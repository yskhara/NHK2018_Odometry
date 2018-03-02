/*
 *
 */

#include <math.h>

#include "diag/Trace.h"

#include "Timer.h"

//#include "tf/tf.h"

#include "stm32f10x_conf.h"

#include "Init.h"
#include "Odometry.h"
#include "geometry_msgs/Pose2D.h"
#include "ros.h"


void SetPoseCallback(const geometry_msgs::Pose2D &msg);

ros::NodeHandle nh;

geometry_msgs::Pose2D nav_pose_msg;

ros::Publisher nav_pose_pub("/nav/pose", &nav_pose_msg);
ros::Subscriber<geometry_msgs::Pose2D> nav_set_pose_sub("/nav/set_pose", &SetPoseCallback);

Odometry *odom = new Odometry();

uint32_t led_last = 0;
bool led_stat = false;

void led_tryTurnOn(void)
{
	if(!led_stat)
	{
		uint32_t now = nh.now();
		if(now - led_last > 100)
		{
			// turn on
			GPIOC->BSRR = GPIO_BSRR_BR13;
			led_last = now;
		}
	}
}

void led_process(void)
{
	if(led_stat)
	{
		uint32_t now = nh.now();
		if(now - led_last > 100)
		{
			// turn off
			GPIOC->BSRR = GPIO_BSRR_BS13;
			led_last = now;
		}
	}
}

int main(void)
{
	// rate in Hz
	static constexpr int rate = 20;
	// interval in ms
	static constexpr double interval = 1.0 / rate;

	Timer::start();

	Init::InitGPIO();
	GPIOC->BSRR = GPIO_BSRR_BR13;

	Init::InitTIM();
	Init::InitSPI();
	//InitHardware();

	// Initialize ROS
	nh.initNode();
	nh.advertise(nav_pose_pub);
	nh.subscribe(nav_set_pose_sub);

	nav_pose_msg.x = 0.0f;
	nav_pose_msg.y = 0.0f;
	nav_pose_msg.theta = 0.0f;

	ros::Time last_time = nh.now();
	ros::Time current_time = last_time;//nh.now();

	bool r = odom->Initialize();

	if(!r)
	{
		while(1)
		{
			Timer::sleep(100);
			GPIOB->BSRR = GPIO_BSRR_BR9;
			Timer::sleep(100);
			GPIOB->BSRR = GPIO_BSRR_BS9;
		}
	}

	NVIC_EnableIRQ(TIM4_IRQn);

	while(1)
	{
		current_time = nh.now();

		// Send the message every second
		if(current_time.toSec() - last_time.toSec() > interval)
		{
			float x, y, theta;

			led_tryTurnOn();

			odom->GetPose(&x, &y, &theta);

			nav_pose_msg.x = (double)x;
			nav_pose_msg.y = (double)y;
			nav_pose_msg.theta = (double)theta;

			nav_pose_pub.publish(&nav_pose_msg);

			last_time = current_time;
		}

		nh.spinOnce();

		led_process();
	}

	return 0;
}

void SetPoseCallback(const geometry_msgs::Pose2D &msg)
{
	odom->SetPose((float)msg.x, (float)msg.y, (float)msg.theta);
}

extern "C" void TIM4_IRQHandler(void)
{
	if(TIM4->SR & TIM_SR_UIF)
	{
		odom->Sample();

		TIM4->SR &= ~TIM_SR_UIF;
	}
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
