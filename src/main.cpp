
#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"

#include <math.h>

#include "ros.h"
//#include "std_msgs/String.h"
//#include "std_msgs/UInt32.h"
//#include "std_msgs/Float32.h"
//#include "nav_msgs/Odometry.h"
//#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
//#include "geometry_msgs/Quaternion.h"
//#include "sensor_msgs/Imu.h"
#include "Timer.h"

//#include "tf/tf.h"

#include "stm32f10x_conf.h"

#include "Init.h"
#include "MPU9250.h"

ros::NodeHandle nh;

geometry_msgs::TwistWithCovarianceStamped twist0_msg;
//sensor_msgs::Imu imu0_msg;
//std_msgs::Float32 x_msg;
//std_msgs::Float32 y_msg;
//std_msgs::Float32 yaw_msg;
//geometry_msgs::PoseStamped pose_msg;
//std_msgs::UInt32 txbufcnt_msg;

//ros::Publisher x_pub("/x_enc", &x_msg);
//ros::Publisher y_pub("/y_enc", &y_msg);
//ros::Publisher yaw_pub("/yaw_imu", &yaw_msg);
//ros::Publisher pose_pub("/localization/pose", &pose_msg);
//ros::Publisher txbufcnt_pub("txbufcnt", &txbufcnt_msg);
ros::Publisher twist0_pub("twist", &twist0_msg);
//ros::Publisher imu0_pub("imu", &imu0_msg);

//char hello[13] = "hello world!";

MPU9250 *mpu9250 = nullptr;

#define SPI_MPU9250 (SPI2)

// radius of wheels in metre
static constexpr double WheelRadius = 0.024;
// pulse/rev
static constexpr double PulsePerRevolution = 500 * 4;	// wanna use 500 p/r ones...
/// Kpd = 2_pi_r[mm/rev] / Kp[pulse/rev]
static constexpr double MPerPulse = 2.0 * (double)M_PI * WheelRadius / PulsePerRevolution;

static constexpr int32_t SamplingFrequency = 200;
//static constexpr double SamplingInterval = 1.0 / SamplingFrequency;

static constexpr double MPerSecPerPulse = MPerPulse * SamplingFrequency;

double vx = 0;
double vy = 0;
// positive for COUNTER-clockwise
double vyaw = 0;

int divisor = 0;

static int movavg;

bool InitGyro(void)
{
	mpu9250 = new MPU9250(SPI_MPU9250, GPIOB, GPIO_Pin_12);
	uint8_t whoami = mpu9250->WriteByte(READ_FLAG | MPUREG_WHOAMI, 0x00);

	if(whoami != 0x71)
	{
		return false;
	}

	 // get stable time source
	mpu9250->WriteByte(MPUREG_PWR_MGMT_1, 0x03);  // Set clock source to be PLL with z-axis gyroscope reference, bits 2:0 = 011

	 // Configure Gyro and Accelerometer
	 // Disable FSYNC and set accelerometer and gyro bandwidth to 4000 and 250 Hz, respectively;
	 // DLPF_CFG = bits 2:0 = 000; this sets the sample rate at 8 kHz for both
	 // Maximum delay is 0.97 ms which is just over a 1 kHz maximum rate
	//mpu9250->WriteByte(MPUREG_CONFIG, 0x00);
	mpu9250->WriteByte(MPUREG_CONFIG, 0x03);

	 // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	//mpu9250->WriteByte(MPUREG_SMPLRT_DIV, 0x07);  // Use a 1 kHz rate; the same rate set in CONFIG above
	mpu9250->WriteByte(MPUREG_SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; the same rate set in CONFIG above

	mpu9250->WriteByte(MPUREG_GYRO_CONFIG, BITS_FS_1000DPS);

	Timer::sleep(500);

	static constexpr int NumOfTrial = 10;
	float offset = 0.0;
	for(int i = 0; i < NumOfTrial; i++)
	{
		offset += (int16_t)mpu9250->WriteWord(READ_FLAG | MPUREG_GYRO_ZOUT_H, 0x0000);
		Timer::sleep(5);
		//USART1_wrapper->Tx_Enqueue('.');
	}
	offset *= (float)(1000 / (NumOfTrial * SensitivityScaleFactor));

	movavg = (int32_t)offset;

	return true;
}

void ReadEncoders(void)
{
	volatile int16_t _p1 = static_cast<int16_t>(TIM2->CNT);
	TIM2->CNT = 0;

	volatile int16_t _p2 = static_cast<int16_t>(TIM3->CNT);
	TIM3->CNT = 0;

	// just a simple rotation matrix
	// translate encoder rate to x-y plane
	double _yaw = /*yaw +*/ (M_PI / 4.0);
	double _cos = cos(_yaw);
	double _sin = sin(_yaw);

	vx += ((_p1 * _cos) - (_p2 * _sin)) * MPerSecPerPulse;
	vy += ((_p1 * _sin) + (_p2 * _cos)) * MPerSecPerPulse;
}

void ReadGyro(void)
{
	static constexpr int movband = 100;
	static constexpr float RadPerMilliDeg = M_PI / 180000.0;
	static constexpr float w = 0.01f;

	int dy_raw_mdps = (((int16_t)mpu9250->WriteWord(READ_FLAG | MPUREG_GYRO_ZOUT_H, 0x0000)) * 1000 / SensitivityScaleFactor) + 0.5f;
	//temp = mpu9250->WriteWord(READ_FLAG | MPUREG_TEMP_OUT_H, 0x0000);

	int dy_biased_mdps = dy_raw_mdps - movavg;

	if(dy_biased_mdps < -movband || movband < dy_biased_mdps)
	{
		// yaw is in radian, so convert dy from millideg to rad.
		//yaw_md += (int32_t)((float)((dy_biased_mdps) / SamplingFrequency) + 0.5f);
		//yaw += (dy_biased_mdps / SamplingFrequency);
		//yaw += dy_biased_mdps * RadPerMilliDeg / SamplingFrequency;
		vyaw += dy_biased_mdps * RadPerMilliDeg;
	}
	else
	{
		movavg = ((movavg * (1 - w)) + (dy_raw_mdps * w)) + 0.5f;
	}
}

int main(void)
{
	// rate in Hz
	static constexpr int rate = 30;
	// interval in ms
	static constexpr double interval = 1.0 / rate;

	Timer::start();

	Init::InitGPIO();
	GPIOC->BSRR = GPIO_BSRR_BR13;

	Init::InitTIM();
	Init::InitSPI();

	// Initialize ROS
	nh.initNode();

	//char twist_frame_id[32];
	//char imu_frame_id[32];


	nh.advertise(twist0_pub);
	//nh.advertise(imu0_pub);
	//while(!nh.connected()) {nh.spinOnce();}
/*
	if (!nh.getParam("~twist_frame", reinterpret_cast<char **>(&twist_frame_id)))
	{
	     //default values
		strcpy(twist_frame_id, "base_link");
	}*/
	twist0_msg.header.frame_id = "base_link";
/*
	if (!nh.getParam("~imu_frame", reinterpret_cast<char **>(&imu_frame_id)))
	{
	     //default values
		strcpy(imu_frame_id, "base_link");
	}*/
	//imu0_msg.header.frame_id = "base_link";

	ros::Time last_time = nh.now();
	ros::Time current_time = last_time;

	//twist0_msg.header.frame_id = "twist0_link";
	//imu0_msg.header.frame_id = "imu0_link";

	InitGyro();

	NVIC_EnableIRQ(TIM4_IRQn);

	GPIOC->BSRR = GPIO_BSRR_BR13;

	while(1)
	{
		current_time = nh.now();

		if(current_time.toSec() - last_time.toSec() > interval)
		{
			if(divisor > 0)
			{
				twist0_msg.header.stamp = nh.now();
				twist0_msg.twist.twist.linear.x = vy / divisor;
				twist0_msg.twist.twist.linear.y = vx / divisor;
				twist0_msg.twist.twist.angular.z = vyaw / divisor;

				vx = 0.0;
				vy = 0.0;
				vyaw = 0.0;

				divisor = 0;

				//imu0_msg.header.stamp = nh.now();
				//imu0_msg.angular_velocity.z = vyaw;

				//pose_pub.publish(&pose_msg);

				twist0_pub.publish(&twist0_msg);
				//imu0_pub.publish(&imu0_msg);
			}

			last_time = current_time;
		}
		else
		{
		}

		//TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);
		//NVIC_DisableIRQ(TIM4_IRQn);
		//NVIC_DisableIRQ(USART1_IRQn);
		//while(!(USART1->SR & USART_SR_TXE)) ;
		//while(TIM4->SR & TIM_SR_UIF) { TIM4->SR &= ~TIM_SR_UIF; }
		nh.spinOnce();
		//TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
		//NVIC_EnableIRQ(TIM4_IRQn);
		//NVIC_EnableIRQ(USART1_IRQn);
	}

	return 0;
}

extern "C" void TIM4_IRQHandler(void)
{
	if(TIM4->SR & TIM_SR_UIF)
	{
		GPIOC->BSRR = GPIO_BSRR_BR13;

		ReadEncoders();
		ReadGyro();

		divisor++;

		GPIOC->BSRR = GPIO_BSRR_BS13;

		TIM4->SR &= ~TIM_SR_UIF;
	}
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
