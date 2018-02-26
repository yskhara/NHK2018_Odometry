#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "diag/Trace.h"

#include "Timer.h"

//#include "tf/tf.h"

#include "stm32f10x_conf.h"

#include "Init.h"
#include "MPU9250.h"
#include "geometry_msgs/Pose2D.h"
#include "ros.h"
#include "std_msgs/Bool.h"

ros::NodeHandle nh;

//nav_msgs::Odometry odom_msg;
//std_msgs::Float32 x_msg;
//std_msgs::Float32 y_msg;
//std_msgs::Float32 yaw_msg;
geometry_msgs::Pose2D pose_delta_msg;
std_msgs::Bool error_msg;
//std_msgs::UInt32 txbufcnt_msg;

//ros::Publisher x_pub("/x_enc", &x_msg);
//ros::Publisher y_pub("/y_enc", &y_msg);
//ros::Publisher yaw_pub("/yaw_imu", &yaw_msg);
ros::Publisher pose_delta_pub("/nav/pose_delta", &pose_delta_msg);
ros::Publisher error_pub("/nav/odom/error", &error_msg);
//ros::Publisher txbufcnt_pub("txbufcnt", &txbufcnt_msg);
//ros::Publisher orientation_pub("orientation", &orientation_msg);

//char hello[13] = "hello world!";

MPU9250 *mpu9250 = nullptr;

#define SPI_MPU9250 (SPI2)

// diameter of wheels in metre
static constexpr double WheelDiameter = 0.048;
// pulse/rev
static constexpr double PulsePerRevolution = 500.0 * 4;
/// Kpd = 2_pi_r[mm/rev] / Kp[pulse/rev]
static constexpr double MPerPulse = (double)M_PI * WheelDiameter / PulsePerRevolution;

static constexpr int32_t SamplingFrequency = 200;

double x = 0;
double y = 0;
// positive for COUNTER-clockwise
double yaw = 0;

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
	// translate encoder rates to velocity on x-y plane
	double _yaw = yaw - (M_PI / 4.0);
	double _cos = cos(_yaw);
	double _sin = sin(_yaw);

	x += ((_p1 * _cos) - (_p2 * _sin)) * MPerPulse;
	y += ((_p1 * _sin) + (_p2 * _cos)) * MPerPulse;
}

void ReadGyro(void)
{
	static constexpr int32_t movband = 100;
	static constexpr double RadPerMilliDeg = M_PI / 180000.0;
	static constexpr double RadPerMilliDegPerSec = RadPerMilliDeg / SamplingFrequency;
	static constexpr float w = 0.01f;

	int dy_raw_mdps = (((int16_t)mpu9250->WriteWord(READ_FLAG | MPUREG_GYRO_ZOUT_H, 0x0000)) * 1000 / SensitivityScaleFactor) + 0.5f;
	//temp = mpu9250->WriteWord(READ_FLAG | MPUREG_TEMP_OUT_H, 0x0000);

	int dy_biased_mdps = dy_raw_mdps - movavg;

	if(dy_biased_mdps < -movband || movband < dy_biased_mdps)
	{
		// yaw is in radian, so, convert from mdps to radian.
		yaw += dy_biased_mdps * RadPerMilliDegPerSec;
	}
	else
	{
		movavg = ((movavg * (1 - w)) + (dy_raw_mdps * w)) + 0.5f;
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
	nh.advertise(pose_delta_pub);
	nh.advertise(error_pub);

	pose_delta_msg.x = 0;
	pose_delta_msg.y = 0;
	pose_delta_msg.theta = 0;

	ros::Time last_time = nh.now();
	ros::Time current_time = last_time;//nh.now();

	InitGyro();

	NVIC_EnableIRQ(TIM4_IRQn);

	GPIOC->BSRR = GPIO_BSRR_BR13;

	while(1)
	{
		current_time = nh.now();

		// Send the message every second
		if(current_time.toSec() - last_time.toSec() > interval)
		{
			pose_delta_msg.x = x;
			pose_delta_msg.y = y;
			pose_delta_msg.theta = yaw;

			pose_delta_pub.publish(&pose_delta_msg);

			error_msg.data = Uart::Uart1->Error();
			error_pub.publish(&error_msg);

			x = 0;
			y = 0;
			yaw = 0;

			last_time = current_time;
		}
		else
		{
			GPIOC->BSRR = GPIO_BSRR_BR13;
		}

		nh.spinOnce();

		GPIOC->BSRR = GPIO_BSRR_BS13;
	}

	return 0;
}

extern "C" void TIM4_IRQHandler(void)
{
	if(TIM4->SR & TIM_SR_UIF)
	{

		ReadEncoders();
		ReadGyro();


		TIM4->SR &= ~TIM_SR_UIF;
	}
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
