/*
 * Odometry.cpp
 *
 *  Created on: Feb 14, 2018
 *      Author: yusaku
 */

#include "Odometry.h"
#include "stm32f10x_conf.h"
#include "Timer.h"

#include <cmath>


Odometry::Odometry(void)
{
	this->x = 0.0f;
	this->y = 0.0f;
	this->yaw = 0.0f;

	this->movavg = 0;

	//this->mpu9250 = new MPU9250(SPI_MPU9250, GPIOC, GPIO_PIN_0);
	this->mpu9250 = new MPU9250(SPI_MPU9250, GPIO_MPU9250, PIN_MPU9250);
}

void Odometry::GetGyroBias(float * const avg, float * const stdev) const
{
	static constexpr int NumOfTrial = 256;

	float _avg = 0.0f;
	float _stdev = 0.0f;

	for(int i = 0; i < NumOfTrial; i++)
	{
		float reading = (int16_t)mpu9250->WriteWord(READ_FLAG | MPUREG_GYRO_ZOUT_H, 0x0000) * 1000.0f / SensitivityScaleFactor;

		_avg += reading;
		_stdev += reading * reading;

		Timer::sleep(5);
	}

	_avg /= NumOfTrial;

	_stdev -= NumOfTrial * _avg * _avg;
	_stdev /= NumOfTrial - 1;
	_stdev = sqrtf(_stdev);

	*avg = _avg;
	*stdev = _stdev;
}

bool Odometry::InitGyro(void)
{
	uint8_t whoami = mpu9250->WriteByte(READ_FLAG | MPUREG_WHOAMI, 0x00);

	if(whoami != 0x71)
	{
		delete mpu9250;
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

	Timer::sleep(100);
	//HAL_Delay(100);

	float avg = 0.0f;
	float stdev = 1000.0f;

	for(int i = 0; i < 10; i++)
	{
		this->GetGyroBias(&avg, &stdev);

		if (stdev < 700)
		{
			movavg = (int32_t)avg;

			return true;
		}
	}

	// gyro unit is not in desirable state (not stabilized)
	return false;
}

void Odometry::ReadEncoder(void)
{
	volatile int16_t _p1 = static_cast<int16_t>(TIM3->CNT);
	TIM3->CNT = 0;

	volatile int16_t _p2 = static_cast<int16_t>(TIM4->CNT);
	TIM4->CNT = 0;

	// just a simple rotation matrix
	// translate encoder rates to velocity on x-y plane
	float _yaw = yaw - ((float)M_PI / 4.0f);
	float _cos = cosf(_yaw);
	float _sin = sinf(_yaw);

	x += ((_p1 * _cos) - (_p2 * _sin)) * MPerPulse;
	y += ((_p1 * _sin) + (_p2 * _cos)) * MPerPulse;
}

void Odometry::ReadGyro(void)
{
	static constexpr int32_t movband = 100;
	static constexpr float RadPerMilliDeg = M_PI / 180000.0;
	static constexpr float RadPerMilliDegPerSec = RadPerMilliDeg / SamplingFrequency;
	static constexpr float w = 0.01f;
	static constexpr float halfPi = M_PI / 2.0;

	int dy_raw_mdps = (((int16_t)mpu9250->WriteWord(READ_FLAG | MPUREG_GYRO_ZOUT_H, 0x0000)) * 1000 / SensitivityScaleFactor) + 0.5f;
	//temp = mpu9250->WriteWord(READ_FLAG | MPUREG_TEMP_OUT_H, 0x0000);

	int dy_biased_mdps = dy_raw_mdps - movavg;

	if(dy_biased_mdps < -movband || movband < dy_biased_mdps)
	{
		// yaw is in radian, so, convert from mdps to radian.
		yaw += (float)dy_biased_mdps * RadPerMilliDegPerSec;

		if(yaw > (float)M_PI)
		{
			yaw -= (2.0f * (float)M_PI);
		}
		else if(yaw < -(float)M_PI)
		{
			yaw += (2.0f * (float)M_PI);
		}
	}
	else
	{
		movavg = (int)((((float)movavg * (1 - w)) + ((float)dy_raw_mdps * w)) + 0.5f);
	}
}

bool Odometry::Initialize(void)
{
	return this->InitGyro();
}

void Odometry::Sample(void)
{
	this->ReadEncoder();
	this->ReadGyro();
}

void Odometry::SetPose(const float x, const float y, const float yaw)
{
	this->x = x;
	this->y = y;
	this->yaw = yaw;
}

void Odometry::GetPose(float * const x, float * const y, float * const yaw)
{
	*x = this->x;
	*y = this->y;
	*yaw = this->yaw;
}








