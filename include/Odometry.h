/*
 * Odometry.h
 *
 *  Created on: Feb 14, 2018
 *      Author: yusaku
 */
#pragma once

#include "MPU9250.h"
#include <cmath>

#define SPI_MPU9250  SPI2
#define GPIO_MPU9250 GPIOB
#define PIN_MPU9250  GPIO_Pin_12

class Odometry
{
private:
	float x;
	float y;
	float yaw;

	MPU9250 *mpu9250 = nullptr;

	// diameter of wheels in metre
	static constexpr float WheelDiameter = 0.048;
	// pulse/rev
	static constexpr float PulsePerRevolution = 500.0 * 4;
	/// Kpd = 2_pi_r[mm/rev] / Kp[pulse/rev]
	static constexpr float MPerPulse = M_PI * WheelDiameter / PulsePerRevolution;

	// moving average in milli-degree-per-second
	int movavg;

	void GetGyroBias(float * const avg, float * const stdev) const;
	bool InitGyro(void);

	void ReadEncoder(void);
	void ReadGyro(void);

public:
	Odometry(void);

	bool Initialize(void);

	void Sample(void);
	void SetPose(float x, float y, float yaw);
	void GetPose(float *x, float *y, float *yaw);

	static constexpr int32_t SamplingFrequency = 200;
};



















