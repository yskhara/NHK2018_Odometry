/*
 * MPU9250.cpp
 *
 *  Created on: Sep 17, 2017
 *      Author: yusaku
 */

#include "MPU9250.h"
//#include "Global.h"
#include "Timer.h"
//#include <exception>

MPU9250::MPU9250(SPI_TypeDef * const spi, GPIO_TypeDef * const ss_gpio, const uint32_t ss_pin)
	: _spi(spi), _ss_gpio(ss_gpio), _ss_pin(ss_pin)
{
	this->_gyroZ_bias = 0.0;
}

uint8_t MPU9250::_spiWrite(const uint8_t data) const
{
	while(!(_spi->SR & SPI_SR_TXE)) ;
	_spi->DR = data;
	while(!(_spi->SR & SPI_SR_RXNE)) ;
	return (uint8_t)_spi->DR;
}

void MPU9250::_spiChipSelect(void) const
{
	_ss_gpio->BSRR = (_ss_pin << 16);
}

void MPU9250::_spiChipDeselect(void) const
{
	_ss_gpio->BSRR = _ss_pin;
}

uint8_t MPU9250::WriteByte(const uint8_t addr, const uint8_t data) const
{
	volatile uint8_t result = 0x00;

	_spiChipSelect();

	_spiWrite(addr);

	result = _spiWrite(data);

	_spiChipDeselect();

	return result;
}

uint16_t MPU9250::WriteWord(const uint8_t addr, const uint16_t data) const
{
	volatile uint16_t result = 0x00;

	_spiChipSelect();

	_spiWrite(addr);

	result = (uint16_t)(_spiWrite((uint8_t)((data >> 8) & 0xff)) << 8);

	result = (uint16_t)(result | _spiWrite((uint8_t)(data & 0xff)));

	_spiChipDeselect();

	return result;
}

void MPU9250::ReadBurst(const uint8_t addr, const uint16_t cnt, uint8_t * const dest) const
{
	if(dest == nullptr)
	{
		return;
	}

	const uint8_t _a = addr | READ_FLAG;

	for(int i = 0; i < cnt; i++)
	{
		dest[i] = WriteByte(_a, 0x00);
	}
}

float MPU9250::ReadGyroZ(void)
{
	float raw = (int16_t)WriteWord(READ_FLAG | MPUREG_GYRO_ZOUT_H, 0x0000);
	return (raw / SensitivityScaleFactor) - this->_gyroZ_bias;
}

/*
void MPU9250::CalibrateGyroZ(void)
{
	static constexpr int NumOfTrial = 10;
	float bias_sum = 0.0;
	for(int i = 0; i < NumOfTrial; i++)
	{
		//uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
		uint16_t packet_count, fifo_count;
		int32_t gyroZ_bias  = 0;

		// reset device
		WriteByte(MPUREG_PWR_MGMT_1, 0x80); // Set bit 7 (reset bit); toggle reset device
		timer->sleep(100);

		// get stable time source; Auto select clock source to be PLL gyroscope reference if ready
		// else use the internal oscillator, bits 2:0 = 001
		WriteByte(MPUREG_PWR_MGMT_1, 0x01);
		//WriteByte(MPUREG_PWR_MGMT_2, 0x00);
		WriteByte(MPUREG_PWR_MGMT_2, 0x3e);		// disable x-, y-, z-acc, and x-, y-gyro.
		timer->sleep(200);

		// Configure device for bias calculation
		WriteByte(MPUREG_INT_ENABLE, 0x00);   // Disable all interrupts
		WriteByte(MPUREG_FIFO_EN, 0x00);      // Disable FIFO
		WriteByte(MPUREG_PWR_MGMT_1, 0x00);   // Turn on internal clock source
		WriteByte(MPUREG_I2C_MST_CTRL, 0x00); // Disable I2C master
		WriteByte(MPUREG_USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
		WriteByte(MPUREG_USER_CTRL, 0x0C);    // Reset FIFO and DMP
		timer->sleep(15);

		// Configure MPU6050 gyro and accelerometer for bias calculation
		WriteByte(MPUREG_CONFIG, 0x01);      // Set low-pass filter to 188 Hz
		WriteByte(MPUREG_SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
		WriteByte(MPUREG_GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
		//WriteByte(MPUREG_ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

		//uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
		//uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

		// Configure FIFO to capture accelerometer and gyro data for bias calculation
		WriteByte(MPUREG_USER_CTRL, 0x40);   // Enable FIFO
		//WriteByte(MPUREG_FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
		//timer->sleep(40); // accumulate 40 samples in 40 milliseconds = 480 bytes
		WriteByte(MPUREG_FIFO_EN, 0x10);     // Enable only Z-axis gyro sensor for FIFO  (max size 512 bytes in MPU-9250)
		timer->sleep(200); // accumulate 200 samples in 200 milliseconds = 400 bytes

		// At end of sample accumulation, turn off FIFO sensor read
		WriteByte(MPUREG_FIFO_EN, 0x0000);        // Disable gyro and accelerometer sensors for FIFO
		//ReadRegs(MPUREG_FIFO_COUNTH, data, 2); // read FIFO sample count
		//fifo_count = ((uint16_t)data[0] << 8) | data[1];
		fifo_count = WriteWord(READ_FLAG | MPUREG_FIFO_COUNTH, 0x00);
		packet_count = fifo_count / 2;// Sets the amount of full gyro data packets for averaging

		for (int i = 0; i < packet_count; i++)
		{
			int16_t data = WriteWord(READ_FLAG | MPUREG_FIFO_R_W, 0x0000);

			gyroZ_bias  += data; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
		}

		gyroZ_bias /= (int32_t)packet_count; // Normalize sums to get average count biases

		// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
		//data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
		//data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
		//data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
		//data[3] = (-gyro_bias[1]/4)       & 0xFF;
		//data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
		//data[5] = (-gyro_bias[2]/4)       & 0xFF;

		//WriteReg(MPUREG_ZG_OFFS_USRH, data[4]);
		//WriteReg(MPUREG_ZG_OFFS_USRL, data[5]);
		uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec

		//int16_t bias = (int16_t)(-gyroZ_bias / 4);

		//this->_gyroZ_bias = (float)gyroZ_bias / (float)gyrosensitivity;
		bias_sum += (float)gyroZ_bias / (float)gyrosensitivity;

		//this->_gyroZ_bias = 0.38;

		//WriteWord(MPUREG_ZG_OFFS_USRH, (uint16_t)bias);

		//timer->sleep(100);
	}

	this->_gyroZ_bias = bias_sum / (float)NumOfTrial;
}
*/

float MPU9250::MeasureGyroZOffsetFloat(void)
{
	return (float)this->MeasureGyroZOffsetInt();
}

/*
int32_t MPU9250::MeasureGyroZOffsetInt(void)
{
	//uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
	int32_t packet_count, fifo_count;

	// reset device
	WriteByte(MPUREG_PWR_MGMT_1, 0x80); // Set bit 7 (reset bit); toggle reset device
	timer->sleep(100);

	// get stable time source; Auto select clock source to be PLL gyroscope reference if ready
	// else use the internal oscillator, bits 2:0 = 001
	WriteByte(MPUREG_PWR_MGMT_1, 0x01);
	//WriteByte(MPUREG_PWR_MGMT_2, 0x00);
	WriteByte(MPUREG_PWR_MGMT_2, 0x3e);		// disable x-, y-, z-acc, and x-, y-gyro.
	timer->sleep(200);

	// Configure device for bias calculation
	WriteByte(MPUREG_INT_ENABLE, 0x00);   // Disable all interrupts
	WriteByte(MPUREG_FIFO_EN, 0x00);      // Disable FIFO
	WriteByte(MPUREG_PWR_MGMT_1, 0x00);   // Turn on internal clock source
	WriteByte(MPUREG_I2C_MST_CTRL, 0x00); // Disable I2C master
	WriteByte(MPUREG_USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
	WriteByte(MPUREG_USER_CTRL, 0x0C);    // Reset FIFO and DMP
	timer->sleep(15);

	// Configure MPU6050 gyro and accelerometer for bias calculation
	WriteByte(MPUREG_CONFIG, 0x01);      // Set low-pass filter to 188 Hz
	WriteByte(MPUREG_SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
	WriteByte(MPUREG_GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
	//WriteByte(MPUREG_ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

	// Configure FIFO to capture accelerometer and gyro data for bias calculation
	WriteByte(MPUREG_USER_CTRL, 0x40);   // Enable FIFO
	//WriteByte(MPUREG_FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
	//timer->sleep(40); // accumulate 40 samples in 40 milliseconds = 480 bytes
	WriteByte(MPUREG_FIFO_EN, 0x10);     // Enable only Z-axis gyro sensor for FIFO  (max size 512 bytes in MPU-9250)
	timer->sleep(200); // accumulate 200 samples in 200 milliseconds = 400 bytes

	// At end of sample accumulation, turn off FIFO sensor read
	WriteByte(MPUREG_FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO

	if(WriteByte(0x3a, 0x00) & 0x10 != 0)
	{
		//USART1_wrapper->Tx_EnqueueString("mpu fifo ovf\r\n");
	}

	//ReadRegs(MPUREG_FIFO_COUNTH, data, 2); // read FIFO sample count
	//fifo_count = ((uint16_t)data[0] << 8) | data[1];
	fifo_count = (uint16_t)WriteWord(READ_FLAG | MPUREG_FIFO_COUNTH, 0x00);
	packet_count = fifo_count / 2;// Sets the amount of full gyro data packets for averaging

	int32_t gyroZ_bias = 0;

	for (int i = 0; i < packet_count; i++)
	{
		int16_t data = WriteWord(READ_FLAG | MPUREG_FIFO_R_W, 0x0000);

		gyroZ_bias += data; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
	}

	//gyroZ_bias /= (float)packet_count; // Normalize sums to get average count biases

	int32_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec

	return gyroZ_bias / packet_count / gyrosensitivity;
}
*/

