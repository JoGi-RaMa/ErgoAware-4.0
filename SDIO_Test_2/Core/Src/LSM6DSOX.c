/*
  This file is part of the Arduino_LSM6DSOX library.
  Copyright (c) 2021 Arduino SA. All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "LSM6DSOX.h"
//#define LSM6DSOX_ADDRESS            0x6A - Original address


#define LSM6DSOX_WHO_AM_I_REG       0x0F
#define LSM6DSOX_CTRL1_XL           0x10
#define LSM6DSOX_CTRL2_G            0x11

#define LSM6DSOX_STATUS_REG         0x1E

#define LSM6DSOX_CTRL6_C            0x15
#define LSM6DSOX_CTRL7_G            0x16
#define LSM6DSOX_CTRL8_XL           0x17

#define LSM6DSOX_OUT_TEMP_L					0x20
#define LSM6DSOX_OUT_TEMP_H					0x21

#define LSM6DSOX_OUTX_L_G           0x22
#define LSM6DSOX_OUTX_H_G           0x23
#define LSM6DSOX_OUTY_L_G           0x24
#define LSM6DSOX_OUTY_H_G           0x25
#define LSM6DSOX_OUTZ_L_G           0x26
#define LSM6DSOX_OUTZ_H_G           0x27

#define LSM6DSOX_OUTX_L_XL          0x28
#define LSM6DSOX_OUTX_H_XL          0x29
#define LSM6DSOX_OUTY_L_XL          0x2A
#define LSM6DSOX_OUTY_H_XL          0x2B
#define LSM6DSOX_OUTZ_L_XL          0x2C
#define LSM6DSOX_OUTZ_H_XL          0x2D

/* Acce sensitivities in g/s -------------------------------------------------*/
float acc_FS = 0.0f;
float gyr_FS = 0.0f;
float temp_sen = 256;
float t_off = 25; //�C

int16_t data_acc[3];
int16_t data_gyr[3];
int16_t data_temp[1];


int LSM6DSOX_begin(I2C_HandleTypeDef* I2Cx, uint8_t imu_address) // Wake up the sensor boards -> Configurations
{
//  if (_spi != NULL) {
//    pinMode(_csPin, OUTPUT);
//    digitalWrite(_csPin, HIGH);
//    _spi->begin();
//  } else {
//    _wire->begin();
//  }
	
	// I2C_HandleTypeDef* I2CX = I2Cx;
	

  if (LSM6DSOX_readRegister8(I2Cx, imu_address, LSM6DSOX_WHO_AM_I_REG) != 0x6C) {
    //end();
		// I2C_HandleTypeDef* I2Cx = I2Cx; // Resende
		LSM6DSOX_end(I2Cx, imu_address);
    return 0;
  }
	
	// Set the ODR config register to ODR/4
  // HPCFXL = 0000 -> ODR/4 Bandwidth
	// HP_REF_MODE_CL = 0 -> Enables high-pass filter
	// FASTSETTL_MODE_XL = 0 -> Enables LPF2 and fast-setting mode 
	// HP_SLOPE_XL_EN = 0
	// XL_FS_MODE = 0
	// LOW_PASS_ON_6D = 0
	// 00000000 -> 0x00
	LSM6DSOX_writeRegister8(I2Cx, imu_address, LSM6DSOX_CTRL8_XL, 0x00); // Resende
	
  // Set the Accelerometer control register to work at 1.66kHz, 16g,and in bypass mode and enable ODR/4
  // low pass filter (check figure9 of LSM6DSOX's datasheet)
	// ODR_XL|FS_XS|LPF2_XL_EN|0:
	// ODR_XL = 1000 for 1.66kHz de fs
	// Considering XL_FS_MODE='0'
	// For 16g: FS_XS = 01
	// For 4g: FS_XS = 10
	// For 2g: FS_XS = 00
	// LPF2_XL_EN = 0 -> disable LPF2 in acc acquisition
	// 10000100 -> 0x84
  LSM6DSOX_writeRegister8(I2Cx, imu_address, LSM6DSOX_CTRL1_XL, 0x84);
	acc_FS = 16; //g
	
	// Set gyroscope power mode to high performance and bandwidth to 16 mHz
  // G_HM_MODE = 0
	// HP_EN = 0
	// HPM_GN = 00 => (16mHz)
	// OIS_ON_EN = 0
	// USR_OFF_ON_OUT = 0
	// OIS_ON = 0
	// 00000000 -> 0x00
	LSM6DSOX_writeRegister8(I2Cx, imu_address, LSM6DSOX_CTRL7_G, 0x00);
	
	// If it is desired to use LPF1 in gyroscope acquisition, please set CTRL_6
	
	// Set the gyroscope control register to work at 1.66kHz, 2000 dps and in bypass mode
	// ODR_G|FS_G|FS_125|0:
	// ODR_G = 1000 for 1.66kHz de fs
	// For 250dps: FS_G = 00
	// For 500dps: FS_G = 01
	// For 1000dps: FS_G = 10
	// For 2000dps: FS_G = 11
	// FS_125 = 0
	// 10001100 -> 0x8C
  LSM6DSOX_writeRegister8(I2Cx, imu_address, LSM6DSOX_CTRL2_G, 0x74);
  gyr_FS = 500; // dps

  return 1;
}



uint8_t LSM6DSOX_readRegister8(I2C_HandleTypeDef* I2Cx, uint8_t imu_address, uint8_t reg) { // Altered to include various i2c peripheral addresses - Resende

  uint8_t x;
  HAL_StatusTypeDef result;
	// I2C_HandleTypeDef* I2Cx = I2Cx; // TODO: Remove this after and see if it works - Resende
  
  result = HAL_I2C_Master_Transmit( I2Cx, (imu_address << 1), &reg, 1, 10 ); // Original device address - (LSM6DSOX_ADDRESS << 1) - Resende
  if( result != HAL_OK ) 
  {
    /*Failed to send register value to driver*/
    x = 0;
  }
  
  result = HAL_I2C_Master_Receive ( I2Cx, (imu_address << 1) , &x  , 1, 10 ); // TODO: Try to commute the least valued bit to 1 - read mode
  if( result != HAL_OK ) 
  {
    /*Failed to send register value to driver*/
    x = 0;
  }
  
  return x;
}


void LSM6DSOX_writeRegister8(I2C_HandleTypeDef* I2Cx, uint8_t imu_address, uint8_t reg, uint8_t val) { // Altered to include various i2c peripheral addresses - Resende
   
  HAL_StatusTypeDef result;
  // I2C_HandleTypeDef* I2Cx = I2Cx; // TODO: Remove this after and see if this works - Resende
  uint8_t data[2];
  
  data[0] = reg;
  data[1] = val;
  
  result = HAL_I2C_Master_Transmit( I2Cx, (imu_address << 1), data, 2, 10 ); // Original device address - (LSM6DSOX_ADDRESS << 1) - Resende
  if( result != HAL_OK ) 
  {
    /*Failed to send register value to driver*/    
  }
  
}

void LSM6DSOX_end(I2C_HandleTypeDef* I2Cx, uint8_t imu_address)
{
		
		// I2C_HandleTypeDef* I2Cx = I2Cx // Resende
	
    LSM6DSOX_writeRegister8(I2Cx, imu_address, LSM6DSOX_CTRL2_G, 0x00);
    LSM6DSOX_writeRegister8(I2Cx, imu_address, LSM6DSOX_CTRL1_XL, 0x00);
    // Desconnectar I2C _wire->end();
  
}

//returns data in g
int LSM6DSOX_readAcceleration(I2C_HandleTypeDef* I2Cx, uint8_t imu_address, float* x, float* y, float* z)
{	
	// I2C_HandleTypeDef* I2Cx = I2Cx // Resende
  int16_t data[3];

  if (!LSM6DSOX_readRegisters(I2Cx, imu_address, LSM6DSOX_OUTX_L_XL, (uint8_t*)data, sizeof(data))) {
    x = 0;
    y = 0;
    z = 0;

    return 0;
  }

  *x = data[0] * acc_FS / 32768.0; // (2^16)/2 = 32768 (it is dividev by two since data is int16 - signed values - with allowed negative values)
  *y = data[1] * acc_FS / 32768.0;
  *z = data[2] * acc_FS / 32768.0;

  return 1;
	
}

//returns data in g
int16_t LSM6DSOX_readAcceleration2(I2C_HandleTypeDef* I2Cx, uint8_t imu_address)
{
  
	//I2C_HandleTypeDef* I2Cx = I2Cx // Resende
  if (!LSM6DSOX_readRegisters(I2Cx, imu_address, LSM6DSOX_OUTX_L_XL, (uint8_t*)data_acc, sizeof(data_acc))) {
    data_acc[0] = 0;
    data_acc[1] = 0;
    data_acc[2] = 0;

    return 0;
  }
	else {
		return *data_acc;
	}
}


int LSM6DSOX_accelerationAvailable(I2C_HandleTypeDef* I2Cx, uint8_t imu_address)
{
	//I2C_HandleTypeDef* I2Cx = I2Cx // Resende
  if (LSM6DSOX_readRegister8(I2Cx, imu_address, LSM6DSOX_STATUS_REG) & 0x01) {
    return 1;
  }

  return 0;
}


//returns data in dps
int LSM6DSOX_readGyroscope(I2C_HandleTypeDef* I2Cx, uint8_t imu_address, float* x, float* y, float* z)
{		
		//I2C_HandleTypeDef* I2Cx = I2Cx //Resende
	  int16_t data[3];

  if (!LSM6DSOX_readRegisters(I2Cx, imu_address, LSM6DSOX_OUTX_L_G, (uint8_t*)data, sizeof(data))) {
    x = 0;
    y = 0;
    z = 0;

    return 0;
  }
	
	*x = data[0] * gyr_FS / 32768.0;  // (2^16)/2 = 32768 (it is dividev by two since data is int16 - signed values - with allowed negative values)
  *y = data[1] * gyr_FS / 32768.0;  // (2^16)/2 = 32768 (it is dividev by two since data is int16 - signed values - with allowed negative values)
  *z = data[2] * gyr_FS / 32768.0; // (2^16)/2 = 32768 (it is dividev by two since data is int16 - signed values - with allowed negative values) 
	
  return 1;
	
}

//returns data in dps
int16_t LSM6DSOX_readGyroscope2(I2C_HandleTypeDef* I2Cx, uint8_t imu_address)
{
	
	//I2C_HandleTypeDef* I2Cx = I2Cx // Resende
	if (!LSM6DSOX_readRegisters(I2Cx, imu_address, LSM6DSOX_OUTX_L_G, (uint8_t*)data_gyr, sizeof(data_gyr))) {
    data_gyr[0] = 0;
    data_gyr[1] = 0;
    data_gyr[2] = 0;

    return 0;
  }
	else {
		return *data_gyr;
	}
}

int LSM6DSOX_gyroscopeAvailable(I2C_HandleTypeDef* I2Cx, uint8_t imu_address)
{
	//I2C_HandleTypeDef* I2Cx = I2Cx
  if (LSM6DSOX_readRegister8(I2Cx, imu_address, LSM6DSOX_STATUS_REG) & 0x02) {
    return 1;
  }

  return 0;
}


//returns data in �C
int LSM6DSOX_readTemperature(I2C_HandleTypeDef* I2Cx, uint8_t imu_address, float* x)
{ 
	//I2C_HandleTypeDef* I2Cx = I2Cx //Resende
  int16_t data[1];

  if (!LSM6DSOX_readRegisters(I2Cx, imu_address, LSM6DSOX_OUT_TEMP_L, (uint8_t*)data, sizeof(data))) {
    x = 0;

    return 0;
  }

  *x = data[0] / temp_sen + t_off;

  return 1;
}

int16_t LSM6DSOX_readTemperature2(I2C_HandleTypeDef* I2Cx, uint8_t imu_address)
{
	//I2C_HandleTypeDef* I2Cx = I2Cx // Resende
	if (!LSM6DSOX_readRegisters(I2Cx, imu_address, LSM6DSOX_OUT_TEMP_L, (uint8_t*)data_temp, sizeof(data_temp))) {

    data_temp[0] = 0;
    return 0;
  }
	else {
		return *data_temp;
	}
	
}


int LSM6DSOX_temperatureAvailable(I2C_HandleTypeDef* I2Cx, uint8_t imu_address)
{
	//I2C_HandleTypeDef* I2Cx = I2Cx
  if (LSM6DSOX_readRegister8(I2Cx, imu_address, LSM6DSOX_STATUS_REG) & 0x04) {
    return 1;
  }

  return 0;
}



int LSM6DSOX_readRegisters(I2C_HandleTypeDef* I2Cx, uint8_t imu_address , uint8_t address, uint8_t* data, uint16_t length)
{	
	uint8_t x;
  HAL_StatusTypeDef result;
	//I2C_HandleTypeDef* I2Cx = I2Cx // TODO: Remove this after and see if this works
  
  result = HAL_I2C_Master_Transmit( I2Cx, (imu_address << 1), &address, 1, 10 ); // Original address - (LSM6DSOX_ADDRESS << 1) - Resende
  if( result != HAL_OK ) 
  {
    /*Failed to send register value to driver*/
    //x = 0;
		return 0;
  }
  
  result = HAL_I2C_Master_Receive ( I2Cx, (imu_address << 1), data, length, 10);
  if( result != HAL_OK ) 
  {
    /*Failed to send register value to driver*/
    //x = 0;
		return 0;
  }
  
  return 1;
	//return x;
}

/**
  * @brief  This function is executed to calibrate a specific imu.
  * @retval None
  */
//void Calibrate_IMU(I2C_HandleTypeDef* I2Cx, uint8_t imu_address)

