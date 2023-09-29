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


//class LSM6DSOXClass {
//  public:
//    LSM6DSOXClass(TwoWire& wire, uint8_t slaveAddress);
//    LSM6DSOXClass(SPIClass& spi, int csPin, int irqPin);
//    virtual ~LSM6DSOXClass();

//    int begin();
//    void end();

//    // Accelerometer
//    virtual int readAcceleration(float& x, float& y, float& z); // Results are in g (earth gravity).
//    virtual float accelerationSampleRate(); // Sampling rate of the sensor.
//    virtual int accelerationAvailable(); // Check for available data from accelerometer

//    // Gyroscope
//    virtual int readGyroscope(float& x, float& y, float& z); // Results are in degrees/second.
//    virtual float gyroscopeSampleRate(); // Sampling rate of the sensor.
//    virtual int gyroscopeAvailable(); // Check for available data from gyroscope


//  private:
//    int readRegister(uint8_t address);
//    int readRegisters(uint8_t address, uint8_t* data, size_t length);
//    int writeRegister(uint8_t address, uint8_t value);


//  private:
//    TwoWire* _wire;
//    SPIClass* _spi;
//    uint8_t _slaveAddress;
//    int _csPin;
//    int _irqPin;

//    SPISettings _spiSettings;
//};

//extern LSM6DSOXClass IMU;


#include "i2c.h"
int LSM6DSOX_begin(I2C_HandleTypeDef* I2Cx, uint8_t imu_address); // Configure IMUs 
void LSM6DSOX_writeRegister8(I2C_HandleTypeDef* I2Cx, uint8_t imu_address, uint8_t reg, uint8_t val);
uint8_t LSM6DSOX_readRegister8(I2C_HandleTypeDef* I2Cx, uint8_t imu_address, uint8_t reg);
void LSM6DSOX_end(I2C_HandleTypeDef* I2Cx, uint8_t imu_address);

int LSM6DSOX_readAcceleration(I2C_HandleTypeDef* I2Cx, uint8_t imu_address, float* x, float* y, float* z); // Results are in g (earth gravity)
int LSM6DSOX_accelerationAvailable(I2C_HandleTypeDef* I2Cx, uint8_t imu_address); // Check for available data from accelerometer

int LSM6DSOX_readGyroscope(I2C_HandleTypeDef* I2Cx, uint8_t imu_address, float* x, float* y, float* z); // Sampling rate of the sensor.
int LSM6DSOX_gyroscopeAvailable(I2C_HandleTypeDef* I2Cx, uint8_t imu_address);  // Check for available data from gyroscope

int LSM6DSOX_readTemperature(I2C_HandleTypeDef* I2Cx, uint8_t imu_address, float* x);
int LSM6DSOX_temperatureAvailable(I2C_HandleTypeDef* I2Cx, uint8_t imu_address);

int LSM6DSOX_readRegisters(I2C_HandleTypeDef* I2Cx, uint8_t imu_address, uint8_t address, uint8_t* data, uint16_t length);

int16_t LSM6DSOX_readAcceleration2(I2C_HandleTypeDef* I2Cx, uint8_t imu_address);
int16_t LSM6DSOX_readGyroscope2(I2C_HandleTypeDef* I2Cx, uint8_t imu_address);
int16_t LSM6DSOX_readTemperature2(I2C_HandleTypeDef* I2Cx, uint8_t imu_address);



