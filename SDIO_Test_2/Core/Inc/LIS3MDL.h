#include "i2c.h"
#include <stdbool.h>
// bool LIS3MDL_init(deviceType device, sa1State sa1);
void LIS3MDL_enableDefault(I2C_HandleTypeDef* I2Cx, uint8_t imu_address);
void LIS3MDL_writeReg(I2C_HandleTypeDef* I2Cx, uint8_t mag_address, uint8_t reg, uint8_t val);
uint8_t LIS3MDL_readReg(I2C_HandleTypeDef* I2Cx, uint8_t mag_address, uint8_t reg);
int LIS3MDL_read(I2C_HandleTypeDef* I2Cx, uint8_t mag_address, float* x, float* y, float* z);
int16_t LIS3MDL_read2(I2C_HandleTypeDef* I2Cx, uint8_t mag_address);
int LIS3MDL_readRegisters(I2C_HandleTypeDef* I2Cx, uint8_t mag_address, uint8_t address, uint8_t* data, uint16_t length);
void LIS3MDL_vector_normalize(float *a);

