#include <LIS3MDL.h>


// Defines ////////////////////////////////////////////////////////////////

// The Arduino two-wire interface uses a 7-bit number for the address,
// and sets the last bit correctly based on reads and writes
#define LIS3MDL_SA1_HIGH_ADDRESS  0b0011110
#define LIS3MDL_SA1_LOW_ADDRESS   0b0011100

#define TEST_REG_ERROR -1

#define LIS3MDL_WHO_ID  0x3D

enum deviceType { device_LIS3MDL, device_auto };
enum sa1State { sa1_low, sa1_high, sa1_auto };

// register addresses
enum regAddr
{
	WHO_AM_I    = 0x0F,

	CTRL_REG1   = 0x20,
	CTRL_REG2   = 0x21,
	CTRL_REG3   = 0x22,
	CTRL_REG4   = 0x23,
	CTRL_REG5   = 0x24,

	STATUS_REG  = 0x27,
	OUT_X_L     = 0x28,
	OUT_X_H     = 0x29,
	OUT_Y_L     = 0x2A,
	OUT_Y_H     = 0x2B,
	OUT_Z_L     = 0x2C,
	OUT_Z_H     = 0x2D,
	TEMP_OUT_L  = 0x2E,
	TEMP_OUT_H  = 0x2F,
	INT_CFG     = 0x30,
	INT_SRC     = 0x31,
	INT_THS_L   = 0x32,
	INT_THS_H   = 0x33,
};

#define I2C_ADDRESS_0  0x1C // Default I2C mag address
#define I2C_ADDRESS_1  0x1E // Second possible I2C mag address

float mag_FS = 0.0;
int16_t data_mag[3];

/*
Enables the LIS3MDL's magnetometer. Also:
- Selects ultra-high-performance mode for all axes
- Sets ODR (output data rate) to default power-on value of 10 Hz
- Sets magnetometer full scale (gain) to default power-on value of +/- 4 gauss
- Enables continuous conversion mode
Note that this function will also reset other settings controlled by
the registers it writes to.
*/
void LIS3MDL_enableDefault(I2C_HandleTypeDef* I2Cx, uint8_t imu_address)
{
		// TEMP_EN|OM|DO|FAST_ODR|ST
		// TEMP_EN = 0 -> disabled temperature sensor
		// OM = 11 -> ultrahigh performance for X and Y axes
		// DO = 111 -> ODR (output data rate) = 80Hz
		// FAST_ODR = 0 -> disabled ODR higher than 80Hz
		// ST = 0 ->  disabled self-test
		// 01111100 -> 0x7C
		LIS3MDL_writeReg(I2Cx, imu_address, CTRL_REG1, 0x7C);

		// 0|FS|0|REBOOT|SOFT_RST|0|0
		// FS = 11 (+/- 16 gauss full scale)
		// For 16gauss: FS_XS = 11
		// For 12gauss: FS_XS = 10
		// For 8 gauss: FS_XS = 01
		// For 4 gauss: FS_XS = 00
		// REBOOT = 0
		// SOFT_RST = 0
		// 0110 0000 -> 0x60
		LIS3MDL_writeReg(I2Cx, imu_address, CTRL_REG2, 0x60);
		mag_FS = 16; //gauss
	
		// 0x00 = 0b00000000
		// MD = 00 (continuous-conversion mode)
		LIS3MDL_writeReg(I2Cx, imu_address, CTRL_REG3, 0x00);

		// 0x0C = 0b00001100
		// OMZ = 11 (ultra-high-performance mode for Z)
		LIS3MDL_writeReg(I2Cx, imu_address, CTRL_REG4, 0x0C);
	}

// Writes a mag register
void LIS3MDL_writeReg(I2C_HandleTypeDef* I2Cx, uint8_t mag_address, uint8_t reg, uint8_t val)
{

	HAL_StatusTypeDef result;
  // I2C_HandleTypeDef* I2Cx = I2Cx
	
  uint8_t data[2];
  
  data[0] = reg;
  data[1] = val;
  
  result = HAL_I2C_Master_Transmit( I2Cx, (mag_address << 1),  data, 2, 10 );
  if( result != HAL_OK ) 
  {
    /*Failed to send register value to driver*/    
  }
	
	
}

// Reads a mag register
uint8_t LIS3MDL_readReg(I2C_HandleTypeDef* I2Cx, uint8_t mag_address, uint8_t reg)
{

	uint8_t x;
  HAL_StatusTypeDef result;
	//I2C_HandleTypeDef* I2Cx = I2Cx
  
  result = HAL_I2C_Master_Transmit( I2Cx, (mag_address << 1), &reg, 1, 10 );
  if( result != HAL_OK ) 
  {
    /*Failed to send register value to driver*/
    x = 0;
  }
  
  result = HAL_I2C_Master_Receive ( I2Cx, (mag_address << 1) , &x  , 1, 10 );
  if( result != HAL_OK ) 
  {
    /*Failed to send register value to driver*/
    x = 0;
  }
  
  return x;
	
}

// Reads the 3 mag channels and stores them in vector m
int LIS3MDL_read(I2C_HandleTypeDef* I2Cx, uint8_t mag_address, float* x, float* y, float* z)
{
	
	int16_t data[3];
	//I2C_HandleTypeDef* I2Cx = I2Cx

  if (!LIS3MDL_readRegisters(I2Cx, mag_address, OUT_X_L | 0x80, (uint8_t*)data, sizeof(data))) {
    x = 0;
    y = 0;
    z = 0;

    return 0;
  }

	// 1 gauss equals to 0.0001 T
  *x = data[0] * mag_FS / 32768.0; // (2^16)/2 = 32768 (it is dividev by two since data is int16 - signed values - with allowed negative values)
  *y = data[1] * mag_FS / 32768.0; // (2^16)/2 = 32768 (it is dividev by two since data is int16 - signed values - with allowed negative values)
  *z = data[2] * mag_FS / 32768.0; // (2^16)/2 = 32768 (it is dividev by two since data is int16 - signed values - with allowed negative values)

  return 1;
	
}


// Reads the 3 mag channels and stores them in vector m
int16_t LIS3MDL_read2(I2C_HandleTypeDef* I2Cx, uint8_t mag_address)
{
	
	//I2C_HandleTypeDef* I2Cx = I2Cx;
	
	if (!LIS3MDL_readRegisters(I2Cx, mag_address, OUT_X_L | 0x80, (uint8_t*)data_mag, sizeof(data_mag))) {
    data_mag[0] = 0;
    data_mag[1] = 0;
    data_mag[2] = 0;

    return 0;
  }
	else {
		return *data_mag;
	}
}


int LIS3MDL_readRegisters(I2C_HandleTypeDef* I2Cx, uint8_t mag_address, uint8_t address, uint8_t* data, uint16_t length)
{
	
	//I2C_HandleTypeDef* I2Cx = I2Cx
	uint8_t x;
  HAL_StatusTypeDef result;
  
  result = HAL_I2C_Master_Transmit( I2Cx, (mag_address << 1), &address, 1, 10 );
  if( result != HAL_OK ) 
  {
    /*Failed to send register value to driver*/
		return 0;
  }
  
  result = HAL_I2C_Master_Receive ( I2Cx, (mag_address << 1) , data  , length, 10 );
  if( result != HAL_OK ) 
  {
    /*Failed to send register value to driver*/
		return 0;
  }
  
  return 1;
}
