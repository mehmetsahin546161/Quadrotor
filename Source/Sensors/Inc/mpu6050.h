#ifndef _MPU6050_H_
#define _MPU6050_H_

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"
#include "com_interface.h"
/* Exported define -----------------------------------------------------------*/

/* MPU6050  Registers */
#define MPU6050_SMPLRT_DIV_REG 			0x19
#define MPU6050_GYRO_CONFIG_REG 		0x1B
#define MPU6050_ACCEL_CONFIG_REG 		0x1C
#define MPU6050_ACCEL_XOUT_H_REG 		0x3B
#define MPU6050_TEMP_OUT_H_REG 			0x41
#define MPU6050_GYRO_XOUT_H_REG 		0x43
#define MPU6050_PWR_MGMT_1_REG 			0x6B
#define MPU6050_WHO_AM_I_REG 				0x75

#define MPU6050_I2C_DEV_ADDR 				0xD0

#define MPU6050_START_OF_ACCEL_DATA_REGS	MPU6050_ACCEL_XOUT_H_REG
#define MPU6050_START_OF_GYRO_DATA_REGS		MPU6050_GYRO_XOUT_H_REG

#define MPU6050_ACCEL_DATA_SCALE_FACTOR   (0.00006103515625)		/* Full Scale Range ±2g */
#define MPU6050_GYRO_DATA_SCALE_FACTOR   	(0.0076335877)				/* Full Scale Range ± 250°/s */

/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/* Register Enums */
typedef enum
{
	MPU6050_INTERNAL_8MHZ 					= 0,
	MPU6050_PLL_X_GYRO_REF 					= 1,
	MPU6050_PLL_Y_GYRO_REF 					= 2,
	MPU6050_PLL_Z_GYRO_REF 					= 3,
	MPU6050_PLL_EXTERNAL_32_768KHZ 	= 4,
	MPU6050_PLL_EXTERNAL_19_2MHZ 		= 5,
	MPU6050_STOP_CLOCK							= 7

}MPU6050_ClockSelection;

typedef enum
{
	MPU6050_ACCEL_FULL_SCALE_2G  = 0,
	MPU6050_ACCEL_FULL_SCALE_4G  = 1,
	MPU6050_ACCEL_FULL_SCALE_8G  = 2,
	MPU6050_ACCEL_FULL_SCALE_16G = 3
	
}MPU6050_AccelFullScaleRange;

typedef enum
{
	MPU6050_GYRO_FULL_SCALE_250_DEG_PER_SEC  = 0,
	MPU6050_GYRO_FULL_SCALE_500_DEG_PER_SEC	 = 1,
	MPU6050_GYRO_FULL_SCALE_1000_DEG_PER_SEC = 2,
	MPU6050_GYRO_FULL_SCALE_2000_DEG_PER_SEC = 3
	
}MPU6050_GyroFullScaleRange;


/* Register Structs */
typedef union
{
	struct
	{
		MPU6050_ClockSelection clockSelection:3;
		bool tempDisable:1;
		bool reserved1:1;
		bool cycle:1;
		bool sleep:1;
		bool deviceReset:1;
	
	}BIT;
	
	uint8_t BYTE;

}MPU6050_PowerManagement1Reg;

typedef union
{
	struct
	{
		uint8_t reserved1:3;
		MPU6050_AccelFullScaleRange fullScaleRange:2;
		bool selTestX:1;
		bool selTestY:1;
		bool selTestZ:1;
	
	}BIT;
	
	uint8_t BYTE;

}MPU6050_AccelConfigReg;

typedef union
{
	struct
	{
		uint8_t reserved1:3;
		MPU6050_GyroFullScaleRange fullScaleRange:2;
		bool selTestX:1;
		bool selTestY:1;
		bool selTestZ:1;
	
	}BIT;
	
	uint8_t BYTE;

}MPU6050_GyroConfigReg;

typedef struct
{
	double rawXData;
	double rawYData;
	double rawZData;

}MPU6050_RawAccelDatas;

typedef struct
{
	double rawXData;
	double rawYData;
	double rawZData;

}MPU6050_RawGyroDatas;

/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void MPU6050_InitSensor(COM_Handle * MPU6050);

void MPU6050_SetSampleRateDivider(COM_Handle * MPU6050, uint8_t sampleRateDiv);
uint8_t MPU6050_GetSampleRateDivider(COM_Handle * MPU6050);

void MPU6050_SetPowerManagement(COM_Handle * MPU6050, const MPU6050_PowerManagement1Reg * powerManagement);
void MPU6050_GetPowerManagement(COM_Handle * MPU6050, MPU6050_PowerManagement1Reg * powerManagement);

void MPU6050_SetAccelConfig(COM_Handle * MPU6050, const MPU6050_AccelConfigReg * accelConfig);
void MPU6050_GetAccelConfig(COM_Handle * MPU6050, MPU6050_AccelConfigReg * accelConfig);

void MPU6050_SetGyroConfig(COM_Handle * MPU6050, const MPU6050_GyroConfigReg * gyroConfig);
void MPU6050_GetGyroConfig(COM_Handle * MPU6050, MPU6050_GyroConfigReg * gyroConfig);

void MPU6050_GetRawAccelDatas(COM_Handle * MPU6050, MPU6050_RawAccelDatas * rawDatas);
void MPU6050_GetRawGyroDatas(COM_Handle * MPU6050, MPU6050_RawGyroDatas * rawDatas);

#endif /* _MPU6050_H_ */
