#ifndef _HMC5883L_H_
#define _HMC5883L_H_

/* Includes ------------------------------------------------------------------*/
#include 	"stdint.h"
#include 	"com_interface.h"
#include	"imu.h"

/* Exported constants --------------------------------------------------------*/
#define 	HMC5883L_CONFIG_REGISTER_A_ADDR						0x00
#define 	HMC5883L_CONFIG_REGISTER_B_ADDR						0x01
#define 	HMC5883L_MODE_REGISTER_ADDR								0x02
#define 	HMC5883L_DATA_OUTPUT_X_MSB_ADDR						0x03
#define 	HMC5883L_DATA_OUTPUT_X_LSB_ADDR						0x04
#define 	HMC5883L_DATA_OUTPUT_Z_MSB_ADDR						0x05
#define 	HMC5883L_DATA_OUTPUT_Z_LSB_ADDR						0x06
#define 	HMC5883L_DATA_OUTPUT_Y_MSB_ADDR						0x07
#define 	HMC5883L_DATA_OUTPUT_Y_LSB_ADDR						0x08
#define 	HMC5883L_STATUS_REGISTER_ADDR							0x09
#define 	HMC5883L_IDENTIFICATION_REGISTER_A_ADDR		0x0A
#define 	HMC5883L_IDENTIFICATION_REGISTER_B_ADDR		0x0B
#define 	HMC5883L_IDENTIFICATION_REGISTER_C_ADDR		0x0C

#define 	HMC5883L_I2C_DEV_ADDR   									0x1E		/* Not Shifted */
#define		HMC5883L_IDENTIFICATION_COUNT							0x03

#define   HMC5883L_START_OF_DATA_REGS 							HMC5883L_DATA_OUTPUT_X_MSB_ADDR

#define 	HMC5883L_MAGNETIC_DATA_SCALE_FACTOR				(0.00092)			/* 1 LSB -----> 9.2e-4 Gauss */
/* Exported macro ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Register Enums */
typedef enum
{
	HMC5883L_1_SAMPLE_PER_OUTPUT = 0,
	HMC5883L_2_SAMPLE_PER_OUTPUT = 1,
	HMC5883L_4_SAMPLE_PER_OUTPUT = 2,
	HMC5883L_8_SAMPLE_PER_OUTPUT = 3,

}HMC5883L_SampleCountPerOutput;

typedef enum
{
	HMC5883L_CONT_MODE_0_75Hz 	= 0,
	HMC5883L_CONT_MODE_1_5Hz 		= 1,
	HMC5883L_CONT_MODE_3Hz 			= 2,
	HMC5883L_CONT_MODE_7_5Hz 		= 3,
	HMC5883L_CONT_MODE_15Hz 		= 4,
	HMC5883L_CONT_MODE_30Hz 		= 5,
	HMC5883L_CONT_MODE_75Hz 		= 6,
	HMC5883L_CONT_MODE_RESERVED = 7,

}HMC5883L_ContModeOutputRate;

typedef enum
{
	HMC5883L_NORMAL_MEAUSUREMENT 					= 0,
	HMC5883L_POSITIVE_BIAS_MEAUSUREMENT		= 1,
	HMC5883L_NEGATIVE_BIAS_MEAUSUREMENT 	= 2,
	HMC5883L_RESERVED_MEAUSUREMENT 				= 3

}HMC5883L_MeausurementMode;

typedef enum
{
	HMC5883L_BIT_PER_GAUSS_1370 = 0,
	HMC5883L_BIT_PER_GAUSS_1090 = 1,
	HMC5883L_BIT_PER_GAUSS_820  = 2,
	HMC5883L_BIT_PER_GAUSS_660  = 3,
	HMC5883L_BIT_PER_GAUSS_440  = 4,
	HMC5883L_BIT_PER_GAUSS_390  = 5,
	HMC5883L_BIT_PER_GAUSS_330  = 6,
	HMC5883L_BIT_PER_GAUSS_230  = 7

}HMC5883L_GainSettings;

typedef enum
{
	HMC5883L_HIGH_SPEED_I2C_DISABLE = 0,
	HMC5883L_HIGH_SPEED_I2C_ENABLE 	= 1,
	
}HMC5883L_HighSpeedI2C;

typedef enum
{
	HMC5883L_CONT_MEAUSUREMENT_MODE 		= 0,
	HMC5883L_SINGLE_MEAUSUREMENT_MODE 	= 1,
	HMC5883L_IDLE_MODE 	= 2,

}HMC5883L_OperatingMode;

typedef enum
{
	HMC5883L_NOT_READY 	= 0,
	HMC5883L_READY 			= 1,

}HMC5883L_ReadyStatus;

typedef enum
{
	HMC5883L_UNLOCKED = 0,
	HMC5883L_LOCKED 	= 1,

}HMC5883L_LockStatus;

typedef enum
{
	HMC5883L_IDENTIFICATION_REG_A = 0,
	HMC5883L_IDENTIFICATION_REG_B = 1,
	HMC5883L_IDENTIFICATION_REG_C = 2

}HMC5883L_IdentificationIndices;



/* Register Structs */
typedef union
{
	struct
	{
		HMC5883L_MeausurementMode meausurementMode:2;
		HMC5883L_ContModeOutputRate contModeOutputRate:3;
		HMC5883L_SampleCountPerOutput sampleCountPerOutput:1;
		uint8_t reserved:1;
	
	}BIT;
	
	uint8_t	BYTE;

}HMC5883L_ConfigAReg;

typedef union
{
	struct
	{
		uint8_t reserved:5;
		HMC5883L_GainSettings gainSettings:3;
	
	}BIT;

	uint8_t	BYTE;
	
}HMC5883L_ConfigBReg;

typedef union
{
	struct
	{
		HMC5883L_HighSpeedI2C highSpeedI2C:1;
		uint8_t reserved:5;
		HMC5883L_OperatingMode operatingMode:2;
	
	}BIT;
	
	uint8_t	BYTE;

}HMC5883L_ModeReg;

typedef struct
{
	double rawXData;
	double rawYData;
	double rawZData;

}HMC5883L_RawDatas;

typedef union
{
	struct
	{
		HMC5883L_ReadyStatus ready:1;
		HMC5883L_LockStatus  lock;
		uint8_t reserved:6;
	
	}BIT;
	
	uint8_t	BYTE;

}HMC5883L_StatusReg;

typedef struct
{
	uint8_t identification[HMC5883L_IDENTIFICATION_COUNT];

}HMC5883L_Identification;

/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void HMC5883L_InitSensor(COM_Handle * HMC5883L);

void HMC5883L_SetConfiguration(COM_Handle * HMC5883L, const HMC5883L_ConfigAReg * configA, const HMC5883L_ConfigBReg * configB);
void HMC5883L_GetConfiguration(COM_Handle * HMC5883L, HMC5883L_ConfigAReg * configA, HMC5883L_ConfigBReg * configB);

void HMC5883L_SetMode(COM_Handle * HMC5883L, const HMC5883L_ModeReg * mode);
void HMC5883L_GetMode(COM_Handle * HMC5883L, HMC5883L_ModeReg * mode);

void HMC5883L_GetRawDatas(COM_Handle * HMC5883L, HMC5883L_RawDatas * rawDatas);

void HMC5883L_GetStatus(COM_Handle * HMC5883L, HMC5883L_StatusReg * status);

void HMC5883L_GetIdentification(COM_Handle * HMC5883L, HMC5883L_Identification * identification);

#endif //_HMC5883L_H_
