#ifndef ITG3205_H
#define ITG3205_H

/* Private includes ----------------------------------------------------------*/
#include "stm32f407xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"
#include <stdbool.h>
#include "defines.h"
#include "com_input.h"

/* Exported constants --------------------------------------------------------*/

#define 	WHO_AM_I			0x00
#define		SMPLRT_DIV		0x15
#define		DLPF_FS				0x16
#define		INT_CFG				0x17
#define		INT_STATUS  	0x1A
#define		TEMP_OUT_H		0x1B
#define 	TEMP_OUT_L		0x1C
#define		GYRO_XOUT_H		0x1D
#define		GYRO_XOUT_L		0x1E
#define 	GYRO_YOUT_H		0x1F
#define		GYRO_YOUT_L 	0x20
#define		GYRO_ZOUT_H		0x21
#define		GYRO_ZOUT_L		0x22
#define		PWR_MGM				0x3E

#define 	ITG3205_I2C_DEV_ADDR_GND  				0x68
#define 	ITG3205_I2C_DEV_ADDR_VCC  				0x69

#define   ITG3205_START_OF_DATA_REGS 				TEMP_OUT_H

#define 	ITG3205_GYRO_DATA_SCALE_FACTOR		(0.069565)			/* 1 LSB -----> 0.069565 ?/sec */

   
/* Exported types ------------------------------------------------------------*/

typedef enum
{
	ITG3205_FS_RESERVED_0 				= 0,
	ITG3205_FS_RESERVED_1 				= 1,
	ITG3205_FS_RESERVED_2 				= 2,
	ITG3205_FS_2000_DEG_PER_SEC 	= 3

}ITG3205_FullScaleSel;

typedef enum
{
	ITG3205_LOWPASS_256HZ_SAMPLE_RATE_8kHZ			= 0,
	ITG3205_LOWPASS_188HZ_SAMPLE_RATE_1kHZ			= 1,
	ITG3205_LOWPASS_98HZ_SAMPLE_RATE_1kHZ				= 2,
	ITG3205_LOWPASS_42HZ_SAMPLE_RATE_1kHZ				= 3,
	ITG3205_LOWPASS_20HZ_SAMPLE_RATE_1kHZ				= 4,
	ITG3205_LOWPASS_10HZ_SAMPLE_RATE_1kHZ				= 5,
	ITG3205_LOWPASS_5HZ_SAMPLE_RATE_1kHZ				= 6,
	ITG3205_LOWPASS_AND_SAMPLE_RATE_RESERVED		= 0,
	
}ITG3205_LowPassFilter;


typedef union
{
	struct
	{
		ITG3205_LowPassFilter lowPassFilter : 3;
		ITG3205_FullScaleSel fullScale : 2;
		uint8_t	dummy : 3;
	
	}BIT;

	uint8_t BYTE;
	
}ITG3205_FullScaleAndLowPassReg;

typedef enum
{
	ITG3205_INTERRUPT_ACTIVE_HIGH = 0,
	ITG3205_INTERRUPT_ACTIVE_LOW  = 1,

}ITG3205_IntActLevel;

typedef enum
{
	ITG3205_INTERRUPT_PIN_PUSH_PULL 	= 0,
	ITG3205_INTERRUPT_PIN_OPEN_DRAIN  = 1,

}ITG3205_IntDriveType;

typedef enum
{
	ITG3205_LATCH_INT_50US_PULSE 		= 0,
	ITG3205_LATCH_INT_UNTIL_CLEARED = 1

}ITG3205_IntLatchMode;

typedef enum
{
	ITG3205_CLEAR_WHEN_STATUS_REG_READ 	= 0,
	ITG3205_CLEAR_WHEN_ANY_REG_READ 		= 1,

}ITG3205_IntLatchClearMode;

typedef enum
{
	ITG3205_NOT_ENABLE_INT_WHEN_DEV_READY = 0,
	ITG3205_ENABLE_INT_WHEN_DEV_READY = 1,

}ITG3205_EnableIntDevReady;

typedef enum
{
	ITG3205_NOT_ENABLE_INT_WHEN_DATA_READY = 0,
	ITG3205_ENABLE_INT_WHEN_DATA_READY = 1,

}ITG3205_EnableIntDataReady;

typedef union
{
	struct
	{
		ITG3205_EnableIntDataReady enableIntDataReady : 1;
		uint8_t dummy2 : 1;
		ITG3205_EnableIntDevReady enableintDevReady : 1;
		uint8_t dummy1 : 1;
		ITG3205_IntLatchClearMode intLatchClearMode: 1;
		ITG3205_IntLatchMode intLatchMode : 1;
		ITG3205_IntDriveType intDriveType : 1;
		ITG3205_IntActLevel intActLevel : 1;
		
	}BIT;

	uint8_t BYTE;
	
}ITG3205_InterruptConfigReg;

typedef enum
{
	ITG3205_INTERRUPT_DEV_NOT_READY = 0,
	ITG3205_INTERRUPT_DEV_READY = 1,

}ITG3205_IntDevReady;

typedef enum
{
	ITG3205_INTERRUPT_DATA_NOT_READY = 0,
	ITG3205_INTERRUPT_DATA_READY = 1,

}ITG3205_IntDataReady;


typedef union
{
	struct
	{
		uint8_t dummy1 : 5;
		ITG3205_IntDevReady intDevReady : 1;
		uint8_t dummy2 : 1;
		ITG3205_IntDataReady intDataReady : 1;
		
	}BIT;

	uint8_t BYTE;

}ITG3205_IntStatusReg;

typedef enum
{
	ITG3205_NOT_RESET_REGISTERS = 0,
	ITG3205_RESET_REGISTERS = 1,

}ITG3205_Reset;

typedef enum
{
	ITG3205_DISABLE_SLEEP_MODE = 0,
	ITG3205_ENABLE_SLEEP_MODE = 1,

}ITG3205_Sleep;

typedef enum
{
	ITG3205_GYRO_AXIS_NORMAL_MODE = 0,
	ITG3205_GYRO_AXIS_STANDBY_MODE = 1

}ITG3205_GyroStandbyMode;

typedef enum
{
	ITG3205_INTERNAL_CLOCK 			= 0,
	ITG3205_PLL_X_REF 					= 1,
	ITG3205_PLL_Y_REF 					= 2,
	ITG3205_PLL_Z_REF 					= 3,
	ITG3205_PLL_EXT_32_768kHZ 	= 4,
	ITG3205_PLL_EXT_19_2MHZ 		= 5,
	ITG3205_CLK_SRC_RESERVED_1	= 6,
	ITG3205_CLK_SRC_RESERVED_2	= 7,

}ITG3205_ClockSelection;

typedef struct
{
	float rawTemp;
	float rawXData;
	float rawYData;
	float rawZData;

}ITG3205_RawDatas;

typedef union
{
	struct
	{
		ITG3205_ClockSelection clockSrc : 3;
		ITG3205_GyroStandbyMode zAxisStandby : 1;
		ITG3205_GyroStandbyMode yAxisStandby : 1;
		ITG3205_GyroStandbyMode xAxisStandby : 1;
		ITG3205_Sleep devSleepMod : 1;
		ITG3205_Reset reset;
		
	}BIT;
	
	int8_t BYTE;
	
}ITG3205_PowerManagementReg;

/* Exported functions --------------------------------------------------------*/
void ITG3205_InitSensor(const COM_Input_HandleTypeDef * ITG3205);

uint8_t ITG3205_WhoAmI(const COM_Input_HandleTypeDef * ITG3205);

void ITG3205_SetSampleRateDivider(const COM_Input_HandleTypeDef * ITG3205, uint8_t sampleRateDiv);
uint8_t ITG3205_GetSampleRateDivider(const COM_Input_HandleTypeDef * ITG3205);

void ITG3205_SetFullScaleAndLowPass(const COM_Input_HandleTypeDef * ITG3205, const ITG3205_FullScaleAndLowPassReg * fullScaleAndLowPass);
void ITG3205_GetFullScaleAndLowPass(const COM_Input_HandleTypeDef * ITG3205, ITG3205_FullScaleAndLowPassReg * fullScaleAndLowPass);

void ITG3205_SetInterruptConfig(const COM_Input_HandleTypeDef * ITG3205, const ITG3205_InterruptConfigReg * intConfig);
void ITG3205_GetInterruptConfig(const COM_Input_HandleTypeDef * ITG3205, ITG3205_InterruptConfigReg * intConfig);

void ITG3205_GetInterruptStatus(const COM_Input_HandleTypeDef * ITG3205, ITG3205_IntStatusReg * intStatus);

void ITG3205_GetRawDatas(const COM_Input_HandleTypeDef * ITG3205, ITG3205_RawDatas * rawDatas);

void ITG3205_SetPowerManagement(const COM_Input_HandleTypeDef * ITG3205, const ITG3205_PowerManagementReg * powerManagement);
void ITG3205_GetPowerManagement(const COM_Input_HandleTypeDef * ITG3205, ITG3205_PowerManagementReg * powerManagement);

#endif //ITG3205_H
