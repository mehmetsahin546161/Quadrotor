#ifndef _ADXL345_H_
#define _ADXL345_H_

/* Private includes ----------------------------------------------------------*/
#include "stm32f407xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"
#include <stdbool.h>
#include "defines.h"
#include "com_input.h"

/* Exported constants --------------------------------------------------------*/
				
#define 	ADXL345_I2C_DEV_ADDR_GND  				0x53			//Not shifted
#define 	ADXL345_I2C_DEV_ADDR_VCC  				0x1D			//Not shifted
	
#define 	TAP_THRESH_SCALE_FACTOR 					62.5
#define 	OFFSET_SCALE_FACTOR								15.6
#define		DUR_TIME_SCALE_FACTOR							625.0
#define		LATENT_TIME_SCALE_FACTOR					1.25
#define		WINDOW_TIME_SCALE_FACTOR					1.25
#define		ACTIVITY_THRESH_SCALE_FACTOR 			62.5
#define		INACTIVITY_THRESH_SCALE_FACTOR 		62.5
#define		TIME_INACTIVITY_SCALE_FACTOR 			1
#define 	ADXL345_ACCEL_DATA_SCALE_FACTOR		(0.0312)

//Register Map										
															
#define		DEVID															0x00		
#define		THRESH_TAP												0x1D		
#define		OFSX															0x1E		
#define		OFSY															0x1F
#define		OFSZ															0x20
#define		DUR																0x21
#define		LATENT														0x22
#define		WINDOW 														0x23
#define		THRESH_ACT 												0x24
#define		THRESH_INACT 											0x25
#define		TIME_INACT 												0x26
#define		ACT_INACT_CTL 										0x27
#define		THRESH_FF 												0x28
#define		TIME_FF 													0x29
#define		TAP_AXES 													0x2A
#define		ACT_TAP_STATUS										0x2B
#define		BW_RATE           								0x2C
#define		POWER_CTL         								0x2D
#define		INT_ENABLE        								0x2E
#define		INT_MAP           								0x2F
#define		INT_SOURCE        								0x30
#define		DATA_FORMAT       								0x31
#define		DATAX0            								0x32
#define		DATAX1            								0x33
#define		DATAY0            								0x34
#define		DATAY1            								0x35
#define		DATAZ0            								0x36
#define		DATAZ1            								0x37
#define		FIFO_CTL          								0x38
#define		FIFO_STATUS       								0x39

#define 	ADXL345_INT_PIN_CNT								0x02
#define 	ADXL345_AXIS_COUNT								0x03
#define 	ADXL345_REG_MSB_BIT								7
#define 	ADXL345_AXIS_DATAS_NOT_READY			((float)(0xDEAD))

#define   ADXL345_START_OF_DATA_REGS 				DATAX0
#define 	ADXL345_RESET_ALL_INTERRUPTS 			0x00
#define 	ADXL345_SET_ALL_INTERRUPTS				0xFF

/* Exported macros -----------------------------------------------------------*/
//#define 	I2C_READ_DEV_ADDR(I2C_ADDR)	 		( (I2C_ADDR<<1)&(~0x01) )
//#define 	I2C_WRITE_DEV_ADDR(I2C_ADDR)	 	( (I2C_ADDR<<1)|( 0x01) )

/* Exported types ------------------------------------------------------------*/
typedef enum
{
	ADDR_PIN_GND = 0x00,
	ADDR_PIN_VCC

}AddrPinState;

typedef enum
{
	ADXL345_X_AXIS,
	ADXL345_Y_AXIS,
	ADXL345_Z_AXIS

}ADXL345_Axis;

typedef enum
{
	ADXL345_INT1	= 0,
	ADXL345_INT2 	= 1

}ADXL345_InterruptPin;

typedef union
{
	struct
	{
		bool overrun:1;
		bool watermark:1;
		bool freefall:1;
		bool inactivity:1;
		bool activity:1;
		bool doubleTap:1;
		bool singleTap:1;
		bool dataReady:1;
		
	}BIT;
	
	uint8_t	BYTE;
	
}ADXL345_InterruptReg;

typedef enum
{
	ADXL345_INT1_PIN = 0,
	ADXL345_INT2_PIN = 1,

}ADXL345_IntPins;

typedef struct
{
	float rawXData;
	float rawYData;
	float rawZData;

}ADXL345_RawDatas;

typedef enum
{
	ADXL345_2G_RANGE = 0,
	ADXL345_4G_RANGE = 1,
	ADXL345_8G_RANGE = 2,
	ADXL345_16G_RANGE = 3,

}ADXL345_G_Range;

typedef enum
{
	ADXL345_RIGHT_JUSTIFIED = 0,
	ADXL345_LEFT_JUSTIFIED 	= 1,

}ADXL345_JustifyMode;

typedef enum
{
	ADXL345_FULL_RES_DISABLED = 0,
	ADXL345_FULL_RES_ENABLED = 1,

}ADXL345_FullRes;

typedef enum
{
	ADXL345_INTERRUPT_ACTIVE_HIGH = 0,
	ADXL345_INTERRUPT_ACTIVE_LOW = 1,
	
}ADXL345_InterruptInvert;

typedef enum
{
	ADXL345_4_WIRE_SPI = 0,
	ADXL345_3_WIRE_SPI = 0,

}ADXL345_SPI_Mode;

typedef enum
{
	ADXL345_DISABLE_SELF_TEST = 0,
	ADXL345_ENABLE_SELF_TEST = 1,
	
}ADXL345_SelfTest;

typedef union
{
	struct
	{
		ADXL345_G_Range range:2;
		ADXL345_JustifyMode justify:1;
		ADXL345_FullRes fullRes:1;
		bool dummy:1;
		ADXL345_InterruptInvert intInvert:1;
		ADXL345_SPI_Mode spi:1;
		ADXL345_SelfTest selfTest:1;
	
	}BIT;
	
	uint8_t	BYTE;

}ADXL345_DataFormatReg;


typedef enum
{
	ADXL345_STANDBY_MODE = 0,
	ADXL345_MEASUREMENT_MODE = 1,

}ADXL345_WorkingMode;

typedef enum
{
	ADXL345_AUTO_SLEEP_DISABLED = 0,
	ADXL345_AUTO_SLEEP_ENABLED = 1,

}ADXL345_AutoSleepMode;

typedef enum
{
	ADXL345_ACT_INACT_NOT_LINKED = 0,
	ADXL345_ACT_INACT_LINKED = 1,

}ADXL345_LinkMode;

typedef enum
{
	ADXL345_NORMAL_MODE = 0,
	ADXL345_SLEEP_MODE	= 1

}ADXL345_PowerMode;

typedef enum
{
	ADXL345_SLEEP_MODE_8HZ = 0,
	ADXL345_SLEEP_MODE_4HZ = 1,
	ADXL345_SLEEP_MODE_2HZ = 2,
	ADXL345_SLEEP_MODE_1HZ = 3,

}ADXL345_SleepModeFreq;

typedef union
{
	struct
	{
		ADXL345_SleepModeFreq wakeup:2;
		ADXL345_PowerMode	sleep:1;
		ADXL345_WorkingMode meausure:1;
		ADXL345_AutoSleepMode autoSleep:1;
		ADXL345_LinkMode link:1;
		uint8_t dummy:2;
		
	}BIT;
	
	uint8_t	BYTE;

}ADXL345_PowerCtrReg;

/* Exported functions --------------------------------------------------------*/
void ADXL345_InitSensor(const COM_Input_HandleTypeDef * ADXL345);

uint8_t ADXL345_WhoAmI(const COM_Input_HandleTypeDef * ADXL345);

void ADXL345_SetTapThreshold(const COM_Input_HandleTypeDef * ADXL345, double tapThresh);
double ADXL345_GetTapThreshold(const COM_Input_HandleTypeDef * ADXL345);

void ADXL345_SetOffset(const COM_Input_HandleTypeDef * ADXL345, ADXL345_Axis axis, double offset);
double ADXL345_GetOffset(const COM_Input_HandleTypeDef * ADXL345, ADXL345_Axis axis);

void ADXL345_SetMaxTapDuration(const COM_Input_HandleTypeDef * ADXL345, uint32_t maxTapDur);
uint32_t ADXL345_GetMaxTapDuration(const COM_Input_HandleTypeDef * ADXL345);

void ADXL345_SetLatencyTime(const COM_Input_HandleTypeDef * ADXL345, double latTime);
double ADXL345_GetLatencyTime(const COM_Input_HandleTypeDef * ADXL345);

void ADXL345_SetWindowTime(const COM_Input_HandleTypeDef * ADXL345, double winTime);
double ADXL345_GetWindowTime(const COM_Input_HandleTypeDef * ADXL345);

void ADXL345_SetActivityThreshold(const COM_Input_HandleTypeDef * ADXL345, double actThresh);
double ADXL345_GetActivityThreshold(const COM_Input_HandleTypeDef * ADXL345);

void ADXL345_SetInactivityThreshold(const COM_Input_HandleTypeDef * ADXL345, double inactThresh);
double ADXL345_GetInactivityThreshold(const COM_Input_HandleTypeDef * ADXL345);

void ADXL345_SetInactivityTime(const COM_Input_HandleTypeDef * ADXL345, uint8_t minInactTime);
uint8_t ADXL345_GetInactivityTime(const COM_Input_HandleTypeDef * ADXL345);

void ADXL345_ConfigInterrupts(const COM_Input_HandleTypeDef * ADXL345, const ADXL345_InterruptReg * intReg);
void ADXL345_GetInterruptStatus(const COM_Input_HandleTypeDef * ADXL345, ADXL345_InterruptReg * intReg);

void ADXL345_MapInterruptPins(const COM_Input_HandleTypeDef * ADXL345, const ADXL345_InterruptReg pinMap);
void ADXL345_GetRawDatas(const COM_Input_HandleTypeDef * ADXL345, ADXL345_RawDatas * rawDatas);

void ADXL345_SetDataFormat(const COM_Input_HandleTypeDef * ADXL345, const ADXL345_DataFormatReg * dataFormat);
void ADXL345_GetDataFormat(const COM_Input_HandleTypeDef * ADXL345, ADXL345_DataFormatReg * dataFormat);

void ADXL345_SetPowerControl(const COM_Input_HandleTypeDef * ADXL345, const ADXL345_PowerCtrReg * powerControl);
void ADXL345_GetPowerControl(const COM_Input_HandleTypeDef * ADXL345, ADXL345_PowerCtrReg * powerControl);










#endif //_ADXL345_H_
