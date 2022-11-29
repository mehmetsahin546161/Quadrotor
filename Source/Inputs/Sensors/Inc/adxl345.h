#ifndef _ADXL345_H_
#define _ADXL345_H_

/* Private includes ----------------------------------------------------------*/
#include "stm32f407xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"
#include <stdbool.h>

/* Exported constants --------------------------------------------------------*/
#define 	INVALID_DATA											0x00
						
#define 	I2C_DEV_ADDR_GND  								0x53			//Not shifted
#define 	I2C_DEV_ADDR_VCC  								0x1D			//Not shifted
	
#define 	TAP_THRESH_SCALE_FACTOR 					62.5
#define 	OFFSET_SCALE_FACTOR								15.6
#define		DUR_TIME_SCALE_FACTOR							625.0
#define		LATENT_TIME_SCALE_FACTOR					1.25
#define		WINDOW_TIME_SCALE_FACTOR					1.25
#define		ACTIVITY_THRESH_SCALE_FACTOR 			62.5
#define		INACTIVITY_THRESH_SCALE_FACTOR 		62.5
#define		TIME_INACTIVITY_SCALE_FACTOR 			1

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
	
#define 	ADXL345_REGISTER_SIZE							1		//1-Byte
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
	X_AXIS,
	Y_AXIS,
	Z_AXIS

}ADXL345_Axis;

typedef struct
{
	I2C_HandleTypeDef * i2cHandle;
	uint8_t 						i2cDevAddr;
	
}ADXL345_HandleTypeDef;

typedef enum
{
	RESET_ALL_INTERRUPTS = 0x00,
	
	OVERRUN 			= 0x01,
	WATERMARK 		= 0x02,
	FREEFALL 			= 0x04,
	INACTIVITY 		= 0x08,
	ACTIVITY			= 0x10,
	DOUBLE_TAP		= 0x20,
	SINGLE_TAP		= 0x40,
	DATA_READY		= 0x80,
	
	SET_ALL_INTERRUPTS = 0xFF,
	
}ADXL345_Interrupts;

typedef struct
{
	bool overrun:1;
	bool watermark:1;
	bool freefall:1;
	bool inactivity:1;
	bool activity:1;
	bool doubleTap:1;
	bool singleTap:1;
	bool dataReady:1;
	
}ADXL345_InterruptBits;

typedef union
{
	ADXL345_InterruptBits 	BIT;
	uint8_t 								BYTE;
	
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


/* Exported functions --------------------------------------------------------*/
void 											ADXL345_SW_Init									(void);
uint8_t 									ADXL345_WhoAmI									(ADXL345_HandleTypeDef * ADXL345);

void 											ADXL345_SetTapThreshold					(ADXL345_HandleTypeDef * ADXL345, double tapThresh);
double 										ADXL345_GetTapThreshold					(ADXL345_HandleTypeDef * ADXL345);

void    									ADXL345_SetOffset								(ADXL345_HandleTypeDef * ADXL345, ADXL345_Axis axis, double offset);
double										ADXL345_GetOffset								(ADXL345_HandleTypeDef * ADXL345, ADXL345_Axis axis);

void 											ADXL345_SetMaxTapDuration				(ADXL345_HandleTypeDef * ADXL345, uint32_t maxTapDur);
uint32_t 									ADXL345_GetMaxTapDuration				(ADXL345_HandleTypeDef * ADXL345);

void 											ADXL345_SetLatencyTime					(ADXL345_HandleTypeDef * ADXL345, double latTime);
double 										ADXL345_GetLatencyTime					(ADXL345_HandleTypeDef * ADXL345);

void 											ADXL345_SetWindowTime						(ADXL345_HandleTypeDef * ADXL345, double winTime);
double 										ADXL345_GetWindowTime						(ADXL345_HandleTypeDef * ADXL345);

void 											ADXL345_SetActivityThreshold		(ADXL345_HandleTypeDef * ADXL345, double actThresh);
double 										ADXL345_GetActivityThreshold		(ADXL345_HandleTypeDef * ADXL345);

void 											ADXL345_SetInactivityThreshold	(ADXL345_HandleTypeDef * ADXL345, double inactThresh);
double 										ADXL345_GetInactivityThreshold	(ADXL345_HandleTypeDef * ADXL345);

void 											ADXL345_SetInactivityTime				(ADXL345_HandleTypeDef * ADXL345, uint8_t minInactTime);
uint8_t 									ADXL345_GetInactivityTime				(ADXL345_HandleTypeDef * ADXL345);

void 											ADXL345_ConfigInterrupts				(ADXL345_HandleTypeDef * ADXL345, ADXL345_InterruptReg intReg);
ADXL345_InterruptReg 			ADXL345_GetInterruptStatus			(ADXL345_HandleTypeDef * ADXL345);

void 											ADXL345_MapInterruptPins				(ADXL345_HandleTypeDef * ADXL345, ADXL345_InterruptReg pinMap[ADXL345_INT_PIN_CNT]);
ADXL345_RawDatas					ADXL345_GetRawDatas							(ADXL345_HandleTypeDef * ADXL345);
















#endif //_ADXL345_H_
