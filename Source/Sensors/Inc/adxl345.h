#ifndef _ADXL345_H_
#define _ADXL345_H_

/* Private includes ----------------------------------------------------------*/
#include 	"stm32f407xx.h"
#include 	"stm32f4xx_hal.h"
#include 	"stm32f4xx_hal_i2c.h"
#include 	<stdbool.h>
#include 	"defines.h"
#include	"imu.h"
#include 	"com_interface.h"

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
														
#define		ADXL345_DEVID_ADDR										0x00		
#define		ADXL345_THRESH_TAP_ADDR								0x1D		
#define		ADXL345_OFSX_ADDR											0x1E		
#define		ADXL345_OFSY_ADDR											0x1F
#define		ADXL345_OFSZ_ADDR											0x20
#define		ADXL345_DUR_ADDR											0x21
#define		ADXL345_LATENT_ADDR										0x22
#define		ADXL345_WINDOW_ADDR 									0x23
#define		ADXL345_THRESH_ACT_ADDR 							0x24
#define		ADXL345_THRESH_INACT_ADDR 						0x25
#define		ADXL345_TIME_INACT_ADDR 							0x26
#define		ADXL345_ACT_INACT_CTL_ADDR 						0x27
#define		ADXL345_THRESH_FF_ADDR 								0x28
#define		ADXL345_TIME_FF_ADDR 									0x29
#define		ADXL345_TAP_AXES_ADDR 								0x2A
#define		ADXL345_ACT_TAP_STATUS_ADDR						0x2B
#define		ADXL345_BW_RATE_ADDR           				0x2C
#define		ADXL345_POWER_CTL_ADDR         				0x2D
#define		ADXL345_INT_ENABLE_ADDR        				0x2E
#define		ADXL345_INT_MAP_ADDR           				0x2F
#define		ADXL345_INT_SOURCE_ADDR        				0x30
#define		ADXL345_DATA_FORMAT_ADDR       				0x31
#define		ADXL345_DATAX0_ADDR            				0x32
#define		ADXL345_DATAX1_ADDR            				0x33
#define		ADXL345_DATAY0_ADDR            				0x34
#define		ADXL345_DATAY1_ADDR            				0x35
#define		ADXL345_DATAZ0_ADDR            				0x36
#define		ADXL345_DATAZ1_ADDR            				0x37
#define		ADXL345_FIFO_CTL_ADDR          				0x38
#define		ADXL345_FIFO_STATUS_ADDR       				0x39
		
#define 	ADXL345_INT_PIN_CNT								0x02
#define 	ADXL345_AXIS_COUNT								0x03
#define 	ADXL345_REG_MSB_BIT								7
#define 	ADXL345_AXIS_DATAS_NOT_READY			((float)(0xDEAD))

#define   ADXL345_START_OF_DATA_ADDR 				ADXL345_DATAX0_ADDR
#define 	ADXL345_RESET_ALL_INTERRUPTS 			0x00
#define 	ADXL345_SET_ALL_INTERRUPTS				0xFF

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
	ADXL345_INT1_PIN = 0,
	ADXL345_INT2_PIN = 1,

}ADXL345_InterruptPin;



/* Register Enums */
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

typedef enum
{
	ADXL345_SLEEP_MODE_8HZ = 0,
	ADXL345_SLEEP_MODE_4HZ = 1,
	ADXL345_SLEEP_MODE_2HZ = 2,
	ADXL345_SLEEP_MODE_1HZ = 3,

}ADXL345_SleepModeFreq;

typedef enum
{
	ADXL345_NORMAL_MODE = 0,
	ADXL345_SLEEP_MODE	= 1

}ADXL345_PowerMode;

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

/* Register Structs */

/* 

Functional Grouping of Registers
¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯
	Tap;
		•THRESH_TAP
		•DUR
		•LATENT
		•WINDOW
		•TAP_AXES
		•ACT_TAP_STATUS

	Data;
		•OFSX
		•OFSY
		•OFSZ
		•DATA_FORMAT
		•DATAX0
		•DATAX1
		•DATAY0
    •DATAY1
    •DATAZ0
    •DATAZ1
		
	Activity/Inactivity
		•THRESH_ACT 		
		•THRESH_INACT 
		•TIME_INACT 		
		•ACT_INACT_CTL
		•ACT_TAP_STATUS
		
	Freefall
		•THRESH_FF 
	  •TIME_FF 	
		
	Device/Power
		•DEVID
	  •BW_RATE  
	  •POWER_CTL
		
	Interrupt
		•INT_ENABLE 
	  •INT_MAP    
	  •INT_SOURCE 
	
	FIFO
		•FIFO_CTL   
	  •FIFO_STATUS
		
*/
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

typedef struct
{
	double rawXData;
	double rawYData;
	double rawZData;

}ADXL345_RawDatas;

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



/* Exported functions --------------------------------------------------------*/
void ADXL345_InitSensor(COM_Handle * ADXL345);

uint8_t ADXL345_WhoAmI(COM_Handle * ADXL345);

void ADXL345_SetTapThreshold(COM_Handle * ADXL345, double tapThresh);
double ADXL345_GetTapThreshold(COM_Handle * ADXL345);

void ADXL345_SetOffset(COM_Handle * ADXL345, ADXL345_Axis axis, double offset);
double ADXL345_GetOffset(COM_Handle * ADXL345, ADXL345_Axis axis);

void ADXL345_SetMaxTapDuration(COM_Handle * ADXL345, uint32_t maxTapDur);
uint32_t ADXL345_GetMaxTapDuration(COM_Handle * ADXL345);

void ADXL345_SetLatencyTime(COM_Handle * ADXL345, double latTime);
double ADXL345_GetLatencyTime(COM_Handle * ADXL345);

void ADXL345_SetWindowTime(COM_Handle * ADXL345, double winTime);
double ADXL345_GetWindowTime(COM_Handle * ADXL345);

void ADXL345_SetActivityThreshold(COM_Handle * ADXL345, double actThresh);
double ADXL345_GetActivityThreshold(COM_Handle * ADXL345);

void ADXL345_SetInactivityThreshold(COM_Handle * ADXL345, double inactThresh);
double ADXL345_GetInactivityThreshold(COM_Handle * ADXL345);

void ADXL345_SetInactivityTime(COM_Handle * ADXL345, uint8_t minInactTime);
uint8_t ADXL345_GetInactivityTime(COM_Handle * ADXL345);

void ADXL345_ConfigInterrupts(COM_Handle * ADXL345, const ADXL345_InterruptReg * intReg);
void ADXL345_GetInterruptStatus(COM_Handle * ADXL345, ADXL345_InterruptReg * intReg);

void ADXL345_MapInterruptPins(COM_Handle * ADXL345, const ADXL345_InterruptReg pinMap);
void ADXL345_GetRawDatas(COM_Handle * ADXL345, ADXL345_RawDatas * rawDatas);

void ADXL345_SetDataFormat(COM_Handle * ADXL345, const ADXL345_DataFormatReg * dataFormat);
void ADXL345_GetDataFormat(COM_Handle * ADXL345, ADXL345_DataFormatReg * dataFormat);

void ADXL345_SetPowerControl(COM_Handle * ADXL345, const ADXL345_PowerCtrReg * powerControl);
void ADXL345_GetPowerControl(COM_Handle * ADXL345, ADXL345_PowerCtrReg * powerControl);

void ADXL345_GetAccelOffsetValues(COM_Handle * ADXL345, IMU_AxisDatas * biasDatas);
void ADXL345_GetAngle(COM_Handle * ADXL345, const IMU_AxisDatas * axisBias, IMU_AxisAngles * axisAngle);

#endif //_ADXL345_H_
