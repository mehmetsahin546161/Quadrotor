#ifndef COM_INPUT_H
#define COM_INPUT_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os2.h"
#include "adxl345.h"
#include "itg3205.h"
#include "hmc5883l.h"

/* Exported defines ----------------------------------------------------------*/
#define COM_INPUT_TX_REQU_GET 	(1<<0)
#define COM_INPUT_TX_COMPLETED 	(1<<1)
#define COM_INPUT_TX_ACK 				(1<<2)

#define COM_INPUT_RX_REQU_GET 	(1<<0)
#define COM_INPUT_RX_COMPLETED 	(1<<1)
#define COM_INPUT_RX_ACK 				(1<<2)

#define COM_INPUT_MAX_TX_TIM		100				//100ms
#define COM_INPUT_MAX_RX_TIM		100				//100ms

#define COM_INPUT_MAX_MULTI_BYTE				8

#define COM_INPUT_MAX_DEVICE_COUNT					5
#define COM_INPUT_MAX_DEVICE_TYPE_COUNT			3
/* Exported types ------------------------------------------------------------*/
typedef void (*COM_Input_InitHandler)(const COM_Input_HandleTypeDef *);
typedef DataStatus (*COM_Input_ReadDataHandler)(const COM_Input_HandleTypeDef *, void *);

typedef struct
{
	COM_Input_InitHandler initHandler;
	COM_Input_ReadDataHandler readDataHandler;

}COM_Input_Handlers;

typedef struct
{
	//TODO:It may be generalized for other communication intefaces.
	COM_Input_DeviceTypes		comInputDevType;
	I2C_HandleTypeDef * 		i2cHandle;
	uint8_t 								i2cDevAddr;
	
}COM_Input_HandleTypeDef;

typedef struct
{
	uint8_t data[COM_INPUT_MAX_MULTI_BYTE];
	uint8_t dataSize;
	uint8_t memAddress;

}COM_Input_TempDataTypeDef;

typedef struct
{
	//TODO:It may be generalized for other communication intefaces.
	
	I2C_HandleTypeDef *hi2c;
	uint16_t  DevAddress;
	uint16_t  MemAddress;
	uint16_t  MemAddSize;
	uint8_t   Data[COM_INPUT_MAX_MULTI_BYTE];
	uint16_t  Size;

}COM_Input_DataTypeDef;


typedef enum
{
	COM_INPUT_MSG_IDLE,
	COM_INPUT_REQU_GET,
	COM_INPUT_COMPLETED
	
}COM_Input_MsgStates;

typedef enum
{
	DEVICE_ADXL345 = 0,
	DEVICE_ITG3205 = 1,
	DEVICE_HMC5883L = 2,

}COM_Input_DeviceTypes;

typedef struct
{
	COM_Input_HandleTypeDef * comInputHandle;
	COM_Input_DeviceTypes		  comInputDevType;

}COM_Input_DeviceInfo;

typedef union
{
	ADXL345_RawDatas adxl345RawData;
	ITG3205_RawDatas itg3205RawData;

}COM_Input_RawDatasTypeDef;

/* Exported variables --------------------------------------------------------*/
extern ADXL345_RawDatas adxl345RawDatas;

/* Exported functions prototypes ---------------------------------------------*/
extern void COM_Input_Init();
extern void COM_Input_AddDevices(COM_Input_HandleTypeDef * comInputDev);
extern uint8_t COM_Input_GetAddedDevices(void);

extern void COM_Input_RegisterSetter(const COM_Input_HandleTypeDef * COM_Input, const COM_Input_TempDataTypeDef * setter);
extern void COM_Input_RegisterGetter(const COM_Input_HandleTypeDef * COM_Input, COM_Input_TempDataTypeDef * getter);

#endif /* COM_INPUT_H */