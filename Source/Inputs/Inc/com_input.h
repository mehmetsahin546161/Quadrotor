#ifndef COM_INPUT_H
#define COM_INPUT_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os2.h"
//#include "hmc5883l.h"

/* Exported defines ----------------------------------------------------------*/
#define EVENT_FLAG_REQU_GET 	(1<<0)
#define EVENT_FLAG_COMPLETED 	(1<<1)
#define EVENT_FLAG_ACK 				(1<<3)

#define COM_INPUT_MAX_MULTI_BYTE				8

#define COM_INPUT_MAX_DEVICE_TYPE_COUNT			3

#define COM_INPUT_I2C_DATA(X)								(X->comInputData.i2c)		//X represents address of ComInput_Handle object.

#define COM_INPUT_TYPE_COUNT								3	

#define COM_INPUT_I2C_CHANNEL_COUNT					1
#define COM_INPUT_SPI_CHANNEL_COUNT					1
#define COM_INPUT_UART_CHANNEL_COUNT				1

/* Exported types ------------------------------------------------------------*/
typedef enum
{
	COM_INPUT_TYPE_I2C 	= 0x00,
	COM_INPUT_TYPE_SPI 	= 0x01,
	COM_INPUT_TYPE_UART = 0x02,
	
}ComInput_Type;

typedef enum
{
	COM_INPUT_MESSAGE_IDLE,
	COM_INPUT_REQU_GET,
	COM_INPUT_COMPLETED
	
}ComInput_MessageState;

typedef struct
{
	I2C_HandleTypeDef * 	comHandle;
	uint16_t  				    devAddress;
	uint16_t  				    memAddress;
	uint16_t  				    memAddSize;
	uint8_t   				    data[COM_INPUT_MAX_MULTI_BYTE];
	uint16_t  				    dataSize;

}ComInput_I2CData;

typedef union
{
	ComInput_I2CData i2c;

}ComInput_Data;

typedef struct
{
	ComInput_Type 	comInputType;
	uint8_t					channelNo;

}ComInput_TypeAndChannel;

typedef struct
{
	ComInput_TypeAndChannel typeAndChannel;
	ComInput_Data comInputData;
	
}ComInput_Handle;



/* Exported variables --------------------------------------------------------*/
extern ComInput_Handle * comInputs[COM_INPUT_TYPE_COUNT];

/* Exported functions prototypes ---------------------------------------------*/
extern void ComInput_Init(void);
extern void ComInput_AddInputDevice(ComInput_Handle * COM_Input);
extern void ComInput_RegisterSetter(ComInput_Handle * COM_Input);
extern void ComInput_RegisterGetter(ComInput_Handle * COM_Input);

#endif /* COM_INPUT_H */