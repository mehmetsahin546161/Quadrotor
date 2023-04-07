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

#define COM_MAX_BYTE_CNT				8

#define COM_I2C_DATA(X)								(X->comData.i2c)		//X represents address of COM_Handle object.

#define COM_TYPE_COUNT								3	

#define COM_I2C_CHANNEL_COUNT					1
#define COM_SPI_CHANNEL_COUNT					1
#define COM_UART_CHANNEL_COUNT				1

#define I2C_MAX_DEV_COUNT					5			/* Per Channel */
#define SPI_MAX_DEV_COUNT					2			/* Per Channel */
#define UART_MAX_DEV_COUNT			  2			/* Per Channel */

#define COM_I2C1_CHANNEL_INDEX		0

/* Exported types ------------------------------------------------------------*/
typedef enum
{
	COM_TYPE_I2C 	= 0x00,
	COM_TYPE_SPI 	= 0x01,
	COM_TYPE_UART = 0x02,
	
}COM_Type;

typedef enum
{
	COM_MESSAGE_IDLE,
	COM_WAIT_LINE,
	COM_REQU_GET,
	COM_MESSAGE_COMPLETED
	
}COM_MessageState;

typedef struct
{
	I2C_HandleTypeDef * 	comHandle;
	uint16_t  				    devAddress;
	uint16_t  				    memAddress;
	uint16_t  				    memAddSize;
	uint8_t   				    data[COM_MAX_BYTE_CNT];
	uint16_t  				    dataSize;
	
}COM_I2CData;

typedef union
{
	COM_I2CData i2c;

}COM_Data;

typedef struct
{
	COM_Type 	comType;
	uint8_t		channelNo;

}COM_TypeChannelIndex;

typedef struct
{
	COM_Data 								comData;
	COM_TypeChannelIndex	* comTypeChannel;
	
}COM_Handle;


/* Exported variables --------------------------------------------------------*/
extern COM_TypeChannelIndex * comTypeAndChannels[COM_TYPE_COUNT];
extern uint8_t * comRXBuff[COM_TYPE_COUNT];

/* Exported functions prototypes ---------------------------------------------*/
extern void COM_Init(void);
extern bool COM_RegisterSetter(COM_Handle * COM_Input);
extern bool COM_RegisterGetter(COM_Handle * COM_Input);

#endif /* COM_INPUT_H */