#include "com_interface.h"
#include "main.h"
#include "cmsis_os2.h"

#include <stdio.h>
#include <string.h>

/* Private define ------------------------------------------------------------*/
#define MAX_COM_DATA_CNT			20
#define MAX_COM_DURATION			1000  /* ms */
#define MAX_NAME_SIZE					50

/* Private function prototypes -----------------------------------------------*/
static void COM_SendHandler(void* arg);
static void COM_ReceiveHandler(void* arg);
static uint8_t COM_GetI2CParentChannel(I2C_HandleTypeDef *hi2c);

/* Exported variables --------------------------------------------------------*/
extern I2C_HandleTypeDef 		hi2c1;

I2C_HandleTypeDef * COM_I2CChannelHandlers[COM_I2C_CHANNEL_COUNT] = {&hi2c1};

/* ---------- Buffer Pointers ---------- */
uint8_t  comI2CRXBuff[COM_I2C_CHANNEL_COUNT];
uint8_t  comSPIRXBuff[COM_SPI_CHANNEL_COUNT];
uint8_t  comUARTRXBuff[COM_UART_CHANNEL_COUNT];

uint8_t * comRXBuff[COM_TYPE_COUNT] =
{
	[COM_TYPE_I2C] 	= comI2CRXBuff,
	[COM_TYPE_SPI] 	= comSPIRXBuff,
	[COM_TYPE_UART] = comUARTRXBuff,
};


/* ---------- COM Type And Channels ---------- */
COM_PortInfo comI2CChannels[COM_I2C_CHANNEL_COUNT];
COM_PortInfo comSPIChannels[COM_SPI_CHANNEL_COUNT];
COM_PortInfo comUARTChannels[COM_UART_CHANNEL_COUNT];

COM_PortInfo * comPorts[COM_TYPE_COUNT] =
{
	[COM_TYPE_I2C] 	= comI2CChannels,
	[COM_TYPE_SPI] 	= comSPIChannels,
	[COM_TYPE_UART] = comUARTChannels,
};

/* Private variables ---------------------------------------------------------*/

/* ---------- Thread ---------- */
static osThreadId_t	TID_ComI2CTx[COM_I2C_CHANNEL_COUNT];
static osThreadId_t	TID_ComI2CRx[COM_I2C_CHANNEL_COUNT];

static osThreadId_t	TID_ComSPITx[COM_SPI_CHANNEL_COUNT];
static osThreadId_t	TID_ComSPIRx[COM_SPI_CHANNEL_COUNT];

static osThreadId_t	TID_ComUARTTx[COM_UART_CHANNEL_COUNT];
static osThreadId_t	TID_ComUARTRx[COM_UART_CHANNEL_COUNT];


static osThreadId_t * TID_ComTx[COM_TYPE_COUNT] =
{
	[COM_TYPE_I2C] 	= TID_ComI2CTx,
	[COM_TYPE_SPI] 	= TID_ComSPITx,
	[COM_TYPE_UART] = TID_ComUARTTx,
};

static osThreadId_t * TID_ComRx[COM_TYPE_COUNT] =
{
	[COM_TYPE_I2C] 	= TID_ComI2CRx,
	[COM_TYPE_SPI] 	= TID_ComSPIRx,
	[COM_TYPE_UART] = TID_ComUARTRx,
};

/* ---------- Queue ---------- */
static osMessageQueueId_t 	MSG_ComI2CTx[COM_I2C_CHANNEL_COUNT];
static osMessageQueueId_t 	MSG_ComI2CRx[COM_I2C_CHANNEL_COUNT];

static osMessageQueueId_t 	MSG_ComSPITx[COM_SPI_CHANNEL_COUNT];
static osMessageQueueId_t 	MSG_ComSPIRx[COM_SPI_CHANNEL_COUNT];

static osMessageQueueId_t 	MSG_ComUARTTx[COM_UART_CHANNEL_COUNT];
static osMessageQueueId_t 	MSG_ComUARTRx[COM_UART_CHANNEL_COUNT];


static osMessageQueueId_t * MSG_ComTx[COM_TYPE_COUNT] =
{
	[COM_TYPE_I2C] 	= MSG_ComI2CTx,
	[COM_TYPE_SPI] 	= MSG_ComSPITx,
	[COM_TYPE_UART] = MSG_ComUARTTx,
};

static osMessageQueueId_t * MSG_ComRx[COM_TYPE_COUNT] =
{
	[COM_TYPE_I2C] 	= MSG_ComI2CRx,
	[COM_TYPE_SPI] 	= MSG_ComSPIRx,
	[COM_TYPE_UART] = MSG_ComUARTRx,
};

/* ---------- Event Flags ---------- */
static osEventFlagsId_t		EVT_ComI2CTx[COM_I2C_CHANNEL_COUNT];
static osEventFlagsId_t		EVT_ComI2CRx[COM_I2C_CHANNEL_COUNT];

static osEventFlagsId_t		EVT_ComSPITx[COM_SPI_CHANNEL_COUNT];
static osEventFlagsId_t		EVT_ComSPIRx[COM_SPI_CHANNEL_COUNT];

static osEventFlagsId_t		EVT_ComUARTTx[COM_UART_CHANNEL_COUNT];
static osEventFlagsId_t		EVT_ComUARTRx[COM_UART_CHANNEL_COUNT];

static osEventFlagsId_t * EVT_ComTx[COM_TYPE_COUNT] = 
{
	[COM_TYPE_I2C] 	= EVT_ComI2CTx,
	[COM_TYPE_SPI] 	= EVT_ComSPITx,
	[COM_TYPE_UART] = EVT_ComUARTTx,
};

static osEventFlagsId_t * EVT_ComRx[COM_TYPE_COUNT] = 
{
	[COM_TYPE_I2C] 	= EVT_ComI2CRx,
	[COM_TYPE_SPI] 	= EVT_ComSPIRx,
	[COM_TYPE_UART] = EVT_ComUARTRx,
};

/* ---------- Message State ---------- */
static COM_MessageState i2cTxMessageState[COM_I2C_CHANNEL_COUNT] 	= {COM_MESSAGE_IDLE};
static COM_MessageState i2cRxMessageState[COM_I2C_CHANNEL_COUNT] 	= {COM_MESSAGE_IDLE};
			
static COM_MessageState spiTxMessageState[COM_SPI_CHANNEL_COUNT] 	= {COM_MESSAGE_IDLE};
static COM_MessageState spiRxMessageState[COM_SPI_CHANNEL_COUNT] 	= {COM_MESSAGE_IDLE};
			
static COM_MessageState uartTxMessageState[COM_UART_CHANNEL_COUNT] = {COM_MESSAGE_IDLE};
static COM_MessageState uartRxMessageState[COM_UART_CHANNEL_COUNT] = {COM_MESSAGE_IDLE};
				
static COM_MessageState * txMessageState[COM_TYPE_COUNT] = 
{
	[COM_TYPE_I2C] 	= i2cTxMessageState,
	[COM_TYPE_SPI] 	= spiTxMessageState,
	[COM_TYPE_UART] = uartTxMessageState,
};

static COM_MessageState * rxMessageState[COM_TYPE_COUNT] = 
{
	[COM_TYPE_I2C] 	= i2cRxMessageState,
	[COM_TYPE_SPI] 	= spiRxMessageState,
	[COM_TYPE_UART] = uartRxMessageState,
};

/* ---------- Thread Attribute ---------- */
static osThreadAttr_t comI2CTxThreadAttr[COM_I2C_CHANNEL_COUNT];
static osThreadAttr_t comI2CRxThreadAttr[COM_I2C_CHANNEL_COUNT];
											
static osThreadAttr_t comSPITxThreadAttr[COM_SPI_CHANNEL_COUNT];
static osThreadAttr_t comSPIRxThreadAttr[COM_SPI_CHANNEL_COUNT];

static osThreadAttr_t comUARTTxThreadAttr[COM_UART_CHANNEL_COUNT];
static osThreadAttr_t comUARTRxThreadAttr[COM_UART_CHANNEL_COUNT];

static osThreadAttr_t * comTXThreadAttr[COM_TYPE_COUNT] = 
{
	[COM_TYPE_I2C] 	= comI2CTxThreadAttr,
	[COM_TYPE_SPI] 	= comSPITxThreadAttr,
	[COM_TYPE_UART] = comUARTTxThreadAttr,
};

static osThreadAttr_t * comRXThreadAttr[COM_TYPE_COUNT] = 
{
	[COM_TYPE_I2C] 	= comI2CRxThreadAttr,
	[COM_TYPE_SPI] 	= comSPIRxThreadAttr,
	[COM_TYPE_UART] = comUARTRxThreadAttr,
};

/* Char Arrays for Name */
char comI2CTxThreadName[COM_I2C_CHANNEL_COUNT][MAX_NAME_SIZE];
char comI2CRxThreadName[COM_I2C_CHANNEL_COUNT][MAX_NAME_SIZE];

char comSPITxThreadName[COM_SPI_CHANNEL_COUNT][MAX_NAME_SIZE];
char comSPIRxThreadName[COM_SPI_CHANNEL_COUNT][MAX_NAME_SIZE];

char comUARTTxThreadName[COM_UART_CHANNEL_COUNT][MAX_NAME_SIZE];
char comUARTRxThreadName[COM_UART_CHANNEL_COUNT][MAX_NAME_SIZE];

/* ---------- Queue Attribute ---------- */
static osMessageQueueAttr_t comI2CTxQueueAttr[COM_I2C_CHANNEL_COUNT];
static osMessageQueueAttr_t comI2CRxQueueAttr[COM_I2C_CHANNEL_COUNT];
														 
static osMessageQueueAttr_t comSPITxQueueAttr[COM_SPI_CHANNEL_COUNT];
static osMessageQueueAttr_t comSPIRxQueueAttr[COM_SPI_CHANNEL_COUNT];
													 
static osMessageQueueAttr_t comUARTTxQueueAttr[COM_UART_CHANNEL_COUNT];
static osMessageQueueAttr_t comUARTRxQueueAttr[COM_UART_CHANNEL_COUNT];


static osMessageQueueAttr_t * comTxQueueAttr[COM_TYPE_COUNT] = 
{
	[COM_TYPE_I2C] 	= comI2CTxQueueAttr,
	[COM_TYPE_SPI] 	= comSPITxQueueAttr,
	[COM_TYPE_UART] = comUARTTxQueueAttr,
};

static osMessageQueueAttr_t * comRxQueueAttr[COM_TYPE_COUNT] = 
{
	[COM_TYPE_I2C] 	= comI2CRxQueueAttr,
	[COM_TYPE_SPI] 	= comSPIRxQueueAttr,
	[COM_TYPE_UART] = comUARTRxQueueAttr,
};

/* Char Arrays for Name */
static char comI2CTxQueueName[COM_I2C_CHANNEL_COUNT][MAX_NAME_SIZE];
static char comI2CRxQueueName[COM_I2C_CHANNEL_COUNT][MAX_NAME_SIZE];
                             
static char comSPITxQueueName[COM_SPI_CHANNEL_COUNT][MAX_NAME_SIZE];
static char comSPIRxQueueName[COM_SPI_CHANNEL_COUNT][MAX_NAME_SIZE];

static char comUARTTxQueueName[COM_UART_CHANNEL_COUNT][MAX_NAME_SIZE];
static char comUARTRxQueueName[COM_UART_CHANNEL_COUNT][MAX_NAME_SIZE];

/* ---------- Event Flag Attribute ---------- */
static osEventFlagsAttr_t comI2CTxEventAttr[COM_I2C_CHANNEL_COUNT];
static osEventFlagsAttr_t comI2CRxEventAttr[COM_I2C_CHANNEL_COUNT];
												 
static osEventFlagsAttr_t comSPITxEventAttr[COM_SPI_CHANNEL_COUNT];
static osEventFlagsAttr_t comSPIRxEventAttr[COM_SPI_CHANNEL_COUNT];
											 
static osEventFlagsAttr_t comUARTTxEventAttr[COM_UART_CHANNEL_COUNT];
static osEventFlagsAttr_t comUARTRxEventAttr[COM_UART_CHANNEL_COUNT];


static osEventFlagsAttr_t * comTxEvenFlagstAttr[COM_TYPE_COUNT] = 
{
	[COM_TYPE_I2C] 	= comI2CTxEventAttr,
	[COM_TYPE_SPI] 	= comSPITxEventAttr,
	[COM_TYPE_UART] = comUARTTxEventAttr,
};

static osEventFlagsAttr_t * comRxEvenFlagstAttr[COM_TYPE_COUNT] = 
{
	[COM_TYPE_I2C] 	= comI2CRxEventAttr,
	[COM_TYPE_SPI] 	= comSPIRxEventAttr,
	[COM_TYPE_UART] = comUARTRxEventAttr,
};

/* Char Arrays for Name */
static char comI2CTxEventFlagName[COM_I2C_CHANNEL_COUNT][MAX_NAME_SIZE];
static char comI2CRxEventFlagName[COM_I2C_CHANNEL_COUNT][MAX_NAME_SIZE];

static char comSPITxEventFlagName[COM_SPI_CHANNEL_COUNT][MAX_NAME_SIZE];
static char comSPIRxEventFlagName[COM_SPI_CHANNEL_COUNT][MAX_NAME_SIZE];

static char comUARTTxEventFlagName[COM_UART_CHANNEL_COUNT][MAX_NAME_SIZE];
static char comUARTRxEventFlagName[COM_UART_CHANNEL_COUNT][MAX_NAME_SIZE];

/* Exported functions --------------------------------------------------------*/

/**------------------------------------------------------------------------------
  * @brief	
  * @param[IN] 
  * @retval 	
  *------------------------------------------------------------------------------*/
void COM_Init()
{
	for(uint8_t comType=0; comType<COM_TYPE_COUNT; comType++)
	{
		/* I2C */
		for(uint8_t channelNo=0; channelNo<COM_I2C_CHANNEL_COUNT; channelNo++)
		{
			/* Type And Channel Index */
			comPorts[comType][channelNo].comType 		= comType;
			comPorts[comType][channelNo].channelNo 	= channelNo;
			
			/* ---------- Create Queues ---------- */
			//sprintf(comTXQueueName[comType][channelNo], "ComType:%d - Channel:%d - TX Queue",comType, channelNo);
			//comTxQueueAttr[comType][channelNo].name = comTXQueueName[comType][channelNo];
			
			//sprintf(comRXQueueName[comType][channelNo], "ComType:%d - Channel:%d - RX Queue",comType, channelNo);
			//comRxQueueAttr[comType][channelNo].name = comRXQueueName[comType][channelNo];
			
			MSG_ComTx[comType][channelNo] = osMessageQueueNew(MAX_COM_DATA_CNT, sizeof(COM_Handle), &(comTxQueueAttr[comType][channelNo]) );
			MSG_ComRx[comType][channelNo] = osMessageQueueNew(MAX_COM_DATA_CNT, sizeof(COM_Handle), &(comRxQueueAttr[comType][channelNo]) );
			
			/* ---------- Create Event Flag ---------- */
			//sprintf(comTXEventFlagName[comType][channelNo], "ComType:%d - Channel:%d - TX EventFlag",comType, channelNo);
			//comTxEvenFlagstAttr[comType][channelNo].name = comTXEventFlagName[comType][channelNo];
			
			//sprintf(comRXEventFlagName[comType][channelNo], "ComType:%d - Channel:%d - RX EventFlag",comType, channelNo);
			//comRxEvenFlagstAttr[comType][channelNo].name = comRXEventFlagName[comType][channelNo];
			
			EVT_ComTx[comType][channelNo] = osEventFlagsNew( &(comTxEvenFlagstAttr[comType][channelNo]) );
			EVT_ComRx[comType][channelNo] = osEventFlagsNew( &(comRxEvenFlagstAttr[comType][channelNo]) );
			
			/* ---------- Create Threads ---------- */
			//sprintf(comTXThreadName[comType][channelNo], "ComType:%d - Channel:%d - TX Thread", comType, channelNo);
			//comTXThreadAttr[comType][channelNo].name 			= comTXThreadName[comType][channelNo];
			comTXThreadAttr[comType][channelNo].priority 	= osPriorityAboveNormal;
			
			//sprintf(comRXThreadName[comType][channelNo], "ComType:%d - Channel:%d - RX Thread", comType, channelNo);
			//comRXThreadAttr[comType][channelNo].name = comRXThreadName[comType][channelNo];
			comRXThreadAttr[comType][channelNo].priority = osPriorityAboveNormal;
			
			TID_ComTx[comType][channelNo] = osThreadNew(COM_SendHandler, 		(void*)&(comPorts[comType][channelNo]), &(comTXThreadAttr[comType][channelNo]) );
			TID_ComRx[comType][channelNo] = osThreadNew(COM_ReceiveHandler, (void*)&(comPorts[comType][channelNo]), &(comRXThreadAttr[comType][channelNo]) );
		}
		
		
		/* SPI */
		for(uint8_t channelNo=0; channelNo<COM_SPI_CHANNEL_COUNT; channelNo++)
		{
		}
		
		
		/* UART */
		for(uint8_t channelNo=0; channelNo<COM_UART_CHANNEL_COUNT; channelNo++)
		{
		}
	}
}

/**------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *------------------------------------------------------------------------------v*/
bool COM_RegisterSetter(COM_Handle * comHandle)
{
	COM_Type 	comType 	= comHandle->port.comType;
	uint8_t		channelNo = comHandle->port.channelNo;
	
	osMessageQueuePut(MSG_ComTx[comType][channelNo], comHandle, NULL, 0);
	osEventFlagsSet(EVT_ComTx[comType][channelNo], COM_EVENT_FLAG_REQU_GET);
	
	/* Wait untill sending is completed */
	if(osEventFlagsWait(EVT_ComTx[comType][channelNo], COM_EVENT_FLAG_ACK, osFlagsWaitAll, MAX_COM_DURATION) & osFlagsError)
	{
		return false;
	}
	else
	{
		return true;
	}
}

/**------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *------------------------------------------------------------------------------*/
bool COM_RegisterGetter(COM_Handle * comHandle)
{
	COM_Type 	comType 	= comHandle->port.comType;
	uint8_t		channelNo = comHandle->port.channelNo;
	
	osMessageQueuePut(MSG_ComRx[comType][channelNo], comHandle, NULL, 0);
	osEventFlagsSet(EVT_ComRx[comType][channelNo], COM_EVENT_FLAG_REQU_GET);
	
	/* Wait untill reading is completed */
	if(osEventFlagsWait(EVT_ComRx[comType][channelNo], COM_EVENT_FLAG_ACK, osFlagsWaitAll, MAX_COM_DURATION) & osFlagsError)
	{
		return false;
	}
	else
	{
		memcpy(comHandle->comData.i2c.data , &(comRXBuff[comType][channelNo]), comHandle->comData.i2c.dataSize); 
		return true;
	}
}

/**------------------------------------------------------------------------------
  * @brief  Memory Tx Transfer completed callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  *------------------------------------------------------------------------------*/
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	uint8_t i2cChannel = COM_GetI2CParentChannel(hi2c);
	osEventFlagsSet(EVT_ComTx[COM_TYPE_I2C][i2cChannel], COM_EVENT_FLAG_COMPLETED);
}

/**------------------------------------------------------------------------------
  * @brief  Memory Rx Transfer completed callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  *------------------------------------------------------------------------------*/
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	uint8_t i2cChannel = COM_GetI2CParentChannel(hi2c);
	osEventFlagsSet(EVT_ComRx[COM_TYPE_I2C][i2cChannel], COM_EVENT_FLAG_COMPLETED);
}

/* Private functions ---------------------------------------------------------*/

/**------------------------------------------------------------------------------
  * @brief	
  * @param[IN] 
  * @retval 	
  *------------------------------------------------------------------------------*/
static void COM_SendHandler(void* arg)
{
	COM_PortInfo * portInfo = (COM_PortInfo*)arg;
	
	COM_Type 		comType 	= portInfo->comType;
	uint8_t			channelNo = portInfo->channelNo;
	COM_Handle 	tempComHandle = {0};
	
	while(true)
	{
		switch(txMessageState[comType][channelNo])
		{
			case COM_MESSAGE_IDLE:
			{
				if( osMessageQueueGet(MSG_ComTx[comType][channelNo], &tempComHandle, NULL, NULL) == osOK )
				{
					txMessageState[comType][channelNo] = COM_REQU_GET;
				}
				else
				{
					osEventFlagsWait(EVT_ComTx[comType][channelNo], COM_EVENT_FLAG_REQU_GET, osFlagsWaitAll, osWaitForever);
				}
				
			}break;	
			
			case COM_WAIT_LINE:
			{
			
			}break;
			
			case COM_REQU_GET:
			{
				HAL_StatusTypeDef writeStatus = HAL_I2C_Mem_Write_DMA( tempComHandle.comData.i2c.comHandle, 
																															 tempComHandle.comData.i2c.devAddress<<1,
																															 tempComHandle.comData.i2c.memAddress, 
																															 tempComHandle.comData.i2c.memAddSize,
																															 tempComHandle.comData.i2c.data, 
																															 tempComHandle.comData.i2c.dataSize );
			
				if(osEventFlagsWait(EVT_ComTx[comType][channelNo], COM_EVENT_FLAG_COMPLETED, osFlagsWaitAll, MAX_COM_DURATION)  & osFlagsError)
				{
					txMessageState[comType][channelNo] = COM_MESSAGE_IDLE;
				}
				else
				{
					txMessageState[comType][channelNo] = COM_MESSAGE_COMPLETED;
				}
				
			}break;	
			
			case COM_MESSAGE_COMPLETED:
			{
				osEventFlagsSet(EVT_ComTx[comType][channelNo], COM_EVENT_FLAG_ACK);
				txMessageState[comType][channelNo] = COM_MESSAGE_IDLE;
			}break;	
		}
	}
}

/**------------------------------------------------------------------------------
  * @brief	
  * @param[IN] 
  * @retval 	
  *------------------------------------------------------------------------------*/
static void COM_ReceiveHandler(void* arg)
{
	COM_PortInfo * portInfo = (COM_PortInfo*)arg;
	
	COM_Type 		comType 	= portInfo->comType;
	uint8_t			channelNo = portInfo->channelNo;
	COM_Handle 	tempComHandle = {0};
	
	while(true)
	{
		switch(rxMessageState[comType][channelNo])
		{
			case COM_MESSAGE_IDLE:
			{
				if( osMessageQueueGet(MSG_ComRx[comType][channelNo], &tempComHandle, NULL, NULL) == osOK )
				{
					rxMessageState[comType][channelNo] = COM_REQU_GET;
				}
				else
				{
					osEventFlagsWait(EVT_ComRx[comType][channelNo], COM_EVENT_FLAG_REQU_GET, osFlagsWaitAll, osWaitForever);
				}
				
			}break;	
			
			case COM_WAIT_LINE:
			{
			
			}break;
			
			case COM_REQU_GET:
			{
				HAL_StatusTypeDef readStatus = HAL_I2C_Mem_Read_DMA(	tempComHandle.comData.i2c.comHandle, 
																															tempComHandle.comData.i2c.devAddress<<1,
																															tempComHandle.comData.i2c.memAddress, 
																															tempComHandle.comData.i2c.memAddSize,
																															tempComHandle.comData.i2c.data, 
																															tempComHandle.comData.i2c.dataSize );
			
				if(osEventFlagsWait(EVT_ComRx[comType][channelNo], COM_EVENT_FLAG_COMPLETED, osFlagsWaitAll, MAX_COM_DURATION) & osFlagsError)
				{
					rxMessageState[comType][channelNo] = COM_MESSAGE_IDLE;
				}
				else
				{
					rxMessageState[comType][channelNo] = COM_MESSAGE_COMPLETED;
				}
				
			}break;	
			
			case COM_MESSAGE_COMPLETED:
			{
				memcpy( &(comRXBuff[comType][channelNo]), tempComHandle.comData.i2c.data, tempComHandle.comData.i2c.dataSize);
				osEventFlagsSet(EVT_ComRx[comType][channelNo], COM_EVENT_FLAG_ACK);
				rxMessageState[comType][channelNo] = COM_MESSAGE_IDLE;
			}break;	
		}
	}
}

/**------------------------------------------------------------------------------
  * @brief	
  * @param[IN] 
  * @retval 	
  *------------------------------------------------------------------------------*/
static uint8_t COM_GetI2CParentChannel(I2C_HandleTypeDef *hi2c)
{
	for(uint8_t i2cChannel=0; i2cChannel<COM_I2C_CHANNEL_COUNT; i2cChannel++)
	{
		if(COM_I2CChannelHandlers[i2cChannel] == hi2c)
		{
			return i2cChannel;
		}
	}
	
	return 0xFF;
}
