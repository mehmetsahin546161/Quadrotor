#include "com_input.h"
#include "main.h"
#include "cmsis_os2.h"
#include "adxl345.h"
#include "adxl345.h"
#include "itg3205.h"

#include <stdio.h>
#include <string.h>


/* Private define ------------------------------------------------------------*/
#define MAX_COM_INPUT_DATA_CNT			20
#define MAX_COM_DURATION						1000  /* ms */
/* Exported variables --------------------------------------------------------*/

/* ---------- ComInput_Handle ---------- */

ComInput_Handle comInputI2CHandle[COM_INPUT_I2C_CHANNEL_COUNT];
ComInput_Handle comInputSPIHandle[COM_INPUT_SPI_CHANNEL_COUNT];
ComInput_Handle comInputUARTHandle[COM_INPUT_UART_CHANNEL_COUNT];

ComInput_Handle * comInputs[COM_INPUT_TYPE_COUNT] =
{
	[COM_INPUT_TYPE_I2C] 	= comInputI2CHandle,
	[COM_INPUT_TYPE_SPI] 	= comInputSPIHandle,
	[COM_INPUT_TYPE_UART] = comInputUARTHandle,
};

/* Private function prototypes -----------------------------------------------*/
static void ComIn_SendHandler(void* arg);
static void ComIn_ReceiveHandler(void* arg);
static uint8_t ComInput_GetI2CParentChannel(I2C_HandleTypeDef *hi2c);
/* Private variables ---------------------------------------------------------*/

/* ---------- Thread ---------- */
static osThreadId_t	TID_ComInputI2CTx[COM_INPUT_I2C_CHANNEL_COUNT];
static osThreadId_t	TID_ComInputI2CRx[COM_INPUT_I2C_CHANNEL_COUNT];

static osThreadId_t	TID_ComInputSPITx[COM_INPUT_SPI_CHANNEL_COUNT];
static osThreadId_t	TID_ComInputSPIRx[COM_INPUT_SPI_CHANNEL_COUNT];

static osThreadId_t	TID_ComInputUARTTx[COM_INPUT_UART_CHANNEL_COUNT];
static osThreadId_t	TID_ComInputUARTRx[COM_INPUT_UART_CHANNEL_COUNT];


static osThreadId_t * TID_ComInputTx[COM_INPUT_TYPE_COUNT] =
{
	[COM_INPUT_TYPE_I2C] 	= TID_ComInputI2CTx,
	[COM_INPUT_TYPE_SPI] 	= TID_ComInputSPITx,
	[COM_INPUT_TYPE_UART] = TID_ComInputUARTTx,
};

static osThreadId_t * TID_ComInputRx[COM_INPUT_TYPE_COUNT] =
{
	[COM_INPUT_TYPE_I2C] 	= TID_ComInputI2CRx,
	[COM_INPUT_TYPE_SPI] 	= TID_ComInputSPIRx,
	[COM_INPUT_TYPE_UART] = TID_ComInputUARTRx,
};

/* ---------- Queue ---------- */
static osMessageQueueId_t 	MSG_ComInputI2CTx[COM_INPUT_I2C_CHANNEL_COUNT];
static osMessageQueueId_t 	MSG_ComInputI2CRx[COM_INPUT_I2C_CHANNEL_COUNT];

static osMessageQueueId_t 	MSG_ComInputSPITx[COM_INPUT_SPI_CHANNEL_COUNT];
static osMessageQueueId_t 	MSG_ComInputSPIRx[COM_INPUT_SPI_CHANNEL_COUNT];

static osMessageQueueId_t 	MSG_ComInputUARTTx[COM_INPUT_UART_CHANNEL_COUNT];
static osMessageQueueId_t 	MSG_ComInputUARTRx[COM_INPUT_UART_CHANNEL_COUNT];

static osMessageQueueId_t * MSG_ComInputTx[COM_INPUT_TYPE_COUNT] =
{
	[COM_INPUT_TYPE_I2C] 	= MSG_ComInputI2CTx,
	[COM_INPUT_TYPE_SPI] 	= MSG_ComInputSPITx,
	[COM_INPUT_TYPE_UART] = MSG_ComInputUARTTx,
};

static osMessageQueueId_t * MSG_ComInputRx[COM_INPUT_TYPE_COUNT] =
{
	[COM_INPUT_TYPE_I2C] 	= MSG_ComInputI2CRx,
	[COM_INPUT_TYPE_SPI] 	= MSG_ComInputSPIRx,
	[COM_INPUT_TYPE_UART] = MSG_ComInputUARTRx,
};


/* ---------- Event Flags ---------- */
static osEventFlagsId_t		EVT_ComInputI2CTx[COM_INPUT_I2C_CHANNEL_COUNT];
static osEventFlagsId_t		EVT_ComInputI2CRx[COM_INPUT_I2C_CHANNEL_COUNT];

static osEventFlagsId_t		EVT_ComInputSPITx[COM_INPUT_SPI_CHANNEL_COUNT];
static osEventFlagsId_t		EVT_ComInputSPIRx[COM_INPUT_SPI_CHANNEL_COUNT];

static osEventFlagsId_t		EVT_ComInputUARTTx[COM_INPUT_UART_CHANNEL_COUNT];
static osEventFlagsId_t		EVT_ComInputUARTRx[COM_INPUT_UART_CHANNEL_COUNT];

static osEventFlagsId_t * EVT_ComInputTx[COM_INPUT_TYPE_COUNT] = 
{
	[COM_INPUT_TYPE_I2C] 	= EVT_ComInputI2CTx,
	[COM_INPUT_TYPE_SPI] 	= EVT_ComInputSPITx,
	[COM_INPUT_TYPE_UART] = EVT_ComInputUARTTx,
};

static osEventFlagsId_t * EVT_ComInputRx[COM_INPUT_TYPE_COUNT] = 
{
	[COM_INPUT_TYPE_I2C] 	= EVT_ComInputI2CRx,
	[COM_INPUT_TYPE_SPI] 	= EVT_ComInputSPIRx,
	[COM_INPUT_TYPE_UART] = EVT_ComInputUARTRx,
};



/* ---------- Message State ---------- */
static ComInput_MessageState i2cTxMessageState[COM_INPUT_I2C_CHANNEL_COUNT] 	= {COM_INPUT_MESSAGE_IDLE};
static ComInput_MessageState i2cRxMessageState[COM_INPUT_I2C_CHANNEL_COUNT] 	= {COM_INPUT_MESSAGE_IDLE};

static ComInput_MessageState spiTxMessageState[COM_INPUT_SPI_CHANNEL_COUNT] 	= {COM_INPUT_MESSAGE_IDLE};
static ComInput_MessageState spiRxMessageState[COM_INPUT_SPI_CHANNEL_COUNT] 	= {COM_INPUT_MESSAGE_IDLE};

static ComInput_MessageState uartTxMessageState[COM_INPUT_UART_CHANNEL_COUNT] = {COM_INPUT_MESSAGE_IDLE};
static ComInput_MessageState uartRxMessageState[COM_INPUT_UART_CHANNEL_COUNT] = {COM_INPUT_MESSAGE_IDLE};

static ComInput_MessageState usbTxMessageState[COM_INPUT_UART_CHANNEL_COUNT] 	= {COM_INPUT_MESSAGE_IDLE};
static ComInput_MessageState usbRxMessageState[COM_INPUT_UART_CHANNEL_COUNT] 	= {COM_INPUT_MESSAGE_IDLE};

static ComInput_MessageState * txMessageState[COM_INPUT_TYPE_COUNT] = 
{
	[COM_INPUT_TYPE_I2C] 	= i2cTxMessageState,
	[COM_INPUT_TYPE_SPI] 	= spiTxMessageState,
	[COM_INPUT_TYPE_UART] = uartTxMessageState,
};

static ComInput_MessageState * rxMessageState[COM_INPUT_TYPE_COUNT] = 
{
	[COM_INPUT_TYPE_I2C] 	= i2cRxMessageState,
	[COM_INPUT_TYPE_SPI] 	= spiRxMessageState,
	[COM_INPUT_TYPE_UART] = uartRxMessageState,
};



/* ---------- Thread Attribute ---------- */
static osThreadAttr_t ComInputI2CTxThreadAttr[COM_INPUT_I2C_CHANNEL_COUNT];
static osThreadAttr_t ComInputI2CRxThreadAttr[COM_INPUT_I2C_CHANNEL_COUNT];

static osThreadAttr_t ComInputSPITxThreadAttr[COM_INPUT_SPI_CHANNEL_COUNT];
static osThreadAttr_t ComInputSPIRxThreadAttr[COM_INPUT_SPI_CHANNEL_COUNT];

static osThreadAttr_t ComInputUARTTxThreadAttr[COM_INPUT_UART_CHANNEL_COUNT];
static osThreadAttr_t ComInputUARTRxThreadAttr[COM_INPUT_UART_CHANNEL_COUNT];

static osThreadAttr_t * ComInputTXThreadAttr[COM_INPUT_TYPE_COUNT] = 
{
	[COM_INPUT_TYPE_I2C] 	= ComInputI2CTxThreadAttr,
	[COM_INPUT_TYPE_SPI] 	= ComInputSPITxThreadAttr,
	[COM_INPUT_TYPE_UART] = ComInputUARTTxThreadAttr,
};

static osThreadAttr_t * ComInputRXThreadAttr[COM_INPUT_TYPE_COUNT] = 
{
	[COM_INPUT_TYPE_I2C] 	= ComInputI2CRxThreadAttr,
	[COM_INPUT_TYPE_SPI] 	= ComInputSPIRxThreadAttr,
	[COM_INPUT_TYPE_UART] = ComInputUARTRxThreadAttr,
};

/* ---------- Queue Attribute ---------- */
static osMessageQueueAttr_t ComInputI2CTxQueueAttr[COM_INPUT_I2C_CHANNEL_COUNT];
static osMessageQueueAttr_t ComInputI2CRxQueueAttr[COM_INPUT_I2C_CHANNEL_COUNT];

static osMessageQueueAttr_t ComInputSPITxQueueAttr[COM_INPUT_SPI_CHANNEL_COUNT];
static osMessageQueueAttr_t ComInputSPIRxQueueAttr[COM_INPUT_SPI_CHANNEL_COUNT];

static osMessageQueueAttr_t ComInputUARTTxQueueAttr[COM_INPUT_UART_CHANNEL_COUNT];
static osMessageQueueAttr_t ComInputUARTRxQueueAttr[COM_INPUT_UART_CHANNEL_COUNT];


static osMessageQueueAttr_t * ComInputTxQueueAttr[COM_INPUT_TYPE_COUNT] = 
{
	[COM_INPUT_TYPE_I2C] 	= ComInputI2CTxQueueAttr,
	[COM_INPUT_TYPE_SPI] 	= ComInputSPITxQueueAttr,
	[COM_INPUT_TYPE_UART] = ComInputUARTTxQueueAttr,
};

static osMessageQueueAttr_t * ComInputRxQueueAttr[COM_INPUT_TYPE_COUNT] = 
{
	[COM_INPUT_TYPE_I2C] 	= ComInputI2CRxQueueAttr,
	[COM_INPUT_TYPE_SPI] 	= ComInputSPIRxQueueAttr,
	[COM_INPUT_TYPE_UART] = ComInputUARTRxQueueAttr,
};


/* ---------- Event Flag Attribute ---------- */
static osEventFlagsAttr_t ComInputI2CTxEventAttr[COM_INPUT_I2C_CHANNEL_COUNT];
static osEventFlagsAttr_t ComInputI2CRxEventAttr[COM_INPUT_I2C_CHANNEL_COUNT];

static osEventFlagsAttr_t ComInputSPITxEventAttr[COM_INPUT_SPI_CHANNEL_COUNT];
static osEventFlagsAttr_t ComInputSPIRxEventAttr[COM_INPUT_SPI_CHANNEL_COUNT];

static osEventFlagsAttr_t ComInputUARTTxEventAttr[COM_INPUT_UART_CHANNEL_COUNT];
static osEventFlagsAttr_t ComInputUARTRxEventAttr[COM_INPUT_UART_CHANNEL_COUNT];


static osEventFlagsAttr_t * ComInputTxEvenFlagstAttr[COM_INPUT_TYPE_COUNT] = 
{
	[COM_INPUT_TYPE_I2C] 	= ComInputI2CTxEventAttr,
	[COM_INPUT_TYPE_SPI] 	= ComInputSPITxEventAttr,
	[COM_INPUT_TYPE_UART] = ComInputUARTTxEventAttr,
};

static osEventFlagsAttr_t * ComInputRxEvenFlagstAttr[COM_INPUT_TYPE_COUNT] = 
{
	[COM_INPUT_TYPE_I2C] 	= ComInputI2CRxEventAttr,
	[COM_INPUT_TYPE_SPI] 	= ComInputSPIRxEventAttr,
	[COM_INPUT_TYPE_UART] = ComInputUARTRxEventAttr,
};



/* Exported functions --------------------------------------------------------*/

/**------------------------------------------------------------------------------
  * @brief	
  * @param[IN] 
  * @retval 	
  *------------------------------------------------------------------------------*/
void ComInput_Init()
{
	char tempTxt[100]={0};
	
	/* ---------- Create Threads ---------- */
	for(uint8_t typeNo=0; typeNo<COM_INPUT_TYPE_COUNT; typeNo++)
	{
		for(uint8_t channelNo=0; channelNo<COM_INPUT_I2C_CHANNEL_COUNT; channelNo++)
		{
			sprintf(tempTxt, "Com_Input_Type_%d_Channel_%d_Tx_Thread",typeNo, channelNo);
			ComInputTXThreadAttr[typeNo][channelNo].name = tempTxt;
			ComInputTXThreadAttr[typeNo][channelNo].priority = osPriorityNormal;
			
			sprintf(tempTxt, "Com_Input_Type_%d_Channel_%d_Rx_Thread",typeNo, channelNo);
			ComInputRXThreadAttr[typeNo][channelNo].name = tempTxt;
			ComInputRXThreadAttr[typeNo][channelNo].priority = osPriorityNormal;
			
			TID_ComInputTx[typeNo][channelNo] = osThreadNew( ComIn_SendHandler, (void*)&(comInputs[typeNo][channelNo].typeAndChannel) , &(ComInputTXThreadAttr[typeNo][channelNo]) );
			if(TID_ComInputTx[typeNo][channelNo] == NULL)
			{
				//Thread couldn't be created.
			}
	
			TID_ComInputRx[typeNo][channelNo] = osThreadNew( ComIn_ReceiveHandler, (void*)&(comInputs[typeNo][channelNo].typeAndChannel), &(ComInputRXThreadAttr[typeNo][channelNo]) );
			if(TID_ComInputRx[typeNo][channelNo] == NULL)
			{
				//Thread couldn't be created.
			}
		}
	}
	
	/* ---------- Create Queues ---------- */
	for(uint8_t typeNo=0; typeNo<COM_INPUT_TYPE_COUNT; typeNo++)
	{
		for(uint8_t channelNo=0; channelNo<COM_INPUT_I2C_CHANNEL_COUNT; channelNo++)
		{
			sprintf(tempTxt, "Com_Input_Type_%d_Channel_%d_Tx_Queue",typeNo, channelNo);
			ComInputTxQueueAttr[typeNo][channelNo].name = tempTxt;
			
			sprintf(tempTxt, "Com_Input_Type_%d_Channel_%d_Rx_Queue",typeNo, channelNo);
			ComInputRxQueueAttr[typeNo][channelNo].name = tempTxt;
			
			MSG_ComInputTx[typeNo][channelNo] = osMessageQueueNew( MAX_COM_INPUT_DATA_CNT, sizeof(ComInput_Handle), &(ComInputTxQueueAttr[typeNo][channelNo]) );
			if(MSG_ComInputTx[typeNo][channelNo] == NULL)
			{
				//Thread couldn't be created.
			}
	
			MSG_ComInputRx[typeNo][channelNo] = osMessageQueueNew(MAX_COM_INPUT_DATA_CNT, sizeof(ComInput_Handle), &(ComInputRxQueueAttr[typeNo][channelNo]) );
			if(MSG_ComInputRx[typeNo][channelNo] == NULL)
			{
				//Thread couldn't be created.
			}
		}
	}
	
	
	
	/* ---------- Create Event Flag ---------- */
	for(uint8_t typeNo=0; typeNo<COM_INPUT_TYPE_COUNT; typeNo++)
	{
		for(uint8_t channelNo=0; channelNo<COM_INPUT_I2C_CHANNEL_COUNT; channelNo++)
		{
			sprintf(tempTxt, "Com_Input_Type_%d_Channel_%d_Tx_Event_Flag",typeNo, channelNo);
			ComInputTxEvenFlagstAttr[typeNo][channelNo].name = tempTxt;
			
			sprintf(tempTxt, "Com_Input_Type_%d_Channel_%d_Rx_Event_Flag",typeNo, channelNo);
			ComInputRxEvenFlagstAttr[typeNo][channelNo].name = tempTxt;
			
			EVT_ComInputTx[typeNo][channelNo] = osEventFlagsNew( &(ComInputTxEvenFlagstAttr[typeNo][channelNo]) );
			if(EVT_ComInputTx[typeNo][channelNo] == NULL)
			{
				//Thread couldn't be created.
			}
	
			EVT_ComInputRx[typeNo][channelNo] = osEventFlagsNew( &(ComInputRxEvenFlagstAttr[typeNo][channelNo]) );
			if(EVT_ComInputRx[typeNo][channelNo] == NULL)
			{
				//Thread couldn't be created.
			}
		}
	}
	
}

/**------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *------------------------------------------------------------------------------v*/
void ComInput_RegisterSetter(ComInput_Handle * COM_Input)
{
	ComInput_Type	comInputType = COM_Input->typeAndChannel.comInputType;
	uint8_t	channelNo = COM_Input->typeAndChannel.channelNo;
	
	osMessageQueuePut(MSG_ComInputTx[comInputType][channelNo], COM_Input, NULL, 0);
	osEventFlagsSet(EVT_ComInputTx[comInputType][channelNo], EVENT_FLAG_REQU_GET);
	
	/* Wait untill sending is completed */
	osEventFlagsWait(EVT_ComInputTx[comInputType][channelNo], EVENT_FLAG_ACK, osFlagsWaitAll, MAX_COM_DURATION);
}

/**------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *------------------------------------------------------------------------------*/
void ComInput_RegisterGetter(ComInput_Handle * COM_Input)
{
	ComInput_Type	comInputType = COM_Input->typeAndChannel.comInputType;
	uint8_t	channelNo = COM_Input->typeAndChannel.channelNo;
	
	osMessageQueuePut(MSG_ComInputRx[comInputType][channelNo], COM_Input, NULL, 0);
	osEventFlagsSet(EVT_ComInputRx[comInputType][channelNo], EVENT_FLAG_REQU_GET);
	
	/* Wait untill reading is completed */
	osEventFlagsWait(EVT_ComInputRx[comInputType][channelNo], EVENT_FLAG_ACK, osFlagsWaitAll, MAX_COM_DURATION);
}

/**------------------------------------------------------------------------------
  * @brief  Memory Tx Transfer completed callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  *------------------------------------------------------------------------------*/
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	uint8_t i2cChannel = ComInput_GetI2CParentChannel(hi2c);
	osEventFlagsSet(EVT_ComInputTx[COM_INPUT_TYPE_I2C][i2cChannel], EVENT_FLAG_COMPLETED);
}

/**------------------------------------------------------------------------------
  * @brief  Memory Rx Transfer completed callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  *------------------------------------------------------------------------------*/
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	uint8_t i2cChannel = ComInput_GetI2CParentChannel(hi2c);
	osEventFlagsSet(EVT_ComInputRx[COM_INPUT_TYPE_I2C][i2cChannel], EVENT_FLAG_COMPLETED);
}

/* Private functions ---------------------------------------------------------*/

/**------------------------------------------------------------------------------
  * @brief	
  * @param[IN] 
  * @retval 	
  *------------------------------------------------------------------------------*/
static void ComIn_SendHandler(void* arg)
{
	ComInput_TypeAndChannel * typeAndChannel = (ComInput_TypeAndChannel*)arg;
	
	ComInput_Type *	comInputType;
	comInputType = &(typeAndChannel->comInputType);
	
	uint8_t * channelNo;
	channelNo = &(typeAndChannel->channelNo);
	
	while(true)
	{
		switch(txMessageState[*comInputType][*channelNo])
		{
			case COM_INPUT_MESSAGE_IDLE:
			{
				if( osMessageQueueGet(MSG_ComInputTx[*comInputType][*channelNo], &(comInputs[*comInputType][*channelNo]), NULL, NULL) == osOK )
				{
					txMessageState[*comInputType][*channelNo] = COM_INPUT_REQU_GET;
				}
				else
				{
					osEventFlagsWait(EVT_ComInputTx[*comInputType][*channelNo], EVENT_FLAG_REQU_GET, osFlagsWaitAll, osWaitForever);
				}
				
			
			}break;	
			
			case COM_INPUT_REQU_GET:
			{
				HAL_StatusTypeDef writeStatus = HAL_I2C_Mem_Write_DMA( comInputs[*comInputType][*channelNo].comInputData.i2c.comHandle, 
																															 comInputs[*comInputType][*channelNo].comInputData.i2c.devAddress<<1,
																															 comInputs[*comInputType][*channelNo].comInputData.i2c.memAddress, 
																															 comInputs[*comInputType][*channelNo].comInputData.i2c.memAddSize,
																															 comInputs[*comInputType][*channelNo].comInputData.i2c.data, 
																															 comInputs[*comInputType][*channelNo].comInputData.i2c.dataSize );
			
				volatile uint32_t retFlag;
				
				retFlag = osEventFlagsWait(EVT_ComInputTx[*comInputType][*channelNo], EVENT_FLAG_COMPLETED, osFlagsWaitAll, MAX_COM_DURATION);
				
				if(retFlag == osFlagsErrorTimeout)
				{
					txMessageState[*comInputType][*channelNo] = COM_INPUT_MESSAGE_IDLE;
				}
				else
				{
					txMessageState[*comInputType][*channelNo] = COM_INPUT_COMPLETED;
				}
				
			
			}break;	
			
			case COM_INPUT_COMPLETED:
			{
				osEventFlagsSet(EVT_ComInputTx[*comInputType][*channelNo], EVENT_FLAG_ACK);
			
				txMessageState[*comInputType][*channelNo] = COM_INPUT_MESSAGE_IDLE;
			}break;	
		}
	}
}

/**------------------------------------------------------------------------------
  * @brief	
  * @param[IN] 
  * @retval 	
  *------------------------------------------------------------------------------*/
static void ComIn_ReceiveHandler(void* arg)
{
	ComInput_TypeAndChannel * typeAndChannel = (ComInput_TypeAndChannel*)arg;
	
	ComInput_Type *	comInputType;
	comInputType = &(typeAndChannel->comInputType);
	
	uint8_t * channelNo;
	channelNo = &(typeAndChannel->channelNo);
	
	while(true)
	{
		switch(rxMessageState[*comInputType][*channelNo])
		{
			case COM_INPUT_MESSAGE_IDLE:
			{
				if( osMessageQueueGet(MSG_ComInputRx[*comInputType][*channelNo], &(comInputs[*comInputType][*channelNo]), NULL, NULL) == osOK )
				{
					rxMessageState[*comInputType][*channelNo] = COM_INPUT_REQU_GET;
				}
				else
				{
					osEventFlagsWait(EVT_ComInputRx[*comInputType][*channelNo], EVENT_FLAG_REQU_GET, osFlagsWaitAll, osWaitForever);
				}
				
			}break;	
			
			case COM_INPUT_REQU_GET:
			{
				HAL_StatusTypeDef readStatus = HAL_I2C_Mem_Read_DMA(	comInputs[*comInputType][*channelNo].comInputData.i2c.comHandle, 
																															comInputs[*comInputType][*channelNo].comInputData.i2c.devAddress<<1,
																															comInputs[*comInputType][*channelNo].comInputData.i2c.memAddress, 
																															comInputs[*comInputType][*channelNo].comInputData.i2c.memAddSize,
																															comInputs[*comInputType][*channelNo].comInputData.i2c.data, 
																															comInputs[*comInputType][*channelNo].comInputData.i2c.dataSize );
			
				uint32_t retFlag = osEventFlagsWait(EVT_ComInputRx[*comInputType][*channelNo], EVENT_FLAG_COMPLETED, osFlagsWaitAll, MAX_COM_DURATION);
				
				if(retFlag == osFlagsErrorTimeout)
				{
					rxMessageState[*comInputType][*channelNo] = COM_INPUT_MESSAGE_IDLE;
				}
				else
				{
					rxMessageState[*comInputType][*channelNo] = COM_INPUT_COMPLETED;
				}
				
			
			}break;	
			
			case COM_INPUT_COMPLETED:
			{
				osEventFlagsSet(EVT_ComInputRx[*comInputType][*channelNo], EVENT_FLAG_ACK);
				rxMessageState[*comInputType][*channelNo] = COM_INPUT_MESSAGE_IDLE;
			}break;	
		}
	}
}

/**------------------------------------------------------------------------------
  * @brief	
  * @param[IN] 
  * @retval 	
  *------------------------------------------------------------------------------*/
static uint8_t ComInput_GetI2CParentChannel(I2C_HandleTypeDef *hi2c)
{
	for(uint8_t i2cChannel=0; i2cChannel<COM_INPUT_I2C_CHANNEL_COUNT; i2cChannel++)
	{
		if(comInputs[COM_INPUT_TYPE_I2C][i2cChannel].comInputData.i2c.comHandle == hi2c)
		{
			return i2cChannel;
		}
	}
	
	return 0xFF;
}

/**------------------------------------------------------------------------------
  * @brief	
  * @param[IN] 
  * @retval 	
  *------------------------------------------------------------------------------*/
void ComInput_AddInputDevice(ComInput_Handle * COM_Input)
{
	memcpy( &comInputs[COM_Input->typeAndChannel.comInputType][COM_Input->typeAndChannel.channelNo], COM_Input, sizeof(ComInput_Handle) );
}