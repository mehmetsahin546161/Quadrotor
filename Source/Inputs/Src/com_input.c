#include "com_input.h"
#include "main.h"
#include "cmsis_os2.h"
#include "adxl345.h"
#include "adxl345.h"
#include "itg3205.h"

/* Private define ------------------------------------------------------------*/
#define MAX_COM_INPUT_DATA_CNT			20

/* Exported variables --------------------------------------------------------*/



/* Private function prototypes -----------------------------------------------*/
static void COM_Input_SendThread(void* arg);
static void COM_Input_ReceiveThread(void* arg);

/* Private variables ---------------------------------------------------------*/
osThreadId_t	TID_comInputTxThread;
osThreadId_t	TID_comInputRxThread;

osMessageQueueId_t 	MSG_comInputTx;
osMessageQueueId_t 	MSG_comInputRx;

osEventFlagsId_t		EVT_comInputTx;
osEventFlagsId_t		EVT_comInputRx;

osMutexId_t					MTX_comInputTx;
osMutexId_t					MTX_comInputRx;

const osThreadAttr_t comInputTxThreadAttr =
{
	.name = "Com_Input_Tx_Thread",
	.priority = osPriorityNormal
};

const osThreadAttr_t comInputRxThreadAttr = 
{
	.name = "Com_Input_Rx_Thread",
	.priority = osPriorityNormal
};

const osMessageQueueAttr_t comInputTxDatasAttr = 
{
	.name = "Com_Input_Tx_Queue"
};

const osMessageQueueAttr_t comInputRxDatasAttr = 
{
	.name = "Com_Input_Rx_Queue"
};

const osEventFlagsAttr_t comInputTxEvtAttr = 
{
	.name = "Com_Input_Tx_Event"
};

const osEventFlagsAttr_t comInputRxEvtAttr =
{
	.name = "Com_Input_Rx_Event"
};

const osMutexAttr_t	comInputTxMtxAttr = 
{
	.name = "Com_Input_Tx_Mutex",
	.attr_bits = osMutexPrioInherit
};

const osMutexAttr_t	comInputRxMtxAttr = 
{
	.name = "Com_Input_Rx_Mutex",
	.attr_bits = osMutexPrioInherit
};

static uint8_t comInputDevIndex = 0;
static COM_Input_HandleTypeDef * COM_Input_Devices[COM_INPUT_MAX_DEVICE_COUNT] = {NULL};

static COM_Input_Handlers comInputHandlers[COM_INPUT_MAX_DEVICE_TYPE_COUNT] = 
{
	[COM_DEVICE_TYPE_ADXL345] 	= {.initHandler = ADXL345_InitSensor },
	[COM_DEVICE_TYPE_ITG3205] 	= {.initHandler = ITG3205_InitSensor },
	[COM_DEVICE_TYPE_HMC5883L]  = {.initHandler = NULL }
};

/* Exported functions --------------------------------------------------------*/

/**
  * @brief	
  * @param[IN] 
  * @retval 	
  */
void COM_Input_Init()
{
	TID_comInputTxThread = osThreadNew(COM_Input_SendThread, NULL, &comInputTxThreadAttr);
	
	if(TID_comInputTxThread == NULL)
	{
		//Thread couldn't be created.
	}
	
	TID_comInputRxThread = osThreadNew(COM_Input_ReceiveThread, NULL, &comInputRxThreadAttr);
	
	if(TID_comInputRxThread == NULL)
	{
		//Thread couldn't be created.
	}
	
	MSG_comInputTx = osMessageQueueNew(MAX_COM_INPUT_DATA_CNT, sizeof(COM_Input_DataTypeDef), &comInputTxDatasAttr);
	
	if(MSG_comInputTx == NULL)
	{
		//Queue couldn't be created.
	}
	
	MSG_comInputRx = osMessageQueueNew(MAX_COM_INPUT_DATA_CNT, sizeof(COM_Input_DataTypeDef), &comInputRxDatasAttr);
	
	if(MSG_comInputRx == NULL)
	{
		//Queue couldn't be created.
	}
	
	EVT_comInputTx = osEventFlagsNew(&comInputTxEvtAttr);
	
	if(EVT_comInputTx == NULL)
	{
		//Event flag couldn't be created.
	}
	
	EVT_comInputRx = osEventFlagsNew(&comInputRxEvtAttr);
	
	if(EVT_comInputRx == NULL)
	{
		//Event flag couldn't be created.
	}
	
	MTX_comInputTx = osMutexNew(&comInputTxMtxAttr);
	
	if(MTX_comInputTx == NULL)
	{
		//Mutex couldn't be created.
	}
	
	MTX_comInputRx = osMutexNew(&comInputRxMtxAttr);
	
	if(MTX_comInputRx == NULL)
	{
		//Mutex couldn't be created.
	}
}

/**
  * @brief  		Every sensor handler is registered to this module.
  * @param[IN] 	comInputDev
  * @retval 		None
  */
void COM_Input_AddDevice(COM_Input_HandleTypeDef * comInputDev)
{
	COM_Input_Devices[comInputDevIndex] = comInputDev;
	
	/* We don't need to both add and initialize the sensor modules. Just add. */
	comInputHandlers[comInputDev->comInputDevType].initHandler(comInputDev);
	
	comInputDevIndex++;
}

/**
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  */
uint8_t COM_Input_GetAddedDevices()
{
	return comInputDevIndex;
}

/**
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  */
void COM_Input_RegisterSetter(const COM_Input_HandleTypeDef * COM_Input, const COM_Input_TempDataTypeDef * setter)
{
	osMutexAcquire(MTX_comInputTx, osWaitForever);
		
	COM_Input_DataTypeDef sendMsg = {	.hi2c = COM_Input->i2cHandle,
																		.DevAddress = COM_Input->i2cDevAddr << 1,
																		.MemAddress	= setter->memAddress,
																		.MemAddSize = I2C_MEMADD_SIZE_8BIT,
																		.Size = setter->dataSize };
	
	/* Multi-Byte initialization is not valid */
	for(uint8_t sizeCnt=0; sizeCnt<setter->dataSize; sizeCnt++)
	{
		sendMsg.Data[sizeCnt] = (setter->data)[sizeCnt];
	}
	
	osMessageQueuePut(MSG_comInputTx, &sendMsg, NULL, 0);
				
	osEventFlagsSet(EVT_comInputTx, COM_INPUT_TX_REQU_GET);
	osEventFlagsWait(EVT_comInputTx, COM_INPUT_TX_ACK, osFlagsWaitAll, COM_INPUT_MAX_TX_TIM);
																		
	osMutexRelease(MTX_comInputTx);	

}

/**
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  */
void COM_Input_RegisterGetter(const COM_Input_HandleTypeDef * COM_Input, COM_Input_TempDataTypeDef * getter)
{
	osMutexAcquire(MTX_comInputRx, osWaitForever);
	
	COM_Input_DataTypeDef receivedMsg = {	.hi2c = COM_Input->i2cHandle,
																				.DevAddress = COM_Input->i2cDevAddr << 1,
																				.MemAddress	= getter->memAddress  ,
																				.MemAddSize = I2C_MEMADD_SIZE_8BIT,
																				.Size =  getter->dataSize};
	
	osMessageQueuePut(MSG_comInputRx, &receivedMsg, NULL, 0);
				
	osEventFlagsSet(EVT_comInputRx, COM_INPUT_RX_REQU_GET);
	osEventFlagsWait(EVT_comInputRx, COM_INPUT_RX_ACK, osFlagsWaitAll, COM_INPUT_MAX_RX_TIM);
	
	osMessageQueueGet(MSG_comInputRx, &receivedMsg, NULL, 0);
	
	for(uint8_t sizeCnt=0; sizeCnt<getter->dataSize; sizeCnt++)
	{
		(getter->data)[sizeCnt] = receivedMsg.Data[sizeCnt];
	}
	
	osMutexRelease(MTX_comInputRx);
}



/**
  * @brief  Memory Tx Transfer completed callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	osEventFlagsSet(EVT_comInputTx, COM_INPUT_TX_COMPLETED);
}

/**
  * @brief  Memory Rx Transfer completed callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	osEventFlagsSet(EVT_comInputRx, COM_INPUT_RX_COMPLETED);
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief	
  * @param[IN] 
  * @retval 	
  */
static void COM_Input_SendThread(void* arg)
{
	static COM_Input_MsgStates txMsgState = COM_INPUT_MSG_IDLE;
	
	while(true)
	{
		switch(txMsgState)
		{
			case COM_INPUT_MSG_IDLE:
			{
				osEventFlagsWait(EVT_comInputTx, COM_INPUT_TX_REQU_GET, osFlagsWaitAll, osWaitForever);
				
				if(osMessageQueueGetCount(MSG_comInputTx) != 0)
				{
					txMsgState = COM_INPUT_REQU_GET;
				}
			
			}break;	
			
			case COM_INPUT_REQU_GET:
			{
				COM_Input_DataTypeDef sendMsg = {0};
				osMessageQueueGet(MSG_comInputTx, &sendMsg, NULL, 0);
		
				HAL_I2C_Mem_Write_DMA( sendMsg.hi2c, 
															 sendMsg.DevAddress,
															 sendMsg.MemAddress, 
															 sendMsg.MemAddSize,
															 sendMsg.Data, 
															 sendMsg.Size );
			
				osEventFlagsWait(EVT_comInputTx, COM_INPUT_TX_COMPLETED, osFlagsWaitAll, osWaitForever);
				
				txMsgState = COM_INPUT_COMPLETED;
			
			}break;	
			
			case COM_INPUT_COMPLETED:
			{
				osEventFlagsSet(EVT_comInputTx, COM_INPUT_TX_ACK);
			
				txMsgState = COM_INPUT_MSG_IDLE;
			}break;	
		}
	}
}

/**
  * @brief	
  * @param[IN] 
  * @retval 	
  */
static void COM_Input_ReceiveThread(void* arg)
{
	static COM_Input_MsgStates rxMsgState = COM_INPUT_MSG_IDLE;
	
	while(true)
	{
		switch(rxMsgState)
		{
			case COM_INPUT_MSG_IDLE:
			{
				osEventFlagsWait(EVT_comInputRx, COM_INPUT_RX_REQU_GET, osFlagsWaitAll, osWaitForever);
				
				if(osMessageQueueGetCount(MSG_comInputRx) != 0)
				{
					rxMsgState = COM_INPUT_REQU_GET;
				}
			
			}break;	
			
			case COM_INPUT_REQU_GET:
			{
				COM_Input_DataTypeDef receivedMsg = {0};
				osMessageQueueGet(MSG_comInputRx, &receivedMsg, NULL, 0);
		
				HAL_I2C_Mem_Read_DMA(	receivedMsg.hi2c, 
															receivedMsg.DevAddress,
															receivedMsg.MemAddress, 
															receivedMsg.MemAddSize,
															receivedMsg.Data, 
															receivedMsg.Size );
			
				osEventFlagsWait(EVT_comInputRx, COM_INPUT_RX_COMPLETED, osFlagsWaitAll, osWaitForever);
				
				//There is no function in CMSIS-RTOS v2 like ...peek().
				//So, we put the message again to notify caller function.
				osMessageQueuePut(MSG_comInputRx, &receivedMsg, NULL, 0);
				
				rxMsgState = COM_INPUT_COMPLETED;
			
			}break;	
			
			case COM_INPUT_COMPLETED:
			{
				osEventFlagsSet(EVT_comInputRx, COM_INPUT_RX_ACK);
			
				rxMsgState = COM_INPUT_MSG_IDLE;
			}break;	
		}
	}
}