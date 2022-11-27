#include "comm_input.h"
#include "main.h"
#include "cmsis_os2.h"
#include "adxl345.h"


/* Private define ------------------------------------------------------------*/
#define MAX_COM_INPUT_DATA_CNT			20

/* Exported variables --------------------------------------------------------*/
extern ADXL345_HandleTypeDef ADXL345;


/* Private variables ---------------------------------------------------------*/
osMessageQueueId_t MSG_comInputDatas;



const osMessageQueueAttr_t comInputDatasAttr = {
	.name = "Com_Input_Datas"
};


/* Private function prototypes -----------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

/**
  * @brief	
  * @param[IN] 
  * @retval 	
  */
void COM_Input_Init()
{
	MSG_comInputDatas = osMessageQueueNew(MAX_COM_INPUT_DATA_CNT, sizeof(COM_Input_DataTypeDef), &comInputDatasAttr);
	
	if(MSG_comInputDatas == NULL)
	{
		//Queue couldn't be created.
	}
	
}

/**
  * @brief  Memory Tx Transfer completed callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	//uint32_t 	pMem0 = *((uint32_t*)(hi2c->hdmatx->Instance->M0AR));
	//pMem0 = pMem0 >> 8;
	uint8_t sadd = *(hi2c->pBuffPtr);
	
	ADXL345_GetMaxTapDuration(&ADXL345);
	
}

/**
  * @brief  Memory Rx Transfer completed callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	uint32_t 	pMem0 = *((uint32_t*)(hi2c->hdmatx->Instance->M0AR));
	pMem0 = pMem0 >> 8;
	
}

/**
  * @brief	
  * @param[IN] 
  * @retval 	
  */
void COM_Input_SendThread(void* arg)
{
	
}

/**
  * @brief	
  * @param[IN] 
  * @retval 	
  */
void COM_Input_ReceiveThread(void* arg)
{

}

