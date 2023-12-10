/* Includes ------------------------------------------------------------------*/
#include "bmx160.h"
#include "pole_placement.h"

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
extern I2C_HandleTypeDef 		hi2c1;

BMX160_Handle BMX160 = 
{
	.i2c = &hi2c1,
};

AHRS_Handle	AHRS = 
{
	.quaternions.q1 = 1
};

PolePlacement_Handle PolePlacement = 
{
	.samplingTime = IMU_READING_PERIODE,
	.inputCnt = 3,
	.stateCnt = 6,
	
	.stateVector[0] = &(AHRS.eulerAngles.roll),

};

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

/**--------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *--------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void APP_Main(void* arg)
{
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_SET);
	
	BMX160_CreateSensor(&BMX160);
	PolePlc_CreateController(&PolePlacement);
	
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);
	
	while(true)
	{
	}

}
