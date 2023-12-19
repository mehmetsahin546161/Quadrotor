/* Includes ------------------------------------------------------------------*/
#include "bmx160.h"
#include "pole_placement.h"
#include "tim.h"
#include "esc.h"

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
	.samplingTime = IMU_READING_PERIOD,
	.quaternions.q1 = 1,
	.biasQuaternions.q1 = 1,
};

PolePlacement_Handle PolePlacement = 
{
	.samplingTime = DISC_CTRL_PERIOD,
	.inputCnt = 3,
	.stateCnt = 6,
	
	.stateVector[0] = &(AHRS.eulerAngles.roll),
	.stateVector[1] = &(AHRS.eulerAngles.pitch),
	.stateVector[2] = &(AHRS.eulerAngles.yaw),
	.stateVector[3] = &(AHRS.bodyRate.p),
	.stateVector[4] = &(AHRS.bodyRate.q),
	.stateVector[5] = &(AHRS.bodyRate.r),
	
	.K[0][0] = 0.1816,
	.K[0][1] = 0,
	.K[0][2] = 0,
	.K[0][3] = 0.0999,
	.K[0][4] = 0,
	.K[0][5] = 0,
	
	.K[1][0] = 0,
	.K[1][1] = 9.0733,
	.K[1][2] = 0,
	.K[1][3] = 0,
	.K[1][4] = 0.9503,
	.K[1][5] = 0,
	
	.K[2][0] = 0,
	.K[2][1] = 0,
	.K[2][2] = 18.1412,
	.K[2][3] = 0,
	.K[2][4] = 0,
	.K[2][5] = 1.9008,
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
	HAL_GPIO_WritePin(GPIOD, DEBUG_LED_4_Pin|DEBUG_LED_3_Pin|DEBUG_LED_2_Pin|DEBUG_LED_1_Pin, GPIO_PIN_SET);
	
	ESC_CalibrateThrottle(&htim1);
	BMX160_Init(&BMX160);
	AHRS_Init(&AHRS);
	
	HAL_GPIO_WritePin(GPIOD, DEBUG_LED_4_Pin|DEBUG_LED_3_Pin|DEBUG_LED_2_Pin|DEBUG_LED_1_Pin, GPIO_PIN_RESET);
	
	PolePlc_Init(&PolePlacement);
	
	
	
	while(true)
	{
	}

}
