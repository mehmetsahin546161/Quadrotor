/**
  ******************************************************************************
  * @file           : esc.c
  * @brief          : Interface drive Electronic Speed Controller
  ******************************************************************************
  * @attention
  *
  * ESC frequency is 50 Hz. 
  * TIMx->ARR = 2000 
  *	CCRx = 100 => Duty %5
  * CCRx = 200 => Duty %10
  * 
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "esc.h"
#include "tim.h"
#include "cmsis_os2.h"

/* Private define ------------------------------------------------------------*/
#define PWM_CCR_OFFSET			100

/* Private macro -------------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

/**--------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief  		Throttle range settings(Throttle range should be reset whenever a new transmitter is being used);
  * 							•	Switch on the transmitter, move throttle stick to the top position.
  * 							•	Connect battery pack to the ESC, and wait for about 2 seconds.
  * 							•	The “Beep-Beep-” tone should be emitted, means the top point of throttle range has been confirmed.
  * 							•	If you wait more than 5 seconds, it enters programing mode.
  * 							•	Move throttle stick to the bottom position, several “beep-” tones should be emitted to present the amount of battery cells.
  * 							•	A long “Beep-” tone should be emitted, means the lowest point of throttle range has been correctly confirmed.
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *--------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void ESC_CalibrateThrottle(TIM_HandleTypeDef * htim)
{
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_4);
	
	__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, 100);
	__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, 100);
	__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, 100);
	__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, 100);
	
	//osDelay(3000);
	//
	//__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, 105);
	//__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, 105);
	//__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, 105);
	//__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, 105);
}