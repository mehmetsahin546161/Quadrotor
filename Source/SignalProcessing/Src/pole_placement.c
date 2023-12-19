/* Includes ------------------------------------------------------------------*/
#include "pole_placement.h"
#include "defines.h"
#include "stdio.h"
#include "string.h"
#include "math.h"
#include "calc.h"
#include "bmx160.h"
#include "app_main.h"
#include "tim.h"

/* Private define ------------------------------------------------------------*/
#define POLE_PLACEMENT_THREAD_STACK_SIZE	512

#define EVT_FLAG_POLE_PLACEMENT_PERIOD_ELAPSED		(1<<0)
#define EVT_FLAG_POLE_PLACEMENT_FAULT							(1<<1)

#define W_TO_CCR(w) (0.087*w + 100)
/* Private macro -------------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
static void PolePlc_ThreadFunc(void* arg);
static void PolePlc_PeriodicTimerFunc(void* arg);
static void PolePlc_CalculateControlSignal(PolePlacement_Handle * controller);

/* Exported functions --------------------------------------------------------*/

/**--------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief
	* @note				
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *--------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void PolePlc_Init(PolePlacement_Handle * controller)
{
	/* Create OS resources */
	controller->osResource.EVT_DiscController = osEventFlagsNew(NULL);
	controller->osResource.TIM_DiscController = osTimerNew(PolePlc_PeriodicTimerFunc, osTimerPeriodic, controller, NULL);
	controller->osResource.TID_DiscController = osThreadNew(PolePlc_ThreadFunc, controller, NULL);
	
	PolePlc_EnableController(controller);
}

/**--------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief
	* @note				
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *--------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void PolePlc_EnableController(PolePlacement_Handle * controller)
{
	controller->enabled = true;
	osTimerStart(controller->osResource.TIM_DiscController, (uint32_t)SEC_TO_MS(controller->samplingTime));
}

/**--------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief
	* @note				
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *--------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void PolePlc_DisableController(PolePlacement_Handle * controller)
{
	osTimerStop(controller->osResource.TIM_DiscController);
	controller->enabled = false;
}

/**--------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief
	* @note				
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *--------------------------------------------------------------------------------------------------------------------------------------------------------------*/
bool PolePlc_IsControllerEnabled(PolePlacement_Handle * controller)
{
	return controller->enabled;
}

/* Private functions ---------------------------------------------------------*/

/**--------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief
	* @note				
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *--------------------------------------------------------------------------------------------------------------------------------------------------------------*/
volatile 	double w1, w2, w3, w4;
double z;
static void PolePlc_ThreadFunc(void* arg)
{
	PolePlacement_Handle * controller = (PolePlacement_Handle *)arg;
	uint32_t eventFlags = 0;
	

	double k = 0.00000625;
	double h = 0.000000757;
	double u2, u3, u4;
	//double z;
	double ccr1, ccr2, ccr3, ccr4;//%5 = 100, %10=200
	
	while(true)
	{
		eventFlags = osEventFlagsWait(controller->osResource.EVT_DiscController, EVT_FLAG_POLE_PLACEMENT_PERIOD_ELAPSED|EVT_FLAG_POLE_PLACEMENT_FAULT, osFlagsWaitAny, osWaitForever);
		
		if(PolePlc_IsControllerEnabled(controller) == true)
		{
			if(eventFlags & EVT_FLAG_POLE_PLACEMENT_PERIOD_ELAPSED)
			{
				PolePlc_CalculateControlSignal(controller);
				
				/* u[n] signals(controller->inputVector[i]) are evaluated. */
				u2 = controller->inputVector[0];
				u3 = controller->inputVector[1];
				u4 = controller->inputVector[2];
				
				z=(h*u3 - h*u2 - k*u4 < 0)?0:h*u3 - h*u2 - k*u4;
				
				w1 = sqrt((h*u2 - h*u3 + k*u4 + 2*h*k*z)/(2*h*k));
				w2 = sqrt((u2 + k*z)/k);
				w3 = sqrt((h*u2 + h*u3 + k*u4 + 2*h*k*z)/(2*h*k));
				w4 = sqrt(z);
				
				ccr1 = W_TO_CCR(w1);
				ccr2 = W_TO_CCR(w2);
				ccr3 = W_TO_CCR(w3);
				ccr4 = W_TO_CCR(w4);
				
				
				ccr1 = (ccr1<100)?100:(ccr1>200)?200:ccr1;
				ccr2 = (ccr2<100)?100:(ccr2>200)?200:ccr2;
				ccr3 = (ccr3<100)?100:(ccr3>200)?200:ccr3;
				ccr4 = (ccr4<100)?100:(ccr4>200)?200:ccr4;
				
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr1);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr2);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ccr3);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, ccr4);
				
			}
			else if(eventFlags & EVT_FLAG_POLE_PLACEMENT_FAULT)
			{
				
			}
		}
	}	
}

/**--------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief
	* @note				
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *--------------------------------------------------------------------------------------------------------------------------------------------------------------*/
static void PolePlc_PeriodicTimerFunc(void* arg)
{
	PolePlacement_Handle * controller = (PolePlacement_Handle *)arg;
	
	osEventFlagsSet(controller->osResource.EVT_DiscController, EVT_FLAG_POLE_PLACEMENT_PERIOD_ELAPSED);
}

/**--------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief
	* @note					Integral and reference input signals will be added insALLAH <^-^>		
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *--------------------------------------------------------------------------------------------------------------------------------------------------------------*/
static void PolePlc_CalculateControlSignal(PolePlacement_Handle * controller)
{
	/* u[n] = -K*x[n] */
	float tempInput = 0;
	
	for(uint8_t i=0; i<controller->inputCnt; i++)
	{
		for(uint8_t k=0; k<controller->stateCnt; k++)
		{
			tempInput += -1*(controller->K[i][k])*(*controller->stateVector[k]);
		}
		
		controller->inputVector[i] = tempInput;
		tempInput = 0;
	}
}
