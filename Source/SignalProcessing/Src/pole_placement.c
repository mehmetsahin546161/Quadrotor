/* Includes ------------------------------------------------------------------*/
#include "pole_placement.h"
#include "defines.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "calc.h"
#include "bmx160.h"
#include "app_main.h"

/* Private define ------------------------------------------------------------*/
#define POLE_PLACEMENT_THREAD_STACK_SIZE	512

#define EVT_FLAG_POLE_PLACEMENT_PERIOD_ELAPSED		(1<<0)
#define EVT_FLAG_POLE_PLACEMENT_FAULT							(1<<1)

/* Private macro -------------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

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
static void PolePlc_ThreadFunc(void* arg)
{
	PolePlacement_Handle * controller = (PolePlacement_Handle *)arg;
	uint32_t eventFlags = 0;
	
	while(true)
	{
		eventFlags = osEventFlagsWait(controller->osResource.EVT_DiscController, EVT_FLAG_POLE_PLACEMENT_PERIOD_ELAPSED|EVT_FLAG_POLE_PLACEMENT_FAULT, osFlagsWaitAny, osWaitForever);
		
		if(PolePlc_IsControllerEnabled(controller) == true)
		{
			if(eventFlags & EVT_FLAG_POLE_PLACEMENT_PERIOD_ELAPSED)
			{
				PolePlc_CalculateControlSignal(controller);
				
				/* u[n] signals(controller->inputVector[i]) are evaluated. */
				
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
