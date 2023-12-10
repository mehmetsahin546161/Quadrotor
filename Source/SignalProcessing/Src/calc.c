#include "calc.h"


/* Exported functions --------------------------------------------------------*/

/**------------------------------------------------------------------------------
  * @brief  			Calcuulates discrete derivative with the method of Euler backward.
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *------------------------------------------------------------------------------*/
float Calc_GetDiscreteDerivative(float currVal, float * prevVal, float samplingTime)
{
	//              ----------------
	//							|						   |
	//	u[n] ---->  |  Derivative  |  ----> y[n]
	//              |              |
	//							----------------
	// 
	//   				 u[n] - u[n-1]
	//  y[n] =	---------------
	//					       T         
	
	float out = ( currVal - *prevVal)/samplingTime;
	*prevVal = currVal;
	return out;
}

/**------------------------------------------------------------------------------
  * @brief  			Calcuulates discrete integral with the method of Euler backward.
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *------------------------------------------------------------------------------*/
float Calc_GetDiscreteIntegral(float currVal, float * prevSum, float samplingTime)
{
	//              --------------
	//							|					   |
	//	u[n] ---->  |  Integral  |  ----> y[n]
	//              |            |
	//							--------------
	//
	//  y[n] = y[n-1] + T*u[n]
	
	float out = (*prevSum) + ( samplingTime * (currVal) );
	*prevSum = out;
	return out;
}
