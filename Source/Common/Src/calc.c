#include "calc.h"


/* Exported functions --------------------------------------------------------*/

/**------------------------------------------------------------------------------
  * @brief  			Calcuulates discrete derivative with the method of Euler backward.
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *------------------------------------------------------------------------------*/
double Calc_GetDiscreteDerivative(double currVal, double * prevVal, double samplingTime)
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
	
	double out = ( currVal - *prevVal)/samplingTime;
	*prevVal = currVal;
	return out;
}

/**------------------------------------------------------------------------------
  * @brief  			Calcuulates discrete integral with the method of Euler backward.
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *------------------------------------------------------------------------------*/
double Calc_GetDiscreteIntegral(double currVal, double * prevSum, double samplingTime)
{
	//              --------------
	//							|					   |
	//	u[n] ---->  |  Integral  |  ----> y[n]
	//              |            |
	//							--------------
	//
	//  y[n] = y[n-1] + T*u[n]
	
	double out = (*prevSum) + ( samplingTime * (currVal) );
	*prevSum = out;
	return out;
}
