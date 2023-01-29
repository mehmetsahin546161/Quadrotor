#include "calc.h"


/* Exported functions --------------------------------------------------------*/


/**------------------------------------------------------------------------------
  * @brief  			
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *------------------------------------------------------------------------------*/
int8_t Calc_GetByte2sComplement(uint8_t data)
{
	int8_t retVal = (data & SIGN_BIT_MASK_IN_BYTE ) ?
									-1.0*( (uint8_t)( ~(data-1) )):			// Negative number
									data;																// Positive number

	return retVal;
}

/**------------------------------------------------------------------------------
  * @brief  			
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *------------------------------------------------------------------------------*/
int16_t Calc_GetHalfWord2sComplement(uint16_t data)
{
	int16_t retVal = (data & SIGN_BIT_MASK_IN_HALFWORD) ?
									-1.0*( (uint16_t)( ~(data-1) )):			// Negative number
									data;																	// Positive number

	return retVal;
}

/**------------------------------------------------------------------------------
  * @brief  			
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *------------------------------------------------------------------------------*/
int32_t Calc_GetWord2sComplement(uint32_t data)
{
	int32_t retVal = (data & SIGN_BIT_MASK_IN_WORD) ?
									-1.0*( (uint32_t)( ~(data-1) )):			// Negative number
									data;																// Positive number

	return retVal;
}

/**------------------------------------------------------------------------------
  * @brief  			Calcuulates discrete derivative with the method of Euler backward.
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *------------------------------------------------------------------------------*/
double Calc_GetDiscreteDerivative(double * currVal, double * prevVal, double samplingTime)
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
	
	double out = ( *currVal - *prevVal)/samplingTime;
	*prevVal = *currVal;
	return out;
}

/**------------------------------------------------------------------------------
  * @brief  			Calcuulates discrete integral with the method of Euler backward.
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *------------------------------------------------------------------------------*/
double Calc_GetDiscreteIntegral(double * currVal, double * prevSum, double samplingTime)
{
	//              --------------
	//							|					   |
	//	u[n] ---->  |  Integral  |  ----> y[n]
	//              |            |
	//							--------------
	//
	//  y[n] = y[n-1] + T*u[n]
	
	double out = (*prevSum) + ( samplingTime * (*currVal) );
	*prevSum = out;
	return out;
}
