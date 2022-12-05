#include "calc.h"


/* Exported functions --------------------------------------------------------*/


/**
  * @brief  		2'complement simply inverts each bit of given binary number, which will be 01010001. Then add 1 to the LSB of this result
	*	@param[IN]  data	Data whis is stored in the forma of 2'complement.
  * @retval 		Actual data.
  */
double Get_Byte2sComplement(uint8_t data)
{
	double retVal = (data & SIGN_BIT_MASK_IN_BYTE ) ?
									-1.0*( (uint8_t)( ~(data-1) )):			// Negative number
									data;																// Positive number

	return retVal;
}

/**
  * @brief  		2'complement simply inverts each bit of given binary number, which will be 01010001. Then add 1 to the LSB of this result
	*	@param[IN]  data	Data whis is stored in the forma of 2'complement.
  * @retval 		Actual data.
  */
double Get_HalfWord2sComplement(uint16_t data)
{
	double retVal = (data & SIGN_BIT_MASK_IN_HALFWORD) ?
									-1.0*( (uint16_t)( ~(data-1) )):			// Negative number
									data;																// Positive number

	return retVal;
}

/**
  * @brief  		2'complement simply inverts each bit of given binary number, which will be 01010001. Then add 1 to the LSB of this result
	*	@param[IN]  data	Data whis is stored in the forma of 2'complement.
  * @retval 		Actual data.
  */
double Get_Word2sComplement(uint32_t data)
{
	double retVal = (data & SIGN_BIT_MASK_IN_WORD) ?
									-1.0*( (uint32_t)( ~(data-1) )):			// Negative number
									data;																// Positive number

	return retVal;
}
