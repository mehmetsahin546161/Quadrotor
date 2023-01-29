#include 	"itg3205.h"
#include 	"cmsis_os2.h"
#include 	"com_input.h"
#include	"defines.h"
#include   "calc.h"

/* Private constants ---------------------------------------------------------*/


/* Exported functions --------------------------------------------------------*/

/**------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief  		Sets basic functionalities.
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void ITG3205_InitSensor(ComInput_Handle * ITG3205)
{
	/* Register sensor for com input. */
	ComInput_AddInputDevice(ITG3205);
	
	ITG3205_FullScaleAndLowPassReg fullScaleAndLowPassReg = 
	{
		.BIT.fullScale = ITG3205_FS_2000_DEG_PER_SEC,
		.BIT.lowPassFilter = ITG3205_LOWPASS_256HZ_SAMPLE_RATE_8kHZ
	};
	
	ITG3205_SetFullScaleAndLowPass(ITG3205, &fullScaleAndLowPassReg);
}

/**------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
uint8_t ITG3205_WhoAmI(ComInput_Handle * ITG3205)
{
	COM_INPUT_I2C_DATA(ITG3205).dataSize = 1;
	COM_INPUT_I2C_DATA(ITG3205).memAddress = ITG3205_WHO_AM_I;
	COM_INPUT_I2C_DATA(ITG3205).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	ComInput_RegisterGetter(ITG3205);
											
	return COM_INPUT_I2C_DATA(ITG3205).data[0];
}

/**------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void ITG3205_SetSampleRateDivider(ComInput_Handle * ITG3205, uint8_t sampleRateDiv)
{
	uint8_t sendVal = sampleRateDiv;
		
	COM_INPUT_I2C_DATA(ITG3205).data[0] = sendVal;
	COM_INPUT_I2C_DATA(ITG3205).dataSize = 1;
	COM_INPUT_I2C_DATA(ITG3205).memAddress = ITG3205_SMPLRT_DIV;
	COM_INPUT_I2C_DATA(ITG3205).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	ComInput_RegisterSetter(ITG3205);
}

/**------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
uint8_t ITG3205_GetSampleRateDivider(ComInput_Handle * ITG3205)
{
	COM_INPUT_I2C_DATA(ITG3205).dataSize = 1;
	COM_INPUT_I2C_DATA(ITG3205).memAddress = ITG3205_SMPLRT_DIV;
	COM_INPUT_I2C_DATA(ITG3205).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	ComInput_RegisterGetter(ITG3205);
	
	return COM_INPUT_I2C_DATA(ITG3205).data[0];
}

/**------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void ITG3205_SetFullScaleAndLowPass(ComInput_Handle * ITG3205, const ITG3205_FullScaleAndLowPassReg * fullScaleAndLowPass)
{
	uint8_t sendVal = fullScaleAndLowPass->BYTE;
	
	COM_INPUT_I2C_DATA(ITG3205).data[0] = sendVal;
	COM_INPUT_I2C_DATA(ITG3205).dataSize = 1;
	COM_INPUT_I2C_DATA(ITG3205).memAddress = ITG3205_DLPF_FS;
	COM_INPUT_I2C_DATA(ITG3205).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	ComInput_RegisterSetter(ITG3205);
}

/**------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void ITG3205_GetFullScaleAndLowPass(ComInput_Handle * ITG3205, ITG3205_FullScaleAndLowPassReg * fullScaleAndLowPass)
{
	COM_INPUT_I2C_DATA(ITG3205).dataSize = 1;
	COM_INPUT_I2C_DATA(ITG3205).memAddress = ITG3205_DLPF_FS;
	COM_INPUT_I2C_DATA(ITG3205).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	ComInput_RegisterGetter(ITG3205);
											
	fullScaleAndLowPass->BYTE = COM_INPUT_I2C_DATA(ITG3205).data[0];
}

/**------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void ITG3205_SetInterruptConfig(ComInput_Handle * ITG3205, const ITG3205_InterruptConfigReg * intConfig)
{
	uint8_t sendVal = intConfig->BYTE;
		
	COM_INPUT_I2C_DATA(ITG3205).data[0] = sendVal;
	COM_INPUT_I2C_DATA(ITG3205).dataSize = 1;
	COM_INPUT_I2C_DATA(ITG3205).memAddress = ITG3205_INT_CFG;
	COM_INPUT_I2C_DATA(ITG3205).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	ComInput_RegisterSetter(ITG3205);
}

/**------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void ITG3205_GetInterruptConfig(ComInput_Handle * ITG3205, ITG3205_InterruptConfigReg * intConfig)
{
	COM_INPUT_I2C_DATA(ITG3205).dataSize = 1;
	COM_INPUT_I2C_DATA(ITG3205).memAddress = ITG3205_INT_CFG;
	COM_INPUT_I2C_DATA(ITG3205).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	ComInput_RegisterGetter(ITG3205);
											
	intConfig->BYTE = COM_INPUT_I2C_DATA(ITG3205).data[0];
}

/**------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void ITG3205_GetInterruptStatus(ComInput_Handle * ITG3205, ITG3205_IntStatusReg * intStatus)
{
	COM_INPUT_I2C_DATA(ITG3205).dataSize = 1;
	COM_INPUT_I2C_DATA(ITG3205).memAddress = ITG3205_INT_STATUS;
	COM_INPUT_I2C_DATA(ITG3205).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	ComInput_RegisterGetter(ITG3205);
											
	intStatus->BYTE = COM_INPUT_I2C_DATA(ITG3205).data[0];
}

/**------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void ITG3205_GetRawDatas(ComInput_Handle * ITG3205, ITG3205_RawDatas * rawDatas)
{
	COM_INPUT_I2C_DATA(ITG3205).dataSize = 8;
	COM_INPUT_I2C_DATA(ITG3205).memAddress = ITG3205_START_OF_DATA_REGS;
	COM_INPUT_I2C_DATA(ITG3205).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	ComInput_RegisterGetter(ITG3205);
							
	uint16_t temperature 	= COM_INPUT_I2C_DATA(ITG3205).data[0]<<8 | COM_INPUT_I2C_DATA(ITG3205).data[1];
	uint16_t xData 				= COM_INPUT_I2C_DATA(ITG3205).data[2]<<8 | COM_INPUT_I2C_DATA(ITG3205).data[3];
	uint16_t yData 				= COM_INPUT_I2C_DATA(ITG3205).data[4]<<8 | COM_INPUT_I2C_DATA(ITG3205).data[5];
	uint16_t zData 				= COM_INPUT_I2C_DATA(ITG3205).data[6]<<8 | COM_INPUT_I2C_DATA(ITG3205).data[7];
	
	rawDatas->rawXData = Calc_GetHalfWord2sComplement(xData)*ITG3205_GYRO_DATA_SCALE_FACTOR;
	rawDatas->rawYData = Calc_GetHalfWord2sComplement(yData)*ITG3205_GYRO_DATA_SCALE_FACTOR;
	rawDatas->rawZData = Calc_GetHalfWord2sComplement(zData)*ITG3205_GYRO_DATA_SCALE_FACTOR;
}

/**------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void ITG3205_SetPowerManagement(ComInput_Handle * ITG3205, const ITG3205_PowerManagementReg * powerManagement)
{
	uint8_t sendVal = powerManagement->BYTE;
		
	COM_INPUT_I2C_DATA(ITG3205).data[0] = sendVal;
	COM_INPUT_I2C_DATA(ITG3205).dataSize = 1;
	COM_INPUT_I2C_DATA(ITG3205).memAddress = ITG3205_PWR_MGM;
	COM_INPUT_I2C_DATA(ITG3205).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	ComInput_RegisterSetter(ITG3205);
}

/**------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void ITG3205_GetPowerManagement(ComInput_Handle * ITG3205, ITG3205_PowerManagementReg * powerManagement)
{
	COM_INPUT_I2C_DATA(ITG3205).dataSize = 1;
	COM_INPUT_I2C_DATA(ITG3205).memAddress = ITG3205_PWR_MGM;
	COM_INPUT_I2C_DATA(ITG3205).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	ComInput_RegisterGetter(ITG3205);
											
	powerManagement->BYTE = COM_INPUT_I2C_DATA(ITG3205).data[0];
}