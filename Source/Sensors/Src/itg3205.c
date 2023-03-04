#include 	"itg3205.h"
#include 	"cmsis_os2.h"
#include	"defines.h"
#include   "calc.h"

/* Private constants ---------------------------------------------------------*/


/* Exported functions --------------------------------------------------------*/

/**-----------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *-----------------------------------------------------------------------------------------------------------------*/
void ITG3205_InitSensor(COM_Handle * ITG3205)
{
	/* Power Mnanagement */
	ITG3205_PowerManagementReg powerManagement =
	{
		.BIT.clockSrc 		= ITG3205_INTERNAL_CLOCK,
		.BIT.xAxisStandby = ITG3205_GYRO_AXIS_NORMAL_MODE,
		.BIT.yAxisStandby = ITG3205_GYRO_AXIS_NORMAL_MODE,
		.BIT.zAxisStandby = ITG3205_GYRO_AXIS_NORMAL_MODE,
		.BIT.devSleepMod 	= ITG3205_DISABLE_SLEEP_MODE,
		.BIT.reset 				= ITG3205_NOT_RESET_REGISTERS
	};
	ITG3205_SetPowerManagement(ITG3205, &powerManagement);
	
	/* Sample Rate Divider */
	ITG3205_SetSampleRateDivider(ITG3205, 0x07);
	
	/* Lowpass and Full Scale */
	ITG3205_FullScaleAndLowPassReg fullScaleAndLowPassReg = 
	{
		.BIT.fullScale 			= ITG3205_FS_2000_DEG_PER_SEC,
		.BIT.lowPassFilter 	= ITG3205_LOWPASS_5HZ_SAMPLE_RATE_1kHZ
	};
	ITG3205_SetFullScaleAndLowPass(ITG3205, &fullScaleAndLowPassReg);
}

/**-----------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *-----------------------------------------------------------------------------------------------------------------*/
uint8_t ITG3205_WhoAmI(COM_Handle * ITG3205)
{
	COM_I2C_DATA(ITG3205).dataSize = 1;
	COM_I2C_DATA(ITG3205).memAddress = ITG3205_WHO_AM_I;
	COM_I2C_DATA(ITG3205).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	COM_RegisterGetter(ITG3205);
											
	return COM_I2C_DATA(ITG3205).data[0];
}

/**-----------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *-----------------------------------------------------------------------------------------------------------------*/
void ITG3205_SetSampleRateDivider(COM_Handle * ITG3205, uint8_t sampleRateDiv)
{
	uint8_t sendVal = sampleRateDiv;
		
	COM_I2C_DATA(ITG3205).data[0] = sendVal;
	COM_I2C_DATA(ITG3205).dataSize = 1;
	COM_I2C_DATA(ITG3205).memAddress = ITG3205_SMPLRT_DIV;
	COM_I2C_DATA(ITG3205).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	COM_RegisterSetter(ITG3205);
}

/**-----------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *-----------------------------------------------------------------------------------------------------------------*/
uint8_t ITG3205_GetSampleRateDivider(COM_Handle * ITG3205)
{
	COM_I2C_DATA(ITG3205).dataSize = 1;
	COM_I2C_DATA(ITG3205).memAddress = ITG3205_SMPLRT_DIV;
	COM_I2C_DATA(ITG3205).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	COM_RegisterGetter(ITG3205);
	
	return COM_I2C_DATA(ITG3205).data[0];
}

/**-----------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *-----------------------------------------------------------------------------------------------------------------*/
void ITG3205_SetFullScaleAndLowPass(COM_Handle * ITG3205, const ITG3205_FullScaleAndLowPassReg * fullScaleAndLowPass)
{
	uint8_t sendVal = fullScaleAndLowPass->BYTE;
	
	COM_I2C_DATA(ITG3205).data[0] = sendVal;
	COM_I2C_DATA(ITG3205).dataSize = 1;
	COM_I2C_DATA(ITG3205).memAddress = ITG3205_DLPF_FS;
	COM_I2C_DATA(ITG3205).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	COM_RegisterSetter(ITG3205);
}

/**-----------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *-----------------------------------------------------------------------------------------------------------------*/
void ITG3205_GetFullScaleAndLowPass(COM_Handle * ITG3205, ITG3205_FullScaleAndLowPassReg * fullScaleAndLowPass)
{
	COM_I2C_DATA(ITG3205).dataSize = 1;
	COM_I2C_DATA(ITG3205).memAddress = ITG3205_DLPF_FS;
	COM_I2C_DATA(ITG3205).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	COM_RegisterGetter(ITG3205);
											
	fullScaleAndLowPass->BYTE = COM_I2C_DATA(ITG3205).data[0];
}

/**-----------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *-----------------------------------------------------------------------------------------------------------------*/
void ITG3205_SetInterruptConfig(COM_Handle * ITG3205, const ITG3205_InterruptConfigReg * intConfig)
{
	uint8_t sendVal = intConfig->BYTE;
		
	COM_I2C_DATA(ITG3205).data[0] = sendVal;
	COM_I2C_DATA(ITG3205).dataSize = 1;
	COM_I2C_DATA(ITG3205).memAddress = ITG3205_INT_CFG;
	COM_I2C_DATA(ITG3205).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	COM_RegisterSetter(ITG3205);
}

/**-----------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *-----------------------------------------------------------------------------------------------------------------*/
void ITG3205_GetInterruptConfig(COM_Handle * ITG3205, ITG3205_InterruptConfigReg * intConfig)
{
	COM_I2C_DATA(ITG3205).dataSize = 1;
	COM_I2C_DATA(ITG3205).memAddress = ITG3205_INT_CFG;
	COM_I2C_DATA(ITG3205).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	COM_RegisterGetter(ITG3205);
											
	intConfig->BYTE = COM_I2C_DATA(ITG3205).data[0];
}

/**-----------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *-----------------------------------------------------------------------------------------------------------------*/
void ITG3205_GetInterruptStatus(COM_Handle * ITG3205, ITG3205_IntStatusReg * intStatus)
{
	COM_I2C_DATA(ITG3205).dataSize = 1;
	COM_I2C_DATA(ITG3205).memAddress = ITG3205_INT_STATUS;
	COM_I2C_DATA(ITG3205).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	COM_RegisterGetter(ITG3205);
											
	intStatus->BYTE = COM_I2C_DATA(ITG3205).data[0];
}

/**-----------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *-----------------------------------------------------------------------------------------------------------------*/
void ITG3205_GetRawDatas(COM_Handle * ITG3205, ITG3205_RawDatas * rawDatas)
{
	COM_I2C_DATA(ITG3205).dataSize = 8;
	COM_I2C_DATA(ITG3205).memAddress = ITG3205_START_OF_DATA_REGS;
	COM_I2C_DATA(ITG3205).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	COM_RegisterGetter(ITG3205);
							
	int16_t temperature 	= (int16_t)(COM_I2C_DATA(ITG3205).data[0]<<8 | COM_I2C_DATA(ITG3205).data[1]);
	int16_t xData 				= (int16_t)(COM_I2C_DATA(ITG3205).data[2]<<8 | COM_I2C_DATA(ITG3205).data[3]);
	int16_t yData 				= (int16_t)(COM_I2C_DATA(ITG3205).data[4]<<8 | COM_I2C_DATA(ITG3205).data[5]);
	int16_t zData 				= (int16_t)(COM_I2C_DATA(ITG3205).data[6]<<8 | COM_I2C_DATA(ITG3205).data[7]);
	
	rawDatas->rawXData = xData*ITG3205_GYRO_DATA_SCALE_FACTOR;
	rawDatas->rawYData = yData*ITG3205_GYRO_DATA_SCALE_FACTOR;
	rawDatas->rawZData = zData*ITG3205_GYRO_DATA_SCALE_FACTOR;
}

/**-----------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *-----------------------------------------------------------------------------------------------------------------*/
void ITG3205_SetPowerManagement(COM_Handle * ITG3205, const ITG3205_PowerManagementReg * powerManagement)
{
	uint8_t sendVal = powerManagement->BYTE;
		
	COM_I2C_DATA(ITG3205).data[0] = sendVal;
	COM_I2C_DATA(ITG3205).dataSize = 1;
	COM_I2C_DATA(ITG3205).memAddress = ITG3205_PWR_MGM;
	COM_I2C_DATA(ITG3205).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	COM_RegisterSetter(ITG3205);
}

/**-----------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *-----------------------------------------------------------------------------------------------------------------*/
void ITG3205_GetPowerManagement(COM_Handle * ITG3205, ITG3205_PowerManagementReg * powerManagement)
{
	COM_I2C_DATA(ITG3205).dataSize = 1;
	COM_I2C_DATA(ITG3205).memAddress = ITG3205_PWR_MGM;
	COM_I2C_DATA(ITG3205).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	COM_RegisterGetter(ITG3205);
											
	powerManagement->BYTE = COM_I2C_DATA(ITG3205).data[0];
}