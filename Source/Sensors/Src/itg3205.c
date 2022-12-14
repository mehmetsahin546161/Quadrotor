#include 	"itg3205.h"
#include 	"cmsis_os2.h"
#include 	"com_input.h"
#include	"defines.h"
#include   "calc.h"

/* Private constants ---------------------------------------------------------*/
#define ITG3205_ZERO_RATE_X_VALUE   (0.9)
#define ITG3205_ZERO_RATE_Y_VALUE   (0.7)
#define ITG3205_ZERO_RATE_Z_VALUE   (0.7)

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  		Sets basic functionalities.
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  */
void ITG3205_InitSensor(const COM_Input_HandleTypeDef * ITG3205)
{

}

/**
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  */
uint8_t ITG3205_WhoAmI(const COM_Input_HandleTypeDef * ITG3205)
{
	COM_Input_TempDataTypeDef comData =
	{
		.dataSize = 1,
		.memAddress = WHO_AM_I
	};
	
	COM_Input_RegisterGetter(ITG3205, &comData);
											
	return comData.data[0];
}

/**
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  */
void ITG3205_SetSampleRateDivider(const COM_Input_HandleTypeDef * ITG3205, uint8_t sampleRateDiv)
{
	uint8_t sendVal = sampleRateDiv;
		
	const COM_Input_TempDataTypeDef comData =
	{
		.data[0] = sendVal,
		.dataSize = 1,
		.memAddress = SMPLRT_DIV
	};
		
	COM_Input_RegisterSetter(ITG3205, &comData);
}

/**
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  */
uint8_t ITG3205_GetSampleRateDivider(const COM_Input_HandleTypeDef * ITG3205)
{
	COM_Input_TempDataTypeDef comData =
	{
		.dataSize = 1,
		.memAddress = SMPLRT_DIV
	};
	
	COM_Input_RegisterGetter(ITG3205, &comData);
	
	return comData.data[0];
}

/**
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  */
void ITG3205_SetFullScaleAndLowPass(const COM_Input_HandleTypeDef * ITG3205, const ITG3205_FullScaleAndLowPassReg * fullScaleAndLowPass)
{
	uint8_t sendVal = fullScaleAndLowPass->BYTE;
		
	const COM_Input_TempDataTypeDef comData =
	{
		.data[0] = sendVal,
		.dataSize = 1,
		.memAddress = DLPF_FS
	};
		
	COM_Input_RegisterSetter(ITG3205, &comData);
}

/**
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  */
void ITG3205_GetFullScaleAndLowPass(const COM_Input_HandleTypeDef * ITG3205, ITG3205_FullScaleAndLowPassReg * fullScaleAndLowPass)
{
	COM_Input_TempDataTypeDef comData =
	{
		.dataSize = 1,
		.memAddress = DLPF_FS
	};
	
	COM_Input_RegisterGetter(ITG3205, &comData);
											
	fullScaleAndLowPass->BYTE = comData.data[0];
}

/**
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  */
void ITG3205_SetInterruptConfig(const COM_Input_HandleTypeDef * ITG3205, const ITG3205_InterruptConfigReg * intConfig)
{
	uint8_t sendVal = intConfig->BYTE;
		
	const COM_Input_TempDataTypeDef comData =
	{
		.data[0] = sendVal,
		.dataSize = 1,
		.memAddress = INT_CFG
	};
		
	COM_Input_RegisterSetter(ITG3205, &comData);
}

/**
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  */
void ITG3205_GetInterruptConfig(const COM_Input_HandleTypeDef * ITG3205, ITG3205_InterruptConfigReg * intConfig)
{
	COM_Input_TempDataTypeDef comData =
	{
		.dataSize = 1,
		.memAddress = INT_CFG
	};
	
	COM_Input_RegisterGetter(ITG3205, &comData);
											
	intConfig->BYTE = comData.data[0];
}

/**
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  */
void ITG3205_GetInterruptStatus(const COM_Input_HandleTypeDef * ITG3205, ITG3205_IntStatusReg * intStatus)
{
	COM_Input_TempDataTypeDef comData =
	{
		.dataSize = 1,
		.memAddress = INT_STATUS
	};
	
	COM_Input_RegisterGetter(ITG3205, &comData);
											
	intStatus->BYTE = comData.data[0];
}

/**
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  */
void ITG3205_GetRawDatas(const COM_Input_HandleTypeDef * ITG3205, ITG3205_RawDatas * rawDatas)
{
	COM_Input_TempDataTypeDef comData =
	{
		.dataSize = 8,
		.memAddress = ITG3205_START_OF_DATA_REGS
	};
	
	COM_Input_RegisterGetter(ITG3205, &comData);
							
	uint16_t temperature = comData.data[0]<<8 | comData.data[1];
	uint16_t xData = comData.data[2]<<8 | comData.data[3];
	uint16_t yData = comData.data[4]<<8 | comData.data[5];
	uint16_t zData = comData.data[6]<<8 | comData.data[7];
	
	rawDatas->rawXData = DEGREE_TO_RADIAN(Get_HalfWord2sComplement(xData)*ITG3205_GYRO_DATA_SCALE_FACTOR) - ITG3205_ZERO_RATE_X_VALUE;
	rawDatas->rawYData = DEGREE_TO_RADIAN(Get_HalfWord2sComplement(yData)*ITG3205_GYRO_DATA_SCALE_FACTOR) - ITG3205_ZERO_RATE_Y_VALUE;
	rawDatas->rawZData = DEGREE_TO_RADIAN(Get_HalfWord2sComplement(zData)*ITG3205_GYRO_DATA_SCALE_FACTOR) - ITG3205_ZERO_RATE_Z_VALUE;
}

/**
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  */
void ITG3205_SetPowerManagement(const COM_Input_HandleTypeDef * ITG3205, const ITG3205_PowerManagementReg * powerManagement)
{
	uint8_t sendVal = powerManagement->BYTE;
		
	const COM_Input_TempDataTypeDef comData =
	{
		.data[0] = sendVal,
		.dataSize = 1,
		.memAddress = PWR_MGM
	};
		
	COM_Input_RegisterSetter(ITG3205, &comData);
}

/**
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  */
void ITG3205_GetPowerManagement(const COM_Input_HandleTypeDef * ITG3205, ITG3205_PowerManagementReg * powerManagement)
{
	COM_Input_TempDataTypeDef comData =
	{
		.dataSize = 1,
		.memAddress = PWR_MGM
	};
	
	COM_Input_RegisterGetter(ITG3205, &comData);
											
	powerManagement->BYTE = comData.data[0];
}