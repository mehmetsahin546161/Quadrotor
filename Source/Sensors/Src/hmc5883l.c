/* Includes ------------------------------------------------------------------*/
#include "hmc5883l.h"
#include "calc.h"

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

/**------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void HMC5883L_InitSensor(ComInput_Handle * HMC5883L)
{
	/* Register sensor for com input. */
	ComInput_AddInputDevice(HMC5883L);
}

/**------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void HMC5883L_SetConfiguration(ComInput_Handle * HMC5883L, const HMC5883L_ConfigAReg * configA, const HMC5883L_ConfigBReg * configB)
{
	uint8_t sendVal = 0;
	
	if(configA != NULL)
	{
		sendVal = configA->BYTE;
		
		COM_INPUT_I2C_DATA(HMC5883L).data[0] = sendVal;
		COM_INPUT_I2C_DATA(HMC5883L).dataSize = 1;
		COM_INPUT_I2C_DATA(HMC5883L).memAddress = HMC5883L_CONFIG_REGISTER_A_ADDR;
		COM_INPUT_I2C_DATA(HMC5883L).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
		ComInput_RegisterSetter(HMC5883L);
	}
	
	if(configB != NULL)
	{
		sendVal = configB->BYTE;
		
		COM_INPUT_I2C_DATA(HMC5883L).data[0] = sendVal;
		COM_INPUT_I2C_DATA(HMC5883L).dataSize = 1;
		COM_INPUT_I2C_DATA(HMC5883L).memAddress = HMC5883L_CONFIG_REGISTER_B_ADDR;
		COM_INPUT_I2C_DATA(HMC5883L).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
		ComInput_RegisterSetter(HMC5883L);
	}
}

/**------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void HMC5883L_GetConfiguration(ComInput_Handle * HMC5883L, HMC5883L_ConfigAReg * configA, HMC5883L_ConfigBReg * configB)
{
	COM_INPUT_I2C_DATA(HMC5883L).dataSize = 2;
	COM_INPUT_I2C_DATA(HMC5883L).memAddress = HMC5883L_CONFIG_REGISTER_A_ADDR;
	COM_INPUT_I2C_DATA(HMC5883L).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	ComInput_RegisterGetter(HMC5883L);
	
	configA->BYTE = COM_INPUT_I2C_DATA(HMC5883L).data[0];
	configB->BYTE = COM_INPUT_I2C_DATA(HMC5883L).data[1];

}

/**------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void HMC5883L_SetMode(ComInput_Handle * HMC5883L, const HMC5883L_ModeReg * mode)
{
	uint8_t sendVal = mode->BYTE;
		
	COM_INPUT_I2C_DATA(HMC5883L).data[0] = sendVal;
	COM_INPUT_I2C_DATA(HMC5883L).dataSize = 1;
	COM_INPUT_I2C_DATA(HMC5883L).memAddress = HMC5883L_MODE_REGISTER_ADDR;
	COM_INPUT_I2C_DATA(HMC5883L).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	ComInput_RegisterSetter(HMC5883L);
}

/**------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void HMC5883L_GetMode(ComInput_Handle * HMC5883L, HMC5883L_ModeReg * mode)
{
	COM_INPUT_I2C_DATA(HMC5883L).dataSize = 1;
	COM_INPUT_I2C_DATA(HMC5883L).memAddress = HMC5883L_MODE_REGISTER_ADDR;
	COM_INPUT_I2C_DATA(HMC5883L).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	ComInput_RegisterGetter(HMC5883L);
	
	mode->BYTE = COM_INPUT_I2C_DATA(HMC5883L).data[0];
}

/**------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void HMC5883L_GetRawDatas(ComInput_Handle * HMC5883L, HMC5883L_RawDatas * rawDatas)
{
	COM_INPUT_I2C_DATA(HMC5883L).dataSize = 6;
	COM_INPUT_I2C_DATA(HMC5883L).memAddress = HMC5883L_START_OF_DATA_REGS;
	COM_INPUT_I2C_DATA(HMC5883L).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	ComInput_RegisterGetter(HMC5883L);
	
	uint16_t xData = COM_INPUT_I2C_DATA(HMC5883L).data[0]<<8 | COM_INPUT_I2C_DATA(HMC5883L).data[1];
	uint16_t yData = COM_INPUT_I2C_DATA(HMC5883L).data[2]<<8 | COM_INPUT_I2C_DATA(HMC5883L).data[3];
	uint16_t zData = COM_INPUT_I2C_DATA(HMC5883L).data[4]<<8 | COM_INPUT_I2C_DATA(HMC5883L).data[5];
	
	rawDatas->rawXData = Calc_GetHalfWord2sComplement(xData)*HMC5883L_MAGNETIC_DATA_SCALE_FACTOR;
	rawDatas->rawYData = Calc_GetHalfWord2sComplement(xData)*HMC5883L_MAGNETIC_DATA_SCALE_FACTOR;
	rawDatas->rawZData = Calc_GetHalfWord2sComplement(xData)*HMC5883L_MAGNETIC_DATA_SCALE_FACTOR;
	
	/* Preparation for next meausurement in Single Conversion mode */
	HMC5883L_SetMode(HMC5883L, &(HMC5883L_ModeReg){.BIT.highSpeedI2C = HMC5883L_HIGH_SPEED_I2C_DISABLE,
																								.BIT.operatingMode = HMC5883L_SINGLE_MEAUSUREMENT_MODE});
}

/**------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void HMC5883L_GetStatus(ComInput_Handle * HMC5883L, HMC5883L_StatusReg * status)
{
	COM_INPUT_I2C_DATA(HMC5883L).dataSize = 1;
	COM_INPUT_I2C_DATA(HMC5883L).memAddress = HMC5883L_STATUS_REGISTER_ADDR;
	COM_INPUT_I2C_DATA(HMC5883L).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	ComInput_RegisterGetter(HMC5883L);

	status->BYTE = COM_INPUT_I2C_DATA(HMC5883L).data[0];
}

/**------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void HMC5883L_GetIdentification(ComInput_Handle * HMC5883L, HMC5883L_Identification * identification)
{
	COM_INPUT_I2C_DATA(HMC5883L).dataSize = 3;
	COM_INPUT_I2C_DATA(HMC5883L).memAddress = HMC5883L_IDENTIFICATION_REGISTER_A_ADDR;
	COM_INPUT_I2C_DATA(HMC5883L).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	ComInput_RegisterGetter(HMC5883L);
	
	for(uint8_t i = 0; i < HMC5883L_IDENTIFICATION_COUNT; i++)
	{
		identification->identification[i] = COM_INPUT_I2C_DATA(HMC5883L).data[i];
	}
}