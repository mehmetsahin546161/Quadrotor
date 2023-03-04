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
void HMC5883L_InitSensor(COM_Handle * HMC5883L)
{
	
}

/**------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void HMC5883L_SetConfiguration(COM_Handle * HMC5883L, const HMC5883L_ConfigAReg * configA, const HMC5883L_ConfigBReg * configB)
{
	uint8_t sendVal = 0;
	
	if(configA != NULL)
	{
		sendVal = configA->BYTE;
		
		COM_I2C_DATA(HMC5883L).data[0] = sendVal;
		COM_I2C_DATA(HMC5883L).dataSize = 1;
		COM_I2C_DATA(HMC5883L).memAddress = HMC5883L_CONFIG_REGISTER_A_ADDR;
		COM_I2C_DATA(HMC5883L).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
		COM_RegisterSetter(HMC5883L);
	}
	
	if(configB != NULL)
	{
		sendVal = configB->BYTE;
		
		COM_I2C_DATA(HMC5883L).data[0] = sendVal;
		COM_I2C_DATA(HMC5883L).dataSize = 1;
		COM_I2C_DATA(HMC5883L).memAddress = HMC5883L_CONFIG_REGISTER_B_ADDR;
		COM_I2C_DATA(HMC5883L).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
		COM_RegisterSetter(HMC5883L);
	}
}

/**------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void HMC5883L_GetConfiguration(COM_Handle * HMC5883L, HMC5883L_ConfigAReg * configA, HMC5883L_ConfigBReg * configB)
{
	COM_I2C_DATA(HMC5883L).dataSize = 2;
	COM_I2C_DATA(HMC5883L).memAddress = HMC5883L_CONFIG_REGISTER_A_ADDR;
	COM_I2C_DATA(HMC5883L).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	COM_RegisterGetter(HMC5883L);
	
	configA->BYTE = COM_I2C_DATA(HMC5883L).data[0];
	configB->BYTE = COM_I2C_DATA(HMC5883L).data[1];

}

/**------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void HMC5883L_SetMode(COM_Handle * HMC5883L, const HMC5883L_ModeReg * mode)
{
	uint8_t sendVal = mode->BYTE;
		
	COM_I2C_DATA(HMC5883L).data[0] = sendVal;
	COM_I2C_DATA(HMC5883L).dataSize = 1;
	COM_I2C_DATA(HMC5883L).memAddress = HMC5883L_MODE_REGISTER_ADDR;
	COM_I2C_DATA(HMC5883L).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	COM_RegisterSetter(HMC5883L);
}

/**------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void HMC5883L_GetMode(COM_Handle * HMC5883L, HMC5883L_ModeReg * mode)
{
	COM_I2C_DATA(HMC5883L).dataSize = 1;
	COM_I2C_DATA(HMC5883L).memAddress = HMC5883L_MODE_REGISTER_ADDR;
	COM_I2C_DATA(HMC5883L).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	COM_RegisterGetter(HMC5883L);
	
	mode->BYTE = COM_I2C_DATA(HMC5883L).data[0];
}

/**------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void HMC5883L_GetRawDatas(COM_Handle * HMC5883L, HMC5883L_RawDatas * rawDatas)
{
	COM_I2C_DATA(HMC5883L).dataSize = 6;
	COM_I2C_DATA(HMC5883L).memAddress = HMC5883L_START_OF_DATA_REGS;
	COM_I2C_DATA(HMC5883L).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	COM_RegisterGetter(HMC5883L);
	
	int16_t xData = (int16_t)(COM_I2C_DATA(HMC5883L).data[0]<<8 | COM_I2C_DATA(HMC5883L).data[1]);
	int16_t yData = (int16_t)(COM_I2C_DATA(HMC5883L).data[2]<<8 | COM_I2C_DATA(HMC5883L).data[3]);
	int16_t zData = (int16_t)(COM_I2C_DATA(HMC5883L).data[4]<<8 | COM_I2C_DATA(HMC5883L).data[5]);
	
	rawDatas->rawXData = xData*HMC5883L_MAGNETIC_DATA_SCALE_FACTOR;
	rawDatas->rawYData = yData*HMC5883L_MAGNETIC_DATA_SCALE_FACTOR;
	rawDatas->rawZData = zData*HMC5883L_MAGNETIC_DATA_SCALE_FACTOR;
	
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
void HMC5883L_GetStatus(COM_Handle * HMC5883L, HMC5883L_StatusReg * status)
{
	COM_I2C_DATA(HMC5883L).dataSize = 1;
	COM_I2C_DATA(HMC5883L).memAddress = HMC5883L_STATUS_REGISTER_ADDR;
	COM_I2C_DATA(HMC5883L).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	COM_RegisterGetter(HMC5883L);

	status->BYTE = COM_I2C_DATA(HMC5883L).data[0];
}

/**------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void HMC5883L_GetIdentification(COM_Handle * HMC5883L, HMC5883L_Identification * identification)
{
	COM_I2C_DATA(HMC5883L).dataSize = 3;
	COM_I2C_DATA(HMC5883L).memAddress = HMC5883L_IDENTIFICATION_REGISTER_A_ADDR;
	COM_I2C_DATA(HMC5883L).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	COM_RegisterGetter(HMC5883L);
	
	for(uint8_t i = 0; i < HMC5883L_IDENTIFICATION_COUNT; i++)
	{
		identification->identification[i] = COM_I2C_DATA(HMC5883L).data[i];
	}
}