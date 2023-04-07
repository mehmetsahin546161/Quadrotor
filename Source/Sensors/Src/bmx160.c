/* Includes ------------------------------------------------------------------*/
#include "bmx160.h"
#include <string.h>

/* Private define ------------------------------------------------------------*/

#define BMX160_SOFT_RESET_DELAY_MS		15


/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/**-----------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *-----------------------------------------------------------------------------------------------------------------*/
void BMX160_InitSensor(COM_Handle * BMX160)
{
	/* Soft Reset. */
	BMX160_SetCMD(BMX160, BMX160_SOFT_RESET);
	osDelay(BMX160_SOFT_RESET_DELAY_MS);
	
	/* Accel Power Config */
	BMX160_SetCMD(BMX160, BMX160_SET_ACCEL_PMU_NORMAL);
	osDelay(50);
	
	/* Gyro Power Config */
	BMX160_SetCMD(BMX160, BMX160_SET_GYRO_PMU_NORMAL);
	osDelay(100);
	
	/* Magnetometer Power Config */
	BMX160_SetCMD(BMX160, BMX160_SET_MAG_PMU_NORMAL);
	osDelay(10);
	
	/* MagIf Config */
	BMX160_InitMagneto(BMX160);
	
	
}

/**-----------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *-----------------------------------------------------------------------------------------------------------------*/
void BMX160_InitMagneto(COM_Handle * BMX160)
{
	/* Manual operation is enabled */
	BMX160_MagIf0Reg magIfReg = 
	{
		.BIT.magManuelEn = true
	};
	BMX160_SetMagIF0(BMX160, &magIfReg);
	osDelay(50);
	
	/* Set magnetometer slepp mode via manual operation */
	BMX160_SetMagnetoManualConfig(BMX160, BMX160_MAGNETO_MANUAL_CONFIG_ADDR, BMX160_MAGNETO_SLEEP_MODE);
	
	/* REP_XY regular preset is used */
	BMX160_SetMagnetoManualConfig(BMX160, BMX160_MAGNETO_MANUAL_REP_XY_ADDR, BMX160_MAGNETO_REGULAR_PRESET_XY);
	
	/* REP_Z regular preset is used */
	BMX160_SetMagnetoManualConfig(BMX160, BMX160_MAGNETO_MANUAL_REP_Z_ADDR, BMX160_MAGNETO_REGULAR_PRESET_Z);
	
	/* Prepare MAG_IF[1,2,3] registers for indirect access mode */
	BMX160_SetMagIF3(BMX160, 0x02);
	BMX160_SetMagIF2(BMX160, 0x4C);
	BMX160_SetMagIF1(BMX160, 0x42);
	
	/* Output data rate is set */
	BMX160_MagnetoConfigReg magnetoConfig =
	{
		.BIT.odr = BMX160_MAGNETO_ODR_100_HZ
	};
	BMX160_SetMagnetoConfig(BMX160, &magnetoConfig);
	
	/* Burst read length is set to 8 Bytes */
	memset(&magIfReg, 0, sizeof(BMX160_MagIf0Reg));
	magIfReg.BIT.magRdBurst = BMX160_MAGNETO_BURST_READ_8_BYTE;
	BMX160_SetMagIF0(BMX160, &magIfReg);
	osDelay(50);
}

/**-----------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *-----------------------------------------------------------------------------------------------------------------*/
void BMX160_SetMagnetoConfig(COM_Handle * BMX160, BMX160_MagnetoConfigReg * magnetoConfig)
{
	uint8_t sendVal = magnetoConfig->BYTE;
		
	COM_I2C_DATA(BMX160).data[0] = sendVal;
	COM_I2C_DATA(BMX160).dataSize = 1;
	COM_I2C_DATA(BMX160).memAddress = BMX160_MAGNETO_CONFIG_ADDR;
	COM_I2C_DATA(BMX160).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	COM_RegisterSetter(BMX160);
}

/**-----------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *-----------------------------------------------------------------------------------------------------------------*/
void BMX160_SetCMD(COM_Handle * BMX160, uint8_t data)
{
	uint8_t sendVal = data;
		
	COM_I2C_DATA(BMX160).data[0] = sendVal;
	COM_I2C_DATA(BMX160).dataSize = 1;
	COM_I2C_DATA(BMX160).memAddress = BMX160_CMD_ADDR;
	COM_I2C_DATA(BMX160).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	COM_RegisterSetter(BMX160);
}

/**-----------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *-----------------------------------------------------------------------------------------------------------------*/
void BMX160_SetMagIF0(COM_Handle * BMX160, BMX160_MagIf0Reg * magIfReg)
{
	uint8_t sendVal = magIfReg->BYTE;
		
	COM_I2C_DATA(BMX160).data[0] = sendVal;
	COM_I2C_DATA(BMX160).dataSize = 1;
	COM_I2C_DATA(BMX160).memAddress = BMX160_MAG_IF_0_ADDR;
	COM_I2C_DATA(BMX160).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	COM_RegisterSetter(BMX160);
}

/**-----------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *-----------------------------------------------------------------------------------------------------------------*/
void BMX160_SetMagIF1(COM_Handle * BMX160, uint8_t addrToRead)
{
	uint8_t sendVal = addrToRead;
		
	COM_I2C_DATA(BMX160).data[0] = sendVal;
	COM_I2C_DATA(BMX160).dataSize = 1;
	COM_I2C_DATA(BMX160).memAddress = BMX160_MAG_IF_1_ADDR;
	COM_I2C_DATA(BMX160).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	COM_RegisterSetter(BMX160);	
}

/**-----------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *-----------------------------------------------------------------------------------------------------------------*/
void BMX160_SetMagIF2(COM_Handle * BMX160, uint8_t addrToWrite)
{
	uint8_t sendVal = addrToWrite;
		
	COM_I2C_DATA(BMX160).data[0] = sendVal;
	COM_I2C_DATA(BMX160).dataSize = 1;
	COM_I2C_DATA(BMX160).memAddress = BMX160_MAG_IF_2_ADDR;
	COM_I2C_DATA(BMX160).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	COM_RegisterSetter(BMX160);	
}

/**-----------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *-----------------------------------------------------------------------------------------------------------------*/
void BMX160_SetMagIF3(COM_Handle * BMX160, BMX160_MagnetoManualCmds manualCmd)
{
	uint8_t sendVal = manualCmd;
		
	COM_I2C_DATA(BMX160).data[0] = sendVal;
	COM_I2C_DATA(BMX160).dataSize = 1;
	COM_I2C_DATA(BMX160).memAddress = BMX160_MAG_IF_3_ADDR;
	COM_I2C_DATA(BMX160).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	COM_RegisterSetter(BMX160);	
}

/**-----------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *-----------------------------------------------------------------------------------------------------------------*/
void BMX160_SetMagnetoManualConfig(COM_Handle * BMX160, uint8_t addrToWrite, BMX160_MagnetoManualCmds manualCmd)
{
	BMX160_SetMagIF3(BMX160, manualCmd);
	BMX160_SetMagIF2(BMX160, addrToWrite);
}



