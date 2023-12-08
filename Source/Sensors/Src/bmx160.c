/* Includes ------------------------------------------------------------------*/
#include "bmx160.h"
#include <string.h>
#include <stdlib.h>
#include "calc.h"

/* Private define ------------------------------------------------------------*/
#define BMX160_SOFT_RESET_DELAY_MS		15
#define BIAS_CALC_ITERATION		50

/* Private macro -------------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
extern I2C_HandleTypeDef 		hi2c1;

/* Exported functions --------------------------------------------------------*/

/**-----------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *-----------------------------------------------------------------------------------------------------------------*/
void BMX160_InitSensor(BMX160_Handle * BMX160)
{
	BMX160->comHandle.port.comType = COM_TYPE_I2C;
	BMX160->comHandle.port.channelNo = COM_I2C1_CHANNEL_INDEX;
	BMX160->comHandle.comData.i2c.comHandle = &hi2c1;
	BMX160->comHandle.comData.i2c.devAddress = BMX160_I2C_DEV_ADDR_GND;
	
	/* Soft Reset. */
	BMX160_SetCMD(&(BMX160->comHandle), BMX160_SOFT_RESET);
	osDelay(BMX160_SOFT_RESET_DELAY_MS);
	
	/* Accel Power Config */
	BMX160_SetCMD(&(BMX160->comHandle), BMX160_SET_ACCEL_PMU_NORMAL);
	osDelay(50);
	
	/* Gyro Power Config */
	BMX160_SetCMD(&(BMX160->comHandle), BMX160_SET_GYRO_PMU_NORMAL);
	osDelay(100);
	
	/* Magnetometer Power Config */
	BMX160_SetCMD(&(BMX160->comHandle), BMX160_SET_MAG_PMU_NORMAL);
	osDelay(10);
	
	/* MagIf Config */
	BMX160_InitMagneto(&(BMX160->comHandle));
}

/**-----------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *-----------------------------------------------------------------------------------------------------------------*/
void BMX160_InitMagneto(COM_Handle * BMX160_Com)
{
	/* Manual operation is enabled */
	BMX160_MagIf0Reg magIfReg = 
	{
		.BIT.magManuelEn = true
	};
	BMX160_SetMagIF0(BMX160_Com, &magIfReg);
	osDelay(50);
	
	/* Set magnetometer slepp mode via manual operation */
	BMX160_SetMagnetoManualConfig(BMX160_Com, BMX160_MAGNETO_MANUAL_CONFIG_ADDR, BMX160_MAGNETO_SLEEP_MODE);
	
	/* REP_XY regular preset is used */
	BMX160_SetMagnetoManualConfig(BMX160_Com, BMX160_MAGNETO_MANUAL_REP_XY_ADDR, BMX160_MAGNETO_REGULAR_PRESET_XY);
	
	/* REP_Z regular preset is used */
	BMX160_SetMagnetoManualConfig(BMX160_Com, BMX160_MAGNETO_MANUAL_REP_Z_ADDR, BMX160_MAGNETO_REGULAR_PRESET_Z);
	
	/* Prepare MAG_IF[1,2,3] registers for indirect access mode */
	BMX160_SetMagIF3(BMX160_Com, 0x02);
	BMX160_SetMagIF2(BMX160_Com, 0x4C);
	BMX160_SetMagIF1(BMX160_Com, 0x42);
	
	/* Output data rate is set */
	BMX160_MagnetoConfigReg magnetoConfig =
	{
		.BIT.odr = BMX160_MAGNETO_ODR_100_HZ
	};
	BMX160_SetMagnetoConfig(BMX160_Com, &magnetoConfig);
	
	/* Burst read length is set to 8 Bytes */
	memset(&magIfReg, 0, sizeof(BMX160_MagIf0Reg));
	magIfReg.BIT.magRdBurst = BMX160_MAGNETO_BURST_READ_8_BYTE;
	BMX160_SetMagIF0(BMX160_Com, &magIfReg);
	osDelay(50);
}

/**-----------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *-----------------------------------------------------------------------------------------------------------------*/
void BMX160_SetMagnetoConfig(COM_Handle * BMX160_Com, BMX160_MagnetoConfigReg * magnetoConfig)
{
	uint8_t sendVal = magnetoConfig->BYTE;
		
	COM_I2C_DATA(BMX160_Com).data[0] = sendVal;
	COM_I2C_DATA(BMX160_Com).dataSize = 1;
	COM_I2C_DATA(BMX160_Com).memAddress = BMX160_MAGNETO_CONFIG_ADDR;
	COM_I2C_DATA(BMX160_Com).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	COM_RegisterSetter(BMX160_Com);
}

/**-----------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *-----------------------------------------------------------------------------------------------------------------*/
void BMX160_SetCMD(COM_Handle * BMX160_Com, uint8_t data)
{
	uint8_t sendVal = data;
		
	COM_I2C_DATA(BMX160_Com).data[0] = sendVal;
	COM_I2C_DATA(BMX160_Com).dataSize = 1;
	COM_I2C_DATA(BMX160_Com).memAddress = BMX160_CMD_ADDR;
	COM_I2C_DATA(BMX160_Com).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	COM_RegisterSetter(BMX160_Com);
}

/**-----------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *-----------------------------------------------------------------------------------------------------------------*/
void BMX160_SetMagIF0(COM_Handle * BMX160_Com, BMX160_MagIf0Reg * magIfReg)
{
	uint8_t sendVal = magIfReg->BYTE;
		
	COM_I2C_DATA(BMX160_Com).data[0] = sendVal;
	COM_I2C_DATA(BMX160_Com).dataSize = 1;
	COM_I2C_DATA(BMX160_Com).memAddress = BMX160_MAG_IF_0_ADDR;
	COM_I2C_DATA(BMX160_Com).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	COM_RegisterSetter(BMX160_Com);
}

/**-----------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *-----------------------------------------------------------------------------------------------------------------*/
void BMX160_SetMagIF1(COM_Handle * BMX160_Com, uint8_t addrToRead)
{
	uint8_t sendVal = addrToRead;
		
	COM_I2C_DATA(BMX160_Com).data[0] = sendVal;
	COM_I2C_DATA(BMX160_Com).dataSize = 1;
	COM_I2C_DATA(BMX160_Com).memAddress = BMX160_MAG_IF_1_ADDR;
	COM_I2C_DATA(BMX160_Com).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	COM_RegisterSetter(BMX160_Com);	
}

/**-----------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *-----------------------------------------------------------------------------------------------------------------*/
void BMX160_SetMagIF2(COM_Handle * BMX160_Com, uint8_t addrToWrite)
{
	uint8_t sendVal = addrToWrite;
		
	COM_I2C_DATA(BMX160_Com).data[0] = sendVal;
	COM_I2C_DATA(BMX160_Com).dataSize = 1;
	COM_I2C_DATA(BMX160_Com).memAddress = BMX160_MAG_IF_2_ADDR;
	COM_I2C_DATA(BMX160_Com).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	COM_RegisterSetter(BMX160_Com);	
}

/**-----------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *-----------------------------------------------------------------------------------------------------------------*/
void BMX160_SetMagIF3(COM_Handle * BMX160_Com, BMX160_MagnetoManualCmds manualCmd)
{
	uint8_t sendVal = manualCmd;
		
	COM_I2C_DATA(BMX160_Com).data[0] = sendVal;
	COM_I2C_DATA(BMX160_Com).dataSize = 1;
	COM_I2C_DATA(BMX160_Com).memAddress = BMX160_MAG_IF_3_ADDR;
	COM_I2C_DATA(BMX160_Com).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	COM_RegisterSetter(BMX160_Com);	
}

/**-----------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *-----------------------------------------------------------------------------------------------------------------*/
void BMX160_SetMagnetoManualConfig(COM_Handle * BMX160_Com, uint8_t addrToWrite, BMX160_MagnetoManualCmds manualCmd)
{
	BMX160_SetMagIF3(BMX160_Com, manualCmd);
	BMX160_SetMagIF2(BMX160_Com, addrToWrite);
}

/**-----------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *-----------------------------------------------------------------------------------------------------------------*/
void BMX160_GetRawData(BMX160_Handle * BMX160)
{
	int16_t xData=0,yData=0,zData=0;
	
	/* All datas are read at once */
	COM_I2C_DATA(&(BMX160->comHandle)).dataSize 	 = BMX160_DATA_CNT;
	COM_I2C_DATA(&(BMX160->comHandle)).memAddress  = BMX160_DATA_0_ADDR;
	COM_I2C_DATA(&(BMX160->comHandle)).memAddSize  = I2C_MEMADD_SIZE_8BIT;
	
	COM_RegisterGetter(&(BMX160->comHandle));
			
	/* Magnetometer Datas */
	xData = (int16_t)(COM_I2C_DATA(&(BMX160->comHandle)).data[1]<<8 | COM_I2C_DATA(&(BMX160->comHandle)).data[0]);
	yData = (int16_t)(COM_I2C_DATA(&(BMX160->comHandle)).data[3]<<8 | COM_I2C_DATA(&(BMX160->comHandle)).data[2]);
	zData = (int16_t)(COM_I2C_DATA(&(BMX160->comHandle)).data[5]<<8 | COM_I2C_DATA(&(BMX160->comHandle)).data[4]);
	
	BMX160->rawMag.rawXData = xData*BMX160_MAGN_UT_LSB;
	BMX160->rawMag.rawYData = yData*BMX160_MAGN_UT_LSB;
	BMX160->rawMag.rawZData = zData*BMX160_MAGN_UT_LSB;

	/* Gyroscope Datas */
	xData = (int16_t)(COM_I2C_DATA(&(BMX160->comHandle)).data[9]<<8  | COM_I2C_DATA(&(BMX160->comHandle)).data[8]);
	yData = (int16_t)(COM_I2C_DATA(&(BMX160->comHandle)).data[11]<<8 | COM_I2C_DATA(&(BMX160->comHandle)).data[10]);
	zData = (int16_t)(COM_I2C_DATA(&(BMX160->comHandle)).data[13]<<8 | COM_I2C_DATA(&(BMX160->comHandle)).data[12]);
	
	BMX160->rawGyro.rawXData = DEGREE_TO_RADIAN(xData*BMX160_GYRO_SENSITIVITY_2000DPS);		/* Unit is rad/sec */
	BMX160->rawGyro.rawYData = DEGREE_TO_RADIAN(yData*BMX160_GYRO_SENSITIVITY_2000DPS);
	BMX160->rawGyro.rawZData = DEGREE_TO_RADIAN(zData*BMX160_GYRO_SENSITIVITY_2000DPS);

	/* Accelerometer Datas */
	xData = (int16_t)(COM_I2C_DATA(&(BMX160->comHandle)).data[15]<<8 | COM_I2C_DATA(&(BMX160->comHandle)).data[14]);
	yData = (int16_t)(COM_I2C_DATA(&(BMX160->comHandle)).data[17]<<8 | COM_I2C_DATA(&(BMX160->comHandle)).data[16]);
	zData = (int16_t)(COM_I2C_DATA(&(BMX160->comHandle)).data[19]<<8 | COM_I2C_DATA(&(BMX160->comHandle)).data[18]);
	
	BMX160->rawAccel.rawXData = xData*(BMX160_ACCEL_MG_LSB_2G);	/* Unit is g */
	BMX160->rawAccel.rawYData = yData*(BMX160_ACCEL_MG_LSB_2G);
	BMX160->rawAccel.rawZData = zData*(BMX160_ACCEL_MG_LSB_2G);
	
	//BMX160->rawAccelData.rawXData = xData*(BMX160_ACCEL_MG_LSB_2G * GRAVITY_ACCELERATION_M_S2);	/* Unit is m/s^2 */
	//BMX160->rawAccelData.rawYData = yData*(BMX160_ACCEL_MG_LSB_2G * GRAVITY_ACCELERATION_M_S2);
	//BMX160->rawAccelData.rawZData = zData*(BMX160_ACCEL_MG_LSB_2G * GRAVITY_ACCELERATION_M_S2);	
}

/**-----------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *-----------------------------------------------------------------------------------------------------------------*/
void BMX160_GetAngle(BMX160_Handle * BMX160) 
{
	BMX160_GetRawData(BMX160);
	
	/* Sensor specific data format is converted to AHRS based data format. */
//	BMX160->imuAccelData.xData = BMX160->rawAccelData.rawXData;
//	BMX160->imuAccelData.yData = BMX160->rawAccelData.rawYData;
//	BMX160->imuAccelData.zData = BMX160->rawAccelData.rawZData;
//	
//	BMX160->imuGyroData.xData = BMX160->rawGyroData.rawXData;
//	BMX160->imuGyroData.yData = BMX160->rawGyroData.rawYData;
//	BMX160->imuGyroData.zData = BMX160->rawGyroData.rawZData;
//	
//	BMX160->imuMagnetoData.xData = 0.1;//BMX160->rawMagnetoData.rawXData;
//	BMX160->imuMagnetoData.yData = 0.1;//BMX160->rawMagnetoData.rawYData;
//	BMX160->imuMagnetoData.zData = 0.1;//BMX160->rawMagnetoData.rawZData;
//	
//	/* Quaternions are calculated */
//	AHRS_GetMadgwickQuaternion(&(BMX160->imuAccelData), &(BMX160->imuGyroData), &(BMX160->imuMagnetoData), &(BMX160->Quaternions));
//	
//	/* Convert to Euler angles */
//	AHRS_QuaternionToEulerAngles(&(BMX160->Quaternions), &(BMX160->EulerAngles));
}

/**------------------------------------------------------------------------------
  * @brief  			
	* @note					
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *------------------------------------------------------------------------------*/
//void BMX160_CalculateBias(BMX160_Handle * BMX160)
//{
//	BMX160_RawData sumAccel={0};
//	BMX160_RawData sumGyro={0};
//	BMX160_RawData sumMag={0};
//	
//	for(uint8_t i=0; i<BIAS_CALC_ITERATION; i++)
//	{
//		BMX160_GetRawData(BMX160);
//		
//		sumAccel.rawXData += BMX160->rawAccel.rawXData;
//		sumAccel.rawYData += BMX160->rawAccel.rawYData;
//		sumAccel.rawZData += BMX160->rawAccel.rawZData;
//		
//		sumGyro.rawXData += BMX160->rawGyro.rawXData;
//		sumGyro.rawYData += BMX160->rawGyro.rawYData;
//		sumGyro.rawZData += BMX160->rawGyro.rawZData;
//		
//		sumMag.rawXData += BMX160->rawMag.rawXData;
//		sumMag.rawYData += BMX160->rawMag.rawYData;
//		sumMag.rawZData += BMX160->rawMag.rawZData;
//	}
//	
////	BMX160->accelBiasData.rawXData = (sumAccelDatas.rawXData/(float)BIAS_CALC_ITERATION);
////	BMX160->accelBiasData.rawYData = (sumAccelDatas.rawYData/(float)BIAS_CALC_ITERATION);
////	BMX160->accelBiasData.rawZData = (sumAccelDatas.rawZData/(float)BIAS_CALC_ITERATION);
////	
////	BMX160->accelBiasData.rawZData -= GRAVITY_ACCELERATION_M_S2;
////
////	BMX160->gyroBiasData.rawXData = (sumGyroDatas.rawXData/(float)BIAS_CALC_ITERATION);
////	BMX160->gyroBiasData.rawYData = (sumGyroDatas.rawYData/(float)BIAS_CALC_ITERATION);
////	BMX160->gyroBiasData.rawZData = (sumGyroDatas.rawZData/(float)BIAS_CALC_ITERATION);
//	
//	BMX160->biasMag.rawXData = (sumMag.rawXData/(float)BIAS_CALC_ITERATION);
//	BMX160->biasMag.rawYData = (sumMag.rawYData/(float)BIAS_CALC_ITERATION);
//	BMX160->biasMag.rawZData = (sumMag.rawZData/(float)BIAS_CALC_ITERATION);
//}

/**------------------------------------------------------------------------------
  * @brief  			
	* @note					
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *------------------------------------------------------------------------------*/
//void BMX160_RemoveBias(BMX160_Handle * BMX160)
//{
	/* Remove bias */
//	BMX160->rawMag.rawXData -= BMX160->biasMag.rawXData;
//	BMX160->rawMag.rawYData -= BMX160->biasMag.rawYData;
//	BMX160->rawMag.rawZData -= BMX160->biasMag.rawZData;
//}

/* Private functions ---------------------------------------------------------*/
