/* Includes ------------------------------------------------------------------*/
#include "mpu6050.h"
#include "string.h"

/* Private define ------------------------------------------------------------*/
#define MPU6050_ACCEL_BIAS_CALC_ITERATION			10
#define MPU6050_GYRO_BIAS_CALC_ITERATION			100

#define MPU6050_GRAVITY_ACCELERATION					1			/* g */

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
void MPU6050_InitSensor(COM_Handle * MPU6050)
{
	MPU6050_PowerManagement1Reg powerManagement=
	{
		.BIT.clockSelection = MPU6050_INTERNAL_8MHZ,
		.BIT.tempDisable = false,
		.BIT.cycle = false,
		.BIT.sleep = false,
		.BIT.deviceReset = false
	};
	MPU6050_SetPowerManagement(MPU6050, &powerManagement);
	
	MPU6050_SetSampleRateDivider(MPU6050, 0x07);
	
	MPU6050_AccelConfigReg accelConfig=
	{
		.BIT.fullScaleRange = MPU6050_ACCEL_FULL_SCALE_2G,
		.BIT.selTestX =false,
		.BIT.selTestY =false,
		.BIT.selTestZ =false
	};
	MPU6050_SetAccelConfig(MPU6050, &accelConfig);
	
	MPU6050_GyroConfigReg gyroConfig=
	{
		.BIT.fullScaleRange = MPU6050_GYRO_FULL_SCALE_250_DEG_PER_SEC,
		.BIT.selTestX =false,
		.BIT.selTestY =false,
		.BIT.selTestZ =false
	};
	MPU6050_SetGyroConfig(MPU6050, &gyroConfig);
}

/**-----------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *-----------------------------------------------------------------------------------------------------------------*/
void MPU6050_SetSampleRateDivider(COM_Handle * MPU6050, uint8_t sampleRateDiv)
{
	uint8_t sendVal = sampleRateDiv;
		
	COM_I2C_DATA(MPU6050).data[0] = sendVal;
	COM_I2C_DATA(MPU6050).dataSize = 1;
	COM_I2C_DATA(MPU6050).memAddress = MPU6050_SMPLRT_DIV_ADDR;
	COM_I2C_DATA(MPU6050).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	COM_RegisterSetter(MPU6050);
}

/**-----------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *-----------------------------------------------------------------------------------------------------------------*/
uint8_t MPU6050_GetSampleRateDivider(COM_Handle * MPU6050)
{
	COM_I2C_DATA(MPU6050).dataSize = 1;
	COM_I2C_DATA(MPU6050).memAddress = MPU6050_SMPLRT_DIV_ADDR;
	COM_I2C_DATA(MPU6050).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	COM_RegisterGetter(MPU6050);
	
	return COM_I2C_DATA(MPU6050).data[0];
}

/**-----------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *-----------------------------------------------------------------------------------------------------------------*/
void MPU6050_SetPowerManagement(COM_Handle * MPU6050, const MPU6050_PowerManagement1Reg * powerManagement)
{
	uint8_t sendVal = powerManagement->BYTE;
	
	COM_I2C_DATA(MPU6050).data[0] = sendVal;
	COM_I2C_DATA(MPU6050).dataSize = 1;
	COM_I2C_DATA(MPU6050).memAddress = MPU6050_PWR_MGMT_1_ADDR;
	COM_I2C_DATA(MPU6050).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	COM_RegisterSetter(MPU6050);
}

/**-----------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *-----------------------------------------------------------------------------------------------------------------*/
void MPU6050_GetPowerManagement(COM_Handle * MPU6050, MPU6050_PowerManagement1Reg * powerManagement)
{
	COM_I2C_DATA(MPU6050).dataSize = 1;
	COM_I2C_DATA(MPU6050).memAddress = MPU6050_PWR_MGMT_1_ADDR;
	COM_I2C_DATA(MPU6050).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	COM_RegisterGetter(MPU6050);
											
	powerManagement->BYTE = COM_I2C_DATA(MPU6050).data[0];
}

/**-----------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *-----------------------------------------------------------------------------------------------------------------*/
void MPU6050_SetAccelConfig(COM_Handle * MPU6050, const MPU6050_AccelConfigReg * accelConfig)
{
	uint8_t sendVal = accelConfig->BYTE;
	
	COM_I2C_DATA(MPU6050).data[0] = sendVal;
	COM_I2C_DATA(MPU6050).dataSize = 1;
	COM_I2C_DATA(MPU6050).memAddress = MPU6050_ACCEL_CONFIG_ADDR;
	COM_I2C_DATA(MPU6050).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	COM_RegisterSetter(MPU6050);
}	

/**-----------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *-----------------------------------------------------------------------------------------------------------------*/
void MPU6050_GetAccelConfig(COM_Handle * MPU6050, MPU6050_AccelConfigReg * accelConfig)
{
	COM_I2C_DATA(MPU6050).dataSize = 1;
	COM_I2C_DATA(MPU6050).memAddress = MPU6050_ACCEL_CONFIG_ADDR;
	COM_I2C_DATA(MPU6050).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	COM_RegisterGetter(MPU6050);
											
	accelConfig->BYTE = COM_I2C_DATA(MPU6050).data[0];
}

/**-----------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *-----------------------------------------------------------------------------------------------------------------*/
void MPU6050_SetGyroConfig(COM_Handle * MPU6050, const MPU6050_GyroConfigReg * gyroConfig)
{
	uint8_t sendVal = gyroConfig->BYTE;
	
	COM_I2C_DATA(MPU6050).data[0] = sendVal;
	COM_I2C_DATA(MPU6050).dataSize = 1;
	COM_I2C_DATA(MPU6050).memAddress = MPU6050_GYRO_CONFIG_ADDR;
	COM_I2C_DATA(MPU6050).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	COM_RegisterSetter(MPU6050);
}

/**-----------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *-----------------------------------------------------------------------------------------------------------------*/
void MPU6050_GetGyroConfig(COM_Handle * MPU6050, MPU6050_GyroConfigReg * gyroConfig)
{
	COM_I2C_DATA(MPU6050).dataSize = 1;
	COM_I2C_DATA(MPU6050).memAddress = MPU6050_GYRO_CONFIG_ADDR;
	COM_I2C_DATA(MPU6050).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	COM_RegisterGetter(MPU6050);
											
	gyroConfig->BYTE = COM_I2C_DATA(MPU6050).data[0];
}

/**-----------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *-----------------------------------------------------------------------------------------------------------------*/
void MPU6050_GetRawAccelDatas(COM_Handle * MPU6050, MPU6050_RawAccelDatas * rawDatas)
{
	COM_I2C_DATA(MPU6050).dataSize = 6;
	COM_I2C_DATA(MPU6050).memAddress = MPU6050_START_OF_ACCEL_DATA_ADDR;
	COM_I2C_DATA(MPU6050).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	COM_RegisterGetter(MPU6050);
	
	int16_t xData  = (int16_t)(COM_I2C_DATA(MPU6050).data[0]<<8 | COM_I2C_DATA(MPU6050).data[1]);
	int16_t yData  = (int16_t)(COM_I2C_DATA(MPU6050).data[2]<<8 | COM_I2C_DATA(MPU6050).data[3]);
	int16_t zData  = (int16_t)(COM_I2C_DATA(MPU6050).data[4]<<8 | COM_I2C_DATA(MPU6050).data[5]);
	
	rawDatas->rawXData = xData*MPU6050_ACCEL_DATA_SCALE_FACTOR;
	rawDatas->rawYData = yData*MPU6050_ACCEL_DATA_SCALE_FACTOR;
	rawDatas->rawZData = zData*MPU6050_ACCEL_DATA_SCALE_FACTOR;
}

/**-----------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *-----------------------------------------------------------------------------------------------------------------*/
void MPU6050_GetRawGyroDatas(COM_Handle * MPU6050, MPU6050_RawGyroDatas * rawDatas)
{
	COM_I2C_DATA(MPU6050).dataSize = 6;
	COM_I2C_DATA(MPU6050).memAddress = MPU6050_START_OF_GYRO_DATA_ADDR;
	COM_I2C_DATA(MPU6050).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	COM_RegisterGetter(MPU6050);
	
	int16_t xData  = (int16_t)(COM_I2C_DATA(MPU6050).data[0]<<8 | COM_I2C_DATA(MPU6050).data[1]);
	int16_t yData  = (int16_t)(COM_I2C_DATA(MPU6050).data[2]<<8 | COM_I2C_DATA(MPU6050).data[3]);
	int16_t zData  = (int16_t)(COM_I2C_DATA(MPU6050).data[4]<<8 | COM_I2C_DATA(MPU6050).data[5]);
	
	rawDatas->rawXData = xData*MPU6050_GYRO_DATA_SCALE_FACTOR;
	rawDatas->rawYData = yData*MPU6050_GYRO_DATA_SCALE_FACTOR;
	rawDatas->rawZData = zData*MPU6050_GYRO_DATA_SCALE_FACTOR;
}

/**------------------------------------------------------------------------------
  * @brief  			
	* @note					
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *------------------------------------------------------------------------------*/
void MPU6050_GetAccelOffsetValues(COM_Handle * MPU6050, IMU_AxisDatas * biasDatas)
{
	memset(biasDatas, 0x00, sizeof(IMU_AxisDatas));
	MPU6050_RawAccelDatas tempDatas;
	
	for(uint8_t i=0; i<MPU6050_ACCEL_BIAS_CALC_ITERATION; i++)
	{
		MPU6050_GetRawAccelDatas(MPU6050, &tempDatas);
		
		biasDatas->xData += tempDatas.rawXData;
		biasDatas->yData += tempDatas.rawYData;
		biasDatas->zData += tempDatas.rawZData;
	}
	
	biasDatas->xData /=  (float)MPU6050_ACCEL_BIAS_CALC_ITERATION;
	biasDatas->yData /=  (float)MPU6050_ACCEL_BIAS_CALC_ITERATION;
	biasDatas->zData /=  (float)MPU6050_ACCEL_BIAS_CALC_ITERATION;
	
	biasDatas->zData -= MPU6050_GRAVITY_ACCELERATION;
}

/**------------------------------------------------------------------------------
  * @brief  			
	* @note					
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *------------------------------------------------------------------------------*/
void MPU6050_GetGyroOffsetValues(COM_Handle * MPU6050, IMU_AxisDatas * biasDatas)
{
	memset(biasDatas, 0x00, sizeof(IMU_AxisDatas));
	MPU6050_RawGyroDatas tempDatas;
	
	for(uint8_t i=0; i<MPU6050_GYRO_BIAS_CALC_ITERATION; i++)
	{
		MPU6050_GetRawGyroDatas(MPU6050, &tempDatas);
		
		biasDatas->xData += tempDatas.rawXData;
		biasDatas->yData += tempDatas.rawYData;
		biasDatas->zData += tempDatas.rawZData;
	}
	
	biasDatas->xData /=  (float)MPU6050_GYRO_BIAS_CALC_ITERATION;
	biasDatas->yData /=  (float)MPU6050_GYRO_BIAS_CALC_ITERATION;
	biasDatas->zData /=  (float)MPU6050_GYRO_BIAS_CALC_ITERATION;
}

/**------------------------------------------------------------------------------
  * @brief  			
	* @note					
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *------------------------------------------------------------------------------*/
void MPU6050_GetAccelAngle(COM_Handle * MPU6050, const IMU_AxisDatas * axisBias, IMU_AxisAngles * axisAngle)
{
	IMU_AxisDatas 					axisData = {0};
	MPU6050_RawAccelDatas 	mpu6050TempAccelAxis={0};
	
	
	MPU6050_GetRawAccelDatas(MPU6050, &mpu6050TempAccelAxis);
	
	axisData.xData = mpu6050TempAccelAxis.rawXData;
	axisData.yData = mpu6050TempAccelAxis.rawYData;
	axisData.zData = mpu6050TempAccelAxis.rawZData;
	
	IMU_RemoveBias(&axisData, axisBias);
	
	IMU_GetAngleFromAccelerometer(&axisData, axisAngle);
}

/**------------------------------------------------------------------------------
  * @brief  			
	* @note					
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *------------------------------------------------------------------------------*/
void MPU6050_GetGyroAngle(COM_Handle * MPU6050, const IMU_AxisDatas * axisBias, IMU_AxisAngles * currAxisAngle, IMU_AxisAngles * prevAxisAngle)
{
	IMU_AxisDatas 				axisData = {0};
	MPU6050_RawGyroDatas 	mpu6050TempGyroData={0};
	
	MPU6050_GetRawGyroDatas(MPU6050, &mpu6050TempGyroData);
	
	axisData.xData = mpu6050TempGyroData.rawXData;
	axisData.yData = mpu6050TempGyroData.rawYData;
	axisData.zData = mpu6050TempGyroData.rawZData;
	
	IMU_RemoveBias(&axisData, axisBias);
	
	IMU_GetAngleFromGyro(&axisData, currAxisAngle, prevAxisAngle, Ts_PERIOD);
}
