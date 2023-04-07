/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "imu.h"
#include "math.h"
#include "calc.h"
#include "adxl345.h"
#include "itg3205.h"
#include "hmc5883l.h"
#include "mpu6050.h"

/* Private define ------------------------------------------------------------*/
#define MAGNETO_BIAS_CALC_ITERATION		10

#define IMU_PERIODIC_READ_FLAG				(1<<0)

/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static void IMU_SensorHandler (void *arg);				
static void IMU_GetDatasCallback (void *arg);

/* Private variables ---------------------------------------------------------*/
static osThreadId_t	TID_IMU;
static osTimerId_t 	TIM_GetDatas;

const osThreadAttr_t IMUThreadAttr =
{
	.name = "IMU_Thread",
	.priority = osPriorityAboveNormal2
};

const osTimerAttr_t GetDatasTimerAttr = 
{
	.name = "Timer_Get_Datas"
};

static char uartBuff[300]={0};



/* Exported variables --------------------------------------------------------*/
COM_Handle 	ADXL345_ComHandle;
COM_Handle 	ITG3205_ComHandle;
COM_Handle 	HMC5883L_ComHandle;
COM_Handle	MPU6050_ComHandle;

IMU_AxisDatas 	adxl345AccelBias;
IMU_AxisDatas 	mpu6050AccelBias;
IMU_AxisDatas		itg3205GyroBias;
IMU_AxisDatas 	mpu6050GyroBias;

IMU_AxisAngles 	adxl345RawAccelAngle;
IMU_AxisAngles 	mpu6050RawAccelAngle;
IMU_AxisAngles	itg3205RawGyroAngle;
IMU_AxisAngles 	mpu6050RawGyroAngle;
IMU_AxisAngles 	hmc5883RawMagnetoAngle;

IMU_AxisAngles	itg3205PrevGyroAngle;
IMU_AxisAngles 	mpu6050PrevGyroAngle;

IMU_AxisAngles eulerAngles;

extern I2C_HandleTypeDef 		hi2c1;
extern UART_HandleTypeDef 	huart2;

/* Exported functions --------------------------------------------------------*/

/**------------------------------------------------------------------------------
  * @brief  			
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *------------------------------------------------------------------------------*/
void IMU_Init(void)
{
	/* ADXL345 Init */
	ADXL345_ComHandle.comTypeChannel = &(comTypeAndChannels[COM_TYPE_I2C][COM_I2C1_CHANNEL_INDEX]);
	ADXL345_ComHandle.comData.i2c.comHandle = &hi2c1;
	ADXL345_ComHandle.comData.i2c.devAddress = ADXL345_I2C_DEV_ADDR_GND;
	ADXL345_InitSensor(&ADXL345_ComHandle);
															
	/* ITG3205 Init */			
	ITG3205_ComHandle.comTypeChannel = &(comTypeAndChannels[COM_TYPE_I2C][COM_I2C1_CHANNEL_INDEX]);
	ITG3205_ComHandle.comData.i2c.comHandle = &hi2c1;
	ITG3205_ComHandle.comData.i2c.devAddress = ITG3205_I2C_DEV_ADDR_GND;
	ITG3205_InitSensor(&ITG3205_ComHandle);
															
//	/* HMC5883L Init  */			
//	HMC5883L_ComHandle.comTypeChannel = &(comTypeAndChannels[COM_TYPE_I2C][COM_I2C1_CHANNEL_INDEX]);
//	HMC5883L_ComHandle.comData.i2c.comHandle = &hi2c1;
//	HMC5883L_ComHandle.comData.i2c.devAddress = HMC5883L_I2C_DEV_ADDR;														
//	HMC5883L_InitSensor(&HMC5883L_ComHandle);
	
//	/* MPU6050 Init */
//	MPU6050_ComHandle.comTypeChannel = &(comTypeAndChannels[COM_TYPE_I2C][COM_I2C1_CHANNEL_INDEX]);
//	MPU6050_ComHandle.comData.i2c.comHandle = &hi2c1;
//	MPU6050_ComHandle.comData.i2c.devAddress = MPU6050_I2C_DEV_ADDR;		
//	MPU6050_InitSensor(&MPU6050_ComHandle);
	
	/* Offset values are read at first run */
	ADXL345_GetAccelOffsetValues(&ADXL345_ComHandle, &adxl345AccelBias);
	ITG3205_GetGyroOffsetValues(&ITG3205_ComHandle, &itg3205GyroBias);
//	MPU6050_GetAccelOffsetValues(&MPU6050_ComHandle, &mpu6050AccelBias);
//	MPU6050_GetGyroOffsetValues(&MPU6050_ComHandle, &mpu6050GyroBias);
	
	TID_IMU 			= osThreadNew(IMU_SensorHandler, NULL, &IMUThreadAttr);
	TIM_GetDatas 	= osTimerNew(IMU_GetDatasCallback, osTimerPeriodic, NULL, &GetDatasTimerAttr);
	
	osTimerStart(TIM_GetDatas, Ts_PERIOD*SEC_TO_MS);
}

/**------------------------------------------------------------------------------
  * @brief  			
	* @note					
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *------------------------------------------------------------------------------*/
void IMU_RemoveBias(IMU_AxisDatas * axisData, const IMU_AxisDatas * axisBias)
{
	/* Remove offset datas */
	axisData->xData -= axisBias->xData;
	axisData->yData -= axisBias->yData;
	axisData->zData -= axisBias->zData;
}

/**------------------------------------------------------------------------------
  * @brief  			
	* @note					
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *------------------------------------------------------------------------------*/
void IMU_ConvertRadianToAngle(IMU_AxisAngles * axisAngles)
{
	axisAngles->xAngle = RADIAN_TO_DEGREE(axisAngles->xAngle);
	axisAngles->yAngle = RADIAN_TO_DEGREE(axisAngles->yAngle);
	axisAngles->zAngle = RADIAN_TO_DEGREE(axisAngles->zAngle);
}


/**------------------------------------------------------------------------------
  * @brief  			Gets roll and pitch datas from gravitional acceleration.
	*	@param[IN]  	rawDatas
	*	@param[OUT]		axisAngles
  * @retval 		
  *------------------------------------------------------------------------------*/
void IMU_GetAngleFromAccelerometer(IMU_AxisDatas * rawDatas, IMU_AxisAngles * axisAngles)
{
	axisAngles->xAngle = atan(rawDatas->yData / sqrt(pow(rawDatas->xData, 2) + pow(rawDatas->zData, 2)));
	axisAngles->yAngle = atan(-1 * rawDatas->xData / sqrt(pow(rawDatas->yData, 2) + pow(rawDatas->zData, 2)));
}

/**------------------------------------------------------------------------------
  * @brief  			Gyroscope meausures angular velocity.
	*								We integrate it to find angular position.
	* @note					Giris sinyalinde küçük hata bile olsa çikista çok feci beklenmedik hatalara sebep oluyor. Integrali alinca bunu farkettik.
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *------------------------------------------------------------------------------*/
void IMU_GetAngleFromGyro(IMU_AxisDatas * rawDatas, IMU_AxisAngles * currAngles, IMU_AxisAngles * prevSumAngles, float periode)
{
	currAngles->xAngle = DEGREE_TO_RADIAN( Calc_GetDiscreteIntegral( &(rawDatas->xData), &(prevSumAngles->xAngle), periode) );
	currAngles->yAngle = DEGREE_TO_RADIAN( Calc_GetDiscreteIntegral( &(rawDatas->yData), &(prevSumAngles->yAngle), periode) );
	currAngles->zAngle = DEGREE_TO_RADIAN( Calc_GetDiscreteIntegral( &(rawDatas->zData), &(prevSumAngles->zAngle), periode) );
}


/* Private functions ---------------------------------------------------------*/
																			
/**------------------------------------------------------------------------------
  * @brief  			
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *------------------------------------------------------------------------------*/
static void IMU_SensorHandler (void *arg)
{
	while(true)
	{
		osThreadFlagsWait(IMU_PERIODIC_READ_FLAG, osFlagsWaitAll, osWaitForever);
	
		//TODO:Sensorler ile ilgili FAULT STATE'i ekle
		ADXL345_GetAngle(&ADXL345_ComHandle, &adxl345AccelBias, &adxl345RawAccelAngle);
		ITG3205_GetAngle(&ITG3205_ComHandle , &itg3205GyroBias, &itg3205RawGyroAngle, &itg3205PrevGyroAngle);
//		MPU6050_GetAccelAngle(&MPU6050_ComHandle, &mpu6050AccelBias, &mpu6050RawAccelAngle);
//		MPU6050_GetGyroAngle(&MPU6050_ComHandle, &mpu6050GyroBias, &mpu6050RawGyroAngle, &mpu6050PrevGyroAngle);		
		
		//TODO:Only for debug
		IMU_ConvertRadianToAngle(&adxl345RawAccelAngle);
		IMU_ConvertRadianToAngle(&itg3205RawGyroAngle);
//		IMU_ConvertRadianToAngle(&mpu6050RawAccelAngle);
//		IMU_ConvertRadianToAngle(&mpu6050RawGyroAngle);
		
		/* Complementary Filter */
		//eulerAngles.xAngle = 0.96*gyroAngles.xAngle + 0.04*accelAngles.xAngle;		/* ROLL 	*/
		//eulerAngles.yAngle = 0.96*gyroAngles.yAngle + 0.04*accelAngles.yAngle;		/* PITCH 	*/
	}
}	
																			
/**------------------------------------------------------------------------------
  * @brief  			
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *------------------------------------------------------------------------------*/
static void IMU_GetDatasCallback (void *arg)
{
	osThreadFlagsSet(TID_IMU, IMU_PERIODIC_READ_FLAG);
}

