/* Includes ------------------------------------------------------------------*/
#include "imu.h"
#include "math.h"
#include "calc.h"
#include <stdio.h>
#include <string.h>
#include "com_interface.h"

/* Private define ------------------------------------------------------------*/
#define ACCEL_BIAS_CALC_ITERATION			10
#define GYRO_BIAS_CALC_ITERATION			10
#define MAGNETO_BIAS_CALC_ITERATION		10

#define GRAVITY_ACCELERATION					1			/* g */

#define IMU_PERIODIC_READ_FLAG				(1<<0)

/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static osThreadId_t	TID_IMU;
static osTimerId_t 	TIM_GetDatas;

const osThreadAttr_t IMUThreadAttr =
{
	.name = "IMU_Thread",
	.priority = osPriorityNormal
};

const osTimerAttr_t GetDatasTimerAttr = 
{
	.name = "Timer_Get_Datas"
};

static char uartBuff[300]={0};

static ADXL345_RawDatas 			accelRawDatas;
static ITG3205_RawDatas 			gyroRawDatas;
static HMC5883L_RawDatas			magnetoRawDatas;	
static MPU6050_RawAccelDatas	mpuAccelRawDatas;

static ADXL345_RawDatas 	accelBiasDatas;
static ITG3205_RawDatas 	gyroBiasDatas;
static HMC5883L_RawDatas	magnetoBiasDatas;

static AxisAngles accelAngles;
static AxisAngles gyroAngles;
static AxisAngles gyroPrevAngles;

static AxisAngles eulerAngles;

/* Exported variables --------------------------------------------------------*/
COM_Handle 	ADXL345_ComHandle;
COM_Handle 	ITG3205_ComHandle;
COM_Handle 	HMC5883L_ComHandle;
COM_Handle	MPU6050_ComHandle;

extern I2C_HandleTypeDef 		hi2c1;
extern UART_HandleTypeDef 	huart2;


/* Private function prototypes -----------------------------------------------*/
static void IMU_SensorHandler (void *arg);				
static void IMU_GetDatasCallback (void *arg);

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
	
		ADXL345_GetRawDatas(&ADXL345_ComHandle, &accelRawDatas);
		ITG3205_GetRawDatas(&ITG3205_ComHandle, &gyroRawDatas);
		//HMC5883L_GetRawDatas(&(comInputs[COM_INPUT_TYPE_I2C][HMC5883L_I2C_CHANNEL_NO]), &magnetoRawDatas);
		
		/* Remove offset datas */
		accelRawDatas.rawXData -= accelBiasDatas.rawXData;
		accelRawDatas.rawYData -= accelBiasDatas.rawYData;
		accelRawDatas.rawZData -= accelBiasDatas.rawZData;

		gyroRawDatas.rawXData -= gyroBiasDatas.rawXData;
		gyroRawDatas.rawYData -= gyroBiasDatas.rawYData;
		gyroRawDatas.rawZData -= gyroBiasDatas.rawZData;
		
		//magnetoRawDatas.rawXData -= magnetoBiasDatas.rawXData;
		//magnetoRawDatas.rawYData -= magnetoBiasDatas.rawYData;
		//magnetoRawDatas.rawZData -= magnetoBiasDatas.rawZData;
		
		IMU_GetAngleFromAccelerometer(&accelRawDatas, &accelAngles);
		IMU_GetAngleFromGyro(&gyroRawDatas, &gyroAngles, &gyroPrevAngles, Ts_PERIOD);
		
		//TODO:Only for debug
		accelAngles.xAngle = RADIAN_TO_DEGREE(accelAngles.xAngle);
		accelAngles.yAngle = RADIAN_TO_DEGREE(accelAngles.yAngle);
		
		gyroAngles.xAngle = RADIAN_TO_DEGREE(gyroAngles.xAngle);
		gyroAngles.yAngle = RADIAN_TO_DEGREE(gyroAngles.yAngle);
		
		/* Complementary Filter */
		eulerAngles.xAngle = 0.96*gyroAngles.xAngle + 0.04*accelAngles.xAngle;		/* ROLL 	*/
		eulerAngles.yAngle = 0.96*gyroAngles.yAngle + 0.04*accelAngles.yAngle;		/* PITCH 	*/
		
		//sprintf(uartBuff, "X : %5.1f\nY : %5.1f\nZ : %5.1f\n\n", accelRawDatas.rawXData, accelRawDatas.rawYData, accelRawDatas.rawZData);
		//HAL_UART_Transmit_IT(&huart2, (uint8_t*)uartBuff, 35);
	
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
															
	/* HMC5883L Init  */			
	HMC5883L_ComHandle.comTypeChannel = &(comTypeAndChannels[COM_TYPE_I2C][COM_I2C1_CHANNEL_INDEX]);
	HMC5883L_ComHandle.comData.i2c.comHandle = &hi2c1;
	HMC5883L_ComHandle.comData.i2c.devAddress = HMC5883L_I2C_DEV_ADDR;														
	//HMC5883L_InitSensor(&HMC5883L_ComHandle);
	
//	/* MPU6050 Init */
//	MPU6050_ComHandle.comTypeChannel = &(comTypeAndChannels[COM_TYPE_I2C][COM_I2C1_CHANNEL_INDEX]);
//	MPU6050_ComHandle.comData.i2c.comHandle = &hi2c1;
//	MPU6050_ComHandle.comData.i2c.devAddress = MPU6050_I2C_DEV_ADDR;		
//	MPU6050_InitSensor(&MPU6050_ComHandle);
	
	/* Offset values are read at first run */
	IMU_GetAccelOffsetValues(&ADXL345_ComHandle, &accelBiasDatas);
	IMU_GetGyroOffsetValues(&ITG3205_ComHandle, &gyroBiasDatas);
	
	TID_IMU 			= osThreadNew(IMU_SensorHandler, NULL, &IMUThreadAttr);
	TIM_GetDatas 	= osTimerNew(IMU_GetDatasCallback, osTimerPeriodic, NULL, &GetDatasTimerAttr);
	
	osTimerStart(TIM_GetDatas, Ts_PERIOD*SEC_TO_MS);
}

/**------------------------------------------------------------------------------
  * @brief  			Gets roll and pitch datas from gravitional acceleration.
	*	@param[IN]  	rawDatas
	*	@param[OUT]		axisAngles
  * @retval 		
  *------------------------------------------------------------------------------*/
void IMU_GetAngleFromAccelerometer(ADXL345_RawDatas * rawDatas, AxisAngles * axisAngles)
{
	axisAngles->xAngle = atan(rawDatas->rawYData / sqrt(pow(rawDatas->rawXData, 2) + pow(rawDatas->rawZData, 2)));
	axisAngles->yAngle = atan(-1 * rawDatas->rawXData / sqrt(pow(rawDatas->rawYData, 2) + pow(rawDatas->rawZData, 2)));
}

/**------------------------------------------------------------------------------
  * @brief  			Gyroscope meausures angular velocity.
	*								We integrate it to find angular position.
	* @note					Giris sinyalinde k���k hata bile olsa �ikista �ok feci beklenmedik hatalara sebep oluyor. Integrali alinca bunu farkettik.
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *------------------------------------------------------------------------------*/
void IMU_GetAngleFromGyro(ITG3205_RawDatas * rawDatas, AxisAngles * currAxisAngles, AxisAngles * prevAxisAngles, float periode)
{
	currAxisAngles->xAngle = DEGREE_TO_RADIAN( Calc_GetDiscreteIntegral( &(rawDatas->rawXData), &(prevAxisAngles->xAngle), periode) );
	currAxisAngles->yAngle = DEGREE_TO_RADIAN( Calc_GetDiscreteIntegral( &(rawDatas->rawYData), &(prevAxisAngles->yAngle), periode) );
	currAxisAngles->zAngle = DEGREE_TO_RADIAN( Calc_GetDiscreteIntegral( &(rawDatas->rawZData), &(prevAxisAngles->zAngle), periode) );
}

/**------------------------------------------------------------------------------
  * @brief  			
	* @note					
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *------------------------------------------------------------------------------*/
void IMU_GetAccelOffsetValues(COM_Handle * ADXL345, ADXL345_RawDatas * biasDatas)
{
	memset(biasDatas, 0x00, sizeof(ADXL345_RawDatas));
	ADXL345_RawDatas tempDatas;
	
	for(uint8_t i=0; i<ACCEL_BIAS_CALC_ITERATION; i++)
	{
		
		ADXL345_GetRawDatas(ADXL345, &tempDatas);
		
		biasDatas->rawXData += tempDatas.rawXData;
		biasDatas->rawYData += tempDatas.rawYData;
		biasDatas->rawZData += tempDatas.rawZData;
	}
	
	biasDatas->rawXData /=  (float)ACCEL_BIAS_CALC_ITERATION;
	biasDatas->rawYData /=  (float)ACCEL_BIAS_CALC_ITERATION;
	biasDatas->rawZData /=  (float)ACCEL_BIAS_CALC_ITERATION;
	
	biasDatas->rawZData -= GRAVITY_ACCELERATION;
}

/**------------------------------------------------------------------------------
  * @brief  			
	* @note					
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *------------------------------------------------------------------------------*/
void IMU_GetGyroOffsetValues(COM_Handle * ITG3205, ITG3205_RawDatas * biasDatas)
{
	memset(biasDatas, 0x00, sizeof(ITG3205_RawDatas));
	ITG3205_RawDatas tempDatas;
	
	for(uint8_t i=0; i<GYRO_BIAS_CALC_ITERATION; i++)
	{
		ITG3205_GetRawDatas(ITG3205, &tempDatas);
		
		biasDatas->rawXData += tempDatas.rawXData;
		biasDatas->rawYData += tempDatas.rawYData;
		biasDatas->rawZData += tempDatas.rawZData;
	}
	
	biasDatas->rawXData /=  (float)GYRO_BIAS_CALC_ITERATION;
	biasDatas->rawYData /=  (float)GYRO_BIAS_CALC_ITERATION;
	biasDatas->rawZData /=  (float)GYRO_BIAS_CALC_ITERATION;
}

/**------------------------------------------------------------------------------
  * @brief  			
	* @note					
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *------------------------------------------------------------------------------*/
void IMU_GetMagnetoOffsetValues(COM_Handle * HMC5883L, HMC5883L_RawDatas * biasDatas)
{

}