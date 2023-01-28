/* Includes ------------------------------------------------------------------*/
#include "imu.h"
#include <math.h>
#include "calc.h"
#include "stdio.h"

/* Private define ------------------------------------------------------------*/
#define GYRO_BIAS_CALC_ITERATION		25


#define GY85_PERIODIC_READ_FLAG				(1<<0)

/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static osThreadId_t	TID_GY85;
static osTimerId_t 	TIM_GetDatas;

const osThreadAttr_t GY85ThreadAttr =
{
	.name = "GY85_Thread",
	.priority = osPriorityNormal
};

const osTimerAttr_t GetDatasTimerAttr = 
{
	.name = "Timer_Get_Datas"
};

static char uartBuff[300]={0};

static ADXL345_RawDatas 	accelRawDatas;
static ITG3205_RawDatas 	gyroRawDatas;
static HMC5883L_RawDatas	magnetoRawDatas;	

static ITG3205_RawDatas 	gyroBiasDatas;


static AxisAngles accelAngles;
static AxisAngles gyroAngles;
static AxisAngles gyroPrevAngles;

static AxisAngles eulerAngles;

/* Exported variables --------------------------------------------------------*/
extern I2C_HandleTypeDef 		hi2c1;
extern UART_HandleTypeDef 	huart2;


/* Private function prototypes -----------------------------------------------*/
static void GY85_SensorHandler (void *arg);				
static void GY85_GetDatasCallback (void *arg);

/* Private functions ---------------------------------------------------------*/
																			
/**------------------------------------------------------------------------------
  * @brief  			
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *------------------------------------------------------------------------------*/
static void GY85_SensorHandler (void *arg)
{
	while(true)
	{
		osThreadFlagsWait(GY85_PERIODIC_READ_FLAG, osFlagsWaitAll, osWaitForever);
	
		ADXL345_GetRawDatas(&(comInputs[COM_INPUT_TYPE_I2C][ADXL345_I2C_CHANNEL_NO]), 	&accelRawDatas);
		//ITG3205_GetRawDatas(&(comInputs[COM_INPUT_TYPE_I2C][ITG3205_I2C_CHANNEL_NO]), 	&gyroRawDatas);
		//HMC5883L_GetRawDatas(&(comInputs[COM_INPUT_TYPE_I2C][HMC5883L_I2C_CHANNEL_NO]), &magnetoRawDatas);
		
		/* Remove offset datas */
		//gyroRawDatas.rawXData -= gyroBiasDatas.rawXData;
		//gyroRawDatas.rawYData -= gyroBiasDatas.rawYData;
		//gyroRawDatas.rawZData -= gyroBiasDatas.rawZData;
		
		GY85_GetAngleFromAccelerometer(&accelRawDatas, &accelAngles);
		//GY85_GetAngleFromGyro(&gyroRawDatas, &gyroAngles, &gyroPrevAngles, Ts_PERIOD);
		
		accelAngles.xAngle = RADIAN_TO_DEGREE(accelAngles.xAngle);
		accelAngles.yAngle = RADIAN_TO_DEGREE(accelAngles.yAngle);
		
		//gyroAngles.xAngle = RADIAN_TO_DEGREE(gyroAngles.xAngle);
		//gyroAngles.yAngle = RADIAN_TO_DEGREE(gyroAngles.yAngle);
		
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
static void GY85_GetDatasCallback (void *arg)
{
	osThreadFlagsSet(TID_GY85, GY85_PERIODIC_READ_FLAG);
}

/* Exported functions --------------------------------------------------------*/

/**------------------------------------------------------------------------------
  * @brief  			
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *------------------------------------------------------------------------------*/
void GY85_Init(void)
{
	
	ComInput_Handle tempSensor;
	
	/* ADXL345 Init */
	tempSensor.typeAndChannel = (ComInput_TypeAndChannel){.comInputType = COM_INPUT_TYPE_I2C, .channelNo = ADXL345_I2C_CHANNEL_NO};
	tempSensor.comInputData.i2c.comHandle = &hi2c1;
	tempSensor.comInputData.i2c.devAddress = ADXL345_I2C_DEV_ADDR_GND;
	
	ADXL345_InitSensor(&tempSensor);
															
	/* ITG3205 Init */			
	tempSensor.typeAndChannel = (ComInput_TypeAndChannel){.comInputType = COM_INPUT_TYPE_I2C, .channelNo = ITG3205_I2C_CHANNEL_NO};
	tempSensor.comInputData.i2c.comHandle = &hi2c1;
	tempSensor.comInputData.i2c.devAddress = ITG3205_I2C_DEV_ADDR_GND;

	//ITG3205_InitSensor(&tempSensor);
															
	/* HMC5883L Init  */			
	tempSensor.typeAndChannel = (ComInput_TypeAndChannel){.comInputType = COM_INPUT_TYPE_I2C, .channelNo = HMC5883L_I2C_CHANNEL_NO};
	tempSensor.comInputData.i2c.comHandle = &hi2c1;
	tempSensor.comInputData.i2c.devAddress = HMC5883L_I2C_DEV_ADDR;														
	
	//HMC5883L_InitSensor(&HMC5883L);
	
	/* Offset values read at first run */
	//ITG3205_RawDatas 	gyroTempBiasDatas;
	//
	//for(uint8_t i=0; i<GYRO_BIAS_CALC_ITERATION; i++)
	//{
	//	
	//	ITG3205_GetRawDatas(&ITG3205, 	&gyroTempBiasDatas);
	//	
	//	gyroBiasDatas.rawXData += gyroTempBiasDatas.rawXData;
	//	gyroBiasDatas.rawYData += gyroTempBiasDatas.rawYData;
	//	gyroBiasDatas.rawZData += gyroTempBiasDatas.rawZData;
	//}
	//
	//gyroBiasDatas.rawXData /=  (float)GYRO_BIAS_CALC_ITERATION;
	//gyroBiasDatas.rawYData /=  (float)GYRO_BIAS_CALC_ITERATION;
	//gyroBiasDatas.rawZData /=  (float)GYRO_BIAS_CALC_ITERATION;
	
	TID_GY85 = osThreadNew(GY85_SensorHandler, NULL, &GY85ThreadAttr);
	
	if(TID_GY85 == NULL)
	{
		//Thread couldn't be created.
	}
	
	TIM_GetDatas = osTimerNew(GY85_GetDatasCallback, osTimerPeriodic, NULL, &GetDatasTimerAttr);
	
	if(TIM_GetDatas == NULL)
	{
		//Timer couldn't be created.
	}
	
	
	
	osTimerStart(TIM_GetDatas, Ts_PERIOD*SEC_TO_MS);
}

/**------------------------------------------------------------------------------
  * @brief  			Gets roll and pitch datas from gravitional acceleration.
	*	@param[IN]  	rawDatas
	*	@param[OUT]		axisAngles
  * @retval 		
  *------------------------------------------------------------------------------*/
void GY85_GetAngleFromAccelerometer(ADXL345_RawDatas * rawDatas, AxisAngles * axisAngles)
{
	axisAngles->xAngle = (atan(rawDatas->rawYData / sqrt(pow(rawDatas->rawXData, 2) + pow(rawDatas->rawZData, 2))) * 180 / PI);
	axisAngles->yAngle = (atan(-1 * rawDatas->rawXData / sqrt(pow(rawDatas->rawYData, 2) + pow(rawDatas->rawZData, 2))) * 180 / PI);
}

/**------------------------------------------------------------------------------
  * @brief  			Gyroscope meausures angular velocity.
	*								We integrate it to find angular position.
	* @note					Giris sinyalinde küçük hata bile olsa çikista çok feci beklenmedik hatalara sebep oluyor. Integrali alinca bunu farkettik.
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *------------------------------------------------------------------------------*/
void GY85_GetAngleFromGyro(ITG3205_RawDatas * rawDatas, AxisAngles * currAxisAngles, AxisAngles * prevAxisAngles, float periode)
{
	currAxisAngles->xAngle = DEGREE_TO_RADIAN( Calc_GetDiscreteIntegral( &(rawDatas->rawXData), &(prevAxisAngles->xAngle), periode) );
	currAxisAngles->yAngle = DEGREE_TO_RADIAN( Calc_GetDiscreteIntegral( &(rawDatas->rawYData), &(prevAxisAngles->yAngle), periode) );
	currAxisAngles->zAngle = DEGREE_TO_RADIAN( Calc_GetDiscreteIntegral( &(rawDatas->rawZData), &(prevAxisAngles->zAngle), periode) );
}