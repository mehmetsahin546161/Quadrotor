/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "defines.h"
#include "ahrs.h"
#include "calc.h"
#include "mpu9250.h"
#include "cmsis_os2.h"
#include "bmx160.h"

/* Private define ------------------------------------------------------------*/
#define IMU_PERIODIC_READ_FLAG				(1<<0)
#define MAGNETIC_DECLINATION					(5.53f) 			/* In Istanbul */

/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static void AHRS_IMU_ReadingThread(void *arg);				
static void AHRS_IMU_ReadingTimCallback(void *arg);

/* Private variables ---------------------------------------------------------*/
static osThreadId_t	TID_IMU_Reading;
static osTimerId_t 	TIM_IMU_Reading;
static osEventFlagsId_t EVT_IMU;

const osThreadAttr_t IMU_ReadingThreadAttr =
{
	.name = "IMU_Reading_Thread",
	.priority = osPriorityNormal1
};

const osTimerAttr_t IMU_ReadingTimerAttr = 
{
	.name = "IMU_Reading_Timer"
};

const osEventFlagsAttr_t IMU_ThreadFlagAttr = 
{
	.name = "IMU_Thread_Flag"
};

/* Exported variables --------------------------------------------------------*/
extern I2C_HandleTypeDef 		hi2c1;
AHRS_Handle	AHRS;
BMX160_Handle BMX160;

/* Exported functions --------------------------------------------------------*/

/**------------------------------------------------------------------------------
  * @brief  			
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *------------------------------------------------------------------------------*/
void AHRS_Init(void)
{
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_SET);
	
	/* Init IMU Sensors */
	BMX160_InitSensor(&BMX160);
	
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);
	
	EVT_IMU = osEventFlagsNew(&IMU_ThreadFlagAttr);
	TIM_IMU_Reading = osTimerNew(AHRS_IMU_ReadingTimCallback, osTimerPeriodic, NULL, &IMU_ReadingTimerAttr);
	TID_IMU_Reading = osThreadNew(AHRS_IMU_ReadingThread, NULL, &IMU_ReadingThreadAttr);
	
	osTimerStart(TIM_IMU_Reading, SEC_TO_MS(IMU_READING_PERIODE));
}

/**------------------------------------------------------------------------------
  * @brief  			
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *------------------------------------------------------------------------------*/
void AHRS_GetMadgwickQuaternion(const AHRS_AxisData * accelData, const AHRS_AxisData * gyroData, const AHRS_AxisData * magnetoData, AHRS_Quaternions * quaternions)
{
	/* Short name local variable for readability */
	float q1 = quaternions->q1;
	float q2 = quaternions->q2;
	float q3 = quaternions->q3;
	float q4 = quaternions->q4;
	
	float ax = accelData->xData;
	float ay = accelData->yData;
	float az = accelData->zData;
	
	float gx = gyroData->xData;
	float gy = gyroData->yData;
	float gz = gyroData->zData;
	
	float mx = magnetoData->xData;
	float my = magnetoData->yData;
	float mz = magnetoData->zData;
	
	float beta = 0.604;//sqrt(3.0f / 4.0f) * gyroMeasError; 
	
	float norm;
	float hx, hy, _2bx, _2bz;
	float s1, s2, s3, s4;
	float qDot1, qDot2, qDot3, qDot4;
	
	/* Auxiliary variables to avoid repeated arithmetic */
	float _2q1mx;
	float _2q1my;
	float _2q1mz;
	float _2q2mx;
	float _4bx;
	float _4bz;
	float _2q1 = 2.0f * q1;
	float _2q2 = 2.0f * q2;
	float _2q3 = 2.0f * q3;
	float _2q4 = 2.0f * q4;
	float _2q1q3 = 2.0f * q1 * q3;
	float _2q3q4 = 2.0f * q3 * q4;
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q1q4 = q1 * q4;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q2q4 = q2 * q4;
	float q3q3 = q3 * q3;
	float q3q4 = q3 * q4;
	float q4q4 = q4 * q4;
	
	/* Normalise accelerometer measurement */
	norm = sqrt(ax * ax + ay * ay + az * az);
	if (norm == 0.0f) return; // Handle NaN
	norm = 1.0f/norm;
	ax *= norm;
	ay *= norm;
	az *= norm;
	
	/* Normalise magnetometer measurement */
	norm = sqrt(mx * mx + my * my + mz * mz);
	if (norm == 0.0f) return; // Handle NaN
	norm = 1.0f/norm;
	mx *= norm;
	my *= norm;
	mz *= norm;
	
	/* Reference direction of Earth's magnetic field */
	_2q1mx = 2.0f * q1 * mx;
	_2q1my = 2.0f * q1 * my;
	_2q1mz = 2.0f * q1 * mz;
	_2q2mx = 2.0f * q2 * mx;
	hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
	hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
	_2bx = sqrt(hx * hx + hy * hy);
	_2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
	_4bx = 2.0f * _2bx;
	_4bz = 2.0f * _2bz;
	
	/* Gradient decent algorithm corrective step */
	s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // Normalise step magnitude
	norm = 1.0f/norm;
	s1 *= norm;
	s2 *= norm;
	s3 *= norm;
	s4 *= norm;
	
	/* Compute rate of change of quaternion */
	qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
	qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
	qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
	qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;
	
	/* Integrate to yield quaternion */
	q1 += qDot1 * IMU_READING_PERIODE;
	q2 += qDot2 * IMU_READING_PERIODE;
	q3 += qDot3 * IMU_READING_PERIODE;
	q4 += qDot4 * IMU_READING_PERIODE;
	norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // Normalise quaternion
	norm = 1.0f/norm;
	
	quaternions->q1 = q1 * norm;
	quaternions->q2 = q2 * norm;
	quaternions->q3 = q3 * norm;
	quaternions->q4 = q4 * norm;
}

/**------------------------------------------------------------------------------
  * @brief  			
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *------------------------------------------------------------------------------*/
void AHRS_QuaternionToEulerAngles(const AHRS_Quaternions * quaternions, AHRS_EulerAngles * eulerAngles)
{
	/* Short name local variable for readability */
	double q1 = quaternions->q1;
	double q2 = quaternions->q2;
	double q3 = quaternions->q3;
	double q4 = quaternions->q4;
	
	float a12 =   2.0f * (q2 * q3 + q1 * q4);
	float a22 =   q1 * q1 + q2 * q2 - q3 * q3 - q4 * q4;
	float a31 =   2.0f * (q1 * q2 + q3 * q4);
	float a32 =   2.0f * (q2 * q4 - q1 * q3);
	float a33 =   q1 * q1 - q2 * q2 - q3 * q3 + q4 * q4;

	eulerAngles->pitch = -asinf(a32);
	eulerAngles->roll  = atan2f(a31, a33);
	eulerAngles->yaw   = atan2f(a12, a22);
	eulerAngles->pitch = RADIAN_TO_DEGREE(eulerAngles->pitch);
	eulerAngles->yaw = RADIAN_TO_DEGREE(eulerAngles->yaw);
	eulerAngles->yaw   += MAGNETIC_DECLINATION;

	/* Ensure yaw stays between 0 and 360. */
	if(eulerAngles->yaw < 0)
		eulerAngles->yaw += 360.0f; 
		
	eulerAngles->roll = RADIAN_TO_DEGREE(eulerAngles->roll);
}

/**------------------------------------------------------------------------------
  * @brief  			
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *------------------------------------------------------------------------------*/
void AHRS_GetEulerAngles(AHRS_EulerAngles * eulerAngles, AHRS_Quaternions * quaternions)
{
	AHRS_AxisData accelData, gyroData, magData;
	
	BMX160_GetRawData(&BMX160);
		
	accelData.xData = BMX160.rawAccel.rawXData;
	accelData.yData = BMX160.rawAccel.rawYData;
	accelData.zData = BMX160.rawAccel.rawZData;
	
	gyroData.xData = BMX160.rawGyro.rawXData;
	gyroData.yData = BMX160.rawGyro.rawYData;
	gyroData.zData = BMX160.rawGyro.rawZData;
	
	magData.xData = BMX160.rawMag.rawXData;
	magData.yData = BMX160.rawMag.rawYData;
	magData.zData = BMX160.rawMag.rawZData;
	
	AHRS_GetMadgwickQuaternion(&accelData, &gyroData, &magData, quaternions);
	AHRS_QuaternionToEulerAngles(quaternions, eulerAngles);
}

/* Private functions ---------------------------------------------------------*/
																			
/**------------------------------------------------------------------------------
  * @brief  			
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *------------------------------------------------------------------------------*/
AHRS_Quaternions 		quaternions = {.q1=1};
AHRS_EulerAngles		eulerAngles;

static void AHRS_IMU_ReadingThread(void *arg)
{
	while(true)
	{
		osEventFlagsWait(EVT_IMU, IMU_PERIODIC_READ_FLAG, osFlagsWaitAll, osWaitForever);
		
		AHRS_GetEulerAngles(&eulerAngles, &quaternions);
		
		//BMX160_GetRawData(&BMX160);
		//
		//accelData.xData = BMX160.rawAccel.rawXData;
		//accelData.yData = BMX160.rawAccel.rawYData;
		//accelData.zData = BMX160.rawAccel.rawZData;
		// 
		//gyroData.xData = BMX160.rawGyro.rawXData;
		//gyroData.yData = BMX160.rawGyro.rawYData;
		//gyroData.zData = BMX160.rawGyro.rawZData;
		// 
		//magData.xData = BMX160.rawMag.rawXData;
		//magData.yData = BMX160.rawMag.rawYData;
		//magData.zData = BMX160.rawMag.rawZData;
		// 
		//AHRS_GetMadgwickQuaternion(&accelData, &gyroData, &magData, &quaternions);
		//AHRS_QuaternionToEulerAngles(&quaternions, &eulerAngles);
	}
}
																			
/**------------------------------------------------------------------------------
  * @brief  			
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *------------------------------------------------------------------------------*/
static void AHRS_IMU_ReadingTimCallback(void *arg)
{
	osEventFlagsSet(EVT_IMU, IMU_PERIODIC_READ_FLAG);
}
