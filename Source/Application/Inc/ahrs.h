#ifndef _AHRS_H_
#define _AHRS_H_

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os2.h"
#include "stdbool.h"

/* Exported define -----------------------------------------------------------*/
#define	IMU_READING_FREQ					(30.0)										/* Hz */
#define IMU_READING_PERIOD				(1/IMU_READING_FREQ)			/* Sec */

#define GRAVITY_ACCELERATION_G						1								/* g */
#define GRAVITY_ACCELERATION_M_S2					(9.81)					/* s/s^2 */

/* Exported macro ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

typedef struct
{
	osThreadId_t 			TID_IMU_Reading;
	osThreadId_t 			TID_IMU_BiasCalc;
	osTimerId_t 			TIM_IMU_Reading;
	osTimerId_t 			TIM_IMU_BiasCalcFinished;
	osEventFlagsId_t 	EVT_IMU_Reading;

}AHRS_OS_Resource;

typedef struct
{
	float xData;
	float yData;
	float zData;

}AHRS_AxisData;

typedef struct
{
	float p;
	float q;
	float r;

}AHRS_BodyRate;

typedef struct
{
	float roll;
	float pitch;
	float yaw;

}AHRS_EulerAngles;

typedef struct
{
	float rollRate;
	float pitchRate;
	float yawRate;

}AHRS_EulerAnglesRate;

typedef struct
{
	float q1;
	float q2;
	float q3;
	float q4;

}AHRS_Quaternions;

typedef struct
{
	bool 	enabled;
	float samplingTime;	//Second
	
	/* Raw IMU Data */
	AHRS_AxisData accelData;
	AHRS_AxisData gyroData;
	AHRS_AxisData magData;
	
	/* Euler Angles and Quaternions */
	AHRS_EulerAngles 	eulerAngles;
	AHRS_Quaternions	quaternions;
	
	AHRS_EulerAnglesRate	eulerAnglesRate;
	AHRS_EulerAngles			prevEulerAngles;
	AHRS_BodyRate					bodyRate;
	
	/* Bias angles */
	AHRS_EulerAngles 	biasEulerAngles;
	AHRS_Quaternions	biasQuaternions;
	
	AHRS_OS_Resource osResource;
	
}AHRS_Handle;

/* Exported variables --------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

void AHRS_Init(AHRS_Handle * AHRS);
void AHRS_Enable(AHRS_Handle * AHRS);
void AHRS_Disable(AHRS_Handle * AHRS);
void AHRS_RemoveBiasAngle(AHRS_Handle * AHRS);
void AHRS_GetEulerAngles(AHRS_EulerAngles * eulerAngles, AHRS_Quaternions * quaternions);
void AHRS_GetMadgwickQuaternion(const AHRS_AxisData * accelData, const AHRS_AxisData * gyroData, const AHRS_AxisData * magnetoData, AHRS_Quaternions * quaternions);
void AHRS_QuaternionToEulerAngles(const AHRS_Quaternions * quaternions, AHRS_EulerAngles * eulerAngles);
void AHRS_GetEulerAnglesRate(AHRS_EulerAngles * curentEulerAngles, AHRS_EulerAngles	* prevEulerAngles, AHRS_EulerAnglesRate *	eulerAnglesRate, float samplimgTime);
void AHRS_GetBodyRateFromEulerAnglesRate(AHRS_EulerAnglesRate	* eulerAnglesRate, AHRS_EulerAngles * eulerAngles, AHRS_BodyRate * bodyRate);

#endif /* _AHRS_H_ */
