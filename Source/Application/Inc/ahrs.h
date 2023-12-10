#ifndef _AHRS_H_
#define _AHRS_H_

/* Includes ------------------------------------------------------------------*/

/* Exported define -----------------------------------------------------------*/
#define	IMU_READING_FREQ					(20.0)										/* Hz */
#define IMU_READING_PERIODE				(1/IMU_READING_FREQ)			/* Sec */

#define GRAVITY_ACCELERATION_G						1								/* g */
#define GRAVITY_ACCELERATION_M_S2					(9.81)					/* s/s^2 */

/* Exported macro ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

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
	AHRS_EulerAngles biasAngle;
	
}AHRS_Handle;

/* Exported variables --------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

void AHRS_GetEulerAngles(AHRS_EulerAngles * eulerAngles, AHRS_Quaternions * quaternions);
void AHRS_GetMadgwickQuaternion(const AHRS_AxisData * accelData, const AHRS_AxisData * gyroData, const AHRS_AxisData * magnetoData, AHRS_Quaternions * quaternions);
void AHRS_QuaternionToEulerAngles(const AHRS_Quaternions * quaternions, AHRS_EulerAngles * eulerAngles);
void AHRS_GetEulerAnglesRate(AHRS_EulerAngles * curentEulerAngles, AHRS_EulerAngles	* prevEulerAngles, AHRS_EulerAnglesRate *	eulerAnglesRate, float samplimgTime);
void AHRS_GetBodyRateFromEulerAnglesRate(AHRS_EulerAnglesRate	* eulerAnglesRate, AHRS_EulerAngles * eulerAngles, AHRS_BodyRate * bodyRate);

#endif /* _AHRS_H_ */
