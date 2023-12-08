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
	double xData;
	double yData;
	double zData;

}AHRS_AxisData;

typedef struct
{
	double p;
	double q;
	double r;

}AHRS_BodyRate;

typedef struct
{
	double roll;
	double pitch;
	double yaw;

}AHRS_EulerAngles;

typedef struct
{
	double q1;
	double q2;
	double q3;
	double q4;

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
	
	/* Bias angles */
	AHRS_EulerAngles biasAngle;
	
}AHRS_Handle;

/* Exported variables --------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

void AHRS_Init(void);
void AHRS_GetMadgwickQuaternion(const AHRS_AxisData * accelData, const AHRS_AxisData * gyroData, const AHRS_AxisData * magData, AHRS_Quaternions * quaternions);
void AHRS_QuaternionToEulerAngles(const AHRS_Quaternions * quaternions, AHRS_EulerAngles * eulerAngles);
//void AHRS_GetEulerAngles(IMU_AxisDatas * accelData, IMU_AxisDatas * magnetoData, IMU_EulerAngles * eulerAngles);
//void IMU_GetAngleFromGyro(IMU_AxisDatas * rawDatas, IMU_Angles * currAngles, IMU_Angles * prevSumAngles, float periode);

#endif /* _AHRS_H_ */
