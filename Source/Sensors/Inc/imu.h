#ifndef _IMU_H_
#define _IMU_H_

/* Includes ------------------------------------------------------------------*/
#include "adxl345.h"
#include "itg3205.h"
#include "hmc5883l.h"
#include "mpu6050.h"

/* Exported define -----------------------------------------------------------*/
#define Ts_PERIOD					0.1 		// Sec

#define ADXL345_I2C_DEVICE_INDEX		0
#define ITG3205_I2C_DEVICE_INDEX		1
#define HMC5883L_I2C_DEVICE_INDEX		2
#define MPU6050_I2C_DEVICE_INDEX		3

/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef struct
{
	double xAngle;
	double yAngle;
	double zAngle;

}AxisAngles;

/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

void IMU_Init(void);
void IMU_GetAngleFromAccelerometer(ADXL345_RawDatas * rawDatas, AxisAngles * axisAngles);
void IMU_GetAngleFromGyro(ITG3205_RawDatas * rawDatas, AxisAngles * currAxisAngles, AxisAngles * prevAxisAngles, float periode);

void IMU_GetAccelOffsetValues(COM_Handle * ADXL345, ADXL345_RawDatas * biasDatas);
void IMU_GetGyroOffsetValues(COM_Handle * ITG3205, ITG3205_RawDatas * biasDatas);
void IMU_GetMagnetoOffsetValues(COM_Handle * HMC5883L, HMC5883L_RawDatas * biasDatas);


#endif /* _IMU_H_ */
