#ifndef _IMU_H_
#define _IMU_H_

/* Includes ------------------------------------------------------------------*/
#include "com_interface.h"

/* Exported define -----------------------------------------------------------*/
#define Ts_PERIOD					0.05 		// Sec

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

}IMU_AxisAngles;

typedef struct
{
	double xData;
	double yData;
	double zData;

}IMU_AxisDatas;

/* Exported variables --------------------------------------------------------*/
extern COM_Handle 	ADXL345_ComHandle;
extern COM_Handle 	ITG3205_ComHandle;
extern COM_Handle 	HMC5883L_ComHandle;
extern COM_Handle		MPU6050_ComHandle;

extern IMU_AxisDatas 	adxl345AccelBias;
extern IMU_AxisDatas 	mpu6050AccelBias;
extern IMU_AxisDatas	itg3205GyroBias;
extern IMU_AxisDatas 	mpu6050GyroBias;

extern IMU_AxisAngles 	adxl345RawAccelAngle;
extern IMU_AxisAngles 	mpu6050RawAccelAngle;
extern IMU_AxisAngles		itg3205RawGyroAngle;
extern IMU_AxisAngles 	mpu6050RawGyroAngle;
extern IMU_AxisAngles 	hmc5883RawMagnetoAngle;

extern IMU_AxisAngles		itg3205PrevGyroAngle;
extern IMU_AxisAngles 	mpu6050PrevGyroAngle;

extern IMU_AxisAngles eulerAngles;

/* Exported functions --------------------------------------------------------*/

void IMU_Init(void);
void IMU_RemoveBias(IMU_AxisDatas * axisData, const IMU_AxisDatas * axisBias);
void IMU_GetAngleFromAccelerometer(IMU_AxisDatas * rawDatas, IMU_AxisAngles * axisAngles);
void IMU_GetAngleFromGyro(IMU_AxisDatas * rawDatas, IMU_AxisAngles * currAngles, IMU_AxisAngles * prevSumAngles, float periode);
void IMU_ConvertRadianToAngle(IMU_AxisAngles * axisAngles);

#endif /* _IMU_H_ */
