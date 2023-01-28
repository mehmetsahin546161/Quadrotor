#ifndef _IMU_H_
#define _IMU_H_

/* Includes ------------------------------------------------------------------*/
#include "adxl345.h"
#include "itg3205.h"
#include "hmc5883l.h"

/* Exported define -----------------------------------------------------------*/
#define Ts_PERIOD					0.1 		// Sec

#define ADXL345_I2C_CHANNEL_NO		0
#define ITG3205_I2C_CHANNEL_NO		1
#define HMC5883L_I2C_CHANNEL_NO		2

/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef struct
{
	float xAngle;
	float yAngle;
	float zAngle;

}AxisAngles;

/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

void GY85_Init(void);
void GY85_GetAngleFromAccelerometer(ADXL345_RawDatas * rawDatas, AxisAngles * axisAngles);
void GY85_GetAngleFromGyro(ITG3205_RawDatas * rawDatas, AxisAngles * currAxisAngles, AxisAngles * prevAxisAngles, float periode);


#endif /* _IMU_H_ */
