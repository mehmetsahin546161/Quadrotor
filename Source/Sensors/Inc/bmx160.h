#ifndef _BMX_160_H_
#define _BMX_160_H_

/* Includes ------------------------------------------------------------------*/
#include "ahrs.h"
#include "stdint.h"
#include "stdbool.h"
#include "i2c.h"

/* Exported define -----------------------------------------------------------*/
#define BMX160_I2C_DEV_ADDR_GND  						0x68
#define BMX160_I2C_DEV_ADDR_VCC  						0x69

#define BMX160_DATA_0_ADDR										0x04
#define BMX160_DATA_1_ADDR										0x05
#define BMX160_DATA_2_ADDR										0x06
#define BMX160_DATA_3_ADDR										0x07
#define BMX160_DATA_4_ADDR										0x08
#define BMX160_DATA_5_ADDR										0x09
#define BMX160_DATA_6_ADDR										0x0A
#define BMX160_DATA_7_ADDR										0x0B
#define BMX160_DATA_8_ADDR										0x0C
#define BMX160_DATA_9_ADDR										0x0D
#define BMX160_DATA_10_ADDR										0x0E
#define BMX160_DATA_11_ADDR										0x0F
#define BMX160_DATA_12_ADDR										0x10
#define BMX160_DATA_13_ADDR										0x11
#define BMX160_DATA_14_ADDR										0x12
#define BMX160_DATA_15_ADDR										0x13
#define BMX160_DATA_16_ADDR										0x14
#define BMX160_DATA_17_ADDR										0x15
#define BMX160_DATA_18_ADDR										0x16
#define BMX160_DATA_19_ADDR										0x17
#define BMX160_SENSOR_TIME_0_ADDR							0x18
#define BMX160_SENSOR_TIME_1_ADDR							0x19
#define BMX160_SENSOR_TIME_2_ADDR							0x1A
#define BMX160_CMD_ADDR												0x7E
#define BMX160_MAGNETO_CONFIG_ADDR						0x44
#define BMX160_MAGNETO_MANUAL_CONFIG_ADDR			0x4B	/* Indirectly access to some of magnetometer config registers */
#define BMX160_MAG_IF_0_ADDR									0x4C
#define BMX160_MAG_IF_1_ADDR									0x4D
#define BMX160_MAG_IF_2_ADDR									0x4E
#define BMX160_MAG_IF_3_ADDR									0x4F
#define BMX160_MAGNETO_MANUAL_REP_XY_ADDR			0x51
#define BMX160_MAGNETO_MANUAL_REP_Z_ADDR		 	0x52

#define BMX160_DATA_CNT												23

#define BMX160_ACCEL_MG_LSB_2G      					0.000061035F   ///< Macro for mg per LSB at +/- 2g sensitivity (1 LSB = 0.000061035mg) */
#define BMX160_ACCEL_MG_LSB_4G      					0.000122070F   ///< Macro for mg per LSB at +/- 4g sensitivity (1 LSB = 0.000122070mg) */
#define BMX160_ACCEL_MG_LSB_8G      					0.000244141F   ///< Macro for mg per LSB at +/- 8g sensitivity (1 LSB = 0.000244141mg) */
#define BMX160_ACCEL_MG_LSB_16G     					0.000488281F   ///< Macro for mg per LSB at +/- 16g sensitivity (1 LSB = 0.000488281mg) */

#define BMX160_GYRO_SENSITIVITY_125DPS  			0.0038110F 	///< Gyroscope sensitivity at 125dps */
#define BMX160_GYRO_SENSITIVITY_250DPS  			0.0076220F 	///< Gyroscope sensitivity at 250dps */
#define BMX160_GYRO_SENSITIVITY_500DPS  			0.0152439F 	///< Gyroscope sensitivity at 500dps */
#define BMX160_GYRO_SENSITIVITY_1000DPS 			0.0304878F 	///< Gyroscope sensitivity at 1000dps */
#define BMX160_GYRO_SENSITIVITY_2000DPS 			0.0609756F 	///< Gyroscope sensitivity at 2000dps */

#define BMX160_MAGN_UT_LSB      							(0.3F)  		///< Macro for micro tesla (uT) per LSB (1 LSB = 0.1uT) */

/* Exported macro ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef enum
{
	BMX160_FAST_OFFSET_CALIBRATION 		= 0x03,
	BMX160_SET_ACCEL_PMU_SUSPEND 			= 0x10,
	BMX160_SET_ACCEL_PMU_NORMAL 			= 0x11,
	BMX160_SET_ACCEL_PMU_LOW_POWER 		= 0x12,
	BMX160_SET_GYRO_PMU_SUSPEND 			= 0x14,
	BMX160_SET_GYRO_PMU_NORMAL 				= 0x15,
	BMX160_SET_GYRO_PMU_FAST_STARTUP 	= 0x17,
	BMX160_SET_MAG_PMU_SUSPEND 				= 0x18,
	BMX160_SET_MAG_PMU_NORMAL 				= 0x19,
	BMX160_SET_MAG_PMU_LOW_POWER 			= 0x1A,
	BMX160_PROG_NVM										= 0xA0,
	BMX160_FLUSH_FIFO									=	0xB0,
	BMX160_RESET_INTERRUPT						=	0xB1,
	BMX160_CLEAR_STEP_COUNTER					=	0xB2,
	BMX160_SOFT_RESET									=	0xB6,

}BMX160_CmdModes;

typedef enum
{
	/* Indirect write magnetometer comands */
	BMX160_MAGNETO_SLEEP_MODE  							= 0x01,
	BMX160_MAGNETO_LOW_POWER_PRESET_XY 			= 0x01,
	BMX160_MAGNETO_REGULAR_PRESET_XY 				= 0x04,
	BMX160_MAGNETO_ENHANCED_PRESET_XY 			= 0x07,
	BMX160_MAGNETO_HIGH_ACCURACY_PRESET_XY 	= 0x17,
	BMX160_MAGNETO_LOW_POWER_PRESET_Z 			= 0x02,
	BMX160_MAGNETO_REGULAR_PRESET_Z 				= 0x0E,
	BMX160_MAGNETO_ENHANCED_PRESET_Z 				= 0x1A,
	BMX160_MAGNETO_HIGH_ACCURACY_PRESET_Z 	= 0x52,
	
}BMX160_MagnetoManualCmds;

typedef enum
{
	BMX160_MAGNETO_ODR_25_HZ	=	6,
	BMX160_MAGNETO_ODR_100_HZ	=	8,
	BMX160_MAGNETO_ODR_800_HZ	=	11,

}BMX160_MagnetoOuputDataRate;

typedef enum
{
	BMX160_MAGNETO_BURST_READ_1_BYTE =	0,
	BMX160_MAGNETO_BURST_READ_2_BYTE =	1,
	BMX160_MAGNETO_BURST_READ_6_BYTE =	2,
	BMX160_MAGNETO_BURST_READ_8_BYTE =	3

}BMX160_MagnetoBurstReadLength;

typedef union
{
	struct
	{
		BMX160_MagnetoBurstReadLength magRdBurst:2;
		uint8_t magOffset:4;	
		bool reserved:1;
		bool magManuelEn:1;
	}BIT;
	
	uint8_t BYTE;

}BMX160_MagIf0Reg;

typedef union
{
	struct
	{
		BMX160_MagnetoOuputDataRate 	odr:4;
		uint8_t 											reserved:4;
	}BIT;
	
	uint8_t BYTE;

}BMX160_MagnetoConfigReg;

typedef struct
{
	double rawXData;
	double rawYData;
	double rawZData;
	uint32_t sensortime;
	
}BMX160_RawData;

typedef struct
{
	I2C_HandleTypeDef *	i2c;
	
	/* Raw Data */
	BMX160_RawData 		rawAccel;				/* Linear acceleration(m/s^2) */		
	BMX160_RawData		rawGyro;				/* Angular velocity(rad/s) */
	BMX160_RawData 		rawMag;
	
}BMX160_Handle;

/* Exported variables --------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void BMX160_CreateSensor(BMX160_Handle * BMX160);
void BMX160_GetRawData(BMX160_Handle * BMX160);

#endif /* _BMX_160_H_ */
