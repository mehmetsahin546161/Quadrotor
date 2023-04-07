#ifndef _BMX_160_H_
#define _BMX_160_H_

/* Includes ------------------------------------------------------------------*/
#include "com_interface.h"


/* Exported define -----------------------------------------------------------*/
#define 	BMX160_I2C_DEV_ADDR_GND  				0x68
#define 	BMX160_I2C_DEV_ADDR_VCC  				0x69


#define BMX160_CMD_ADDR												0x7E
#define BMX160_MAGNETO_CONFIG_ADDR						0x44
#define BMX160_MAGNETO_MANUAL_CONFIG_ADDR			0x4B	/* Indirectly access to some of magnetometer config registers */
#define BMX160_MAG_IF_0_ADDR									0x4C
#define BMX160_MAG_IF_1_ADDR									0x4D
#define BMX160_MAG_IF_2_ADDR									0x4E
#define BMX160_MAG_IF_3_ADDR									0x4F
#define BMX160_MAGNETO_MANUAL_REP_XY_ADDR			0x51
#define BMX160_MAGNETO_MANUAL_REP_Z_ADDR		 	0x52
#define BMX160_
#define BMX160_
#define BMX160_
#define BMX160_
#define BMX160_
#define BMX160_
#define BMX160_
#define BMX160_
#define BMX160_
#define BMX160_
#define BMX160_
#define BMX160_
#define BMX160_
#define BMX160_

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

/* Exported variables --------------------------------------------------------*/
/* Exported functions prototypes ---------------------------------------------*/
void BMX160_InitSensor(COM_Handle * BMX160);
void BMX160_InitMagneto(COM_Handle * BMX160);

void BMX160_SetMagnetoConfig(COM_Handle * BMX160, BMX160_MagnetoConfigReg * magnetoConfig);
void BMX160_SetCMD(COM_Handle * BMX160, uint8_t data);
void BMX160_SetMagIF0(COM_Handle * BMX160, BMX160_MagIf0Reg * magIfReg);
void BMX160_SetMagIF1(COM_Handle * BMX160, uint8_t addrToRead);
void BMX160_SetMagIF2(COM_Handle * BMX160, uint8_t addrToWrite);
void BMX160_SetMagIF3(COM_Handle * BMX160, uint8_t dataToWrite);
void BMX160_SetMagnetoManualConfig(COM_Handle * BMX160, uint8_t addrToWrite, BMX160_MagnetoManualCmds manualCmd);
uint8_t BMX160_GetMagnetoManualConfig(COM_Handle * BMX160, uint8_t addrToRead);

#endif /* _BMX_160_H_ */
