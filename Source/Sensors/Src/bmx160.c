/* Includes ------------------------------------------------------------------*/
#include "bmx160.h"
#include "string.h"
#include "stdlib.h"
#include "calc.h"
#include "cmsis_os2.h"

/* Private define ------------------------------------------------------------*/
#define BMX160_SOFT_RESET_DELAY_MS		15

/* Private macro -------------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static void BMX160_InitMagneto(I2C_HandleTypeDef *	i2c);
static void BMX160_SetMagnetoManualConfig(I2C_HandleTypeDef *	i2c, uint8_t addrToWrite, BMX160_MagnetoManualCmds manualCmd);

/* Exported variables --------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

/**--------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *--------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void BMX160_Init(BMX160_Handle * BMX160)
{
	uint8_t writenData = 0;
	
	/* Soft Reset. */
	writenData = BMX160_SOFT_RESET;
	I2C_AsyncMemWrite(BMX160->i2c, BMX160_I2C_DEV_ADDR_GND, BMX160_CMD_ADDR, I2C_MEMADD_SIZE_8BIT, &writenData, 1);
	osDelay(BMX160_SOFT_RESET_DELAY_MS);
	
	/* Accel Power Config */
	writenData = BMX160_SET_ACCEL_PMU_NORMAL;
	I2C_AsyncMemWrite(BMX160->i2c, BMX160_I2C_DEV_ADDR_GND, BMX160_CMD_ADDR, I2C_MEMADD_SIZE_8BIT, &writenData, 1);
	osDelay(50);
	
	/* Gyro Power Config */
	writenData = BMX160_SET_GYRO_PMU_NORMAL;
	I2C_AsyncMemWrite(BMX160->i2c, BMX160_I2C_DEV_ADDR_GND, BMX160_CMD_ADDR, I2C_MEMADD_SIZE_8BIT, &writenData, 1);
	osDelay(100);
	
	/* Magnetometer Power Config */
	writenData = BMX160_SET_MAG_PMU_NORMAL;
	I2C_AsyncMemWrite(BMX160->i2c, BMX160_I2C_DEV_ADDR_GND, BMX160_CMD_ADDR, I2C_MEMADD_SIZE_8BIT, &writenData, 1);
	osDelay(10);
	
	/* MagIf Config */
	BMX160_InitMagneto(BMX160->i2c);
}

/**--------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *--------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void BMX160_GetRawData(BMX160_Handle * BMX160)
{
	int16_t xData=0, yData=0, zData=0;
	uint8_t readData[BMX160_DATA_CNT] = {0};
	
	/* All datas are read at once */
	I2C_AsyncMemRead(BMX160->i2c, BMX160_I2C_DEV_ADDR_GND, BMX160_DATA_0_ADDR, I2C_MEMADD_SIZE_8BIT, readData, BMX160_DATA_CNT);
	
	/* Magnetometer Datas */
	xData = (int16_t)(readData[1]<<8 | readData[0]);
	yData = (int16_t)(readData[3]<<8 | readData[2]);
	zData = (int16_t)(readData[5]<<8 | readData[4]);
	
	BMX160->rawMag.rawXData = xData*BMX160_MAGN_UT_LSB;
	BMX160->rawMag.rawYData = yData*BMX160_MAGN_UT_LSB;
	BMX160->rawMag.rawZData = zData*BMX160_MAGN_UT_LSB;

	/* Gyroscope Datas */
	xData = (int16_t)(readData[9]<<8  | readData[8]);
	yData = (int16_t)(readData[11]<<8 | readData[10]);
	zData = (int16_t)(readData[13]<<8 | readData[12]);
	
	BMX160->rawGyro.rawXData = DEGREE_TO_RADIAN(xData*BMX160_GYRO_SENSITIVITY_2000DPS);		/* Unit is rad/sec */
	BMX160->rawGyro.rawYData = DEGREE_TO_RADIAN(yData*BMX160_GYRO_SENSITIVITY_2000DPS);
	BMX160->rawGyro.rawZData = DEGREE_TO_RADIAN(zData*BMX160_GYRO_SENSITIVITY_2000DPS);

	/* Accelerometer Datas */
	xData = (int16_t)(readData[15]<<8 | readData[14]);
	yData = (int16_t)(readData[17]<<8 | readData[16]);
	zData = (int16_t)(readData[19]<<8 | readData[18]);
	
	BMX160->rawAccel.rawXData = xData*(BMX160_ACCEL_MG_LSB_2G);	/* Unit is g */
	BMX160->rawAccel.rawYData = yData*(BMX160_ACCEL_MG_LSB_2G);
	BMX160->rawAccel.rawZData = zData*(BMX160_ACCEL_MG_LSB_2G);
}

/* Private functions ---------------------------------------------------------*/

/**--------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *--------------------------------------------------------------------------------------------------------------------------------------------------------------*/
static void BMX160_InitMagneto(I2C_HandleTypeDef *	i2c)
{
	uint8_t writenData = 0;
	
	/* Manual operation is enabled */
	BMX160_MagIf0Reg magIfReg = 
	{
		.BIT.magManuelEn = true
	};
	writenData = magIfReg.BYTE;
	I2C_AsyncMemWrite(i2c, BMX160_I2C_DEV_ADDR_GND, BMX160_MAG_IF_0_ADDR, I2C_MEMADD_SIZE_8BIT, &writenData, 1);
	osDelay(50);
	
	/* Set magnetometer slepp mode via manual operation */
	BMX160_SetMagnetoManualConfig(i2c, BMX160_MAGNETO_MANUAL_CONFIG_ADDR, BMX160_MAGNETO_SLEEP_MODE);
	
	/* REP_XY regular preset is used */
	BMX160_SetMagnetoManualConfig(i2c, BMX160_MAGNETO_MANUAL_REP_XY_ADDR, BMX160_MAGNETO_REGULAR_PRESET_XY);
	
	/* REP_Z regular preset is used */
	BMX160_SetMagnetoManualConfig(i2c, BMX160_MAGNETO_MANUAL_REP_Z_ADDR, BMX160_MAGNETO_REGULAR_PRESET_Z);
	
	/* Prepare MAG_IF[1,2,3] registers for indirect access mode */
	writenData = 0x02;
	I2C_AsyncMemWrite(i2c, BMX160_I2C_DEV_ADDR_GND, BMX160_MAG_IF_3_ADDR, I2C_MEMADD_SIZE_8BIT, &writenData, 1);
	
	writenData = 0x4C;
	I2C_AsyncMemWrite(i2c, BMX160_I2C_DEV_ADDR_GND, BMX160_MAG_IF_2_ADDR, I2C_MEMADD_SIZE_8BIT, &writenData, 1);
	
	writenData = 0x42;
	I2C_AsyncMemWrite(i2c, BMX160_I2C_DEV_ADDR_GND, BMX160_MAG_IF_1_ADDR, I2C_MEMADD_SIZE_8BIT, &writenData, 1);

	
	/* Output data rate is set */
	BMX160_MagnetoConfigReg magnetoConfig =
	{
		.BIT.odr = BMX160_MAGNETO_ODR_100_HZ
	};
	I2C_AsyncMemWrite(i2c, BMX160_I2C_DEV_ADDR_GND, BMX160_MAGNETO_CONFIG_ADDR, I2C_MEMADD_SIZE_8BIT, &magnetoConfig.BYTE, 1);

	/* Burst read length is set to 8 Bytes */
	magIfReg.BYTE = 0;
	magIfReg.BIT.magRdBurst = BMX160_MAGNETO_BURST_READ_8_BYTE;
	writenData = magIfReg.BYTE;
	I2C_AsyncMemWrite(i2c, BMX160_I2C_DEV_ADDR_GND, BMX160_MAG_IF_0_ADDR, I2C_MEMADD_SIZE_8BIT, &writenData, 1);
	osDelay(50);
}

/**--------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *--------------------------------------------------------------------------------------------------------------------------------------------------------------*/
static void BMX160_SetMagnetoManualConfig(I2C_HandleTypeDef *	i2c, uint8_t addrToWrite, BMX160_MagnetoManualCmds manualCmd)
{
	uint8_t writenData = 0;
	
	writenData = manualCmd;
	I2C_AsyncMemWrite(i2c, BMX160_I2C_DEV_ADDR_GND, BMX160_MAG_IF_3_ADDR, I2C_MEMADD_SIZE_8BIT, &writenData, 1);
	
	writenData = addrToWrite;
	I2C_AsyncMemWrite(i2c, BMX160_I2C_DEV_ADDR_GND, BMX160_MAG_IF_2_ADDR, I2C_MEMADD_SIZE_8BIT, &writenData, 1);
}
