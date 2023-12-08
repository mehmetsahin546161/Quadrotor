/* Includes ------------------------------------------------------------------*/
#include "mpu9250.h"
#include "math.h"
#include "calc.h"
#include "cmsis_os2.h"

/* Private define ------------------------------------------------------------*/
#define MPU9250_SELF_TEST_ITERATION_CNT		200
#define MPU9250_I2C_COM_TIMEOUT						1000

/* Private macro -------------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
extern I2C_HandleTypeDef 		hi2c1;

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

/**-----------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *-----------------------------------------------------------------------------------------------------------------*/
void MPU9250_CreateSensor(MPU9250_Handle * MPU9250)
{
	MPU9250->i2cHandle = &hi2c1;
	
	uint8_t writtenData;
	uint8_t readData;
	
	/* Self-Test */
	MPU9250_SelfTest(MPU9250->i2cHandle, MPU9250->selfTestResult);
	
	/* Calibration */
	MPU9250_Calibrate(MPU9250->i2cHandle, MPU9250->accelBias, MPU9250->gyroBias);
	
	osDelay(1000);

	/* Init Gyroscope and Accelerometer. */
	MPU9250_Init(MPU9250->i2cHandle);
	
	/* Config INT/Bypass register */
	writtenData = 0x22;
	HAL_I2C_Mem_Write(MPU9250->i2cHandle, MPU9250_ADDRESS, MPU9250_INT_PIN_CFG, 1, &writtenData, 1, MPU9250_I2C_COM_TIMEOUT);
	
	/* Get magnetometer calibration from AK8963 ROM. */
	AK8963_Init(MPU9250->i2cHandle, MPU9250->magCalibration);
	
	AK8963_Calibrate(MPU9250);
	
	osDelay(1000);
	
}

/**-----------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *-----------------------------------------------------------------------------------------------------------------*/
void MPU9250_Init(I2C_HandleTypeDef * i2cHandle)
{
	uint8_t readData;
	uint8_t writtenData;

	/* Wake up device. */
	writtenData = 0x00;
	HAL_I2C_Mem_Write(i2cHandle, MPU9250_ADDRESS, MPU9250_PWR_MGMT_1, 1, &writtenData, 1, MPU9250_I2C_COM_TIMEOUT);
	osDelay(100);

	writtenData = 0x01;
	HAL_I2C_Mem_Write(i2cHandle, MPU9250_ADDRESS, MPU9250_PWR_MGMT_1, 1, &writtenData, 1, MPU9250_I2C_COM_TIMEOUT);
	osDelay(100);

	writtenData = 0x03;
	HAL_I2C_Mem_Write(i2cHandle, MPU9250_ADDRESS, MPU9250_CONFIG, 1, &writtenData, 1, MPU9250_I2C_COM_TIMEOUT);
	osDelay(100);

	writtenData = 0x04;
	HAL_I2C_Mem_Write(i2cHandle, MPU9250_ADDRESS, MPU9250_SMPLRT_DIV, 1, &writtenData, 1, MPU9250_I2C_COM_TIMEOUT);
	osDelay(100);

	HAL_I2C_Mem_Read(i2cHandle, MPU9250_ADDRESS, MPU9250_GYRO_CONFIG, 1, &readData, 1, MPU9250_I2C_COM_TIMEOUT);
	readData = readData & (~0x03); 						// Clear Fchoice bits [1:0]
	readData = readData & ~0x18; 							// Clear GFS bits [4:3]
	readData = readData | GFS_2000DPS << 3; 	// Set full scale range for the gyro
	osDelay(100);

	writtenData = readData;
	HAL_I2C_Mem_Write(i2cHandle, MPU9250_ADDRESS, MPU9250_GYRO_CONFIG, 1, &writtenData, 1, MPU9250_I2C_COM_TIMEOUT);
	osDelay(100);

	HAL_I2C_Mem_Read(i2cHandle, MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, 1, &readData, 1, MPU9250_I2C_COM_TIMEOUT);
	readData = readData & ~0x18;  						// Clear AFS bits [4:3]
	readData = readData | AFS_16G << 3; 			// Set full scale range for the accelerometer

	writtenData = readData;
	HAL_I2C_Mem_Write(i2cHandle, MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, 1, &writtenData, 1, MPU9250_I2C_COM_TIMEOUT);
	osDelay(100);

	HAL_I2C_Mem_Read(i2cHandle, MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG2, 1, &readData, 1, MPU9250_I2C_COM_TIMEOUT);
	readData = readData & ~0x0F; 							// Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
	readData = readData | 0x03;  							// Set accelerometer rate to 1 kHz and bandwidth to 41 Hz

	writtenData = readData;
	HAL_I2C_Mem_Write(i2cHandle, MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG2, 1, &writtenData, 1, MPU9250_I2C_COM_TIMEOUT);
	osDelay(100);
}

/**-----------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *-----------------------------------------------------------------------------------------------------------------*/
void  MPU9250_SelfTest(I2C_HandleTypeDef * 	i2cHandle, float * testResult)
{
	uint8_t	writtenData;
	uint8_t FS = 0;
	uint8_t selfTest[6];
	float factoryTrim[6];
	uint8_t rawTestData[6] = {0, 0, 0, 0, 0, 0};
	int32_t accelAvg[3] = {0}, gyroAvg[3] = {0}, accelSelfTestAvg[3] = {0}, gyroSelfTestAvg[3] = {0};

	volatile HAL_StatusTypeDef status;
	
	/* Set gyro sample rate to 1 kHz */
	writtenData = 0x00;
	status = HAL_I2C_Mem_Write(i2cHandle, MPU9250_ADDRESS, MPU9250_SMPLRT_DIV, 1, &writtenData, 1, MPU9250_I2C_COM_TIMEOUT);
	
	/* Set gyro sample rate to 1 kHz and DLPF to 92 Hz */
	writtenData = 0x02;
	status = HAL_I2C_Mem_Write(i2cHandle, MPU9250_ADDRESS, MPU9250_CONFIG, 1, &writtenData, 1, MPU9250_I2C_COM_TIMEOUT);
	
	/* Set full scale range for the gyro to 250 dps */
	writtenData = FS<<3;
	status = HAL_I2C_Mem_Write(i2cHandle, MPU9250_ADDRESS, MPU9250_GYRO_CONFIG, 1, &writtenData, 1, MPU9250_I2C_COM_TIMEOUT);
	
	/* Set accelerometer rate to 1 kHz and bandwidth to 92 Hz */
	writtenData = 0x02;
	status = HAL_I2C_Mem_Write(i2cHandle, MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG2, 1, &writtenData, 1, MPU9250_I2C_COM_TIMEOUT);
	
	/* Set full scale range for the accelerometer to 2 g */
	writtenData = FS<<3;
	status = HAL_I2C_Mem_Write(i2cHandle, MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, 1, &writtenData, 1, MPU9250_I2C_COM_TIMEOUT);
	
	/* Get average current values of gyro and acclerometer */
	for(uint16_t i = 0; i < MPU9250_SELF_TEST_ITERATION_CNT; i++)
	{
		status = HAL_I2C_Mem_Read(i2cHandle, MPU9250_ADDRESS, MPU9250_ACCEL_XOUT_H, 1, &rawTestData[0], 6, MPU9250_I2C_COM_TIMEOUT);
		
		accelAvg[0] += (int16_t)(((int16_t)rawTestData[0] << 8) | rawTestData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
		accelAvg[1] += (int16_t)(((int16_t)rawTestData[2] << 8) | rawTestData[3]) ;
		accelAvg[2] += (int16_t)(((int16_t)rawTestData[4] << 8) | rawTestData[5]) ;

		status = HAL_I2C_Mem_Read(i2cHandle, MPU9250_ADDRESS, MPU9250_GYRO_XOUT_H, 1, &rawTestData[0], 6, MPU9250_I2C_COM_TIMEOUT);
		
		
		gyroAvg[0] += (int16_t)(((int16_t)rawTestData[0] << 8) | rawTestData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
		gyroAvg[1] += (int16_t)(((int16_t)rawTestData[2] << 8) | rawTestData[3]) ;
		gyroAvg[2] += (int16_t)(((int16_t)rawTestData[4] << 8) | rawTestData[5]) ;
	}
	
	for(uint16_t i = 0; i < 3; i++)
	{
		accelAvg[i] /= (float)(MPU9250_SELF_TEST_ITERATION_CNT);
		gyroAvg[i] /= (float)(MPU9250_SELF_TEST_ITERATION_CNT);
	}
	
	/* Configure the accelerometer for self-test */
	
	/* Enable self test on all three axes and set accelerometer range to +/- 2 g */
	writtenData = 0xE0;
	HAL_I2C_Mem_Write(i2cHandle, MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, 1, &writtenData, 1, MPU9250_I2C_COM_TIMEOUT);
	
	/* Enable self test on all three axes and set gyro range to +/- 250 degrees/s */
	writtenData = 0xE0;
	HAL_I2C_Mem_Write(i2cHandle, MPU9250_ADDRESS, MPU9250_GYRO_CONFIG, 1, &writtenData, 1, MPU9250_I2C_COM_TIMEOUT);
	
	/* Delay a while to let the device stabilize */
	osDelay(25);

	// Get average self-test values of gyro and acclerometer
	for(uint16_t i = 0; i < MPU9250_SELF_TEST_ITERATION_CNT; i++)
	{
		HAL_I2C_Mem_Read(i2cHandle, MPU9250_ADDRESS, MPU9250_ACCEL_XOUT_H, 1, &rawTestData[0], 6, MPU9250_I2C_COM_TIMEOUT);
		accelSelfTestAvg[0] += (int16_t)(((int16_t)rawTestData[0] << 8) | rawTestData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
		accelSelfTestAvg[1] += (int16_t)(((int16_t)rawTestData[2] << 8) | rawTestData[3]) ;
		accelSelfTestAvg[2] += (int16_t)(((int16_t)rawTestData[4] << 8) | rawTestData[5]) ;

		HAL_I2C_Mem_Read(i2cHandle, MPU9250_ADDRESS, MPU9250_GYRO_XOUT_H, 1, &rawTestData[0], 6, MPU9250_I2C_COM_TIMEOUT);
		gyroSelfTestAvg[0] += (int16_t)(((int16_t)rawTestData[0] << 8) | rawTestData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
		gyroSelfTestAvg[1] += (int16_t)(((int16_t)rawTestData[2] << 8) | rawTestData[3]) ;
		gyroSelfTestAvg[2] += (int16_t)(((int16_t)rawTestData[4] << 8) | rawTestData[5]) ;
	}

	for(uint16_t i = 0; i < 3; i++)
	{
		accelSelfTestAvg[i] /= (float)(MPU9250_SELF_TEST_ITERATION_CNT);;
		gyroSelfTestAvg[i] /= (float)(MPU9250_SELF_TEST_ITERATION_CNT);;
	}
	
	/* Configure the gyro and accelerometer for normal operation */
	writtenData = 0x00;
	HAL_I2C_Mem_Write(i2cHandle, MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, 1, &writtenData, 1, MPU9250_I2C_COM_TIMEOUT);
	
	writtenData = 0x00;
	HAL_I2C_Mem_Write(i2cHandle, MPU9250_ADDRESS, MPU9250_GYRO_CONFIG, 1, &writtenData, 1, MPU9250_I2C_COM_TIMEOUT);
	
	/* Delay a while to let the device stabilize */
	osDelay(25);

	/* Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg */
	HAL_I2C_Mem_Read(i2cHandle, MPU9250_ADDRESS, MPU9250_SELF_TEST_X_ACCEL, 1, &selfTest[0], 1, MPU9250_I2C_COM_TIMEOUT);	// X-Axis accel self-test results
	HAL_I2C_Mem_Read(i2cHandle, MPU9250_ADDRESS, MPU9250_SELF_TEST_Y_ACCEL, 1, &selfTest[1], 1, MPU9250_I2C_COM_TIMEOUT);	// Y-Axis accel self-test results
	HAL_I2C_Mem_Read(i2cHandle, MPU9250_ADDRESS, MPU9250_SELF_TEST_Z_ACCEL, 1, &selfTest[2], 1, MPU9250_I2C_COM_TIMEOUT);	// Z-Axis accel self-test results
	HAL_I2C_Mem_Read(i2cHandle, MPU9250_ADDRESS, MPU9250_SELF_TEST_X_GYRO, 1, &selfTest[3], 1, MPU9250_I2C_COM_TIMEOUT);	// X-Axis gyro self-test results
	HAL_I2C_Mem_Read(i2cHandle, MPU9250_ADDRESS, MPU9250_SELF_TEST_Y_GYRO, 1, &selfTest[4], 1, MPU9250_I2C_COM_TIMEOUT);	// Y-Axis gyro self-test results
	HAL_I2C_Mem_Read(i2cHandle, MPU9250_ADDRESS, MPU9250_SELF_TEST_Z_GYRO, 1, &selfTest[5], 1, MPU9250_I2C_COM_TIMEOUT);	// Z-Axis gyro self-test results

	//Retrieve factory self-test value from self-test code reads
	factoryTrim[0] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[0] - 1.0) ));
	factoryTrim[1] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[1] - 1.0) ));
	factoryTrim[2] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[2] - 1.0) ));
	factoryTrim[3] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[3] - 1.0) ));
	factoryTrim[4] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[4] - 1.0) ));
	factoryTrim[5] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[5] - 1.0) ));

	/* Report percent differences */
	for(int i=0; i<3; i++)
	{
     testResult[i]   = 100.0*((float)(accelSelfTestAvg[i] - accelAvg[i]))/factoryTrim[i] - 100.;
     testResult[i+3] = 100.0*((float)(gyroSelfTestAvg[i] - gyroAvg[i]))/factoryTrim[i+3] - 100.;
	}
}

/**-----------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *-----------------------------------------------------------------------------------------------------------------*/
void MPU9250_Calibrate(I2C_HandleTypeDef * 	i2cHandle, float * accelBias, float * gyroBias)
{
  uint8_t writtenData;
	uint8_t tempBuff[12];
	uint16_t fifoCount, packetCount;
	int16_t gyroTemp[3] = {0, 0, 0}, accelTemp[3] = {0, 0, 0};
	uint16_t  gyroSensitivity  = 131;   	// 131 LSB/degrees/sec
	uint16_t  accelSensitivity = 16384;  	// 16384 LSB/g
	int32_t tempAccelBias[3] = {0, 0, 0}, tempGyroBias[3] = {0, 0, 0};
	
	/* Reset device. Write one to bit 7 reset bit; toggle reset device. */
	writtenData = 0x80;
	HAL_I2C_Mem_Write(i2cHandle, MPU9250_ADDRESS, MPU9250_PWR_MGMT_1, 1, &writtenData, 1, MPU9250_I2C_COM_TIMEOUT);
	osDelay(100);

	/* Get stable time source; Auto select clock source to be PLL gyroscope reference if ready else use the internal oscillator, bits 2:0 = 001 */
	writtenData = 0x01;
	HAL_I2C_Mem_Write(i2cHandle, MPU9250_ADDRESS, MPU9250_PWR_MGMT_1, 1, &writtenData, 1, MPU9250_I2C_COM_TIMEOUT);
	
	writtenData = 0x00;
	HAL_I2C_Mem_Write(i2cHandle, MPU9250_ADDRESS, MPU9250_PWR_MGMT_2, 1, &writtenData, 1, MPU9250_I2C_COM_TIMEOUT);
	osDelay(200);

	/* Configure device for bias calculation. */
	writtenData = 0x00;
	HAL_I2C_Mem_Write(i2cHandle, MPU9250_ADDRESS, MPU9250_INT_ENABLE, 1, &writtenData, 1, MPU9250_I2C_COM_TIMEOUT);			// Disable all interrupts
	
	writtenData = 0x00;
	HAL_I2C_Mem_Write(i2cHandle, MPU9250_ADDRESS, MPU9250_FIFO_EN, 1, &writtenData, 1, MPU9250_I2C_COM_TIMEOUT);				// Disable FIFO
	
	writtenData = 0x00;
	HAL_I2C_Mem_Write(i2cHandle, MPU9250_ADDRESS, MPU9250_PWR_MGMT_1, 1, &writtenData, 1, MPU9250_I2C_COM_TIMEOUT);			// Turn on internal clock source
	
	writtenData = 0x00;
	HAL_I2C_Mem_Write(i2cHandle, MPU9250_ADDRESS, MPU9250_I2C_MST_CTRL, 1, &writtenData, 1, MPU9250_I2C_COM_TIMEOUT);		// Disable I2C master
	
	writtenData = 0x00;
	HAL_I2C_Mem_Write(i2cHandle, MPU9250_ADDRESS, MPU9250_USER_CTRL, 1, &writtenData, 1, MPU9250_I2C_COM_TIMEOUT);			// Disable FIFO and I2C master modes
	
	writtenData = 0x0C;
	HAL_I2C_Mem_Write(i2cHandle, MPU9250_ADDRESS, MPU9250_USER_CTRL, 1, &writtenData, 1, MPU9250_I2C_COM_TIMEOUT);			// Reset FIFO and DMP
	osDelay(15);

	/* Configure MPU6050 gyro and accelerometer for bias calculation. */
	writtenData = 0x01;
	HAL_I2C_Mem_Write(i2cHandle, MPU9250_ADDRESS, MPU9250_CONFIG, 1, &writtenData, 1, MPU9250_I2C_COM_TIMEOUT);					// Set low-pass filter to 188 Hz
	
	writtenData = 0x00;
	HAL_I2C_Mem_Write(i2cHandle, MPU9250_ADDRESS, MPU9250_SMPLRT_DIV, 1, &writtenData, 1, MPU9250_I2C_COM_TIMEOUT);			// Set sample rate to 1 kHz
	
	writtenData = 0x00;
	HAL_I2C_Mem_Write(i2cHandle, MPU9250_ADDRESS, MPU9250_GYRO_CONFIG, 1, &writtenData, 1, MPU9250_I2C_COM_TIMEOUT);		// Set gyro full-scale to 250 degrees per second, maximum sensitivity
	
	writtenData = 0x00;
	HAL_I2C_Mem_Write(i2cHandle, MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, 1, &writtenData, 1, MPU9250_I2C_COM_TIMEOUT);		// Set accelerometer full-scale to 2 g, maximum sensitivity

	/* Configure FIFO to capture accelerometer and gyro data for bias calculation. */
	writtenData = 0x40;
	HAL_I2C_Mem_Write(i2cHandle, MPU9250_ADDRESS, MPU9250_USER_CTRL, 1, &writtenData, 1, MPU9250_I2C_COM_TIMEOUT);			// Enable FIFO
	
	writtenData = 0x78;
	HAL_I2C_Mem_Write(i2cHandle, MPU9250_ADDRESS, MPU9250_FIFO_EN, 1, &writtenData, 1, MPU9250_I2C_COM_TIMEOUT);				// Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
	osDelay(40);																																																			// Accumulate 40 samples in 40 milliseconds = 480 bytes

	/* At end of sample accumulation, turn off FIFO sensor read. */
	writtenData = 0x00;
	HAL_I2C_Mem_Write(i2cHandle, MPU9250_ADDRESS, MPU9250_FIFO_EN, 1, &writtenData, 1, MPU9250_I2C_COM_TIMEOUT);				// Disable gyro and accelerometer sensors for FIFO
	HAL_I2C_Mem_Read(i2cHandle, MPU9250_ADDRESS, MPU9250_FIFO_COUNTH, 1, &tempBuff[0], 2, MPU9250_I2C_COM_TIMEOUT);			// Read FIFO sample count
	fifoCount = ((uint16_t)tempBuff[0] << 8) | tempBuff[1];
	packetCount = fifoCount/12;																																													// How many sets of full gyro and accelerometer data for averaging

	for (uint16_t i=0; i<packetCount; i++)
	{
		HAL_I2C_Mem_Read(i2cHandle, MPU9250_ADDRESS, MPU9250_FIFO_R_W, 1, &tempBuff[0], 12, MPU9250_I2C_COM_TIMEOUT);

		/* Form signed 16-bit integer for each sample in FIFO. */
		accelTemp[0] = (int16_t)(((int16_t)tempBuff[0] << 8) | tempBuff[1]);
		accelTemp[1] = (int16_t)(((int16_t)tempBuff[2] << 8) | tempBuff[3]);
		accelTemp[2] = (int16_t)(((int16_t)tempBuff[4] << 8) | tempBuff[5]);
		gyroTemp[0]  = (int16_t)(((int16_t)tempBuff[6] << 8) | tempBuff[7]);
		gyroTemp[1]  = (int16_t)(((int16_t)tempBuff[8] << 8) | tempBuff[9]);
		gyroTemp[2]  = (int16_t)(((int16_t)tempBuff[10] << 8) | tempBuff[11]);

		/* Sum individual signed 16-bit biases to get accumulated signed 32-bit biases. */
		tempAccelBias[0] += (int32_t)(accelTemp[0]);
		tempAccelBias[1] += (int32_t)(accelTemp[1]);
		tempAccelBias[2] += (int32_t)(accelTemp[2]);
		tempGyroBias[0]  += (int32_t)(gyroTemp[0]);
		tempGyroBias[1]  += (int32_t)(gyroTemp[1]);
		tempGyroBias[2]  += (int32_t)(gyroTemp[2]);
	}

	/* Normalize sums to get average count biases. */
	tempAccelBias[0] /= (int32_t)packetCount;
	tempAccelBias[1] /= (int32_t)packetCount;
	tempAccelBias[2] /= (int32_t)packetCount;
	tempGyroBias[0]  /= (int32_t)packetCount;
	tempGyroBias[1]  /= (int32_t)packetCount;
	tempGyroBias[2]  /= (int32_t)packetCount;

	/* Remove gravity from the z-axis accelerometer bias calculation. */
	if(tempAccelBias[2] > 0L) {tempAccelBias[2] -= (int32_t) accelSensitivity;}
	else {tempAccelBias[2] += (int32_t) accelSensitivity;}

	/* Construct the gyro biases for pushing to the hardware gyro bias registers, which are reset to zero upon device startup. */
	tempBuff[0] = (-tempGyroBias[0]/4 >> 8) & 0xFF; 		// Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
	tempBuff[1] = (-tempGyroBias[0]/4) & 0xFF; 					// Biases are additive, so change sign on calculated average gyro biases
	tempBuff[2] = (-tempGyroBias[1]/4 >> 8) & 0xFF;
	tempBuff[3] = (-tempGyroBias[1]/4) & 0xFF;
	tempBuff[4] = (-tempGyroBias[2]/4 >> 8) & 0xFF;
	tempBuff[5] = (-tempGyroBias[2]/4) & 0xFF;

	/* Push gyro biases to hardware registers. */
	writtenData = tempBuff[0];
	HAL_I2C_Mem_Write(i2cHandle, MPU9250_ADDRESS, MPU9250_XG_OFFSET_H, 1, &writtenData, 1, MPU9250_I2C_COM_TIMEOUT);
	
	writtenData = tempBuff[1];
	HAL_I2C_Mem_Write(i2cHandle, MPU9250_ADDRESS, MPU9250_XG_OFFSET_L, 1, &writtenData, 1, MPU9250_I2C_COM_TIMEOUT);
	
	writtenData = tempBuff[2];
	HAL_I2C_Mem_Write(i2cHandle, MPU9250_ADDRESS, MPU9250_YG_OFFSET_H, 1, &writtenData, 1, MPU9250_I2C_COM_TIMEOUT);
	
	writtenData = tempBuff[3];
	HAL_I2C_Mem_Write(i2cHandle, MPU9250_ADDRESS, MPU9250_YG_OFFSET_L, 1, &writtenData, 1, MPU9250_I2C_COM_TIMEOUT);
	
	writtenData = tempBuff[4];
	HAL_I2C_Mem_Write(i2cHandle, MPU9250_ADDRESS, MPU9250_ZG_OFFSET_H, 1, &writtenData, 1, MPU9250_I2C_COM_TIMEOUT);
	
	writtenData = tempBuff[5];
	HAL_I2C_Mem_Write(i2cHandle, MPU9250_ADDRESS, MPU9250_ZG_OFFSET_L, 1, &writtenData, 1, MPU9250_I2C_COM_TIMEOUT);

	/* Output scaled gyro biases for display in the main program. */
	gyroBias[0] = (float) tempGyroBias[0]/(float) gyroSensitivity;
	gyroBias[1] = (float) tempGyroBias[1]/(float) gyroSensitivity;
	gyroBias[2] = (float) tempGyroBias[2]/(float) gyroSensitivity;

	/* Construct the accelerometer biases for pushing to the hardware accelerometer bias registers. */
	int32_t accelBiasReg[3] = {0, 0, 0}; // To hold the factory accelerometer trim biases
	
	/* Read factory accelerometer trim values. */
	HAL_I2C_Mem_Read(i2cHandle, MPU9250_ADDRESS, MPU9250_XA_OFFSET_H, 1, &tempBuff[0], 2, MPU9250_I2C_COM_TIMEOUT); 
	accelBiasReg[0] = (int32_t) (((int16_t)tempBuff[0] << 8) | tempBuff[1]);
	
	HAL_I2C_Mem_Read(i2cHandle, MPU9250_ADDRESS, MPU9250_YA_OFFSET_H, 1, &tempBuff[0], 2, MPU9250_I2C_COM_TIMEOUT);
	accelBiasReg[1] = (int32_t) (((int16_t)tempBuff[0] << 8) | tempBuff[1]);
	
	HAL_I2C_Mem_Read(i2cHandle, MPU9250_ADDRESS, MPU9250_ZA_OFFSET_H, 1, &tempBuff[0], 2, MPU9250_I2C_COM_TIMEOUT);
	accelBiasReg[2] = (int32_t) (((int16_t)tempBuff[0] << 8) | tempBuff[1]);

	uint32_t mask = 1uL;							// Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers.
	uint8_t maskBit[3] = {0, 0, 0};		// Define array to hold mask bit for each accelerometer bias axis.

	for(uint8_t i=0; i<3; i++)
	{
		/* If temperature compensation bit is set, record that fact in maskBit. */
		if((accelBiasReg[i] & mask)) maskBit[i] = 0x01;
	}

	/* Construct total accelerometer bias, including calculated average accelerometer bias from above. */
	/* Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale). */
	accelBiasReg[0] -= (tempAccelBias[0]/8);
	accelBiasReg[1] -= (tempAccelBias[1]/8);
	accelBiasReg[2] -= (tempAccelBias[2]/8);

	tempBuff[0] = (accelBiasReg[0] >> 8) & 0xFF;
	tempBuff[1] = (accelBiasReg[0]) & 0xFF;
	tempBuff[1] = tempBuff[1] | maskBit[0]; 				/* Preserve temperature compensation bit when writing back to accelerometer bias registers. */
	tempBuff[2] = (accelBiasReg[1] >> 8) & 0xFF;
	tempBuff[3] = (accelBiasReg[1]) & 0xFF;
	tempBuff[3] = tempBuff[3] | maskBit[1]; 				/* Preserve temperature compensation bit when writing back to accelerometer bias registers. */
	tempBuff[4] = (accelBiasReg[2] >> 8) & 0xFF;
	tempBuff[5] = (accelBiasReg[2]) & 0xFF;
	tempBuff[5] = tempBuff[5] | maskBit[2];				 	/* Preserve temperature compensation bit when writing back to accelerometer bias registers. */

	/* Push accelerometer biases to hardware registers. */
	writtenData = tempBuff[0];
	HAL_I2C_Mem_Write(i2cHandle, MPU9250_ADDRESS, MPU9250_XA_OFFSET_H, 1, &writtenData, 1, MPU9250_I2C_COM_TIMEOUT);
	
	writtenData = tempBuff[1];
	HAL_I2C_Mem_Write(i2cHandle, MPU9250_ADDRESS, MPU9250_XA_OFFSET_L, 1, &writtenData, 1, MPU9250_I2C_COM_TIMEOUT);
	
	writtenData = tempBuff[2];
	HAL_I2C_Mem_Write(i2cHandle, MPU9250_ADDRESS, MPU9250_YA_OFFSET_H, 1, &writtenData, 1, MPU9250_I2C_COM_TIMEOUT);
	
	writtenData = tempBuff[3];
	HAL_I2C_Mem_Write(i2cHandle, MPU9250_ADDRESS, MPU9250_YA_OFFSET_L, 1, &writtenData, 1, MPU9250_I2C_COM_TIMEOUT);
	
	writtenData = tempBuff[4];
	HAL_I2C_Mem_Write(i2cHandle, MPU9250_ADDRESS, MPU9250_ZA_OFFSET_H, 1, &writtenData, 1, MPU9250_I2C_COM_TIMEOUT);
	
	writtenData = tempBuff[5];
	HAL_I2C_Mem_Write(i2cHandle, MPU9250_ADDRESS, MPU9250_ZA_OFFSET_L, 1, &writtenData, 1, MPU9250_I2C_COM_TIMEOUT);

	/* Output scaled accel biases for display in the main program. */
	accelBias[0] = (float) tempAccelBias[0]/(float) accelSensitivity;
	accelBias[1] = (float) tempAccelBias[1]/(float) accelSensitivity;
	accelBias[2] = (float) tempAccelBias[2]/(float) accelSensitivity;
}

/**-----------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *-----------------------------------------------------------------------------------------------------------------*/
void AK8963_Init(I2C_HandleTypeDef * i2cHandle, float * calibData)
{
	volatile HAL_StatusTypeDef status;
	
  uint8_t writtenData;
	
  /* Extracted factory calibration data is hold. */
  uint8_t rawMagCalData[3];

  /* Power down magnetometer. */
  writtenData = 0x00;
  status = HAL_I2C_Mem_Write(i2cHandle, AK8963_ADDRESS, AK8963_CNTL, 1, &writtenData, 1, MPU9250_I2C_COM_TIMEOUT);
  osDelay(100);

	/* Enter Fuse ROM access mode. */
  writtenData = 0x0F;
  status = HAL_I2C_Mem_Write(i2cHandle, AK8963_ADDRESS, AK8963_CNTL, 1, &writtenData, 1, MPU9250_I2C_COM_TIMEOUT);
  osDelay(100);

	/* Read calibration values for each axes. */
  status = HAL_I2C_Mem_Read(i2cHandle, AK8963_ADDRESS, AK8963_ASAX, 1, &rawMagCalData[0], 3, MPU9250_I2C_COM_TIMEOUT);
  calibData[0] =  (float)(rawMagCalData[0] - 128)/256.0 + 1.0;
  calibData[1] =  (float)(rawMagCalData[1] - 128)/256.0 + 1.0;
  calibData[2] =  (float)(rawMagCalData[2] - 128)/256.0 + 1.0;

	/* Power down magnetometer. */
  writtenData = 0x00;
  HAL_I2C_Mem_Write(i2cHandle, AK8963_ADDRESS, AK8963_CNTL, 1, &writtenData, 1, MPU9250_I2C_COM_TIMEOUT);
  osDelay(100);

  /* Configure the magnetometer for continuous read and highest resolution. */
	/* Set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register. */
  /* Enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates. */
  writtenData = MFS_16BITS << 4 | 0x06;
  HAL_I2C_Mem_Write(i2cHandle, AK8963_ADDRESS, AK8963_CNTL, 1, &writtenData, 1, MPU9250_I2C_COM_TIMEOUT);
  osDelay(10);
}

/**-----------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *-----------------------------------------------------------------------------------------------------------------*/
void AK8963_Calibrate(MPU9250_Handle * MPU9250)
{
  uint16_t sampleCount = 0;
  int32_t magBias[3] = {0, 0, 0}, magScale[3] = {0, 0, 0};
  int16_t magMax[3] = {-32767, -32767, -32767}, magMin[3] = {32767, 32767, 32767};
		
	/* Wait for a short duration for magnetometer data. */
  osDelay(4000);
	sampleCount = 1500;  // At 100 Hz ODR, new mag data is available every 10 ms
   
	for(uint16_t i=0; i<sampleCount; i++)
	{
		/* Read the magnetometer data. */
    AK8963_GetMagnetometerData(MPU9250); 
		
    for (uint8_t j=0; j<3; j++)
		{
      if(MPU9250->rawSignedMag[j] > magMax[j])
				magMax[j] = MPU9250->rawSignedMag[j];
			
      if(MPU9250->rawSignedMag[j] < magMin[j])
				magMin[j] = MPU9250->rawSignedMag[j];
    }
		
		/* At 100 Hz ODR, new mag data is available every 10 ms. */
    osDelay(12);  
	}

	/* Get hard iron correction. */
	magBias[0] = (magMax[0] + magMin[0])/2;  // Get average x mag bias in counts
	magBias[1] = (magMax[1] + magMin[1])/2;  // Get average y mag bias in counts
	magBias[2] = (magMax[2] + magMin[2])/2;  // Get average z mag bias in counts
	
	/* Save mag biases in G for main program. */
	MPU9250->magBias[0] = (float) magBias[0]*MFS_16BITS*MPU9250->magCalibration[0];  
	MPU9250->magBias[1] = (float) magBias[1]*MFS_16BITS*MPU9250->magCalibration[1];
	MPU9250->magBias[2] = (float) magBias[2]*MFS_16BITS*MPU9250->magCalibration[2];
	
	/* Get soft iron correction estimate. */
	magScale[0] = (magMax[0] - magMin[0])/2;  // Get average x axis max chord length in counts
	magScale[1] = (magMax[1] - magMin[1])/2;  // Get average y axis max chord length in counts
	magScale[2] = (magMax[2] - magMin[2])/2;  // Get average z axis max chord length in counts
	
	float avgRad = magScale[0] + magScale[1] + magScale[2];
	avgRad /= 3.0;
	
	MPU9250->magScale[0] = avgRad/((float)magScale[0]);
	MPU9250->magScale[1] = avgRad/((float)magScale[1]);
	MPU9250->magScale[2] = avgRad/((float)magScale[2]);
}

/**-----------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *-----------------------------------------------------------------------------------------------------------------*/
void MPU9250_GetAccelerometerData(MPU9250_Handle * MPU9250)
{
  uint8_t rawAccelData[6];
	
	/* Read the six raw data registers into data array. */
  HAL_I2C_Mem_Read(MPU9250->i2cHandle, MPU9250_ADDRESS, MPU9250_ACCEL_XOUT_H, 1, &rawAccelData[0], 6, MPU9250_I2C_COM_TIMEOUT);
  
	MPU9250->rawSignedAccel[0] = ((int16_t)rawAccelData[0] << 8) | rawAccelData[1];  // Turn the MSB and LSB into a signed 16-bit value
  MPU9250->rawSignedAccel[1] = ((int16_t)rawAccelData[2] << 8) | rawAccelData[3];
  MPU9250->rawSignedAccel[2] = ((int16_t)rawAccelData[4] << 8) | rawAccelData[5];

	MPU9250->rawAccel.rawXData = (float)MPU9250->rawSignedAccel[0]*MPU9250_ACCEL_SCALE_FACTOR; 	// - accelBias[0];
	MPU9250->rawAccel.rawYData = (float)MPU9250->rawSignedAccel[1]*MPU9250_ACCEL_SCALE_FACTOR;	// - accelBias[1];
	MPU9250->rawAccel.rawZData = (float)MPU9250->rawSignedAccel[2]*MPU9250_ACCEL_SCALE_FACTOR;	// - accelBias[2];
}

/**-----------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *-----------------------------------------------------------------------------------------------------------------*/
void MPU9250_GetGyroscopeData(MPU9250_Handle * MPU9250)
{
  uint8_t rawGyroData[6];
	
	/* Read the six raw data registers sequentially into data array. */
  HAL_I2C_Mem_Read(MPU9250->i2cHandle, MPU9250_ADDRESS, MPU9250_GYRO_XOUT_H, 1, &rawGyroData[0], 6, MPU9250_I2C_COM_TIMEOUT);  
	
  MPU9250->rawSignedGyro[0] = ((int16_t)rawGyroData[0] << 8) | rawGyroData[1];  // Turn the MSB and LSB into a signed 16-bit value
  MPU9250->rawSignedGyro[1] = ((int16_t)rawGyroData[2] << 8) | rawGyroData[3];
  MPU9250->rawSignedGyro[2] = ((int16_t)rawGyroData[4] << 8) | rawGyroData[5];
	
	MPU9250->rawGyro.rawXData = (float)MPU9250->rawSignedGyro[0]*MPU9250_GYRO_SCALE_FACTOR;
	MPU9250->rawGyro.rawYData = (float)MPU9250->rawSignedGyro[1]*MPU9250_GYRO_SCALE_FACTOR;
	MPU9250->rawGyro.rawZData = (float)MPU9250->rawSignedGyro[2]*MPU9250_GYRO_SCALE_FACTOR;
}

/**-----------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *-----------------------------------------------------------------------------------------------------------------*/
void AK8963_GetMagnetometerData(MPU9250_Handle * MPU9250)
{
	uint8_t readData;
	uint8_t temp;
	uint8_t rawMagData[7];
	volatile HAL_StatusTypeDef status;
	
	status = HAL_I2C_Mem_Read(MPU9250->i2cHandle, AK8963_ADDRESS, AK8963_ST1, 1, &readData, 1, MPU9250_I2C_COM_TIMEOUT);
	
	if((readData & 0x01) == 0x01 )
	{
		/* Read the six raw data and ST2 registers sequentially into data array. */
		status = HAL_I2C_Mem_Read(MPU9250->i2cHandle, AK8963_ADDRESS, AK8963_XOUT_L, 1, &rawMagData[0], 7, MPU9250_I2C_COM_TIMEOUT);  
		temp = rawMagData[6];
		
		if(!(temp & 0x08))
		{
			MPU9250->rawSignedMag[0] = ((int16_t)rawMagData[1] << 8) | rawMagData[0];
			MPU9250->rawSignedMag[1] = ((int16_t)rawMagData[3] << 8) | rawMagData[2];
			MPU9250->rawSignedMag[2] = ((int16_t)rawMagData[5] << 8) | rawMagData[4];
	
			MPU9250->rawMag.rawXData = (float)MPU9250->rawSignedMag[0]*MPU9250_ACCEL_SCALE_FACTOR*MPU9250->magCalibration[0] - MPU9250->magBias[0];
			MPU9250->rawMag.rawYData = (float)MPU9250->rawSignedMag[1]*MPU9250_ACCEL_SCALE_FACTOR*MPU9250->magCalibration[1] - MPU9250->magBias[1];
			MPU9250->rawMag.rawZData = (float)MPU9250->rawSignedMag[2]*MPU9250_ACCEL_SCALE_FACTOR*MPU9250->magCalibration[2] - MPU9250->magBias[2];
			
			MPU9250->rawMag.rawXData *= MPU9250->magScale[0];
			MPU9250->rawMag.rawYData *=	MPU9250->magScale[1];
			MPU9250->rawMag.rawZData *=	MPU9250->magScale[2];
		}
	}
}



/**-----------------------------------------------------------------------------------------------------------------
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *-----------------------------------------------------------------------------------------------------------------*/
void MPU9250_GetAllData(MPU9250_Handle * MPU9250, AHRS_Handle	* AHRS)
{
	uint8_t intData;
	
	/* If intPin goes high, all data registers have new data. */
	HAL_I2C_Mem_Read(MPU9250->i2cHandle, MPU9250_ADDRESS, MPU9250_INT_STATUS, 1, &intData, 1, MPU9250_I2C_COM_TIMEOUT);
	
	if(intData & 0x01)
	{  
		MPU9250_GetAccelerometerData(MPU9250);
		MPU9250_GetGyroscopeData(MPU9250);
		AK8963_GetMagnetometerData(MPU9250);
	}
	
	/* Sensors x (y)-axis of the accelerometer/gyro is aligned with the y (x)-axis of the magnetometer;
	   the magnetometer z-axis (+ down) is misaligned with z-axis (+ up) of accelerometer and gyro.
	   Since MPU9250's mag. and IMU modules are different and seperate (AK8963 and MPU6050), their...
	   ...coordinate systems also different. So, to compensate this, order should be my - mx - mz. */
	
	AHRS_AxisData tempAccel = { .xData = MPU9250->rawAccel.rawXData,
															.yData = MPU9250->rawAccel.rawYData,
															.zData = MPU9250->rawAccel.rawZData,};
	
	AHRS_AxisData	tempGyro 	= { .xData = DEGREE_TO_RADIAN(MPU9250->rawGyro.rawXData), 
															.yData = DEGREE_TO_RADIAN(MPU9250->rawGyro.rawYData), 
															.zData = DEGREE_TO_RADIAN(MPU9250->rawGyro.rawZData) };
	
	AHRS_AxisData	tempMag 	= { .xData = MPU9250->rawMag.rawYData, 
															.yData = MPU9250->rawMag.rawXData,
															.zData = MPU9250->rawMag.rawXData };
	
	/* Calculate quaternions based on Madgwick's filter. */
	AHRS_GetMadgwickQuaternion(&tempAccel, &tempGyro, &tempMag, &(AHRS->quaternions));
	
	/* Convert quaternions to Euler angles. */
	AHRS_QuaternionToEulerAngles(&(AHRS->quaternions), &(AHRS->eulerAngles));
}
