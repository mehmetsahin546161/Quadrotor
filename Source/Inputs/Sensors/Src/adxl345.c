#include "adxl345.h"


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  None
  * @param  None
  * @param  None
  * @retval None
  */
void ADXL345_Init(void)
{
	/* Add thread related operations */
}

/**
  * @brief  		The DEVID register holds a fixed device ID code of 0xE5
  * @param[IN]  ADXL345 Sensor handler
  * @retval 		None
  */
void ADXL345_WhoAmI(ADXL345_HandleTypeDef * ADXL345)
{
	//Mesaji queueye koy setle
	//COM_Input_SendThread de ise bekle. mesaj olunca al. IT li fonksiyonu çagir.  onunda callbackinde flagi setle. woow bu kisimi state machine yaparsam iyi olur. 
	//Return type da ekle
	//Timeout da ekle.
	
	//states
	//IDLE
	//MESSAGE REQUEST IS ACCEPTED
	//MESSAGE IS STARTED TO PROCESS
	//TIMEOUT VEYA MESAGE IS COMPLETED 
	//gibi gibi falan yaani.
	
	
	
	
	
	//uint8_t buff;
	//
	//HAL_I2C_Master_Receive(	ADXL345->i2cHandle, 
	//												ADXL345->i2cDevAddr << 1,
	//												&buff,
	//												1,
	//												100);
	//
	//return buff;
}

 
/**
  * @brief  		The THRESH_TAP register is eight bits and holds the threshold value for tap interrupts. 
	*							The data format is unsigned, so the magnitude of the tap event is compared with the value in THRESH_TAP.
	*							The scale factor is 62.5 mg/LSB (that is, 0xFF = +16 g).
	*	@param[IN]  ADXL345 		Sensor handler.
  * @param[IN]  dTapThresh	Minimum acceleration value represented in mg unit to create tap interrupt.
  * @retval 		None
  */
void ADXL345_SetTapThreshold(ADXL345_HandleTypeDef * ADXL345, double tapThresh)
{
	if( (tapThresh>=0) && (tapThresh<=16) )
	{
	
		uint8_t regVal = (1/TAP_THRESH_SCALE_FACTOR)*tapThresh;
		
		HAL_I2C_Mem_Write( ADXL345->i2cHandle, 
											 ADXL345->i2cDevAddr << 1,
											 THRESH_TAP,
											 I2C_MEMADD_SIZE_8BIT,
											 &regVal,
											 1,
											 100 );
	}
}

/**
  * @brief  		The THRESH_TAP register is eight bits and holds the threshold value for tap interrupts. 
	*							The data format is unsigned, so the magnitude of the tap event is compared with the value in THRESH_TAP.
	*							The scale factor is 62.5 mg/LSB (that is, 0xFF = +16 g).
	*	@param[IN]  ADXL345 Sensor handler.
  * @retval 		Minimum acceleration value represented in mg unit to create tap interrupt.
  */
double ADXL345_GetTapThreshold(ADXL345_HandleTypeDef * ADXL345)
{
	uint8_t regVal = 0;
	
	HAL_I2C_Mem_Read( ADXL345->i2cHandle,
										ADXL345->i2cDevAddr << 1, 
										THRESH_TAP,
										I2C_MEMADD_SIZE_8BIT,
										&regVal,
										1,
										100 );
	
	return regVal*TAP_THRESH_SCALE_FACTOR;

}

/**
  * @brief  		The OFSX, OFSY, and OFSZ registers are each eight bits and offer user-set offset adjustments in twos complement format 
	*							with a scale factor of 15.6 mg/LSB (that is, 0x7F = +2 g)
	*	@note				2'complement simply inverts each bit of given binary number, which will be 01010001. Then add 1 to the LSB of this result.
	*	@param[IN]  ADXL345 	Sensor handler.
  * @param[IN]  dOffset		Offset value represented in mg unit.
  * @retval 		None
  */
void ADXL345_SetOffset(ADXL345_HandleTypeDef * ADXL345, ADXL345_Axis axis, double offset)
{
	if( (offset<=2) && (offset>=-2) )
	{
		uint8_t regVal = 	(offset<0) ? (uint8_t)(~((uint8_t)(-1*offset*(1/OFFSET_SCALE_FACTOR)))) + 1 :
											(offset>0) ? (offset*(1/OFFSET_SCALE_FACTOR))	:	0;
		
		uint8_t memAddress = 	(axis==X_AXIS) ? OFSX :
													(axis==Y_AXIS) ? OFSY :
													(axis==Z_AXIS) ? OFSZ	:	INVALID_DATA;
		
		if(memAddress != INVALID_DATA)
		{
			HAL_I2C_Mem_Write( 	ADXL345->i2cHandle, 
													ADXL345->i2cDevAddr << 1,
													memAddress,
													I2C_MEMADD_SIZE_8BIT,
													&regVal,
													1,
													100 );
		}
	}
}

/**
  * @brief  		The OFSX, OFSY, and OFSZ registers are each eight bits and offer user-set offset adjustments in twos complement format 
	*							with a scale factor of 15.6 mg/LSB (that is, 0x7F = +2 g)
	*	@note				2'complement simply inverts each bit of given binary number, which will be 01010001. Then add 1 to the LSB of this result
	*	@param[IN]  ADXL345 	Sensor handler.
  * @param[IN]  eAxis 		The axes whose offset is queued
  * @retval 		Offset value represented in mg unit.
  */
double	ADXL345_GetOffset(ADXL345_HandleTypeDef * ADXL345, ADXL345_Axis axis)
{
	uint8_t regVal = 0;
	
	uint8_t memAddress = 	(axis==X_AXIS) ? OFSX :
												(axis==Y_AXIS) ? OFSY :
												(axis==Z_AXIS) ? OFSZ	:	INVALID_DATA;
		
	if(memAddress != INVALID_DATA)
	{
		HAL_I2C_Mem_Read( ADXL345->i2cHandle,
											ADXL345->i2cDevAddr << 1, 
											memAddress,
											I2C_MEMADD_SIZE_8BIT,
											&regVal,
											1,
											100 );
	
		/* Check MSB bit to find the sign of the value. */
		double offsetVal = 	((regVal) & (1<<ADXL345_REG_MSB_BIT)) ?
													(-1*( (uint8_t)( ~(regVal-1) )*OFFSET_SCALE_FACTOR))	:			// Negative number
													(regVal*OFFSET_SCALE_FACTOR);																// Positive number
		
		return offsetVal;
	}
	
	return -61;
}

/**
  * @brief  		The DUR register is eight bits and contains an unsigned time value representing 
	*							the maximum time that an event must be above the THRESH_TAP threshold to qualify as a tap event.
	*							The scale factor is 625 µs/LSB.
	*	@param[IN]  ADXL345 		Sensor handler.
  * @param[IN]  u4MaxTapDur Maximum tap duration represented in microsecond unit.
  * @retval 		None
  */
void ADXL345_SetMaxTapDuration(ADXL345_HandleTypeDef * ADXL345, uint32_t maxTapDur)
{
	uint8_t	regVal = maxTapDur/DUR_TIME_SCALE_FACTOR;
	
	//HAL_I2C_Mem_Write(	ADXL345->i2cHandle, 
	//										ADXL345->i2cDevAddr << 1,
	//										DUR,
	//										I2C_MEMADD_SIZE_8BIT,
	//										&regVal,
	//										1,
	//										100 );
	
	HAL_I2C_Mem_Write_DMA (	ADXL345->i2cHandle,
													ADXL345->i2cDevAddr << 1,
													DUR,
													I2C_MEMADD_SIZE_8BIT,
													&regVal, 
													1 );
	
	
	
}

/**
  * @brief  		The DUR register is eight bits and contains an unsigned time value representing 
	*							the maximum time that an event must be above the THRESH_TAP threshold to qualify as a tap event.
	*							The scale factor is 625 µs/LSB.
	*	@param[IN]  ADXL345 	Sensor handler.
  * @retval 		Maximum tap duration represented in microsecond unit.
  */
uint32_t ADXL345_GetMaxTapDuration(ADXL345_HandleTypeDef * ADXL345)
{
	//HAL_I2C_Mem_Read( ADXL345->i2cHandle,
	//									ADXL345->i2cDevAddr << 1, 
	//									DUR,
	//									I2C_MEMADD_SIZE_8BIT,
	//									&regVal,
	//									1,
	//									100 );
	//
	//return regVal*DUR_TIME_SCALE_FACTOR;
	
	HAL_I2C_Mem_Read_DMA(	ADXL345->i2cHandle, 
												ADXL345->i2cDevAddr << 1,
												DUR, 
												I2C_MEMADD_SIZE_8BIT,
												&ADXL345->memRead, 
												1);
	
	//TODO:Only for debug
	return 0;
}

/**
  * @brief  		The latent register is eight bits and contains an unsigned time value representing 
	*							the wait time from the detection of a tap event to the start of the time window (defined by the window register) 
	*							during which a possible second tap event can be detected. 
	*							The scale factor is 1.25 ms/LSB.
	*	@param[IN]  ADXL345 	Sensor handler.
  * @param[IN]  dLatTime	Latency time represented in milisecond unit
  * @retval 		None
  */
void ADXL345_SetLatencyTime(ADXL345_HandleTypeDef * ADXL345, double latTime)
{
	uint8_t	regVal = latTime/LATENT_TIME_SCALE_FACTOR;
	
	HAL_I2C_Mem_Write(	ADXL345->i2cHandle, 
											ADXL345->i2cDevAddr << 1,
											LATENT,
											I2C_MEMADD_SIZE_8BIT,
											&regVal,
											1,		
											100 );
}

/**
  * @brief  		The latent register is eight bits and contains an unsigned time value representing 
	*							the wait time from the detection of a tap event to the start of the time window (defined by the window register) 
	*							during which a possible second tap event can be detected. 
	*							The scale factor is 1.25 ms/LSB.
	*	@param[IN]  ADXL345 Sensor handler.
  * @retval 		Latency time represented in milisecond unit
  */
double ADXL345_GetLatencyTime(ADXL345_HandleTypeDef * ADXL345)
{
	uint8_t regVal = 0;
	
	HAL_I2C_Mem_Read( ADXL345->i2cHandle,
										ADXL345->i2cDevAddr << 1, 
										LATENT,
										I2C_MEMADD_SIZE_8BIT,
										&regVal,
										1,
										100 );
	
	return regVal*LATENT_TIME_SCALE_FACTOR;
}

/**
  * @brief  		The window register is eight bits and contains an unsigned time value representing 
	*							the amount of time after the expiration of the latency time (determined by the latent register)
	*							during which a second valid tap can begin.
	*							The scale factor is 1.25 ms/LSB.
	*	@param[IN]  ADXL345 	Sensor handler.
  * @param[IN]  dWinTime	Window time represented in milisecond unit
  * @retval 		None
  */
void ADXL345_SetWindowTime(ADXL345_HandleTypeDef * ADXL345, double winTime)
{
	uint8_t	regVal = winTime/WINDOW_TIME_SCALE_FACTOR;
	
	HAL_I2C_Mem_Write(	ADXL345->i2cHandle, 
											ADXL345->i2cDevAddr << 1,
											WINDOW,
											I2C_MEMADD_SIZE_8BIT,
											&regVal,
											1,		
											100 );
}

/**
  * @brief  		The window register is eight bits and contains an unsigned time value representing 
	*							the amount of time after the expiration of the latency time (determined by the latent register)
	*							during which a second valid tap can begin.
	*							The scale factor is 1.25 ms/LSB.
	*	@param[IN]  ADXL345 Sensor handler.
  * @retval 		Window time represented in milisecond unit
  */
double ADXL345_GetWindowTime(ADXL345_HandleTypeDef * ADXL345)
{
	uint8_t regVal = 0;
	
	HAL_I2C_Mem_Read( ADXL345->i2cHandle,
										ADXL345->i2cDevAddr << 1, 
										WINDOW,
										I2C_MEMADD_SIZE_8BIT,
										&regVal,
										1,
										100 );
	
	return regVal*WINDOW_TIME_SCALE_FACTOR;
}

/**
  * @brief  		The THRESH_ACT register is eight bits and holds the threshold value for detecting activity.
	*							The data format is unsigned, so the magnitude of the activity event is compared with the value in the THRESH_ACT register.
	*							The scale factor is 62.5 mg/LSB.
	*	@param[IN]  ADXL345 		Sensor handler.
  * @param[IN]  dActThresh	Minimum threshold value for detecting activity.
  * @retval 		None
  */
void ADXL345_SetActivityThreshold(ADXL345_HandleTypeDef * ADXL345, double dActThresh)
{
	uint8_t	regVal = dActThresh/ACTIVITY_THRESH_SCALE_FACTOR;
	
	HAL_I2C_Mem_Write(	ADXL345->i2cHandle, 
											ADXL345->i2cDevAddr << 1,
											THRESH_ACT,
											I2C_MEMADD_SIZE_8BIT,
											&regVal,
											1,		
											100 );
}

/**
  * @brief  		The THRESH_ACT register is eight bits and holds the threshold value for detecting activity.
	*							The data format is unsigned, so the magnitude of the activity event is compared with the value in the THRESH_ACT register.
	*							The scale factor is 62.5 mg/LSB.
	*	@param[IN]  ADXL345 Sensor handler.
  * @retval 		Minimum threshold value for detecting activity.
  */
double ADXL345_GetActivityThreshold(ADXL345_HandleTypeDef * ADXL345)
{
	uint8_t regVal = 0;
	
	HAL_I2C_Mem_Read( ADXL345->i2cHandle,
										ADXL345->i2cDevAddr << 1, 
										WINDOW,
										I2C_MEMADD_SIZE_8BIT,
										&regVal,
										1,
										100 );
	
	return regVal*WINDOW_TIME_SCALE_FACTOR;
}

/**
  * @brief  		The THRESH_INACT register is eight bits and holds the threshold value for detecting inactivity.
	*							The data format is unsigned, so the magnitude of the inactivity event is compared with the value in the THRESH_INACT register.
	*							The scale factor is 62.5 mg/LSB.
	*	@param[IN]  ADXL345 			Sensor handler.
  * @param[IN]  dInactThresh	Minimum threshold value for detecting activity.
  * @retval 		None
  */
void ADXL345_SetInactivityThreshold(ADXL345_HandleTypeDef * ADXL345, double dInactThresh)
{
	uint8_t	regVal = dInactThresh/INACTIVITY_THRESH_SCALE_FACTOR;
	
	HAL_I2C_Mem_Write(	ADXL345->i2cHandle, 
											ADXL345->i2cDevAddr << 1,
											THRESH_INACT,
											I2C_MEMADD_SIZE_8BIT,
											&regVal,
											1,		
											100 );
}

/**
  * @brief  		The THRESH_INACT register is eight bits and holds the threshold value for detecting inactivity.
	*							The data format is unsigned, so the magnitude of the inactivity event is compared with the value in the THRESH_INACT register.
	*							The scale factor is 62.5 mg/LSB.
	*	@param[IN]  ADXL345 Sensor handler.
  * @retval 		Minimum threshold value for detecting inactivity.
  */
double ADXL345_GetInactivityThreshold(ADXL345_HandleTypeDef * ADXL345)
{
	uint8_t regVal = 0;
	
	HAL_I2C_Mem_Read( ADXL345->i2cHandle,
										ADXL345->i2cDevAddr << 1, 
										THRESH_INACT,
										I2C_MEMADD_SIZE_8BIT,
										&regVal,
										1,
										100 );
	
	return regVal*INACTIVITY_THRESH_SCALE_FACTOR;
}

/**
  * @brief  		The TIME_INACT register is eight bits and contains an unsigned time value representing the amount of time that 
	*							acceleration must be less than the value in the THRESH_INACT register for inactivity to be declared.
	*							The scale factor is 1 sec/LSB.
	*							Unlike the other interrupt functions, which use unfiltered data (see the Threshold section), the inactivity function uses filtered output data.
	*							At least one output sample must be generated for the inactivity interrupt to be triggered.
	*							This results in the function appearing unresponsive if the TIME_INACT register is set to a value less than the time constant of the output data rate.
	*	@param[IN]  ADXL345 			Sensor handler.
  * @param[IN]  dMinInactTime	Minimum time value for detecting inactivity.
  * @retval 		None
  */
void ADXL345_SetInactivityTime(ADXL345_HandleTypeDef * ADXL345, double dMinInactTime)
{
	uint8_t	regVal = dMinInactTime/TIME_INACTIVITY_SCALE_FACTOR;
	
	HAL_I2C_Mem_Write(	ADXL345->i2cHandle, 
											ADXL345->i2cDevAddr << 1,
											TIME_INACT,
											I2C_MEMADD_SIZE_8BIT,
											&regVal,
											1,		
											100 );
}

/**
  * @brief  		The TIME_INACT register is eight bits and contains an unsigned time value representing the amount of time that 
	*							acceleration must be less than the value in the THRESH_INACT register for inactivity to be declared.
	*							The scale factor is 1 sec/LSB.
	*							Unlike the other interrupt functions, which use unfiltered data (see the Threshold section), the inactivity function uses filtered output data.
	*							At least one output sample must be generated for the inactivity interrupt to be triggered.
	*							This results in the function appearing unresponsive if the TIME_INACT register is set to a value less than the time constant of the output data rate.
	*	@param[IN]  ADXL345 Sensor handler.
  * @retval 		Minimum time value for detecting inactivity.
  */
double ADXL345_GetInactivityTime(ADXL345_HandleTypeDef * ADXL345)
{
	uint8_t regVal = 0;
	
	HAL_I2C_Mem_Read( ADXL345->i2cHandle,
										ADXL345->i2cDevAddr << 1, 
										TIME_INACT,
										I2C_MEMADD_SIZE_8BIT,
										&regVal,
										1,
										100 );
	
	return regVal*TIME_INACTIVITY_SCALE_FACTOR;
}

/**
  * @brief  		Configures the interrupts.
	*							Writing corresponding bits TRUE enables interrupt and FALSE means disables interrupt.
	*	@param[IN]  ADXL345 	Sensor handler.
	*	@param[IN]  pIntReg 	Contains which interrupt sources are enabled or disabled.
  * @retval 		None
  */
void ADXL345_ConfigInterrupts(ADXL345_HandleTypeDef * ADXL345, ADXL345_InterruptReg intReg)
{
	uint8_t regVal = intReg.BYTE;
	
	HAL_I2C_Mem_Write( ADXL345->i2cHandle, 
										 ADXL345->i2cDevAddr << 1,
										 INT_ENABLE,
										 I2C_MEMADD_SIZE_8BIT,
										 &regVal,
										 1,		
										 100 );
}




/**
  * @brief  		Reads the content of interrupt source register
	*	@param[IN]  ADXL345 Sensor handler.
  * @retval 		Status of interrupt bits.
  */
ADXL345_InterruptReg ADXL345_GetInterruptStatus(ADXL345_HandleTypeDef * ADXL345)
{
	uint8_t regVal =  0;
	
	HAL_I2C_Mem_Read( ADXL345->i2cHandle,
										ADXL345->i2cDevAddr << 1, 
										INT_SOURCE,
										I2C_MEMADD_SIZE_8BIT,
										&regVal,
										1,
										100 );
	
	ADXL345_InterruptReg intReg = {.BYTE=regVal};
	
	return intReg;
}



/**
  * @brief  		Configures which interrupts trigger which pins.
	*							Any bits set to 0 in this register send their respective interrupts to the INT1 pin,
	*							whereas bits set to 1 send their respective interrupts to the INT2 pin.
	*	@param[IN]  ADXL345 	Sensor handler.
	*	@param[IN]  pinMap		Set bits of index of zero represents INT1 pins.
	*												Set bits of index of one  represents INT2 pins.
  * @retval 		None
  */
void ADXL345_MapInterruptPins(ADXL345_HandleTypeDef * ADXL345, ADXL345_InterruptReg pinMap[ADXL345_INT_PIN_CNT])
{
	/* It is recommended that interrupt bits be configured with the interrupts disabled,
		 preventing interrupts from being accidentally triggered during configuration.
		 This can be done by writing a value of 0x00 to the INT_ENABLE register.*/
	ADXL345_InterruptReg unIntReg = {.BYTE = RESET_ALL_INTERRUPTS};
	ADXL345_ConfigInterrupts(ADXL345, unIntReg);

	uint8_t regVal = (~pinMap[ADXL345_INT1_PIN].BYTE) |
										( pinMap[ADXL345_INT2_PIN].BYTE);
	
	HAL_I2C_Mem_Write( 	ADXL345->i2cHandle, 
											ADXL345->i2cDevAddr << 1,
											INT_MAP,
											I2C_MEMADD_SIZE_8BIT,
											&regVal,
											1,		
											100 );
}

/**
  * @brief  		
	*	@param[IN]  ADXL345 	Sensor handler.
  * @retval 		
  */
ADXL345_RawDatas	ADXL345_GetRawDatasPolling(ADXL345_HandleTypeDef * ADXL345)
{
	//ADXL345_RawDatas sRawDatas = {	.rawXData = ADXL345_AXIS_DATAS_NOT_READY,
	//																.rawYData = ADXL345_AXIS_DATAS_NOT_READY,
	//																.rawZData = ADXL345_AXIS_DATAS_NOT_READY };
	//
	//ADXL345_InterruptReg intReg = ADXL345_GetInterruptStatus(ADXL345);
	//
	//if(intReg.BIT.dataReady == true)
	//{
	//	// TODO:BURADA X Y Z IÇIN AYRI OKUM KISIMLARINI DA EKLE
	//}
	//
	
	//TODO:Only for debug
	ADXL345_RawDatas ss;
	return ss;
}

/**
  * @brief  		
	*	@param[IN]  ADXL345 	Sensor handler.
  * @retval 		
  */
ADXL345_RawDatas	ADXL345_GetRawDatasIT(ADXL345_HandleTypeDef * ADXL345)
{
	//ADXL345_RawDatas sRawDatas = {	.rawXData = ADXL345_AXIS_DATAS_NOT_READY,
	//																.rawYData = ADXL345_AXIS_DATAS_NOT_READY,
	//																.rawZData = ADXL345_AXIS_DATAS_NOT_READY };
	//
	//ADXL345_InterruptReg intReg = ADXL345_GetInterruptStatus(ADXL345);
	//
	//if(intReg.BIT.dataReady == true)
	//{
	//	// TODO:BURADA X Y Z IÇIN AYRI OKUM KISIMLARINI DA EKLE
	//}
	//
	
	//TODO:Only for debug
	ADXL345_RawDatas ss;
	return ss;
}