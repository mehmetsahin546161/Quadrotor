#include 	"adxl345.h"
#include 	"cmsis_os2.h"
#include 	"com_interface.h"
#include	"defines.h"
#include 	"calc.h"
#include 	"string.h"

/* Private macro -------------------------------------------------------------*/
#define ADXL345_SINGLE_BYTE_WRITE_CNT 				(5)
#define ADXL345_SINGLE_BYTE_READ_CNT 					(5)

#define ADXL345_MULTIPLE_BYTE_WRITE_CNT(X)		(ADXL345_SINGLE_BYTE_WRITE_CNT + X)
#define ADXL345_MULTIPLE_BYTE_READ_CNT(X) 		(ADXL345_SINGLE_BYTE_READ_CNT + X)

#define ADXL345_ACCEL_BIAS_CALC_ITERATION			10
#define ADXL345_GRAVITY_ACCELERATION					1			/* g */
/* Private types -------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/



/* Exported functions --------------------------------------------------------*/

/**------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief  		Sets basic functionalities.
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  *------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void ADXL345_InitSensor(COM_Handle * ADXL345)
{
	const ADXL345_DataFormatReg setDataFormat = 
	{ 
		.BIT.range = ADXL345_16G_RANGE,
		.BIT.justify = ADXL345_RIGHT_JUSTIFIED, 
		.BIT.fullRes = ADXL345_FULL_RES_DISABLED,
		.BIT.intInvert = ADXL345_INTERRUPT_ACTIVE_HIGH
	};
	ADXL345_SetDataFormat(ADXL345, &setDataFormat);
	
	const ADXL345_PowerCtrReg setPowerControl = 
	{
		.BIT.sleep = ADXL345_NORMAL_MODE,
		.BIT.meausure = ADXL345_MEASUREMENT_MODE
	};
	ADXL345_SetPowerControl(ADXL345, &setPowerControl);
}

/**------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief  		The DEVID register holds a fixed device ID code of 0xE5
  * @param[IN]  ADXL345 Sensor handler
  * @retval 		None
  *------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
uint8_t ADXL345_WhoAmI(COM_Handle * ADXL345)
{
	COM_I2C_DATA(ADXL345).dataSize = 1;
	COM_I2C_DATA(ADXL345).memAddress = ADXL345_DEVID_ADDR;
	COM_I2C_DATA(ADXL345).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	COM_RegisterGetter(ADXL345);
											
	return COM_I2C_DATA(ADXL345).data[0];
}

/**------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief  		The THRESH_TAP register is eight bits and holds the threshold value for tap interrupts. 
	*							The data format is unsigned, so the magnitude of the tap event is compared with the value in THRESH_TAP.
	*							The scale factor is 62.5 mg/LSB (that is, 0xFF = +16 g).
	*	@param[IN]  ADXL345 		Sensor handler.
  * @param[IN]  dTapThresh	Minimum acceleration value represented in mg unit to create tap interrupt.
  * @retval 		None
  *------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void ADXL345_SetTapThreshold(COM_Handle * ADXL345, double tapThresh)
{
	if( (tapThresh>=0) && (tapThresh<=16000) )
	{
		uint8_t sendVal = (1.0/TAP_THRESH_SCALE_FACTOR)*tapThresh;
		
		COM_I2C_DATA(ADXL345).data[0] = sendVal;
		COM_I2C_DATA(ADXL345).dataSize = 1;
		COM_I2C_DATA(ADXL345).memAddress = ADXL345_THRESH_TAP_ADDR;
		COM_I2C_DATA(ADXL345).memAddSize = I2C_MEMADD_SIZE_8BIT;
		
		COM_RegisterSetter(ADXL345);
	}
}

/**------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief  		The THRESH_TAP register is eight bits and holds the threshold value for tap interrupts. 
	*							The data format is unsigned, so the magnitude of the tap event is compared with the value in THRESH_TAP.
	*							The scale factor is 62.5 mg/LSB (that is, 0xFF = +16 g).
	*	@param[IN]  ADXL345 Sensor handler.
  * @retval 		Minimum acceleration value represented in mg unit to create tap interrupt.
  *------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
double ADXL345_GetTapThreshold(COM_Handle * ADXL345)
{
	COM_I2C_DATA(ADXL345).dataSize = 1;
	COM_I2C_DATA(ADXL345).memAddress = ADXL345_THRESH_TAP_ADDR;
	COM_I2C_DATA(ADXL345).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	COM_RegisterGetter(ADXL345);
											
	return (COM_I2C_DATA(ADXL345).data[0])*TAP_THRESH_SCALE_FACTOR;
}

/**------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief  		The OFSX, OFSY, and OFSZ registers are each eight bits and offer user-set offset adjustments in twos complement format 
	*							with a scale factor of 15.6 mg/LSB (that is, 0x7F = +2 g)
	*	@note				2'complement simply inverts each bit of given binary number, which will be 01010001. Then add 1 to the LSB of this result.
	*	@param[IN]  ADXL345 	Sensor handler.
  * @param[IN]  dOffset		Offset value represented in mg unit.
  * @retval 		None
  *------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void ADXL345_SetOffset(COM_Handle * ADXL345, ADXL345_Axis axis, double offset)
{
	if( (offset<=2000) && (offset>=-2000) )
	{
		uint8_t sendVal = (offset<0) ? (uint8_t)(~((uint8_t)(-1*offset*(1.0/OFFSET_SCALE_FACTOR)))) + 1 :
											(offset>0) ? (offset*(1.0/OFFSET_SCALE_FACTOR))	:	0;
		
		uint8_t memAddress = (axis==ADXL345_X_AXIS) ? ADXL345_OFSX_ADDR :
												 (axis==ADXL345_Y_AXIS) ? ADXL345_OFSY_ADDR :
												 (axis==ADXL345_Z_AXIS) ? ADXL345_OFSZ_ADDR :
																					INVALID_DATA;
		
		if(memAddress != INVALID_DATA)
		{
			COM_I2C_DATA(ADXL345).data[0] = sendVal;
			COM_I2C_DATA(ADXL345).dataSize = 1;
			COM_I2C_DATA(ADXL345).memAddress = memAddress;
			COM_I2C_DATA(ADXL345).memAddSize = I2C_MEMADD_SIZE_8BIT;
			
			COM_RegisterSetter(ADXL345);
		}
	}
}

/**------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief  		The OFSX, OFSY, and OFSZ registers are each eight bits and offer user-set offset adjustments in twos complement format 
	*							with a scale factor of 15.6 mg/LSB (that is, 0x7F = +2 g)
	*	@param[IN]  ADXL345 	Sensor handler.
  * @param[IN]  eAxis 		The axes whose offset is queued
  * @retval 		Offset value represented in mg unit.
  *------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
double ADXL345_GetOffset(COM_Handle * ADXL345, ADXL345_Axis axis)
{
	uint8_t memAddress = 	(axis==ADXL345_X_AXIS) ? ADXL345_OFSX_ADDR :
												(axis==ADXL345_Y_AXIS) ? ADXL345_OFSY_ADDR :
												(axis==ADXL345_Z_AXIS) ? ADXL345_OFSZ_ADDR	: INVALID_DATA;
		
	if(memAddress != INVALID_DATA)
	{
		COM_I2C_DATA(ADXL345).dataSize = 1;
		COM_I2C_DATA(ADXL345).memAddress = memAddress;
		COM_I2C_DATA(ADXL345).memAddSize = I2C_MEMADD_SIZE_8BIT;
		
		COM_RegisterGetter(ADXL345);
																				
		/* Check MSB bit to find the sign of the value. */
		double offsetVal = 	((COM_I2C_DATA(ADXL345).data[0]) & (1<<ADXL345_REG_MSB_BIT)) ?
													(-1.0*( (uint8_t)( ~(COM_I2C_DATA(ADXL345).data[0]-1) )*OFFSET_SCALE_FACTOR))	:			// Negative number
													(COM_I2C_DATA(ADXL345).data[0]*OFFSET_SCALE_FACTOR);																// Positive number
		
		
		
		return offsetVal;
	}
	
	return INVALID_DATA;
}

/**------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief  		The DUR register is eight bits and contains an unsigned time value representing 
	*							the maximum time that an event must be above the THRESH_TAP threshold to qualify as a tap event.
	*							The scale factor is 625 µs/LSB.
	*	@param[IN]  ADXL345 		Sensor handler.
  * @param[IN]  u4MaxTapDur Maximum tap duration represented in microsecond unit.
  * @retval 		None
  *------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void ADXL345_SetMaxTapDuration(COM_Handle * ADXL345, uint32_t maxTapDur)
{
	uint8_t	sendVal = maxTapDur/DUR_TIME_SCALE_FACTOR;
	
	COM_I2C_DATA(ADXL345).data[0] = sendVal;
	COM_I2C_DATA(ADXL345).dataSize = 1;
	COM_I2C_DATA(ADXL345).memAddress = ADXL345_DUR_ADDR;
	COM_I2C_DATA(ADXL345).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	COM_RegisterSetter(ADXL345);
}

/**------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief  		The DUR register is eight bits and contains an unsigned time value representing 
	*							the maximum time that an event must be above the THRESH_TAP threshold to qualify as a tap event.
	*							The scale factor is 625 µs/LSB.
	*	@param[IN]  ADXL345 	Sensor handler.
  * @retval 		Maximum tap duration represented in microsecond unit.
  *------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
uint32_t ADXL345_GetMaxTapDuration(COM_Handle * ADXL345)
{
	COM_I2C_DATA(ADXL345).dataSize = 1;
	COM_I2C_DATA(ADXL345).memAddress = ADXL345_DUR_ADDR;
	COM_I2C_DATA(ADXL345).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	COM_RegisterGetter(ADXL345);
											
	return (COM_I2C_DATA(ADXL345).data[0])*DUR_TIME_SCALE_FACTOR;
}

/**------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief  		The latent register is eight bits and contains an unsigned time value representing 
	*							the wait time from the detection of a tap event to the start of the time window (defined by the window register) 
	*							during which a possible second tap event can be detected. 
	*							The scale factor is 1.25 ms/LSB.
	*	@param[IN]  ADXL345 	Sensor handler.
  * @param[IN]  dLatTime	Latency time represented in milisecond unit
  * @retval 		None
  *------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void ADXL345_SetLatencyTime(COM_Handle * ADXL345, double latTime)
{
	uint8_t	sendVal = latTime/LATENT_TIME_SCALE_FACTOR;
	
	COM_I2C_DATA(ADXL345).data[0] = sendVal;
	COM_I2C_DATA(ADXL345).dataSize = 1;
	COM_I2C_DATA(ADXL345).memAddress = ADXL345_LATENT_ADDR;
	COM_I2C_DATA(ADXL345).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	COM_RegisterSetter(ADXL345);
}

/**------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief  		The latent register is eight bits and contains an unsigned time value representing 
	*							the wait time from the detection of a tap event to the start of the time window (defined by the window register) 
	*							during which a possible second tap event can be detected. 
	*							The scale factor is 1.25 ms/LSB.
	*	@param[IN]  ADXL345 Sensor handler.
  * @retval 		Latency time represented in milisecond unit
  *------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
double ADXL345_GetLatencyTime(COM_Handle * ADXL345)
{
	COM_I2C_DATA(ADXL345).dataSize = 1;
	COM_I2C_DATA(ADXL345).memAddress = ADXL345_LATENT_ADDR;
	COM_I2C_DATA(ADXL345).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	COM_RegisterGetter(ADXL345);																		
																				
	return (COM_I2C_DATA(ADXL345).data[0])*LATENT_TIME_SCALE_FACTOR;
}

/**------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief  		The window register is eight bits and contains an unsigned time value representing 
	*							the amount of time after the expiration of the latency time (determined by the latent register)
	*							during which a second valid tap can begin.
	*							The scale factor is 1.25 ms/LSB.
	*	@param[IN]  ADXL345 	Sensor handler.
  * @param[IN]  dWinTime	Window time represented in milisecond unit
  * @retval 		None
  *------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void ADXL345_SetWindowTime(COM_Handle * ADXL345, double winTime)
{
	uint8_t	sendVal = winTime/WINDOW_TIME_SCALE_FACTOR;
	
	COM_I2C_DATA(ADXL345).data[0] = sendVal;
	COM_I2C_DATA(ADXL345).dataSize = 1;
	COM_I2C_DATA(ADXL345).memAddress = ADXL345_WINDOW_ADDR;
	COM_I2C_DATA(ADXL345).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	COM_RegisterSetter(ADXL345);
}

/**------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief  		The window register is eight bits and contains an unsigned time value representing 
	*							the amount of time after the expiration of the latency time (determined by the latent register)
	*							during which a second valid tap can begin.
	*							The scale factor is 1.25 ms/LSB.
	*	@param[IN]  ADXL345 Sensor handler.
  * @retval 		Window time represented in milisecond unit
  *------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
double ADXL345_GetWindowTime(COM_Handle * ADXL345)
{
	COM_I2C_DATA(ADXL345).dataSize = 1;
	COM_I2C_DATA(ADXL345).memAddress = ADXL345_WINDOW_ADDR;
	COM_I2C_DATA(ADXL345).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	COM_RegisterGetter(ADXL345);
										
	return (COM_I2C_DATA(ADXL345).data[0])*WINDOW_TIME_SCALE_FACTOR;
}

/**------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief  		The THRESH_ACT register is eight bits and holds the threshold value for detecting activity.
	*							The data format is unsigned, so the magnitude of the activity event is compared with the value in the THRESH_ACT register.
	*							The scale factor is 62.5 mg/LSB.
	*	@param[IN]  ADXL345 		Sensor handler.
  * @param[IN]  dActThresh	Minimum threshold value for detecting activity.
  * @retval 		None
  *------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void ADXL345_SetActivityThreshold(COM_Handle * ADXL345, double actThresh)
{
	uint8_t	sendVal = actThresh/ACTIVITY_THRESH_SCALE_FACTOR;
	
	COM_I2C_DATA(ADXL345).data[0] = sendVal;
	COM_I2C_DATA(ADXL345).dataSize = 1;
	COM_I2C_DATA(ADXL345).memAddress = ADXL345_THRESH_ACT_ADDR;
	COM_I2C_DATA(ADXL345).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	COM_RegisterSetter(ADXL345);
}

/**------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief  		The THRESH_ACT register is eight bits and holds the threshold value for detecting activity.
	*							The data format is unsigned, so the magnitude of the activity event is compared with the value in the THRESH_ACT register.
	*							The scale factor is 62.5 mg/LSB.
	*	@param[IN]  ADXL345 Sensor handler.
  * @retval 		Minimum threshold value for detecting activity.
  *------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
double ADXL345_GetActivityThreshold(COM_Handle * ADXL345)
{
	COM_I2C_DATA(ADXL345).dataSize = 1;
	COM_I2C_DATA(ADXL345).memAddress = ADXL345_THRESH_ACT_ADDR;
	COM_I2C_DATA(ADXL345).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	COM_RegisterGetter(ADXL345);
													
	return (COM_I2C_DATA(ADXL345).data[0])*WINDOW_TIME_SCALE_FACTOR;
}

/**------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief  		The THRESH_INACT register is eight bits and holds the threshold value for detecting inactivity.
	*							The data format is unsigned, so the magnitude of the inactivity event is compared with the value in the THRESH_INACT register.
	*							The scale factor is 62.5 mg/LSB.
	*	@param[IN]  ADXL345 			Sensor handler.
  * @param[IN]  dInactThresh	Minimum threshold value for detecting activity.
  * @retval 		None
  *------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void ADXL345_SetInactivityThreshold(COM_Handle * ADXL345, double inactThresh)
{
	uint8_t	sendVal = inactThresh/INACTIVITY_THRESH_SCALE_FACTOR;
	
	COM_I2C_DATA(ADXL345).data[0] = sendVal;
	COM_I2C_DATA(ADXL345).dataSize = 1;
	COM_I2C_DATA(ADXL345).memAddress = ADXL345_THRESH_INACT_ADDR;
	COM_I2C_DATA(ADXL345).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	COM_RegisterSetter(ADXL345);
}

/**------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief  		The THRESH_INACT register is eight bits and holds the threshold value for detecting inactivity.
	*							The data format is unsigned, so the magnitude of the inactivity event is compared with the value in the THRESH_INACT register.
	*							The scale factor is 62.5 mg/LSB.
	*	@param[IN]  ADXL345 Sensor handler.
  * @retval 		Minimum threshold value for detecting inactivity.
  *------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
double ADXL345_GetInactivityThreshold(COM_Handle * ADXL345)
{
	COM_I2C_DATA(ADXL345).dataSize = 1;
	COM_I2C_DATA(ADXL345).memAddress = ADXL345_THRESH_INACT_ADDR;
	COM_I2C_DATA(ADXL345).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	COM_RegisterGetter(ADXL345);
														
	return (COM_I2C_DATA(ADXL345).data[0])*INACTIVITY_THRESH_SCALE_FACTOR;
}

/**------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief  		The TIME_INACT register is eight bits and contains an unsigned time value representing the amount of time that 
	*							acceleration must be less than the value in the THRESH_INACT register for inactivity to be declared.
	*							The scale factor is 1 sec/LSB.
	*							Unlike the other interrupt functions, which use unfiltered data (see the Threshold section), the inactivity function uses filtered output data.
	*							At least one output sample must be generated for the inactivity interrupt to be triggered.
	*							This results in the function appearing unresponsive if the TIME_INACT register is set to a value less than the time constant of the output data rate.
	*	@param[IN]  ADXL345 			Sensor handler.
  * @param[IN]  dMinInactTime	Minimum time value for detecting inactivity.
  * @retval 		None
  *------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void ADXL345_SetInactivityTime(COM_Handle * ADXL345, uint8_t minInactTime)
{
	uint8_t	sendVal = minInactTime/TIME_INACTIVITY_SCALE_FACTOR;
	
	COM_I2C_DATA(ADXL345).data[0] = sendVal;
	COM_I2C_DATA(ADXL345).dataSize = 1;
	COM_I2C_DATA(ADXL345).memAddress = ADXL345_TIME_INACT_ADDR;
	COM_I2C_DATA(ADXL345).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	COM_RegisterSetter(ADXL345);
}

/**------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief  		The TIME_INACT register is eight bits and contains an unsigned time value representing the amount of time that 
	*							acceleration must be less than the value in the THRESH_INACT register for inactivity to be declared.
	*							The scale factor is 1 sec/LSB.
	*							Unlike the other interrupt functions, which use unfiltered data (see the Threshold section), the inactivity function uses filtered output data.
	*							At least one output sample must be generated for the inactivity interrupt to be triggered.
	*							This results in the function appearing unresponsive if the TIME_INACT register is set to a value less than the time constant of the output data rate.
	*	@param[IN]  ADXL345 Sensor handler.
  * @retval 		Minimum time value for detecting inactivity.
  *------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
uint8_t ADXL345_GetInactivityTime(COM_Handle * ADXL345)
{
	COM_I2C_DATA(ADXL345).dataSize = 1;
	COM_I2C_DATA(ADXL345).memAddress = ADXL345_TIME_INACT_ADDR;
	COM_I2C_DATA(ADXL345).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	COM_RegisterGetter(ADXL345);
																				
	return (COM_I2C_DATA(ADXL345).data[0])*TIME_INACTIVITY_SCALE_FACTOR;
}

/**---------------------------------------------------------------------------------------------------------------------
  * @brief  		Configures the interrupts.
	*							Writing corresponding bits TRUE enables interrupt and FALSE means disables interrupt.
	*	@param[IN]  ADXL345 	Sensor handler.
	*	@param[IN]  pIntReg 	Contains which interrupt sources are enabled or disabled.
  * @retval 		None
  *---------------------------------------------------------------------------------------------------------------------*/
void ADXL345_ConfigInterrupts(COM_Handle * ADXL345, const ADXL345_InterruptReg * intReg)
{
	uint8_t	sendVal = intReg->BYTE;
	
	COM_I2C_DATA(ADXL345).data[0] = sendVal;
	COM_I2C_DATA(ADXL345).dataSize = 1;
	COM_I2C_DATA(ADXL345).memAddress = ADXL345_INT_ENABLE_ADDR;
	COM_I2C_DATA(ADXL345).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	COM_RegisterSetter(ADXL345);
}

/**---------------------------------------------------------------------------------------------------------------------
  * @brief  			Reads the content of interrupt source register
	*	@param[IN]  	ADXL345 Sensor handler.
  * @param[OUT] 	intReg	Status of interrupt bits.
  *---------------------------------------------------------------------------------------------------------------------*/
void ADXL345_GetInterruptStatus(COM_Handle * ADXL345, ADXL345_InterruptReg * intReg)
{
	COM_I2C_DATA(ADXL345).dataSize = 1;
	COM_I2C_DATA(ADXL345).memAddress = ADXL345_INT_SOURCE_ADDR;
	COM_I2C_DATA(ADXL345).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	COM_RegisterGetter(ADXL345);
													
	intReg->BYTE=COM_I2C_DATA(ADXL345).data[0];
}



/**---------------------------------------------------------------------------------------------------------------------
  * @brief  		Configures which interrupts trigger which pins.
	*							Any bits set to 0 in this register send their respective interrupts to the INT1 pin,
	*							whereas bits set to 1 send their respective interrupts to the INT2 pin.
	*	@param[IN]  ADXL345 	Sensor handler.
	*	@param[IN]  pinMap		Set bits of index of zero represents INT1 pins.
	*												Set bits of index of one  represents INT2 pins.
  * @retval 		None
  *---------------------------------------------------------------------------------------------------------------------*/
void ADXL345_MapInterruptPins(COM_Handle * ADXL345, const ADXL345_InterruptReg pinMap)
{
	/* It is recommended that interrupt bits be configured with the interrupts disabled,
		 preventing interrupts from being accidentally triggered during configuration.
		 This can be done by writing a value of 0x00 to the INT_ENABLE register.*/

	/* Save current interrupt */
	ADXL345_InterruptReg lastIntStatus;
	ADXL345_GetInterruptStatus(ADXL345, &lastIntStatus);
	
	/* Reset all interrupts */
	ADXL345_InterruptReg intReg = {.BYTE = ADXL345_RESET_ALL_INTERRUPTS};
	ADXL345_ConfigInterrupts(ADXL345, &intReg);

	uint8_t sendVal = pinMap.BYTE;
	
	COM_I2C_DATA(ADXL345).data[0] = sendVal;
	COM_I2C_DATA(ADXL345).dataSize = 1;
	COM_I2C_DATA(ADXL345).memAddress = ADXL345_INT_MAP_ADDR;
	COM_I2C_DATA(ADXL345).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	COM_RegisterSetter(ADXL345);
	
	/* Set all interrupts to their previous state */
	intReg.BYTE = lastIntStatus.BYTE;
	ADXL345_ConfigInterrupts(ADXL345, &intReg);
}

/**------------------------------------------------------------------------------
  * @brief  		
	*	@param[IN]  ADXL345 	Sensor handler.
	*	@param[OUT]	rawDatas	Not filtered datas represented every axis
  * @retval 		
  *------------------------------------------------------------------------------*/
void ADXL345_GetRawDatas(COM_Handle * ADXL345, ADXL345_RawDatas * rawDatas)
{
	COM_I2C_DATA(ADXL345).dataSize = 6;
	COM_I2C_DATA(ADXL345).memAddress = ADXL345_START_OF_DATA_ADDR;
	COM_I2C_DATA(ADXL345).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	COM_RegisterGetter(ADXL345);
											
	int16_t xData = (int16_t)(COM_I2C_DATA(ADXL345).data[1]<<8 | COM_I2C_DATA(ADXL345).data[0]);
	int16_t yData = (int16_t)(COM_I2C_DATA(ADXL345).data[3]<<8 | COM_I2C_DATA(ADXL345).data[2]);
	int16_t zData = (int16_t)(COM_I2C_DATA(ADXL345).data[5]<<8 | COM_I2C_DATA(ADXL345).data[4]);
	
	rawDatas->rawXData = xData*ADXL345_ACCEL_DATA_SCALE_FACTOR;
	rawDatas->rawYData = yData*ADXL345_ACCEL_DATA_SCALE_FACTOR;
	rawDatas->rawZData = zData*ADXL345_ACCEL_DATA_SCALE_FACTOR;
}

/**------------------------------------------------------------------------------
 *	@brief  		
 *	@param[IN]  ADXL345 		Sensor handler.
 *	@param[IN]	dataFormat	The content of DATA_FORMAT register.
 *	@retval 		
 *-------------------------------------------------------------------------------*/
void ADXL345_SetDataFormat(COM_Handle * ADXL345, const ADXL345_DataFormatReg * dataFormat)
{
	uint8_t sendVal = dataFormat->BYTE;
	
	COM_I2C_DATA(ADXL345).data[0] = sendVal;
	COM_I2C_DATA(ADXL345).dataSize = 1;
	COM_I2C_DATA(ADXL345).memAddress = ADXL345_DATA_FORMAT_ADDR;
	COM_I2C_DATA(ADXL345).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	COM_RegisterSetter(ADXL345);
}

/**------------------------------------------------------------------------------
  * @brief  		
	*	@param[IN]  ADXL345 		Sensor handler.
	*	@param[OUT]	dataFormat	The content of DATA_FORMAT register.
  * @retval 		
  *------------------------------------------------------------------------------*/
void ADXL345_GetDataFormat(COM_Handle * ADXL345, ADXL345_DataFormatReg * dataFormat)
{
	COM_I2C_DATA(ADXL345).dataSize = 1;
	COM_I2C_DATA(ADXL345).memAddress = ADXL345_DATA_FORMAT_ADDR;
	COM_I2C_DATA(ADXL345).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	COM_RegisterGetter(ADXL345);
	
	dataFormat->BYTE = COM_I2C_DATA(ADXL345).data[0];
}

/**------------------------------------------------------------------------------
  * @brief  		
	*	@param[IN]  ADXL345 			Sensor handler.
	*	@param[OUT]	powerControl	The content of POWER_CTRL register.
  * @retval 		
  *------------------------------------------------------------------------------*/
void ADXL345_SetPowerControl(COM_Handle * ADXL345, const ADXL345_PowerCtrReg * powerControl)
{
	uint8_t sendVal = powerControl->BYTE;
	
	COM_I2C_DATA(ADXL345).data[0] = sendVal;
	COM_I2C_DATA(ADXL345).dataSize = 1;
	COM_I2C_DATA(ADXL345).memAddress = ADXL345_POWER_CTL_ADDR;
	COM_I2C_DATA(ADXL345).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	COM_RegisterSetter(ADXL345);
}


/**------------------------------------------------------------------------------
  * @brief  		
	*	@param[IN]  ADXL345 			Sensor handler.
	*	@param[OUT]	powerControl	The content of POWER_CTRL register.
  * @retval 		
  *------------------------------------------------------------------------------*/
void ADXL345_GetPowerControl(COM_Handle * ADXL345, ADXL345_PowerCtrReg * powerControl)
{
	COM_I2C_DATA(ADXL345).dataSize = 1;
	COM_I2C_DATA(ADXL345).memAddress = ADXL345_POWER_CTL_ADDR;
	COM_I2C_DATA(ADXL345).memAddSize = I2C_MEMADD_SIZE_8BIT;
	
	COM_RegisterGetter(ADXL345);
	
	powerControl->BYTE = COM_I2C_DATA(ADXL345).data[0];
}

/**------------------------------------------------------------------------------
  * @brief  			
	* @note					
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *------------------------------------------------------------------------------*/
void ADXL345_GetAccelOffsetValues(COM_Handle * ADXL345, IMU_AxisDatas * biasDatas)
{
	memset(biasDatas, 0x00, sizeof(IMU_AxisDatas));
	ADXL345_RawDatas tempDatas;
	
	for(uint8_t i=0; i<ADXL345_ACCEL_BIAS_CALC_ITERATION; i++)
	{
		ADXL345_GetRawDatas(ADXL345, &tempDatas);
		
		biasDatas->xData += tempDatas.rawXData;
		biasDatas->yData += tempDatas.rawYData;
		biasDatas->zData += tempDatas.rawZData;
	}
	
	biasDatas->xData /=  (float)ADXL345_ACCEL_BIAS_CALC_ITERATION;
	biasDatas->yData /=  (float)ADXL345_ACCEL_BIAS_CALC_ITERATION;
	biasDatas->zData /=  (float)ADXL345_ACCEL_BIAS_CALC_ITERATION;
	
	biasDatas->zData -= ADXL345_GRAVITY_ACCELERATION;
}

/**------------------------------------------------------------------------------
  * @brief  			
	* @note					
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *------------------------------------------------------------------------------*/
void ADXL345_GetAngle(COM_Handle * ADXL345, const IMU_AxisDatas * axisBias, IMU_AxisAngles * axisAngle)
{
	IMU_AxisDatas 	 	axisData={0};
	ADXL345_RawDatas 	adxl345TempAccelData={0};
	
	ADXL345_GetRawDatas(ADXL345, &adxl345TempAccelData);
	
	axisData.xData = adxl345TempAccelData.rawXData;
	axisData.yData = adxl345TempAccelData.rawYData;
	axisData.zData = adxl345TempAccelData.rawZData;
	
	IMU_RemoveBias(&axisData, axisBias);
	
	IMU_GetAngleFromAccelerometer(&axisData, axisAngle);
}
