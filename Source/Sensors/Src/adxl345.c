#include 	"adxl345.h"
#include 	"cmsis_os2.h"
#include 	"com_input.h"
#include	"defines.h"
#include   "calc.h"

/* Private macro -------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/



/* Exported functions --------------------------------------------------------*/

/**
  * @brief  		Sets basic functionalities.
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  */
void ADXL345_InitSensor(const COM_Input_HandleTypeDef * ADXL345)
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

/**
  * @brief  		The DEVID register holds a fixed device ID code of 0xE5
  * @param[IN]  ADXL345 Sensor handler
  * @retval 		None
  */
uint8_t ADXL345_WhoAmI(const COM_Input_HandleTypeDef * ADXL345)
{
	COM_Input_TempDataTypeDef comData =
	{
		.dataSize = 1,
		.memAddress = DEVID
	};
	
	COM_Input_RegisterGetter(ADXL345, &comData);
											
	return comData.data[0];
}

/**
  * @brief  		The THRESH_TAP register is eight bits and holds the threshold value for tap interrupts. 
	*							The data format is unsigned, so the magnitude of the tap event is compared with the value in THRESH_TAP.
	*							The scale factor is 62.5 mg/LSB (that is, 0xFF = +16 g).
	*	@param[IN]  ADXL345 		Sensor handler.
  * @param[IN]  dTapThresh	Minimum acceleration value represented in mg unit to create tap interrupt.
  * @retval 		None
  */
void ADXL345_SetTapThreshold(const COM_Input_HandleTypeDef * ADXL345, double tapThresh)
{
	if( (tapThresh>=0) && (tapThresh<=16000) )
	{
		uint8_t sendVal = (1.0/TAP_THRESH_SCALE_FACTOR)*tapThresh;
		
		const COM_Input_TempDataTypeDef comData =
		{
			.data[0] = sendVal,
			.dataSize = 1,
			.memAddress = THRESH_TAP
		};
		
		COM_Input_RegisterSetter(ADXL345, &comData);
	}
}

/**
  * @brief  		The THRESH_TAP register is eight bits and holds the threshold value for tap interrupts. 
	*							The data format is unsigned, so the magnitude of the tap event is compared with the value in THRESH_TAP.
	*							The scale factor is 62.5 mg/LSB (that is, 0xFF = +16 g).
	*	@param[IN]  ADXL345 Sensor handler.
  * @retval 		Minimum acceleration value represented in mg unit to create tap interrupt.
  */
double ADXL345_GetTapThreshold(const COM_Input_HandleTypeDef * ADXL345)
{
	COM_Input_TempDataTypeDef comData =
	{
		.dataSize = 1,
		.memAddress = THRESH_TAP
	};
	
	COM_Input_RegisterGetter(ADXL345, &comData);
											
	return (comData.data[0])*TAP_THRESH_SCALE_FACTOR;
}

/**
  * @brief  		The OFSX, OFSY, and OFSZ registers are each eight bits and offer user-set offset adjustments in twos complement format 
	*							with a scale factor of 15.6 mg/LSB (that is, 0x7F = +2 g)
	*	@note				2'complement simply inverts each bit of given binary number, which will be 01010001. Then add 1 to the LSB of this result.
	*	@param[IN]  ADXL345 	Sensor handler.
  * @param[IN]  dOffset		Offset value represented in mg unit.
  * @retval 		None
  */
void ADXL345_SetOffset(const COM_Input_HandleTypeDef * ADXL345, ADXL345_Axis axis, double offset)
{
	if( (offset<=2000) && (offset>=-2000) )
	{
		uint8_t sendVal = (offset<0) ? (uint8_t)(~((uint8_t)(-1*offset*(1.0/OFFSET_SCALE_FACTOR)))) + 1 :
											(offset>0) ? (offset*(1.0/OFFSET_SCALE_FACTOR))	:	0;
		
		uint8_t memAddress = (axis==ADXL345_X_AXIS) ? OFSX :
												 (axis==ADXL345_Y_AXIS) ? OFSY :
												 (axis==ADXL345_Z_AXIS) ? OFSZ :
																					INVALID_DATA;
		
		if(memAddress != INVALID_DATA)
		{
			const COM_Input_TempDataTypeDef comData =
			{
				.data[0] = sendVal,
				.dataSize = 1,
				.memAddress = memAddress
			};
		
			COM_Input_RegisterSetter(ADXL345, &comData);
		}
	}
}

/**
  * @brief  		The OFSX, OFSY, and OFSZ registers are each eight bits and offer user-set offset adjustments in twos complement format 
	*							with a scale factor of 15.6 mg/LSB (that is, 0x7F = +2 g)
	*	@param[IN]  ADXL345 	Sensor handler.
  * @param[IN]  eAxis 		The axes whose offset is queued
  * @retval 		Offset value represented in mg unit.
  */
double ADXL345_GetOffset(const COM_Input_HandleTypeDef * ADXL345, ADXL345_Axis axis)
{
	uint8_t memAddress = 	(axis==ADXL345_X_AXIS) ? OFSX :
												(axis==ADXL345_Y_AXIS) ? OFSY :
												(axis==ADXL345_Z_AXIS) ? OFSZ	: INVALID_DATA;
		
	if(memAddress != INVALID_DATA)
	{
		COM_Input_TempDataTypeDef comData =
		{
			.dataSize = 1,
			.memAddress = memAddress
		};
	
		COM_Input_RegisterGetter(ADXL345, &comData);
																				
		/* Check MSB bit to find the sign of the value. */
		double offsetVal = 	((comData.data[0]) & (1<<ADXL345_REG_MSB_BIT)) ?
													(-1.0*( (uint8_t)( ~(comData.data[0]-1) )*OFFSET_SCALE_FACTOR))	:			// Negative number
													(comData.data[0]*OFFSET_SCALE_FACTOR);																// Positive number
		
		
		
		return offsetVal;
	}
	
	return INVALID_DATA;
}

/**
  * @brief  		The DUR register is eight bits and contains an unsigned time value representing 
	*							the maximum time that an event must be above the THRESH_TAP threshold to qualify as a tap event.
	*							The scale factor is 625 µs/LSB.
	*	@param[IN]  ADXL345 		Sensor handler.
  * @param[IN]  u4MaxTapDur Maximum tap duration represented in microsecond unit.
  * @retval 		None
  */
void ADXL345_SetMaxTapDuration(const COM_Input_HandleTypeDef * ADXL345, uint32_t maxTapDur)
{
	uint8_t	sendVal = maxTapDur/DUR_TIME_SCALE_FACTOR;
	
	const COM_Input_TempDataTypeDef comData =
	{
		.data[0] = sendVal,
		.dataSize = 1,
		.memAddress = DUR
	};
	
	COM_Input_RegisterSetter(ADXL345, &comData);
}

/**
  * @brief  		The DUR register is eight bits and contains an unsigned time value representing 
	*							the maximum time that an event must be above the THRESH_TAP threshold to qualify as a tap event.
	*							The scale factor is 625 µs/LSB.
	*	@param[IN]  ADXL345 	Sensor handler.
  * @retval 		Maximum tap duration represented in microsecond unit.
  */
uint32_t ADXL345_GetMaxTapDuration(const COM_Input_HandleTypeDef * ADXL345)
{
	COM_Input_TempDataTypeDef comData =
	{
		.dataSize = 1,
		.memAddress = DUR
	};
	
	COM_Input_RegisterGetter(ADXL345, &comData);
											
	return (comData.data[0])*DUR_TIME_SCALE_FACTOR;
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
void ADXL345_SetLatencyTime(const COM_Input_HandleTypeDef * ADXL345, double latTime)
{
	uint8_t	sendVal = latTime/LATENT_TIME_SCALE_FACTOR;
	
	const COM_Input_TempDataTypeDef comData =
	{
		.data[0] = sendVal,
		.dataSize = 1,
		.memAddress = LATENT
	};
	
	COM_Input_RegisterSetter(ADXL345, &comData);
}

/**
  * @brief  		The latent register is eight bits and contains an unsigned time value representing 
	*							the wait time from the detection of a tap event to the start of the time window (defined by the window register) 
	*							during which a possible second tap event can be detected. 
	*							The scale factor is 1.25 ms/LSB.
	*	@param[IN]  ADXL345 Sensor handler.
  * @retval 		Latency time represented in milisecond unit
  */
double ADXL345_GetLatencyTime(const COM_Input_HandleTypeDef * ADXL345)
{
	COM_Input_TempDataTypeDef comData =
	{
		.dataSize = 1,
		.memAddress = LATENT
	};
	
	COM_Input_RegisterGetter(ADXL345, &comData);																		
																				
	return (comData.data[0])*LATENT_TIME_SCALE_FACTOR;
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
void ADXL345_SetWindowTime(const COM_Input_HandleTypeDef * ADXL345, double winTime)
{
	uint8_t	sendVal = winTime/WINDOW_TIME_SCALE_FACTOR;
	
	const COM_Input_TempDataTypeDef comData =
	{
		.data[0] = sendVal,
		.dataSize = 1,
		.memAddress = WINDOW
	};
	
	COM_Input_RegisterSetter(ADXL345, &comData);
}

/**
  * @brief  		The window register is eight bits and contains an unsigned time value representing 
	*							the amount of time after the expiration of the latency time (determined by the latent register)
	*							during which a second valid tap can begin.
	*							The scale factor is 1.25 ms/LSB.
	*	@param[IN]  ADXL345 Sensor handler.
  * @retval 		Window time represented in milisecond unit
  */
double ADXL345_GetWindowTime(const COM_Input_HandleTypeDef * ADXL345)
{
	COM_Input_TempDataTypeDef comData =
	{
		.dataSize = 1,
		.memAddress = WINDOW
	};
	
	COM_Input_RegisterGetter(ADXL345, &comData);
										
	return (comData.data[0])*WINDOW_TIME_SCALE_FACTOR;
}

/**
  * @brief  		The THRESH_ACT register is eight bits and holds the threshold value for detecting activity.
	*							The data format is unsigned, so the magnitude of the activity event is compared with the value in the THRESH_ACT register.
	*							The scale factor is 62.5 mg/LSB.
	*	@param[IN]  ADXL345 		Sensor handler.
  * @param[IN]  dActThresh	Minimum threshold value for detecting activity.
  * @retval 		None
  */
void ADXL345_SetActivityThreshold(const COM_Input_HandleTypeDef * ADXL345, double actThresh)
{
	uint8_t	sendVal = actThresh/ACTIVITY_THRESH_SCALE_FACTOR;
	
	const COM_Input_TempDataTypeDef comData =
	{
		.data[0] = sendVal,
		.dataSize = 1,
		.memAddress = THRESH_ACT
	};
	
	COM_Input_RegisterSetter(ADXL345, &comData);
}

/**
  * @brief  		The THRESH_ACT register is eight bits and holds the threshold value for detecting activity.
	*							The data format is unsigned, so the magnitude of the activity event is compared with the value in the THRESH_ACT register.
	*							The scale factor is 62.5 mg/LSB.
	*	@param[IN]  ADXL345 Sensor handler.
  * @retval 		Minimum threshold value for detecting activity.
  */
double ADXL345_GetActivityThreshold(const COM_Input_HandleTypeDef * ADXL345)
{
	COM_Input_TempDataTypeDef comData =
	{
		.dataSize = 1,
		.memAddress = THRESH_ACT
	};
	
	COM_Input_RegisterGetter(ADXL345, &comData);
													
	return (comData.data[0])*WINDOW_TIME_SCALE_FACTOR;
}

/**
  * @brief  		The THRESH_INACT register is eight bits and holds the threshold value for detecting inactivity.
	*							The data format is unsigned, so the magnitude of the inactivity event is compared with the value in the THRESH_INACT register.
	*							The scale factor is 62.5 mg/LSB.
	*	@param[IN]  ADXL345 			Sensor handler.
  * @param[IN]  dInactThresh	Minimum threshold value for detecting activity.
  * @retval 		None
  */
void ADXL345_SetInactivityThreshold(const COM_Input_HandleTypeDef * ADXL345, double inactThresh)
{
	uint8_t	sendVal = inactThresh/INACTIVITY_THRESH_SCALE_FACTOR;
	
	const COM_Input_TempDataTypeDef comData =
	{
		.data[0] = sendVal,
		.dataSize = 1,
		.memAddress = THRESH_INACT
	};
	
	COM_Input_RegisterSetter(ADXL345, &comData);
}

/**
  * @brief  		The THRESH_INACT register is eight bits and holds the threshold value for detecting inactivity.
	*							The data format is unsigned, so the magnitude of the inactivity event is compared with the value in the THRESH_INACT register.
	*							The scale factor is 62.5 mg/LSB.
	*	@param[IN]  ADXL345 Sensor handler.
  * @retval 		Minimum threshold value for detecting inactivity.
  */
double ADXL345_GetInactivityThreshold(const COM_Input_HandleTypeDef * ADXL345)
{
	COM_Input_TempDataTypeDef comData =
	{
		.dataSize = 1,
		.memAddress = THRESH_INACT
	};
	
	COM_Input_RegisterGetter(ADXL345, &comData);
														
	return (comData.data[0])*INACTIVITY_THRESH_SCALE_FACTOR;
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
void ADXL345_SetInactivityTime(const COM_Input_HandleTypeDef * ADXL345, uint8_t minInactTime)
{
	uint8_t	sendVal = minInactTime/TIME_INACTIVITY_SCALE_FACTOR;
	
	const COM_Input_TempDataTypeDef comData =
	{
		.data[0] = sendVal,
		.dataSize = 1,
		.memAddress = TIME_INACT
	};
	
	COM_Input_RegisterSetter(ADXL345, &comData);
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
uint8_t ADXL345_GetInactivityTime(const COM_Input_HandleTypeDef * ADXL345)
{
	COM_Input_TempDataTypeDef comData =
	{
		.dataSize = 1,
		.memAddress = TIME_INACT
	};
	
	COM_Input_RegisterGetter(ADXL345, &comData);
																				
	return (comData.data[0])*TIME_INACTIVITY_SCALE_FACTOR;
}

/**
  * @brief  		Configures the interrupts.
	*							Writing corresponding bits TRUE enables interrupt and FALSE means disables interrupt.
	*	@param[IN]  ADXL345 	Sensor handler.
	*	@param[IN]  pIntReg 	Contains which interrupt sources are enabled or disabled.
  * @retval 		None
  */
void ADXL345_ConfigInterrupts(const COM_Input_HandleTypeDef * ADXL345, const ADXL345_InterruptReg * intReg)
{
	const COM_Input_TempDataTypeDef comData =
	{
		.data[0] = intReg->BYTE,
		.dataSize = 1,
		.memAddress = INT_ENABLE
	};
	
	COM_Input_RegisterSetter(ADXL345, &comData);
}




/**
  * @brief  			Reads the content of interrupt source register
	*	@param[IN]  	ADXL345 Sensor handler.
  * @param[OUT] 	intReg	Status of interrupt bits.
  */
void ADXL345_GetInterruptStatus(const COM_Input_HandleTypeDef * ADXL345, ADXL345_InterruptReg * intReg)
{
	COM_Input_TempDataTypeDef comData =
	{
		.dataSize = 1,
		.memAddress = INT_SOURCE
	};
	
	COM_Input_RegisterGetter(ADXL345, &comData);
													
	intReg->BYTE=comData.data[0];
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
void ADXL345_MapInterruptPins(const COM_Input_HandleTypeDef * ADXL345, const ADXL345_InterruptReg pinMap)
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

	const COM_Input_TempDataTypeDef comData =
	{
		.data[0] = pinMap.BYTE,
		.dataSize = 1,
		.memAddress = INT_MAP
	};
	
	COM_Input_RegisterSetter(ADXL345, &comData);
	
	/* Set all interrupts to their previous state */
	intReg.BYTE = lastIntStatus.BYTE;
	ADXL345_ConfigInterrupts(ADXL345, &intReg);
}

/**
  * @brief  		
	*	@param[IN]  ADXL345 	Sensor handler.
	*	@param[OUT]	rawDatas	Not filtered datas represented every axis
  * @retval 		
  */
DataStatus ADXL345_GetRawDatas(const COM_Input_HandleTypeDef * ADXL345, ADXL345_RawDatas * rawDatas)
{
	/* The DATA_READY, watermark, and overrun bits are always set
		 if the corresponding events occur, regardless of the INT_ENABLE register settings. */
	ADXL345_InterruptReg intStatus;
	ADXL345_GetInterruptStatus(ADXL345, &intStatus);
	
	if(intStatus.BIT.dataReady == true)
	{
		COM_Input_TempDataTypeDef comData =
		{
			.dataSize = 6,
			.memAddress = ADXL345_START_OF_DATA_REGS
		};
	
		COM_Input_RegisterGetter(ADXL345, &comData);
												
		uint16_t xData = comData.data[1]<<8 | comData.data[0];
		uint16_t yData = comData.data[3]<<8 | comData.data[2];
		uint16_t zData = comData.data[5]<<8 | comData.data[4];
		
		rawDatas->rawXData = Get_HalfWord2sComplement(xData)*ADXL345_ACCEL_DATA_SCALE_FACTOR;//(16.0/512.0);
		rawDatas->rawYData = Get_HalfWord2sComplement(yData)*ADXL345_ACCEL_DATA_SCALE_FACTOR;//(16.0/512.0);
		rawDatas->rawZData = Get_HalfWord2sComplement(zData)*ADXL345_ACCEL_DATA_SCALE_FACTOR;//(16.0/512.0);
					
		return DATA_READY;
	}
	return DATA_NOT_READY;
}

/**
  * @brief  		
	*	@param[IN]  ADXL345 		Sensor handler.
	*	@param[IN]	dataFormat	The content of DATA_FORMAT register.
  * @retval 		
  */
void ADXL345_SetDataFormat(const COM_Input_HandleTypeDef * ADXL345, const ADXL345_DataFormatReg * dataFormat)
{
	const COM_Input_TempDataTypeDef comData =
	{
		.data[0] = dataFormat->BYTE,
		.dataSize = 1,
		.memAddress = DATA_FORMAT
	};
	
	COM_Input_RegisterSetter(ADXL345, &comData);
}

/**
  * @brief  		
	*	@param[IN]  ADXL345 		Sensor handler.
	*	@param[OUT]	dataFormat	The content of DATA_FORMAT register.
  * @retval 		
  */
void ADXL345_GetDataFormat(const COM_Input_HandleTypeDef * ADXL345, ADXL345_DataFormatReg * dataFormat)
{
	COM_Input_TempDataTypeDef comData =
	{
		.dataSize = 1,
		.memAddress = DATA_FORMAT
	};
	
	COM_Input_RegisterGetter(ADXL345, &comData);
	
	dataFormat->BYTE = comData.data[0];
}

/**
  * @brief  		
	*	@param[IN]  ADXL345 			Sensor handler.
	*	@param[OUT]	powerControl	The content of POWER_CTRL register.
  * @retval 		
  */
void ADXL345_SetPowerControl(const COM_Input_HandleTypeDef * ADXL345, const ADXL345_PowerCtrReg * powerControl)
{
	const COM_Input_TempDataTypeDef comData =
	{
		.data[0] = powerControl->BYTE,
		.dataSize = 1,
		.memAddress = POWER_CTL
	};
	
	COM_Input_RegisterSetter(ADXL345, &comData);

}


/**
  * @brief  		
	*	@param[IN]  ADXL345 			Sensor handler.
	*	@param[OUT]	powerControl	The content of POWER_CTRL register.
  * @retval 		
  */
void ADXL345_GetPowerControl(const COM_Input_HandleTypeDef * ADXL345, ADXL345_PowerCtrReg * powerControl)
{
	COM_Input_TempDataTypeDef comData =
	{
		.dataSize = 1,
		.memAddress = POWER_CTL
	};
	
	COM_Input_RegisterGetter(ADXL345, &comData);
	
	powerControl->BYTE = comData.data[0];
}


