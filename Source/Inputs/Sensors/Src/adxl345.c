#include 	"adxl345.h"
#include 	"cmsis_os2.h"
#include 	"comm_input.h"
#include	"defines.h"
#include   "calc.h"

/* Private macro -------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
typedef struct
{
	uint8_t data[COM_INPUT_MAX_MULTI_BYTE];
	uint8_t dataSize;
	uint8_t memAddress;

}ADXL345_ComDataTypeDef;


/* Private function prototypes -----------------------------------------------*/
static void ADXL345_RegisterSetter(const ADXL345_HandleTypeDef * ADXL345, const ADXL345_ComDataTypeDef * setter);
static void ADXL345_RegisterGetter(const ADXL345_HandleTypeDef * ADXL345, ADXL345_ComDataTypeDef * getter);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  */
static void ADXL345_RegisterSetter(const ADXL345_HandleTypeDef * ADXL345, const ADXL345_ComDataTypeDef * setter)
{
	osMutexAcquire(MTX_comInputTx, osWaitForever);
		
	COM_Input_DataTypeDef sendMsg = {	.hi2c = ADXL345->i2cHandle,
																		.DevAddress = ADXL345->i2cDevAddr << 1,
																		.MemAddress	= setter->memAddress,
																		.MemAddSize = I2C_MEMADD_SIZE_8BIT,
																		.Size = setter->dataSize };
	
	/* Multi-Byte initialization is not valid */
	for(uint8_t sizeCnt=0; sizeCnt<setter->dataSize; sizeCnt++)
	{
		sendMsg.Data[sizeCnt] = (setter->data)[sizeCnt];
	}
	
	osMessageQueuePut(MSG_comInputTx, &sendMsg, NULL, 0);
				
	osEventFlagsSet(EVT_comInputTx, COM_INPUT_TX_REQU_GET);
	osEventFlagsWait(EVT_comInputTx, COM_INPUT_TX_ACK, osFlagsWaitAll, COM_INPUT_MAX_TX_TIM);
																		
	osMutexRelease(MTX_comInputTx);	

}

/**
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  */
static void ADXL345_RegisterGetter(const ADXL345_HandleTypeDef * ADXL345, ADXL345_ComDataTypeDef * getter)
{
	osMutexAcquire(MTX_comInputRx, osWaitForever);
	
	COM_Input_DataTypeDef receivedMsg = {	.hi2c = ADXL345->i2cHandle,
																				.DevAddress = ADXL345->i2cDevAddr << 1,
																				.MemAddress	= getter->memAddress  ,
																				.MemAddSize = I2C_MEMADD_SIZE_8BIT,
																				.Size =  getter->dataSize};
	
	osMessageQueuePut(MSG_comInputRx, &receivedMsg, NULL, 0);
				
	osEventFlagsSet(EVT_comInputRx, COM_INPUT_RX_REQU_GET);
	osEventFlagsWait(EVT_comInputRx, COM_INPUT_RX_ACK, osFlagsWaitAll, COM_INPUT_MAX_RX_TIM);
	
	osMessageQueueGet(MSG_comInputRx, &receivedMsg, NULL, 0);
	
	for(uint8_t sizeCnt=0; sizeCnt<getter->dataSize; sizeCnt++)
	{
		(getter->data)[sizeCnt] = receivedMsg.Data[sizeCnt];
	}
	
	osMutexRelease(MTX_comInputRx);
}

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  		None
  * @param[IN] 	None
  * @param[OUT]	None
  * @retval 		None
  */
void ADXL345_Init(void)
{

	
	
}

/**
  * @brief  		The DEVID register holds a fixed device ID code of 0xE5
  * @param[IN]  ADXL345 Sensor handler
  * @retval 		None
  */
uint8_t ADXL345_WhoAmI(const ADXL345_HandleTypeDef * ADXL345)
{
	ADXL345_ComDataTypeDef comData =
	{
		.dataSize = 1,
		.memAddress = DEVID
	};
	
	ADXL345_RegisterGetter(ADXL345, &comData);
											
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
void ADXL345_SetTapThreshold(const ADXL345_HandleTypeDef * ADXL345, double tapThresh)
{
	if( (tapThresh>=0) && (tapThresh<=16000) )
	{
		uint8_t sendVal = (1.0/TAP_THRESH_SCALE_FACTOR)*tapThresh;
		
		const ADXL345_ComDataTypeDef comData =
		{
			.data[0] = sendVal,
			.dataSize = 1,
			.memAddress = THRESH_TAP
		};
		
		ADXL345_RegisterSetter(ADXL345, &comData);
	}
}

/**
  * @brief  		The THRESH_TAP register is eight bits and holds the threshold value for tap interrupts. 
	*							The data format is unsigned, so the magnitude of the tap event is compared with the value in THRESH_TAP.
	*							The scale factor is 62.5 mg/LSB (that is, 0xFF = +16 g).
	*	@param[IN]  ADXL345 Sensor handler.
  * @retval 		Minimum acceleration value represented in mg unit to create tap interrupt.
  */
double ADXL345_GetTapThreshold(const ADXL345_HandleTypeDef * ADXL345)
{
	ADXL345_ComDataTypeDef comData =
	{
		.dataSize = 1,
		.memAddress = THRESH_TAP
	};
	
	ADXL345_RegisterGetter(ADXL345, &comData);
											
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
void ADXL345_SetOffset(const ADXL345_HandleTypeDef * ADXL345, ADXL345_Axis axis, double offset)
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
			const ADXL345_ComDataTypeDef comData =
			{
				.data[0] = sendVal,
				.dataSize = 1,
				.memAddress = memAddress
			};
		
			ADXL345_RegisterSetter(ADXL345, &comData);
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
double ADXL345_GetOffset(const ADXL345_HandleTypeDef * ADXL345, ADXL345_Axis axis)
{
	uint8_t memAddress = 	(axis==ADXL345_X_AXIS) ? OFSX :
												(axis==ADXL345_Y_AXIS) ? OFSY :
												(axis==ADXL345_Z_AXIS) ? OFSZ	: INVALID_DATA;
		
	if(memAddress != INVALID_DATA)
	{
		ADXL345_ComDataTypeDef comData =
		{
			.dataSize = 1,
			.memAddress = memAddress
		};
	
		ADXL345_RegisterGetter(ADXL345, &comData);
																				
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
void ADXL345_SetMaxTapDuration(const ADXL345_HandleTypeDef * ADXL345, uint32_t maxTapDur)
{
	uint8_t	sendVal = maxTapDur/DUR_TIME_SCALE_FACTOR;
	
	const ADXL345_ComDataTypeDef comData =
	{
		.data[0] = sendVal,
		.dataSize = 1,
		.memAddress = DUR
	};
	
	ADXL345_RegisterSetter(ADXL345, &comData);
}

/**
  * @brief  		The DUR register is eight bits and contains an unsigned time value representing 
	*							the maximum time that an event must be above the THRESH_TAP threshold to qualify as a tap event.
	*							The scale factor is 625 µs/LSB.
	*	@param[IN]  ADXL345 	Sensor handler.
  * @retval 		Maximum tap duration represented in microsecond unit.
  */
uint32_t ADXL345_GetMaxTapDuration(const ADXL345_HandleTypeDef * ADXL345)
{
	ADXL345_ComDataTypeDef comData =
	{
		.dataSize = 1,
		.memAddress = DUR
	};
	
	ADXL345_RegisterGetter(ADXL345, &comData);
											
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
void ADXL345_SetLatencyTime(const ADXL345_HandleTypeDef * ADXL345, double latTime)
{
	uint8_t	sendVal = latTime/LATENT_TIME_SCALE_FACTOR;
	
	const ADXL345_ComDataTypeDef comData =
	{
		.data[0] = sendVal,
		.dataSize = 1,
		.memAddress = LATENT
	};
	
	ADXL345_RegisterSetter(ADXL345, &comData);
}

/**
  * @brief  		The latent register is eight bits and contains an unsigned time value representing 
	*							the wait time from the detection of a tap event to the start of the time window (defined by the window register) 
	*							during which a possible second tap event can be detected. 
	*							The scale factor is 1.25 ms/LSB.
	*	@param[IN]  ADXL345 Sensor handler.
  * @retval 		Latency time represented in milisecond unit
  */
double ADXL345_GetLatencyTime(const ADXL345_HandleTypeDef * ADXL345)
{
	ADXL345_ComDataTypeDef comData =
	{
		.dataSize = 1,
		.memAddress = LATENT
	};
	
	ADXL345_RegisterGetter(ADXL345, &comData);																		
																				
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
void ADXL345_SetWindowTime(const ADXL345_HandleTypeDef * ADXL345, double winTime)
{
	uint8_t	sendVal = winTime/WINDOW_TIME_SCALE_FACTOR;
	
	const ADXL345_ComDataTypeDef comData =
	{
		.data[0] = sendVal,
		.dataSize = 1,
		.memAddress = WINDOW
	};
	
	ADXL345_RegisterSetter(ADXL345, &comData);
}

/**
  * @brief  		The window register is eight bits and contains an unsigned time value representing 
	*							the amount of time after the expiration of the latency time (determined by the latent register)
	*							during which a second valid tap can begin.
	*							The scale factor is 1.25 ms/LSB.
	*	@param[IN]  ADXL345 Sensor handler.
  * @retval 		Window time represented in milisecond unit
  */
double ADXL345_GetWindowTime(const ADXL345_HandleTypeDef * ADXL345)
{
	ADXL345_ComDataTypeDef comData =
	{
		.dataSize = 1,
		.memAddress = WINDOW
	};
	
	ADXL345_RegisterGetter(ADXL345, &comData);
										
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
void ADXL345_SetActivityThreshold(const ADXL345_HandleTypeDef * ADXL345, double actThresh)
{
	uint8_t	sendVal = actThresh/ACTIVITY_THRESH_SCALE_FACTOR;
	
	const ADXL345_ComDataTypeDef comData =
	{
		.data[0] = sendVal,
		.dataSize = 1,
		.memAddress = THRESH_ACT
	};
	
	ADXL345_RegisterSetter(ADXL345, &comData);
}

/**
  * @brief  		The THRESH_ACT register is eight bits and holds the threshold value for detecting activity.
	*							The data format is unsigned, so the magnitude of the activity event is compared with the value in the THRESH_ACT register.
	*							The scale factor is 62.5 mg/LSB.
	*	@param[IN]  ADXL345 Sensor handler.
  * @retval 		Minimum threshold value for detecting activity.
  */
double ADXL345_GetActivityThreshold(const ADXL345_HandleTypeDef * ADXL345)
{
	ADXL345_ComDataTypeDef comData =
	{
		.dataSize = 1,
		.memAddress = THRESH_ACT
	};
	
	ADXL345_RegisterGetter(ADXL345, &comData);
													
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
void ADXL345_SetInactivityThreshold(const ADXL345_HandleTypeDef * ADXL345, double inactThresh)
{
	uint8_t	sendVal = inactThresh/INACTIVITY_THRESH_SCALE_FACTOR;
	
	const ADXL345_ComDataTypeDef comData =
	{
		.data[0] = sendVal,
		.dataSize = 1,
		.memAddress = THRESH_INACT
	};
	
	ADXL345_RegisterSetter(ADXL345, &comData);
}

/**
  * @brief  		The THRESH_INACT register is eight bits and holds the threshold value for detecting inactivity.
	*							The data format is unsigned, so the magnitude of the inactivity event is compared with the value in the THRESH_INACT register.
	*							The scale factor is 62.5 mg/LSB.
	*	@param[IN]  ADXL345 Sensor handler.
  * @retval 		Minimum threshold value for detecting inactivity.
  */
double ADXL345_GetInactivityThreshold(const ADXL345_HandleTypeDef * ADXL345)
{
	ADXL345_ComDataTypeDef comData =
	{
		.dataSize = 1,
		.memAddress = THRESH_INACT
	};
	
	ADXL345_RegisterGetter(ADXL345, &comData);
														
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
void ADXL345_SetInactivityTime(const ADXL345_HandleTypeDef * ADXL345, uint8_t minInactTime)
{
	uint8_t	sendVal = minInactTime/TIME_INACTIVITY_SCALE_FACTOR;
	
	const ADXL345_ComDataTypeDef comData =
	{
		.data[0] = sendVal,
		.dataSize = 1,
		.memAddress = TIME_INACT
	};
	
	ADXL345_RegisterSetter(ADXL345, &comData);
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
uint8_t ADXL345_GetInactivityTime(const ADXL345_HandleTypeDef * ADXL345)
{
	ADXL345_ComDataTypeDef comData =
	{
		.dataSize = 1,
		.memAddress = TIME_INACT
	};
	
	ADXL345_RegisterGetter(ADXL345, &comData);
																				
	return (comData.data[0])*TIME_INACTIVITY_SCALE_FACTOR;
}

/**
  * @brief  		Configures the interrupts.
	*							Writing corresponding bits TRUE enables interrupt and FALSE means disables interrupt.
	*	@param[IN]  ADXL345 	Sensor handler.
	*	@param[IN]  pIntReg 	Contains which interrupt sources are enabled or disabled.
  * @retval 		None
  */
void ADXL345_ConfigInterrupts(const ADXL345_HandleTypeDef * ADXL345, const ADXL345_InterruptReg * intReg)
{
	const ADXL345_ComDataTypeDef comData =
	{
		.data[0] = intReg->BYTE,
		.dataSize = 1,
		.memAddress = INT_ENABLE
	};
	
	ADXL345_RegisterSetter(ADXL345, &comData);
}




/**
  * @brief  			Reads the content of interrupt source register
	*	@param[IN]  	ADXL345 Sensor handler.
  * @param[OUT] 	intReg	Status of interrupt bits.
  */
void ADXL345_GetInterruptStatus(const ADXL345_HandleTypeDef * ADXL345, ADXL345_InterruptReg * intReg)
{
	ADXL345_ComDataTypeDef comData =
	{
		.dataSize = 1,
		.memAddress = INT_SOURCE
	};
	
	ADXL345_RegisterGetter(ADXL345, &comData);
													
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
void ADXL345_MapInterruptPins(const ADXL345_HandleTypeDef * ADXL345, const ADXL345_InterruptReg pinMap)
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

	const ADXL345_ComDataTypeDef comData =
	{
		.data[0] = pinMap.BYTE,
		.dataSize = 1,
		.memAddress = INT_MAP
	};
	
	ADXL345_RegisterSetter(ADXL345, &comData);
	
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
DataStatus ADXL345_GetRawDatas(const ADXL345_HandleTypeDef * ADXL345, ADXL345_RawDatas * rawDatas)
{
	/* The DATA_READY, watermark, and overrun bits are always set
		 if the corresponding events occur, regardless of the INT_ENABLE register settings. */
	ADXL345_InterruptReg intStatus;
	ADXL345_GetInterruptStatus(ADXL345, &intStatus);
	
	if(intStatus.BIT.dataReady == true)
	{
		ADXL345_ComDataTypeDef comData =
		{
			.dataSize = 6,
			.memAddress = ADXL345_START_OF_DATA_REGS
		};
	
		ADXL345_RegisterGetter(ADXL345, &comData);
												
		uint16_t xData = comData.data[1]<<8 | comData.data[0];
		uint16_t yData = comData.data[3]<<8 | comData.data[2];
		uint16_t zData = comData.data[5]<<8 | comData.data[4];
		
		rawDatas->rawXData = Get_HalfWord2sComplement(xData)*(16.0/512.0);
		rawDatas->rawYData = Get_HalfWord2sComplement(yData)*(16.0/512.0);
		rawDatas->rawZData = Get_HalfWord2sComplement(zData)*(16.0/512.0);
					
		
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
void ADXL345_SetDataFormat(const ADXL345_HandleTypeDef * ADXL345, const ADXL345_DataFormatReg * dataFormat)
{
	const ADXL345_ComDataTypeDef comData =
	{
		.data[0] = dataFormat->BYTE,
		.dataSize = 1,
		.memAddress = DATA_FORMAT
	};
	
	ADXL345_RegisterSetter(ADXL345, &comData);
}

/**
  * @brief  		
	*	@param[IN]  ADXL345 		Sensor handler.
	*	@param[OUT]	dataFormat	The content of DATA_FORMAT register.
  * @retval 		
  */
void ADXL345_GetDataFormat(const ADXL345_HandleTypeDef * ADXL345, ADXL345_DataFormatReg * dataFormat)
{
	ADXL345_ComDataTypeDef comData =
	{
		.dataSize = 1,
		.memAddress = DATA_FORMAT
	};
	
	ADXL345_RegisterGetter(ADXL345, &comData);
	
	dataFormat->BYTE = comData.data[0];
}

/**
  * @brief  		
	*	@param[IN]  ADXL345 			Sensor handler.
	*	@param[OUT]	powerControl	The content of POWER_CTRL register.
  * @retval 		
  */
void ADXL345_SetPowerControl(const ADXL345_HandleTypeDef * ADXL345, const ADXL345_PowerCtrReg * powerControl)
{
	const ADXL345_ComDataTypeDef comData =
	{
		.data[0] = powerControl->BYTE,
		.dataSize = 1,
		.memAddress = POWER_CTL
	};
	
	ADXL345_RegisterSetter(ADXL345, &comData);

}


/**
  * @brief  		
	*	@param[IN]  ADXL345 			Sensor handler.
	*	@param[OUT]	powerControl	The content of POWER_CTRL register.
  * @retval 		
  */
void ADXL345_GetPowerControl(const ADXL345_HandleTypeDef * ADXL345, ADXL345_PowerCtrReg * powerControl)
{
	ADXL345_ComDataTypeDef comData =
	{
		.dataSize = 1,
		.memAddress = POWER_CTL
	};
	
	ADXL345_RegisterGetter(ADXL345, &comData);
	
	powerControl->BYTE = comData.data[0];
}


