#ifndef COMM_INPUT_H
#define COMM_INPUT_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported types ------------------------------------------------------------*/
typedef struct
{
	//TODO:It may be generalized for other communication intefaces.
	I2C_HandleTypeDef *hi2c;
	uint16_t  DevAddress;
	uint16_t  MemAddress;
	uint16_t  MemAddSize;
	uint8_t * pData;
	uint16_t  Size;

}COM_Input_DataTypeDef;

#endif /* COMM_INPUT_H */