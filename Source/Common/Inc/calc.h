#ifndef CALC_H
#define CALC_H

#include "inttypes.h"

/* Exported constants --------------------------------------------------------*/
#define SIGN_BIT_IN_BYTE  		7
#define SIGN_BIT_IN_HALFWORD  15
#define SIGN_BIT_IN_WORD  		31

#define SIGN_BIT_MASK_IN_BYTE  			(1<<SIGN_BIT_IN_BYTE)
#define SIGN_BIT_MASK_IN_HALFWORD  	(1<<SIGN_BIT_IN_HALFWORD)
#define SIGN_BIT_MASK_IN_WORD  			(1<<SIGN_BIT_IN_WORD)

/* Exported functions --------------------------------------------------------*/
double Get_Byte2sComplement(uint8_t data);
double Get_HalfWord2sComplement(uint16_t data);
double Get_Word2sComplement(uint32_t data);

#endif	/* CALC_H */