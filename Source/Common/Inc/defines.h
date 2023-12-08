#ifndef _DEFINES_H_
#define _DEFINES_H_


/* Exported defines ----------------------------------------------------------*/
#define INVALID_DATA (0xFF)

/* Exported macro ------------------------------------------------------------*/
#define SEC_TO_MS(X)			(X*1000)

/* Exported types ------------------------------------------------------------*/
typedef enum
{
	DATA_READY,
	DATA_NOT_READY

}DataStatus;





#endif /* _DEFINES_H_ */
