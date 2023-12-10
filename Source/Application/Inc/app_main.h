#ifndef _APP_MAIN_H_
#define _APP_MAIN_H_

/* Includes ------------------------------------------------------------------*/
#include "bmx160.h"
/* Exported define -----------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
extern BMX160_Handle 	BMX160;
extern AHRS_Handle		AHRS;

/* Exported functions --------------------------------------------------------*/

void APP_Main(void* arg);


#endif /* _APP_MAIN_H_ */
