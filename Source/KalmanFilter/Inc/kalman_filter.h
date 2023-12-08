#ifndef _KALMAN_H_
#define _KALMAN_H_

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"

/* Exported define -----------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef struct
{
  /* Covariance Variables */
  float Q_angle;      /* Process noise variance for the accelerometer. */
  float Q_bias;       /* Process noise variance for the gyro bias. */
  float R_measure;    /* Measurement noise variance - this is actually the variance of the measurement noise. */

  /* State Variables */
  float angle;        /* The angle calculated by the Kalman filter - part of the 2x1 state vector. */
  float bias;         /* The gyro bias calculated by the Kalman filter - part of the 2x1 state vector. */

  float rate;         /* Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate. */
  float P[2][2];      /* Error covariance matrix - This is a 2x2 matrix. */

} Kalman;


/* Exported variables --------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
void Kalman_CreateFilter(Kalman * kalman);

void Kalman_SetAngle(Kalman * kalman, float angle);
float Kalman_GetAngle(Kalman * kalman, float newAngle, float newRate, float dt);
float Kalman_GetRate(Kalman * kalman);

void Kalman_SetQangle(Kalman * kalman, float Q_angle);
void Kalman_SetQbias(Kalman * kalman, float Q_bias);
void Kalman_SetRmeasure(Kalman * kalman, float R_measure);


float Kalman_GetQangle(Kalman * kalman);
float Kalman_GetQbias(Kalman * kalman);
float Kalman_GetRmeasure(Kalman * kalman);

#endif /* _KALMAN_H_ */
