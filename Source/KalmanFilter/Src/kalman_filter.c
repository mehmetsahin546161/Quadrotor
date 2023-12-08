/* Includes ------------------------------------------------------------------*/
#include "kalman_filter.h"

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

/**------------------------------------------------------------------------------
  * @brief  			
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *------------------------------------------------------------------------------*/
void Kalman_CreateFilter(Kalman * kalman)
{
	/* We will set the variables like so, these can also be tuned by the user. */
	kalman->Q_angle 	= 0.001f;
	kalman->Q_bias  	= 0.003f;
	kalman->R_measure = 0.01f;

	kalman->angle = 0.0f; 		/* Reset the angle */
	kalman->bias 	= 0.0f; 		/* Reset bias. */

	/* Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so.
		 see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical */
	kalman->P[0][0] = 0.0f; 
	kalman->P[0][1] = 0.0f;
	kalman->P[1][0] = 0.0f;
	kalman->P[1][1] = 0.0f;
}

/**------------------------------------------------------------------------------
  * @brief Used to set angle, this should be set as the starting angle
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *------------------------------------------------------------------------------*/
void Kalman_SetAngle(Kalman * kalman, float angle)
{
	kalman->angle = angle;
}

/**------------------------------------------------------------------------------
  * @brief The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds 			
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *------------------------------------------------------------------------------*/
float Kalman_GetAngle(Kalman * kalman, float newAngle, float newRate, float dt)
{
	/* Discrete Kalman filter time update equations - Time Update ("Predict") */
	/* Step 1 : Update xhat - Project the state ahead. */
	kalman->rate = newRate - kalman->bias;
	kalman->angle += dt * kalman->rate;
	
	/* Step 2 : Update estimation error covariance. Project the error covariance ahead	*/
	kalman->P[0][0] += dt * (dt*kalman->P[1][1] - kalman->P[0][1] - kalman->P[1][0] + kalman->Q_angle);
	kalman->P[0][1] -= dt * kalman->P[1][1];
	kalman->P[1][0] -= dt * kalman->P[1][1];
	kalman->P[1][1] += kalman->Q_bias * dt;
	
	/* Discrete Kalman filter measurement update equations - Measurement Update ("Correct") */
	/* Step 4 : Calculate Kalman gain. */
	float S = kalman->P[0][0] + kalman->R_measure; /* Estimate error. */
	
	/* Step 5 */
	float K[2]; /* Kalman gain - This is a 2x1 vector. */
	K[0] = kalman->P[0][0] / S;
	K[1] = kalman->P[1][0] / S;
	
	/* Step 3 : Calculate angle and bias - Update estimate with measurement zk (newAngle) */
	float y = newAngle - kalman->angle; /* Angle difference. */
	
	/* Step 6 */
	kalman->angle += K[0] * y;
	kalman->bias += K[1] * y;
	
	/* Step 7 : Calculate estimation error covariance - Update the error covariance. */
	float P00_temp = kalman->P[0][0];
	float P01_temp = kalman->P[0][1];
	
	kalman->P[0][0] -= K[0] * P00_temp;
	kalman->P[0][1] -= K[0] * P01_temp;
	kalman->P[1][0] -= K[1] * P00_temp;
	kalman->P[1][1] -= K[1] * P01_temp;
	
	return kalman->angle;
}

/**------------------------------------------------------------------------------
  * @brief Return the unbiased rate.
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *------------------------------------------------------------------------------*/
float Kalman_GetRate(Kalman * kalman)
{
	return kalman->rate;
}

/**------------------------------------------------------------------------------
  * @brief Used to tune the Kalman filter Q angle
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *------------------------------------------------------------------------------*/
void Kalman_SetQangle(Kalman * kalman, float Q_angle)
{
	kalman->Q_angle = Q_angle;
}

/**------------------------------------------------------------------------------
  * @brief Used to tune the Kalman filter Q bias
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *------------------------------------------------------------------------------*/
void Kalman_SetQbias(Kalman * kalman, float Q_bias)
{
	kalman->Q_bias = Q_bias;
}
	
/**------------------------------------------------------------------------------
  * @brief Used to tune the Kalman filter R meausurement
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *------------------------------------------------------------------------------*/
void Kalman_SetRmeasure(Kalman * kalman, float R_measure)
{
	kalman->R_measure = R_measure;
}

/**------------------------------------------------------------------------------
  * @brief  			
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *------------------------------------------------------------------------------*/
float Kalman_GetQangle(Kalman * kalman)
{
	return kalman->Q_angle; 
}
	
/**------------------------------------------------------------------------------
  * @brief  			
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *------------------------------------------------------------------------------*/
float Kalman_GetQbias(Kalman * kalman)
{
	return kalman->Q_bias;
}

/**------------------------------------------------------------------------------
  * @brief  			
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *------------------------------------------------------------------------------*/
float Kalman_GetRmeasure(Kalman * kalman)
{
	return kalman->R_measure;
}
