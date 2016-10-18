/*
 * transformation.c
 *
 *  Created on: 18.04.2016
 *      Author: Michael Meiler
 */

/*------------------------------------------------- Transformation -----
|  Function:	transform (float32_t current_a, float32_t current_b, float32_t theta)
|
|  Purpose:		Takes measured currents Ia and Ib (Ic not neccesary, b.c. Ia+Ib+Ic=0)
|				plus angle theta to do Clarke and Park transformation
|
|  Parameters:	[in]	float32_t current_a: Measured Ia
|				[in]	float32_t current_b: Measured Ib
|				[in]	float32_t theta: Measured Theta
|
|
|  Returns:  	"Measured" torque producing current Iq
|
*-------------------------------------------------------------------*/

/*********************************************************************
*
*                   Includes
*
*********************************************************************/
#include <stdint.h>
#include <arm_math.h>
#include <transformation.h>


/*********************************************************************
*
*                   Functions
*
*********************************************************************/
float transform(float32_t current_a, float32_t current_b, float32_t theta) {

	float32_t current_alpha, current_beta;
	float32_t current_Id, current_Iq;

	/* Park Transformation */
	/* Calculates I_alpha and I_beta */
	arm_clarke_f32(current_a, current_b, &current_alpha, &current_beta);

	/* Calculates sine and cosine for park transformation */
	arm_sin_cos_f32(theta, &sinevalue, &cosinevalue);

	/* Park Transformation */
	/* Calculates Id and Iq */
	arm_park_f32(current_alpha, current_beta, &current_Id, &current_Iq, sinevalue, cosinevalue);

	return(current_Iq);
}


