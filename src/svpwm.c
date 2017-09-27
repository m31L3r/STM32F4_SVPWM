/*
 *  svpwm.c
 *
 *  Created on: 09.01.2016
 *      Author: Michael Meiler
 */



#include <math.h>
#include <arm_math.h>
#include <stdint.h>
#include <svpwm.h>



/*-----------------------------------Space Vector Modulation -----
|  Function:	svpwm()
|
|  Purpose:	This subroutine determines the sector (1 out of 6 possible)
|		which the voltage vector should be modulated,
|		then calculates the on/off times, the counter values for switching respectively.
|		Addresses of calculated values are saved in pointer array "switchtime".
|
|
|
|  Description:	_______________________________________________________
|			   |         __|	     __|	     __|
|			   |	   |/  |	   |/  | 	   |/  |
|			   |    0__|  /_\       1__|  /_\       2__|  /_\
|			   |	   |\__| 	   |\__| 	   |\__|
|	 		 * | *	       |	       |	       |
|     		      *    |    *      +---------------|---------------|-------u
|                    *     |     *     |	       +---------------|-------v
|                    *     |     *     |	       |	       +-------w
|                     *    |    *      |	       |	       |
|                        * | *	     __|	     __|	     __|
|			   |  	   |/  |	   |/  |	   |/  |
|			   |    3__|  /_\       4__|  /_\       5__|  /_\
|			   |	   |\__|	   |\__|	   |\__|
|			   |___________|_______________|_______________|
|
|
|  Parameters:	[in]	None.
|			Uses global variables (Theta, U_alpha, U_beta), without changing them.
|
|		[out]	uint16_t *switchtime[3]
|
|  Returns:  	No return function used, but updates counter values in pointer array "switchtime".
|
*-------------------------------------------------------------------*/


void svpwm(void)	{


	uint8_t	sector = Theta/_PIdiv3;
	float32_t	U_ref = hypotf(U_alpha,U_beta);
	if (U_ref > U_max) {
		U_ref = U_max;
	}
	float32_t	angle = Theta - (sector*_PIdiv3);
	float32_t	U_ref_percent = (_SQRT3)*(U_ref/_U_DC); // previous: (2/_SQRT3)
	float32_t	t_1 = U_ref_percent*arm_sin_f32(_PIdiv3-angle)*T_halfsample;
	float32_t	t_2 = U_ref_percent*arm_sin_f32(angle)*T_halfsample;
	float32_t	t_0 = T_halfsample - t_1 - t_2;
	float32_t	t_0_half = t_0/2;


	/* Switching counter values for Timer Interrupts */

	/* Upper switches */
	uint16_t	ontime_t_0_half = (t_0_half) * counterfrequency;
	uint16_t	ontime_value_1 = (t_0_half + t_1) * counterfrequency;
	uint16_t	ontime_value_2 = (t_0_half + t_2) * counterfrequency;
	uint16_t	ontime_value_3 = (t_0_half + t_1 + t_2) * counterfrequency;

	switch (sector)	{

		/*					Upper switches			*/

		/* Sector 1 */
		case 0:		switchtime[0] = &ontime_t_0_half;
					switchtime[1] = &ontime_value_1;
					switchtime[2] = &ontime_value_3;
				break;

		/* Sector 2 */
		case 1:		switchtime[0] = &ontime_value_2;
					switchtime[1] = &ontime_t_0_half;
					switchtime[2] = &ontime_value_3;
				break;

		/* Sector 3 */
		case 2:		switchtime[0] = &ontime_value_3;
					switchtime[1] = &ontime_t_0_half;
					switchtime[2] = &ontime_value_1;
				break;

		/* Sector 4 */
		case 3:		switchtime[0] = &ontime_value_3;
					switchtime[1] = &ontime_value_2;
					switchtime[2] = &ontime_t_0_half;
				break;

		/* Sector 5 */
		case 4:		switchtime[0] = &ontime_value_1;
					switchtime[1] = &ontime_value_3;
					switchtime[2] = &ontime_t_0_half;
				break;

		/* Sector 6 */
		case 5:		switchtime[0] = &ontime_t_0_half;
					switchtime[1] = &ontime_value_3;
					switchtime[2] = &ontime_value_2;
				break;
	}
}
