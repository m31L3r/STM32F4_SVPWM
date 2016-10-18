/*
 * pi_control.h
 *
 *  Created on: 19.05.2016
 *      Author: Michael Meiler
 */

#ifndef SRC_PI_CONTROL_H_
#define SRC_PI_CONTROL_H_


/*********************************************************************
*
*                   Defines
*
*********************************************************************/
#define _OUTMIN 2//value       // limits output alias command signal
#define _OUTMAX 2//value       // limits output alias command signal
#define _ERRORMIN 2//value     // limits control error
#define _ERRORMAX 2//value     // limits control error
#define _K_p 4
#define _K_i 1


/*********************************************************************
*
*                   Variables
*
*********************************************************************/



/*********************************************************************
*
*                   Functions
*
*********************************************************************/
float PI_control(float32_t ref, float32_t real);


#endif /* SRC_PI_CONTROL_H_ */
