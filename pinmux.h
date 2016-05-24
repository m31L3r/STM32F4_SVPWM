/*
 * pinmux.h
 *
 *  Created on: 24.05.2016
 *      Author: Michael Meiler
 */

#ifndef INC_PINMUX_H_
#define INC_PINMUX_H_



/*********************************************************************
*
*                   Config Functions
*
*********************************************************************/
void PWM_Pins_Init(void);

void TIMER_Init(void);

void SVPWM_Init(void);

void configureEncoder(void);

void USART2_Init(void);

void ADC123_Init(void);



#endif /* INC_PINMUX_H_ */
