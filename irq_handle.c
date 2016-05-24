/*
 * irq_handle.c
 *
 *  Created on: 24.05.2016
 *      Author: Michael Meiler
 */


/*********************************************************************
*
*                   Includes
*
*********************************************************************/
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"

/*********************************************************************
*
*                   Defines
*
*********************************************************************/
#define _MAX_BUFFER_LENGTH 10

/*********************************************************************
*
*                   Variables
*
*********************************************************************/
volatile char string_buffer [_MAX_BUFFER_LENGTH + 1];




/*********************************************************************
 *
 *					Tim2 InterruptHandle
 *
 ********************************************************************/
/*
void TIM2_IRQHandler(void) {

	/* Make sure that interrupt flag is set */
/*	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {

	  GPIO_SetBits(GPIOD, GPIO_Pin_0);	// Debugging, watch Calculation Time

	  /* Measure Ia and Ib and Theta */
/*	  Theta = TIM_GetCounter(TIM8) * 0.087890625; // 0.087890625 = 360°/4096
	  Ia = (ADC_GetConversionValue(ADC1)/4096)*_U_currentmeasue; // TO DO: multiply with current/voltage ratio
	  Ib = (ADC_GetConversionValue(ADC1)/4096)*_U_currentmeasue; // TO DO: multiply with current/voltage ratio

	  /* Output angle counter value*/
	  //USART_SendData(USART1, 'ok');

	  /* Transform Ia and Ib in Iq*/
/*	  Iq_real = transform(Ia, Ib, Theta);

	  /* Control Iq and return Uq */
/*	  Uq = PI_control(Iq_ref, Iq_real);

	  /* Transform Uq in U_alpha */
	  // arm_inv_park_f32();

	  /* Calculate a new pulse width */
/*	  svpwm();

	  TIM1->CCR1 = *switchtime[0];
	  TIM1->CCR2 = *switchtime[1];
	  TIM1->CCR3 = *switchtime[2];

	  //Theta += 0.019;
	  //if (Theta >= (2*M_PI))
	  //	Theta = 0;

	  // ADC 0V min 3.3V max
	  // U_alpha = (ADC_GetConversionValue(ADC1)/4096)*_U_DC;

	  /* Clear Interrupt Flag */
/*	  TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

	  GPIO_ResetBits(GPIOD, GPIO_Pin_0);	// Debugging
  }
}
*/


/*********************************************************************
 *
 *					InterruptHandle for External Signal PC8 (Reset Encoder Value)
 *
 ********************************************************************/

void EXTI9_5_IRQHandler(void) {

    /* Make sure that interrupt flag is set */
    if (EXTI_GetITStatus(EXTI_Line8) != RESET) {

    	/* Reset Counter */
    	TIM_SetCounter(TIM8, 0x00);

        /* Clear interrupt flag */
        EXTI_ClearITPendingBit(EXTI_Line12);
    }
}


/*********************************************************************
 *
 *					USART2 IRQ
 *
 ********************************************************************/

void USART2_IRQHandler(void) {


	if (USART_GetITStatus(USART2, USART_IT_RXNE)) {
		static int count = 0;
		char ch = USART2->DR;	// Read clears ITPendingBit

/*------Here you can modify what to do when data is received------*/
		if ((ch != '\n') && (count < _MAX_BUFFER_LENGTH)) {
			string_buffer[count++] = ch;
		}
		else {
			string_buffer[count] = '\0';
			count=0;
			USART_puts(USART2, string_buffer);
		}
	}

}

