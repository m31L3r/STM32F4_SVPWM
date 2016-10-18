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
#include "irq_handle.h"

volatile uint16_t blabla;
/*********************************************************************
 *
 *					Tim2 IRQ
 *
 ********************************************************************/

void TIM2_IRQHandler(void) {

	/* Make sure that interrupt flag is set */
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {

	  GPIO_SetBits(GPIOD, GPIO_Pin_0);	// Debugging, watch Calculation Time

	  /* Measure Ia and Ib and Theta */
	  //Theta = ((uint16_t) TIM_GetCounter(TIM8)) * 0.001533981; // 0.001533981 = 2pi/4096
	  //Ia = (ADC_GetConversionValue(ADC1)/4095)*_MAX_CURRENT;
	  Ib = (ADC_GetConversionValue(ADC2)/4095)*_MAX_CURRENT;
	  blabla = ADC_GetConversionValue(ADC3);

	  /* Output angle counter value*/
	  //USART_SendData(USART2, 'ok');

	  /* Calculates sine and cosine for park transformation */
	  arm_sin_cos_f32(Theta, &sinevalue, &cosinevalue);

	  /* Transform Ia and Ib in Iq */
	  Iq_real = transform(Ia, Ib, Theta);

	  /* Control Iq and return Uq */
	  Uq = PI_control(Iq_ref, Iq_real);

	  /* Transform Uq in U_alpha */
	  arm_inv_park_f32(Ud, Uq, &U_alpha, &U_beta, sinevalue, cosinevalue);

	  /* Calculate a new pulse width */
	  svpwm();

	  TIM1->CCR1 = *switchtime[0];
	  TIM1->CCR2 = *switchtime[1];
	  TIM1->CCR3 = *switchtime[2];

	  Theta += 0.019;
	  if (Theta >= (2*M_PI))
	  	Theta = 0;


	  /* Clear Interrupt Flag */
	  TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

	  GPIO_ResetBits(GPIOD, GPIO_Pin_0);	// Debugging
  }
}



/*********************************************************************
 *
 *					IRQ for External Signal PC8 (Reset Encoder Value)
 *
 ********************************************************************/

void EXTI9_5_IRQHandler(void) {

    /* Make sure that interrupt flag is set */
    if (EXTI_GetITStatus(EXTI_Line8) != RESET) {

    	/* Reset Counter */
    	TIM_SetCounter(TIM8, 0x00);
    	USART_SendData(USART2, 0x01);

        /* Clear interrupt flag */
        EXTI_ClearITPendingBit(EXTI_Line8);
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
		uint8_t ch = USART2->DR;	// Read clears ITPendingBit

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

