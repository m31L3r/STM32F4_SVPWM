/*=============================================================================
 |   Assignment:  TPA_Frequenzumrichter
 |
 |	 Created on:  10.01.2016
 |       Author:  Michael Meiler
 |     Language:  C, Gnu Arm Toolchain C-Compiler
 |
 |   To Compile:  Make sure libc and libm are defined for Linker, as well as arm_cortexM4_math lib
 |				  Compiler Flag: -mfloat-abi=hard for FPU and in settings->targetprocessor set Float ABI = hard
 |
 |   Instructor:  NAME OF YOUR COURSE'S INSTRUCTOR
 |     Due Date:  DATE AND TIME THAT THIS PROGRAM IS/WAS DUE TO BE SUBMITTED
 |
 +-----------------------------------------------------------------------------
 |
 |  Description:
 |
 |        Input:  DESCRIBE THE INPUT THAT THE PROGRAM REQUIRES.
 |
 |       Output:  DESCRIBE THE OUTPUT THAT THE PROGRAM PRODUCES.
 |
 |    Algorithm:  OUTLINE THE APPROACH USED BY THE PROGRAM TO SOLVE THE
 |      PROBLEM.
 |
 |   Required Features Not Included:  DESCRIBE HERE ANY REQUIREMENTS OF
 |      THE ASSIGNMENT THAT THE PROGRAM DOES NOT ATTEMPT TO SOLVE.
 |
 |   Known Bugs:  IF THE PROGRAM DOES NOT FUNCTION CORRECTLY IN SOME
 |      SITUATIONS, DESCRIBE THE SITUATIONS AND PROBLEMS HERE.
 |
 *===========================================================================*/



/*********************************************************************
*
*                   Includes
*
*********************************************************************/
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include <pinmux.h>
#include <math.h>
#include <arm_math.h>
#include <pi_controller.h>
#include <svpwm.h>
#include <transformation.h>


/*********************************************************************
 *
 *					Defines
 *
 ********************************************************************/



/***************************************************************
 *
 *					Global Variables
 *
 ***************************************************************/
float32_t	U_alpha=200, U_beta=0, Theta=0;
float32_t	T_halfsample = 0.00003125;
float32_t	counterfrequency = 168000000;
float32_t	U_max = (1/_SQRT3)*_U_DC;
float32_t	Ia=0, Ib=0, Iq_real=0, Iq_ref=0, Ud=0, Uq=0;	// TO DO: Formula for Iq_ref and MAximum
float32_t	sinevalue=0, cosinevalue=0;


uint16_t	*switchtime[3] = {0,0,0};


/*--------------Function to send strings---------------------*/
void USART_puts(USART_TypeDef *USARTx, volatile uint8_t *string) {

	while(*string) {
		while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET);
		USART_SendData(USARTx, *string);
		string++;
	}

}


/*********************************************************************
 *
 *						MAIN
 *
 ********************************************************************/

int main(void) {

	/* SystemInit() called by "startup_stm32f40xx.S" */

    /* Init PWM */
    PWM_Pins_Init();

    /* Init timer */
    TIMER_Init();

    /* Init SVPWM */
    SVPWM_Init();

    /* Init 16kHz Interrupt */
    INT_TIM2_Config();

    /* Init ADC1 Channel 0 & ADC2 Channel1 */
    ADC123_Init();

    /* Configure Encoder with Tim8 on GPIOC 6 & 7 */
    configureEncoder();

    /* Enable USART2 */
    USART2_Init();

    GPIO_SetBits(GPIOB, GPIO_Pin_10);



    while (1) {


    }
}
