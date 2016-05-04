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
#include <svpwm.h>
#include <math.h>
#include <arm_math.h>


/*********************************************************************
 *
 *					Defines
 *
 ********************************************************************/
/* Maximum Voltage from Current measurement*/
#define _U_currentmeasue 3.3 // TO DO: Find Current Voltage ratio


/***************************************************************
 *
 *					Global Variables
 *
 ***************************************************************/
float32_t	U_alpha=200, U_beta=0, Theta=0;
float32_t	T_halfsample = 0.00003125;
float32_t	counterfrequency = 168000000;
float32_t	U_max = (_SQRT3/2)*_U_DC;
float32_t	Ia=0, Ib=0, Iq_real=0, Iq_ref=0;	// TO DO: Formula for Iq_ref and MAximum


uint16_t	*switchtime[3] = {0,0,0};


/*********************************************************************
 *
 *					Initialize PWM pins
 *
 ********************************************************************/

void PWM_Pins_Init(void) {

    GPIO_InitTypeDef GPIO_InitStruct;

    /* Clock enable for GPIOE */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    /* Connect pins to Alternate function Tim1 */
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource8, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource10, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource12, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);

    /* GPIOE Configuration: Channel 1, 2, 3 and 1N, 2N, 3N as alternate function push-pull */
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9  | GPIO_Pin_10 |
    						  GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOE, &GPIO_InitStruct);


    /* For debugging, watching calculation time respectively */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* Irgendwas hab ich aber vergessen */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);


/*	// Output Internal Clock Frequency on MCO1 (Pin A8)

    //	Enable GPIOs clocks
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_MCO);

    //	Configure MCO (PA8)
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    RCC_MCO1Config(RCC_MCO1Source_PLLCLK, RCC_MCO1Div_4);
*/

}


/*********************************************************************
 *
 *					Initialize Timer1 for PWM
 *
 ********************************************************************/

void TIMER_Init(void) {

    TIM_TimeBaseInitTypeDef TIM_BaseStruct;

    /* Enable clock for TIM1 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    /* Time Base configuration for 168 MHz Clock */
    TIM_BaseStruct.TIM_Prescaler = 0;
    TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
    TIM_BaseStruct.TIM_Period = 5250;		// Reset Value = Auto-Reload-Register
    TIM_BaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_BaseStruct.TIM_RepetitionCounter = 1; // Update event occurs after downcounting underflow

     /* Initialize TIM1 */
    TIM_TimeBaseInit(TIM1, &TIM_BaseStruct);

    /* Tim1 enable counter */
    TIM_Cmd(TIM1, ENABLE);
}


/*********************************************************************
 *
 *					Configure PWM pins with Tim1 and Deadtime
 *
 ********************************************************************/

void SVPWM_Init(void) {

    TIM_OCInitTypeDef TIM_OCStruct;
    TIM_BDTRInitTypeDef TIM_BDTRStruct;

    /* PWM mode 2 = Set on compare match */
    TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCStruct.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCStruct.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCStruct.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCStruct.TIM_OCIdleState = TIM_OCIdleState_Reset;
    TIM_OCStruct.TIM_OCNIdleState = TIM_OCIdleState_Set;
    TIM_OCStruct.TIM_Pulse = 0;		// duty cycle, initialize all with 0

    /* Channel 1 alias E8 & E9 */
    TIM_OC1Init(TIM1, &TIM_OCStruct);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

    /* Channel 2 alias E10 & E11 */
    TIM_OC2Init(TIM1, &TIM_OCStruct);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);

    /* Channel 3 alias E12 & E13 */
    TIM_OC3Init(TIM1, &TIM_OCStruct);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

    /* Automatic Output enable, Break, dead time and lock configuration
    *  For dead time calculation please see reference manual */
    TIM_BDTRStruct.TIM_OSSRState = TIM_OSSRState_Enable;
    TIM_BDTRStruct.TIM_OSSIState = TIM_OSSIState_Enable;
    TIM_BDTRStruct.TIM_LOCKLevel = TIM_LOCKLevel_1;
    /*								Config		Value */
    TIM_BDTRStruct.TIM_DeadTime = 0b11000000 | 0b00000000;
    TIM_BDTRStruct.TIM_Break = TIM_Break_Enable;
    TIM_BDTRStruct.TIM_BreakPolarity = TIM_BreakPolarity_High;
    TIM_BDTRStruct.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;

    TIM_BDTRConfig(TIM1, &TIM_BDTRStruct);

    /* Main Output enable */
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
}


/*********************************************************************
 *
 *					Initialize Timer2 as Interrupt
 *
 ********************************************************************/

void INT_TIM_Config(void){

	NVIC_InitTypeDef NVIC_InitStruct;
	TIM_TimeBaseInitTypeDef TIM_BaseStruct;

	/* Enable the TIM2 global Interrupt */
	NVIC_InitStruct.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);

	/* TIM2 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	/* Time base configuration */
	TIM_BaseStruct.TIM_Prescaler = 0;
	TIM_BaseStruct.TIM_Period = 5249;  // 84 MHz down to 16 KHz
	TIM_BaseStruct.TIM_ClockDivision = 0;
	TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_BaseStruct);

	/* TIM IT enable */
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

	/* TIM2 enable counter */
	TIM_Cmd(TIM2, ENABLE);
}


/*********************************************************************
 *
 *					Tim2 InterruptHandle
 *
 ********************************************************************/

void TIM2_IRQHandler(void) {

	/* Make sure that interrupt flag is set */
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {

	  GPIO_SetBits(GPIOD, GPIO_Pin_0);	// Debugging, watch Calculation Time

	  /* Measure Ia and Ib and Theta */
	  Theta = TIM_GetCounter(TIM8) * 0.087890625; // 0.087890625 = 360/4096
	  Ia = (ADC_GetConversionValue(ADC1)/4096)*_U_currentmeasue; // TO DO: multiply with current/voltage ratio
	  Ib = (ADC_GetConversionValue(ADC1)/4096)*_U_currentmeasue; // TO DO: multiply with current/voltage ratio

	  /* Transform Ia and Ib in Iq*/
	  Iq_real = transform(Ia, Ib, Theta);

	  /* Control Iq and return Uq */
	  Uq = PI_control(Iq_ref, Iq_real);

	  /* Transform Uq in U_alpha */
	  arm_inv_park_f32();

	  /* Calculate a new pulse width */
	  svpwm();

	  TIM1->CCR1 = *switchtime[0];
	  TIM1->CCR2 = *switchtime[1];
	  TIM1->CCR3 = *switchtime[2];

	  //Theta += 0.019;
	  //if (Theta >= (2*M_PI))
	  //	Theta = 0;

	  // ADC 0V min 3.3V max
	  // U_alpha = (ADC_GetConversionValue(ADC1)/4096)*_U_DC;

	  /* Clear Interrupt Flag */
	  TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

	  GPIO_ResetBits(GPIOD, GPIO_Pin_0);	// Debugging
  }
}


/*********************************************************************
 *
 *					Configure Resolver & Interrupt Pin C8 for Z
 *
 ********************************************************************/

void configureEncoder(void) {

    GPIO_InitTypeDef GPIO_InitStruct;
    EXTI_InitTypeDef EXTI_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    /*******************************************
     *	Resolver
     *******************************************/

    /* Clock enable for GPIOC */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

    /* GPIOC Encoder Configuration */
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6| GPIO_Pin_7;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* Alternate Function */
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM8);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM8);

    /* Configure Timer */
    TIM_EncoderInterfaceConfig(TIM8, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

    /* Tim8 counter enable */
    TIM_Cmd(TIM8, ENABLE);

    /********************************************
     *	Reset Pin Z
     ********************************************/

    /* Enable clock for SYSCFG */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    /* GPIOC Input Interrupt Configuration */
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* Tell system that you will use PC8 for EXTI_Line8 */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource0);


    /* PC8 connected to EXTI_Line8 */
    EXTI_InitStruct.EXTI_Line = EXTI_Line8;
    /* Enable interrupt */
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    /* Interrupt mode */
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    /* Triggers on rising edge */
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
    /* Add to EXTI */
    EXTI_Init(&EXTI_InitStruct);

    /* Add IRQ vector to NVIC */
    /* PC8 is connected to EXTI_Line8, which has EXTI9_5_IRQn vector */
    NVIC_InitStruct.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x01;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x01;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
}


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
 *					USART1
 *
 ********************************************************************/

void USART1_Init(void) {

	GPIO_InitTypeDef	GPIO_InitStruct;
	USART_InitTypeDef	USART_InitStruct;
	NVIC_InitTypeDef	NVIC_InitStruct;

	/* Enable clock for GPIOA */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	/* Enable clock for USART1 peripheral */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	/* Tell pins PA9 and PA10 which alternating function */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
	/* Initialize pins as alternating function */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	/**********************************************************
	 * Set Baudrate
	 * Disable Hardware Flow control
	 * Set Mode To TX and RX, so USART will work in full-duplex mode
	 * Disable parity bit
	 * Set 1 stop bit
	 * Set Data bits to 8
	 **********************************************************/
	USART_InitStruct.USART_BaudRate = 9600;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStruct.USART_Parity = USART_Parity_No;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART1, &USART_InitStruct);
	USART_Cmd(USART1, ENABLE);

	/* Enable RX interrupt for communication ;) */
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	/**********************************************************
	 * Set Channel to USART1
	 * Set Channel Cmd to enable. That will enable USART1 channel in NVIC
	 * Set Preemption priority to 2. This means third highest priority
	 **********************************************************/
	NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x02;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_Init(&NVIC_InitStruct);

}

/*--------------Function to send strings---------------------*/
void USART_puts(USART_TypeDef *USARTx, volatile char *string) {
	while(*string) {
		while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET) {
			USART_SendData(USARTx, *string);
		}
		string++;
	}

}


/*------Here you can modify what to do when data is received------*/
#define _MAX_BUFFER_LENGTH 10
volatile char string_buffer [_MAX_BUFFER_LENGTH + 1];

void USART1_IRQHandler(void) {

	if (USART_GetITStatus(USART1, USART_IT_RXNE)) {
		static int count = 0;
		char ch = USART1->DR;	// Read clears ITPendingBit

		if ((ch != '\n') && (count < _MAX_BUFFER_LENGTH)) {
			string_buffer[count++] = ch;
		}
		else {
			string_buffer = '\0';
			count=0;
		}
	}

}


/*********************************************************************
 *
 *					Initialize ADC1 Channel0 & ADC2 Channel1
 *
 ********************************************************************/

void ADC123_Iinit(void) {

	GPIO_InitTypeDef GPIO_InitStruct;
	ADC_InitTypeDef ADC_InitStruct;
	ADC_CommonInitTypeDef ADC_CommonInitStruct;

	/* Enable peripheral clocks */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);

	/* Configure PA0 & PA1 as analog input */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* ADC common configuration */
	ADC_CommonStructInit(&ADC_CommonInitStruct);
	ADC_CommonInit(&ADC_CommonInitStruct);

	/* ADC1 & ADC2 configuration */
	ADC_InitStruct.ADC_ScanConvMode = ENABLE;
	ADC_InitStruct.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStruct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_TRGO;
	ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStruct.ADC_NbrOfConversion = 1;
	ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;
	ADC_Init(ADC1, &ADC_InitStruct);
	ADC_Init(ADC2, &ADC_InitStruct);

	/* ADC1 regular channel 0 configuration */
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_3Cycles);
	/* ADC2 regular channel 1 configuration */
	ADC_RegularChannelConfig(ADC2, ADC_Channel_1, 1, ADC_SampleTime_3Cycles);

	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);
	/* Enable ADC2 */
	ADC_Cmd(ADC2, ENABLE);

	/* Trigger first Conversion */
	ADC_SoftwareStartConv(ADC1);
	ADC_SoftwareStartConv(ADC2);
}


/*********************************************************************
 *
 *					MAIN FUNCTION
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

    /* Init Interrupt */
    INT_TIM_Config();

    /* Init ADC1 Channel 0 */
    ADC123_Init();

    /* Configure Encoder with Tim8 on GPIOC 6 & 7 */
    configureEncoder();

    GPIO_SetBits(GPIOB, GPIO_Pin_10);


    while (1) {

    }
}
