/*
 * pinmux.c
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
 *					PWM pins
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
 *					Timer1 for 16kHz PWM
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
 *					PWM pins with Tim1 and Deadtime
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
    TIM_BDTRStruct.TIM_DeadTime = 0b11000000 | 0b00001010;
    TIM_BDTRStruct.TIM_Break = TIM_Break_Enable;
    TIM_BDTRStruct.TIM_BreakPolarity = TIM_BreakPolarity_High;
    TIM_BDTRStruct.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;

    TIM_BDTRConfig(TIM1, &TIM_BDTRStruct);

    /* Main Output enable */
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
}


/*********************************************************************
 *
 *					Timer2 as Interrupt
 *
 ********************************************************************/

void INT_TIM2_Config(void) {

	NVIC_InitTypeDef NVIC_InitStruct;
	TIM_TimeBaseInitTypeDef TIM_BaseStruct;

	/* Enable the TIM2 global Interrupt */
	NVIC_InitStruct.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x01;
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
 *					Resolver & Interrupt Pin C8 for Z
 *
 ********************************************************************/

void configureEncoder(void) {

    GPIO_InitTypeDef GPIO_InitStruct;
    EXTI_InitTypeDef EXTI_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;
    TIM_ICInitTypeDef TIM_ICInitStruct;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStruct;

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
    TIM_TimeBaseStruct.TIM_Prescaler = 0;
    TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStruct.TIM_Period = 4095;
    TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStruct);
    TIM_ARRPreloadConfig(TIM8, ENABLE);


    /* Debounce filter */
    TIM_ICInitStruct.TIM_Channel=TIM_Channel_1;
    TIM_ICInitStruct.TIM_ICFilter=3;
    TIM_ICInit(TIM8, &TIM_ICInitStruct);
    TIM_ICInitStruct.TIM_Channel=TIM_Channel_2;
    TIM_ICInitStruct.TIM_ICFilter=3;
    TIM_ICInit(TIM8, &TIM_ICInitStruct);

    /* Setup encoder */
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
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource8);


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
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x02;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x01;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

}


/*********************************************************************
 *
 *					USART2
 *
 ********************************************************************/

void USART2_Init(void) {

	GPIO_InitTypeDef	GPIO_InitStruct;
	USART_InitTypeDef	USART_InitStruct;
	NVIC_InitTypeDef	NVIC_InitStruct;
	USART_ClockInitTypeDef USART_ClockInitstruct;

	/* Enable clock for GPIOA */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	/* Enable clock for USART1 peripheral */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	/* Initialize pins as alternating function */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* Tell pins PA9 and PA10 which alternating function */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

	/**********************************************************
	 * Set Baudrate
	 * Disable Hardware Flow control
	 * Set Mode To TX and RX, so USART will work in full-duplex mode
	 * Disable parity bit
	 * Set 1 stop bit
	 * Set Data bits to 8
	 **********************************************************/
	USART_InitStruct.USART_BaudRate = 115200;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStruct.USART_Parity = USART_Parity_No;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART2, &USART_InitStruct);

	/* Enable RX interrupt for communication ;) */
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

	/**********************************************************
	 * Set Channel to USART1
	 * Set Channel Cmd to enable. That will enable USART1 channel in NVIC
	 * Set Preemption priority to 2. This means third highest priority
	 **********************************************************/
	NVIC_InitStruct.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_Init(&NVIC_InitStruct);

	USART_Cmd(USART2, ENABLE);

}


/*********************************************************************
 *
 *					ADC123
 *
 ********************************************************************/

void ADC123_Init(void) {

	GPIO_InitTypeDef		GPIO_InitStruct;
	ADC_InitTypeDef			ADC_InitStruct;
	ADC_CommonInitTypeDef	ADC_CommonInitStruct;
	DMA_InitTypeDef			DMA_InitStruct;

	/* Enable peripheral clocks */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2 | RCC_APB2Periph_ADC3, ENABLE);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

	/* DMA1 Stream 0 Config
	DMA_DeInit(DMA_Channel_0);
	DMA_InitStruct.DMA_Channel = DMA_Channel_0;
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&ADC3->DR;
	DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)ADC_DualConvertedValueTab;
	DMA_InitStruct */

	/* Configure PA0 & PA1 as analog input */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* Configure PC1 as analog input */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStruct);

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
	ADC_InitStruct.ADC_NbrOfConversion = 2;
	ADC_Init(ADC3, &ADC_InitStruct);

	/* ADC1 regular channel configuration */
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_3Cycles);
	/* ADC2 regular channel configuration */
	ADC_RegularChannelConfig(ADC2, ADC_Channel_1, 1, ADC_SampleTime_3Cycles);
	/* ADC3 regular channel configuration*/
	ADC_RegularChannelConfig(ADC3, ADC_Channel_10, 1, ADC_SampleTime_3Cycles);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_11, 2, ADC_SampleTime_3Cycles);

	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);
	/* Enable ADC2 */
	ADC_Cmd(ADC2, ENABLE);
	/* Enable ADC3 */
	ADC_Cmd(ADC3, ENABLE);
	ADC_DMACmd(ADC3, ENABLE);

	/* Trigger first Conversion */
	ADC_SoftwareStartConv(ADC1);
	ADC_SoftwareStartConv(ADC2);
	ADC_SoftwareStartConv(ADC3);

}
