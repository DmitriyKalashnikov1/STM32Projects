/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "stm32f103xb.h"

void delay() {
	for (uint32_t i = 0; i < 500000; i++) {
	}
}

void setupRCCTo72MHz() {
	RCC->CR |= RCC_CR_HSEON; // turn on ext. clock
	//Enable PLL
	RCC->CR |= RCC_CR_PLLON;
	// PLL Source = HSE
	RCC->CFGR |= RCC_CFGR_PLLSRC;
	// HSE clock isn't divided
	RCC->CFGR &= ~(RCC_CFGR_PLLXTPRE);
	//set PLLMUL to 9
	RCC->CFGR |= (RCC_CFGR_PLLMULL9);
	//set PLL as system clock
	RCC->CFGR |= (RCC_CFGR_SW_1);
	//set APB1 prescaller to 2
	RCC->CFGR |= (RCC_CFGR_PPRE1_2);
	// setup adc prescaller to 8
	RCC->CFGR |= RCC_CFGR_ADCPRE_DIV8;

}
uint32_t ADC_Channel0_Read() {
	ADC1->CR2 &= ~(ADC_CR2_ADON); // turn of adc
	// select 0 channel
	ADC1->SQR1 = 0; // 1 regular channel
	ADC1->SQR2 = 0x00000000;
	ADC1->SQR3 = 0x00000000; // 1 convert - channel 0
	ADC1->CR2 |= (ADC_CR2_ADON); // turn on adc
	ADC1->CR2 |= ADC_CR2_SWSTART; // start adc convert
	while (!(ADC1->SR & ADC_SR_EOC)) {
	}; //wait for end of convert

	return ADC1->DR; // read adc data
}

uint32_t ADC_Channel1_Read() {
	ADC1->CR2 &= ~(ADC_CR2_ADON); // turn of adc
	// select 1 channel
	ADC1->SQR1 = 0; // 1 regular channel
	ADC1->SQR2 = 0x00000000;
	ADC1->SQR3 = 0x00000001; // 1 convert - channel 1
	ADC1->CR2 |= (ADC_CR2_ADON); // turn on adc
	ADC1->CR2 |= ADC_CR2_SWSTART; // start adc convert
	while (!(ADC1->SR & ADC_SR_EOC)) {
	}; //wait for end of convert

	return ADC1->DR; // read adc data
}

void ADC_setup() {
	ADC1->CR2 |= (ADC_CR2_ADON); // turn on adc
	//	 adc calib
//	ADC1->CR2 |= (ADC_CR2_CAL); // start calib
//	while(!(ADC1->SR & ADC_CR2_CAL)); //wait for end of calib
//	//enable adc interrupt
//	ADC1->CR1 |= ADC_CR1_EOCIE;
//	NVIC_EnableIRQ(ADC1_2_IRQn);
	ADC1->CR2 |= (ADC_CR2_EXTSEL); // adc started on SWSTART bit
	ADC1->CR2 |= (ADC_CR2_EXTTRIG); // enable external adc turn on for single convert
	ADC1->CR2 &= ~(ADC_CR2_CONT); // continious mode
	ADC1->CR2 &= ~(ADC_CR2_ADON); // turn of adc
}

int main(void) {
	setupRCCTo72MHz();
	RCC->APB2ENR |= (RCC_APB2ENR_IOPAEN | RCC_APB2ENR_ADC1EN); // turn on clock on port A and ADC1
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN; // turn on clock on alternative funcs (interrupts)
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; // turn on clock of timer 1
	//setup PA0 to analog mode
	GPIOA->CRL &= ~(GPIO_CRL_MODE0 | GPIO_CRL_CNF0); //reset PA0 to analog
	//setup PA1 to analog mode
	GPIOA->CRL &= ~(GPIO_CRL_MODE1 | GPIO_CRL_CNF1); //reset PA1 to analog
	//setup PA8 to alternative push-pull output with max speed 50 MHz (need for PWM)
	GPIOA->CRH &= ~(GPIO_CRH_MODE8 | GPIO_CRH_CNF8); //reset PA8
	GPIOA->CRH |= (GPIO_CRH_MODE8_0 | GPIO_CRH_MODE8_1 | GPIO_CRH_CNF8_1); //alternative push-pull output with max speed 50 MHz
	//setup PA9 to alternative push-pull output with max speed 50 MHz (need for PWM)
	GPIOA->CRH &= ~(GPIO_CRH_MODE9 | GPIO_CRH_CNF9); //reset PA9
	GPIOA->CRH |= (GPIO_CRH_MODE9_0 | GPIO_CRH_MODE9_1 | GPIO_CRH_CNF9_1); //alternative push-pull output with max speed 50 MHz
	//general setup of tim1

	TIM1->CR1 &= ~(TIM_CR1_UDIS | TIM_CR1_URS); // not gererate general event or interrupt
	TIM1->CR1 &= ~(TIM_CR1_OPM); //one pulse mode off (Counter will not stop on every update)
	TIM1->CR1 &= ~(TIM_CR1_DIR); //counter used as upcounter
	TIM1->CR1 &= ~(TIM_CR1_CMS); // edge-aligned mode
	// setup timer freq
	TIM1->PSC = 1 - 1; // 72Mhz / 1 = 72MHz -> first prescaler
	TIM1->ARR = 4096 - 1; // 72MHz / 4096 = 17.57kHz -> second prescaler
	// (max tick count in one PWM Pulse) --> finish freq = 17.57kHz,
	//setup PWM
	////setup chanels
	TIM1->CCMR1 &= ~(TIM_CCMR1_CC1S); // chanel 1 to output
	TIM1->CCMR1 &= ~(TIM_CCMR1_CC2S); // chanel 2 to output
	TIM1->BDTR |= TIM_BDTR_MOE; // assing timer output to pins
	//// setup PWM mode 1
	TIM1->CCMR1 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1); // chanel 1 to PWM mode 1
	TIM1->CCMR1 |= (TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1); // chanel 2 to PWM mode 1
	//turn on PWM (active -> high)
	TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC1P);
	TIM1->CCER |= (TIM_CCER_CC2E | TIM_CCER_CC2P);
	//set duty to chanels
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;

	TIM1->CR1 |= TIM_CR1_CEN; // start timer
	// setup adc
	ADC_setup();

	while (1) {
		uint32_t res1 = ADC_Channel0_Read();
		TIM1->CCR1 = res1;
		uint32_t res2 = ADC_Channel1_Read();
		TIM1->CCR2 = res2;
		//TIM1->CCR1 = ADC_Read();
//		delay();

	}
}
