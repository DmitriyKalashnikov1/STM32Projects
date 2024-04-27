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

void delay(){
	for(uint32_t i=0; i < 1000000; i++){}
}

void setupRCCTo72MHz(){
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

uint16_t ADC_Read(){
	uint16_t temp =  0;
	ADC1->CR2 |= ADC_CR2_SWSTART; // start adc convert
	while(!(ADC1->SR & ADC_SR_EOC)); //wait for end of convert
	temp = ADC1->DR; // read adc data
	return temp;
}


int main(void){
	setupRCCTo72MHz();
	RCC->APB2ENR |= (RCC_APB2ENR_IOPAEN | RCC_APB2ENR_ADC1EN);  // turn on clock on port A and ADC1
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN; // turn on clock on alternative funcs (interrupts)
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; // turn on clock of timer 1
	//setup PA8-PA11 to alternative push-pull output with max speed 50 MHz (need for PWM)
		GPIOA->CRH &= ~(GPIO_CRH_MODE8 | GPIO_CRH_CNF8); //reset PA8

		GPIOA->CRH |= (GPIO_CRH_MODE8_0 | GPIO_CRH_MODE8_1 | GPIO_CRH_CNF8_1 ); //alternative push-pull output with max speed 50 MHz
		//general setup of tim1

		TIM1->CR1 &= ~(TIM_CR1_UDIS | TIM_CR1_URS); // not gererate general event or interrupt
		TIM1->CR1 &= ~(TIM_CR1_OPM); //one pulse mode off (Counter will not stop on every update)
		TIM1->CR1 &= ~(TIM_CR1_DIR); //counter used as upcounter
		TIM1->CR1 &= ~(TIM_CR1_CMS); // edge-aligned mode

		//TIM1->CR1 |= TIM_CR1_ARPE; //TIM1_ARR register is buffered

		// setup update interrupt
	//	TIM1->DIER |= TIM_DIER_UIE; // update interrupt enable
		// setup timer freq
		TIM1->PSC = 4 - 1; // 72Mhz / 4 = 18MHz -> first prescaler
		TIM1->ARR = 4096 - 1; // 18MHz / 500 = 36kHz -> second prescaler
		// (max tick count in one PWM Pulse) --> finish freq = 36kHz,

	//	NVIC_EnableIRQ(TIM1_UP_IRQn); //enable update irq



		//setup PWM
		////setup chanels
		TIM1->CCMR1 &= ~(TIM_CCMR1_CC1S); // chanel 1 to output
		TIM1->BDTR |= TIM_BDTR_MOE; // assing timer output to pins
	//	////setup preload
	//	TIM1->CCMR1 |= (TIM_CCMR1_OC1PE); // chanel 1 preload enable
	//	TIM1->CCMR1 |= (TIM_CCMR1_OC2PE); // chanel 2 preload enable
	//	TIM1->CCMR2 |= (TIM_CCMR2_OC3PE); // chanel 3 preload enable
	//	TIM1->CCMR2 |= (TIM_CCMR2_OC4PE); // chanel 4 preload enable
	//	//OCxRef is not affected by the ETRF Input
	//	TIM1->CCMR1 &= ~TIM_CCMR1_OC1CE;
	//	TIM1->CCMR1 &= ~TIM_CCMR1_OC2CE;
	//	TIM1->CCMR2 &= ~TIM_CCMR2_OC3CE;
	//	TIM1->CCMR2 &= ~TIM_CCMR2_OC4CE;
		//// setup PWM mode 1
		TIM1->CCMR1 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1); // chanel 1 to PWM mode 1
		//turn on PWM (active -> high)
		TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC1P);

		//set duty to chanels
		TIM1->CCR1 = 0;

		TIM1->CR1 |= TIM_CR1_CEN; // start timer
	// setup adc
	ADC1->CR2 |= (ADC_CR2_EXTSEL_2 | ADC_CR2_EXTSEL_1 | ADC_CR2_EXTSEL_0); // adc started on SWSTART bit
	ADC1->CR2 |= (ADC_CR2_EXTTRIG); // enable external adc turn on for single convert
	ADC1->CR2 |= (ADC_CR2_ADON); // turn on adc
	// adc calib
	ADC1->CR2 |= (ADC_CR2_CAL); // start calib
	while(!(ADC1->SR & ADC_CR2_CAL)); //wait for end of calib

	while(1){
		uint16_t res = ADC_Read(); // read adc data
		TIM1->CCR1 = res;

	}
}
