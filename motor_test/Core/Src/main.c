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
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void delay(){
	for(uint32_t i=0; i < 2000000; i++){}
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

}

//void TIM1_UP_IRQHandler(){
//	if ((TIM1->SR & TIM_SR_UIF) != 0){
//		TIM1->SR &= ~TIM_SR_UIF; // exit from interrupt
//	}
//
//}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	// setup RCC
	setupRCCTo72MHz();
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;  // turn on clock on port A
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN; // turn on clock on alternative funcs (interrupts)
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; // turn on clock of timer 1

	//setup PA8-PA11 to alternative push-pull output with max speed 50 MHz (need for PWM)
	GPIOA->CRH &= ~(GPIO_CRH_MODE8 | GPIO_CRH_CNF8 | GPIO_CRH_MODE9 | GPIO_CRH_CNF9); //reset PA8-PA9
	GPIOA->CRH &= ~(GPIO_CRH_MODE10 | GPIO_CRH_CNF10 | GPIO_CRH_MODE11 | GPIO_CRH_CNF11); //reset PA10-PA11

	GPIOA->CRH |= (GPIO_CRH_MODE8_0 | GPIO_CRH_MODE8_1 | GPIO_CRH_CNF8_1 ); //alternative push-pull output with max speed 50 MHz
	GPIOA->CRH |= (GPIO_CRH_MODE9_0 | GPIO_CRH_MODE9_1 | GPIO_CRH_CNF9_1 ); //alternative push-pull output with max speed 50 MHz
	GPIOA->CRH |= (GPIO_CRH_MODE10_0 | GPIO_CRH_MODE10_1 | GPIO_CRH_CNF10_1 ); //alternative push-pull output with max speed 50 MHz
	GPIOA->CRH |= (GPIO_CRH_MODE11_0 | GPIO_CRH_MODE11_1 | GPIO_CRH_CNF11_1 ); //alternative push-pull output with max speed 50 MHz

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
	TIM1->ARR = 500 - 1; // 18MHz / 500 = 36kHz -> second prescaler
	// (max tick count in one PWM Pulse) --> finish freq = 36kHz,

//	NVIC_EnableIRQ(TIM1_UP_IRQn); //enable update irq



	//setup PWM
	////setup chanels
	TIM1->CCMR1 &= ~(TIM_CCMR1_CC1S); // chanel 1 to output
	TIM1->CCMR1 &= ~(TIM_CCMR1_CC2S); // chanel 2 to output
	TIM1->CCMR2 &= ~(TIM_CCMR2_CC3S); //chanel 3 to output
	TIM1->CCMR2 &= ~(TIM_CCMR2_CC4S); //chanel 4 to output
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
	TIM1->CCMR1 |= (TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1); // chanel 2 to PWM mode 1
	TIM1->CCMR2 |= (TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1); // chanel 3 to PWM mode 1
	TIM1->CCMR2 |= (TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1); // chanel 4 to PWM mode 1
	//turn on PWM (active -> high)
	TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC1P);
	TIM1->CCER |= (TIM_CCER_CC2E | TIM_CCER_CC2P);
	TIM1->CCER |= (TIM_CCER_CC3E | TIM_CCER_CC3P);
	TIM1->CCER |= (TIM_CCER_CC4E | TIM_CCER_CC4P);

	//set duty to chanels
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;
	TIM1->CCR4 = 255;

	TIM1->CR1 |= TIM_CR1_CEN; // start timer

  while (1)
  {

	  for (uint32_t f = 0; f < 500; f++){
		  TIM1->CCR1 = f;
		  delay();
	  }
	  for (uint32_t f = 500; f > 0 ; f--){
		  TIM1->CCR1 = f;
		  delay();
	  }

  }

}

