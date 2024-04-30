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
#define SystemCoreClock 72000000UL

__IO uint32_t SysTick_CNT = 0; //SysTick tick count var

void SysTick_Handler(){
	if (SysTick_CNT > 0){
		SysTick_CNT--;
	}
}

void setupSysTick(){
	SysTick->LOAD &= ~SysTick_LOAD_RELOAD_Msk; // Reset Load val
	SysTick->LOAD = SystemCoreClock/(1000 - 1); // Set timer period to 1 ms
	SysTick->VAL &= ~SysTick_VAL_CURRENT_Msk; // reset Val
	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk; // select core clock for timer, enable irq, start timer
}



void delay_ms(uint32_t ms){
	SysTick->VAL &= ~SysTick_VAL_CURRENT_Msk; // reset Val
	// set delay period
	SysTick->VAL = SystemCoreClock/(1000 - 1); // Set timer period to 1 ms
	SysTick_CNT = ms;
	while(SysTick_CNT); //sleep for delay time
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



int main(void){
	setupRCCTo72MHz();
	setupSysTick();
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;  // turn on clock on port A

	GPIOA->CRL &= ~(GPIO_CRL_MODE0 | GPIO_CRL_CNF0 | GPIO_CRL_MODE1 | GPIO_CRL_CNF1); //reset PA0, PA1

	GPIOA->CRL |= GPIO_CRL_MODE0_1; //PA0 to output with max speed 2MHz
	GPIOA->CRL |= GPIO_CRL_MODE1_1; //PA1 to output with max speed 2MHz

	GPIOA->ODR |= GPIO_ODR_ODR0; // PA0 to HIGH

	while(1){
		GPIOA->ODR ^= GPIO_ODR_ODR1; //Invert state of PA1;
		delay_ms(1000); // delay 1s

	}
}
