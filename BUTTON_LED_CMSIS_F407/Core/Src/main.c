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
#include "stm32f407xx.h"
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
	RCC->CR |=  RCC_CR_HSEON; //turn on ext. clock
	RCC->PLLCFGR |= (4 << RCC_PLLCFGR_PLLM_Pos) | (72 << RCC_PLLCFGR_PLLN_Pos) | (0 << RCC_PLLCFGR_PLLP_Pos) | (4 << RCC_PLLCFGR_PLLQ_Pos); //setup PLL to 72 MHz
	RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE; // PLL tackts from ext. clock
	RCC->CR |=  RCC_CR_PLLON; //enable pll;
	// Switch SYSCLK to PLL
	RCC->CFGR |= RCC_CFGR_SW_PLL;
}


int main(){
	setupRCCTo72MHz();
	setupSysTick();
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // turn on clock on port A

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN; // turn on clock on port E

	GPIOA->MODER &= ~GPIO_MODER_MODER6; // reset pin6
	GPIOA->MODER &= ~GPIO_MODER_MODER7; // reset pin7

	GPIOA->MODER |= GPIO_MODER_MODER6_0; // PA6 to output

	GPIOA->MODER |= GPIO_MODER_MODER7_0; // PA7 to output

	//PE3 to input pull up
	GPIOE->MODER &= ~GPIO_MODER_MODER3; // reset pin3
	GPIOE->PUPDR |= GPIO_PUPDR_PUPD3_0; // input pull up


	while(1){
		GPIOA->ODR ^= GPIO_ODR_ODR_6; // invert state of pin6

		if (!((GPIOE->IDR & GPIO_IDR_ID3) != 0)){
			GPIOA->ODR ^= GPIO_ODR_ODR_7;
			delay_ms(500);

		}
		delay_ms(1000);
	}
}
