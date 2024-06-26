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

void delay(){
	for(uint32_t i=0; i < 250000; i++){}
}


int main(){

	RCC->CR |=  RCC_CR_HSEON; //turn on ext. clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // turn on clock on port A
	GPIOA->MODER &= ~GPIO_MODER_MODER6; // reset pin6

	GPIOA->MODER |= GPIO_MODER_MODER6_0; // PA6 to output


	while(1){
		GPIOA->ODR ^= GPIO_ODR_ODR_6; // invert state of pin6
		delay();
	}
}
