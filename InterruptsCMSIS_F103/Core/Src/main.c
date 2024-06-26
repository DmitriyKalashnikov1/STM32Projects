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
	for(uint32_t i=0; i < 250000; i++){}
}

void EXTI0_IRQHandler(){

	for (int f = 0; f < 3; f++){
		GPIOA->ODR |= GPIO_ODR_ODR1; //set PA1;
		delay(); // delay 1s
		GPIOA->ODR &= ~GPIO_ODR_ODR1; //reset PA1;
		delay(); // delay 1s
	}
	EXTI->PR |= EXTI_PR_PR0; // SET FLAG FOR EXITING FROM INTERRRUPT
}

void EXTI1_IRQHandler(){

	GPIOA->ODR |= GPIO_ODR_ODR1; //set PA1;
	delay(); // delay 1s
	GPIOA->ODR &= ~GPIO_ODR_ODR1; //reset PA1;
	delay(); // delay 1s
	EXTI->PR |= EXTI_PR_PR1; // SET FLAG FOR EXITING FROM INTERRRUPT
}

int main(){
	// setup RCC
	RCC->CR |= RCC_CR_HSEON; // turn on ext. clock
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;  // turn on clock on port A
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;  // turn on clock on port B
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN; // turn on clock on alternative funcs (interrupts)

	// PA0, PA1 to output
	GPIOA->CRL &= ~(GPIO_CRL_MODE0 | GPIO_CRL_CNF0 | GPIO_CRL_MODE1 | GPIO_CRL_CNF1); //reset PA0, PA1

	GPIOA->CRL |= GPIO_CRL_MODE0_1; //PA0 to output with max speed 2MHz
	GPIOA->CRL |= GPIO_CRL_MODE1_1; //PA1 to output with max speed 2MHz

	GPIOA->ODR |= GPIO_ODR_ODR0; // PA0 to HIGH

	// PB0, PB1 to input

	GPIOB->CRL &= ~(GPIO_CRL_MODE0 | GPIO_CRL_CNF0 | GPIO_CRL_MODE1 | GPIO_CRL_CNF1); //reset PB0, PB1

	GPIOB->CRL |= GPIO_CRL_CNF0_0; //PA0 to input floating
	GPIOB->CRL |= GPIO_CRL_CNF1_0; //PA1 to input floating

	// setup EXTI (interrupts)
	// setup pins for interrupts
	AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI0_PB; // PB0
	AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI1_PB; // PB1

	//enable EXTI
	EXTI->IMR |= EXTI_IMR_MR0; // EXTI0
	EXTI->IMR |= EXTI_IMR_MR1; // EXTI1

	// select interrupts fronts
	EXTI->FTSR |= EXTI_FTSR_TR0; // EXTI0 -> FALLING
	EXTI->FTSR |= EXTI_FTSR_TR1; // EXTI1 -> FALLING

	// turn on interrupts handlers
	NVIC_EnableIRQ(EXTI0_IRQn);
	NVIC_EnableIRQ(EXTI1_IRQn);

	// turn on interrupts
	__enable_irq();

	while (1){


	}
}
