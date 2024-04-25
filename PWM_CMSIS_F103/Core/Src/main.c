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

/* Private includes ----------------------------------------------------------*/

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void delay(){
	for(uint32_t i=0; i < 250000; i++){}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	// setup RCC
	RCC->CR |= RCC_CR_HSEON; // turn on ext. clock
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

	TIM1->CR1 &= ~(TIM_CR1_UDIS | TIM_CR1_URS); // not gererate event or interrupt
	TIM1->CR1 &= ~(TIM_CR1_OPM); //one pulse mode off (Counter will not stop on every update)
	TIM1->CR1 &= ~(TIM_CR1_DIR); //counter used as upcounter
	TIM1->CR1 &= ~(TIM_CR1_CMS); // edge-aligned mode
	TIM1->CR1 |= TIM_CR1_ARPE; //TIM1_ARR register is buffered

	//TIM1->CR1 |= TIM_CR1_CEN // start timer, start PWM

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

