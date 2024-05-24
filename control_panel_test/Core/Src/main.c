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

/*
 * PINOUT:
 * ZoomC -> A_ODR15
 * SP1-SP5 leds -> B_ODR0-2, B_ODR4,5
 * BAT1-5 leds ->  B_ODR6,7,8,9,10
 * BZ, SP+, SP- -> B_ODR11, A_ODR6, A_ODR7
 * VBAT analog -> A_ODR4
 * jA, jB analog -> A_ODR0, A_ODR1
 * */

// setup SWD printf
#include <stdio.h>

#define ITM_Port8(n)    (*((volatile unsigned char *)(0xE0000000+4*n)))
#define ITM_Port16(n)   (*((volatile unsigned short*)(0xE0000000+4*n)))
#define ITM_Port32(n)   (*((volatile unsigned long *)(0xE0000000+4*n)))

#define DEMCR           (*((volatile unsigned long *)(0xE000EDFC)))
#define TRCENA          0x01000000

struct __FILE {
	int handle; /* Add whatever you need here */
};
FILE __stdout;
FILE __stdin;

int fputc(int ch, FILE *f) {
	if (DEMCR & TRCENA) {
		while (ITM_Port32(0) == 0)
			;
		ITM_Port8(0) = ch;
	}
	return (ch);
}
// end setup swd printf

__IO uint32_t SysTick_CNT = 0; //SysTick tick count var
__IO uint32_t speedDisplay = 0; // Speed for out to display leds

void SysTick_Handler() {
	if (SysTick_CNT > 0) {
		SysTick_CNT--;
	}
}

void setupSysTick() {
	SysTick->LOAD &= ~SysTick_LOAD_RELOAD_Msk; // Reset Load val
	SysTick->LOAD = SystemCoreClock / (1000 - 1); // Set timer period to 1 ms
	SysTick->VAL &= ~SysTick_VAL_CURRENT_Msk; // reset Val
	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk
			| SysTick_CTRL_ENABLE_Msk; // select core clock for timer, enable irq, start timer
}

void ADC_setup() {
	ADC1->CR2 |= (ADC_CR2_ADON); // turn on adc
	//	 adc calib
	ADC1->CR2 |= (ADC_CR2_CAL); // start calib
	while (!(ADC1->SR & ADC_CR2_CAL))
		; //wait for end of calib
//	//enable adc interrupt
//	ADC1->CR1 |= ADC_CR1_EOCIE;
//	NVIC_EnableIRQ(ADC1_2_IRQn);
	ADC1->CR2 |= (ADC_CR2_EXTSEL); // adc started on SWSTART bit
	ADC1->CR2 |= (ADC_CR2_EXTTRIG); // enable external adc turn on for single convert
	ADC1->CR2 &= ~(ADC_CR2_CONT); // continious mode
	ADC1->CR2 &= ~(ADC_CR2_ADON); // turn of adc
}

void delay_ms(uint32_t ms) {
	SysTick->VAL &= ~SysTick_VAL_CURRENT_Msk; // reset Val
	// set delay period
	SysTick->VAL = SystemCoreClock / (1000 - 1); // Set timer period to 1 ms
	SysTick_CNT = ms;
	while (SysTick_CNT)
		; //sleep for delay time
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

void beep(uint32_t count, uint32_t ms) { // function for zummer signal
	for (int f = 0; f < count; f++) {
		GPIOA->ODR |= GPIO_ODR_ODR15;
		delay_ms(ms);
		GPIOA->ODR &= ~GPIO_ODR_ODR15;
		delay_ms(ms);
	}
}

void displaySpeed(uint32_t speed) { // function for display speed
	switch (speed) {
	case 0:
		GPIOB->ODR |= GPIO_ODR_ODR0;
		GPIOB->ODR &= ~GPIO_ODR_ODR1;
		GPIOB->ODR &= ~GPIO_ODR_ODR2;
		GPIOB->ODR &= ~GPIO_ODR_ODR4;
		GPIOB->ODR &= ~GPIO_ODR_ODR5;
		break;
	case 1:
		GPIOB->ODR |= GPIO_ODR_ODR0;
		GPIOB->ODR |= GPIO_ODR_ODR1;
		GPIOB->ODR &= ~GPIO_ODR_ODR2;
		GPIOB->ODR &= ~GPIO_ODR_ODR4;
		GPIOB->ODR &= ~GPIO_ODR_ODR5;
		break;
	case 2:
		GPIOB->ODR |= GPIO_ODR_ODR0;
		GPIOB->ODR |= GPIO_ODR_ODR1;
		GPIOB->ODR |= GPIO_ODR_ODR2;
		GPIOB->ODR &= ~GPIO_ODR_ODR4;
		GPIOB->ODR &= ~GPIO_ODR_ODR5;
		break;
	case 3:
		GPIOB->ODR |= GPIO_ODR_ODR0;
		GPIOB->ODR |= GPIO_ODR_ODR1;
		GPIOB->ODR |= GPIO_ODR_ODR2;
		GPIOB->ODR |= GPIO_ODR_ODR4;
		GPIOB->ODR &= ~GPIO_ODR_ODR5;
		break;
	case 4:
		GPIOB->ODR |= GPIO_ODR_ODR0;
		GPIOB->ODR |= GPIO_ODR_ODR1;
		GPIOB->ODR |= GPIO_ODR_ODR2;
		GPIOB->ODR |= GPIO_ODR_ODR4;
		GPIOB->ODR |= GPIO_ODR_ODR5;
		break;
	}
}

uint32_t ADC_Read_VBAT() {
	ADC1->CR2 &= ~(ADC_CR2_ADON); // turn of adc
	// select 4 channel
	ADC1->SQR1 = 0; // 1 regular channel
	ADC1->SQR2 = 0x00000000;
	ADC1->SQR3 = 0x00000100; // 1 convert - channel 4
	ADC1->CR2 |= (ADC_CR2_ADON); // turn on adc
	ADC1->CR2 |= ADC_CR2_SWSTART; // start adc convert
	while (!(ADC1->SR & ADC_SR_EOC)) {
	}; //wait for end of convert

	return ADC1->DR; // read adc data
}

uint32_t ADC_Read_JoystickA() {
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

uint32_t ADC_Read_JoystickB() {
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

void displayBat(uint32_t batVal) {
	if (batVal < 500) {
		GPIOB->ODR |= GPIO_ODR_ODR6;
		GPIOB->ODR &= ~GPIO_ODR_ODR7;
		GPIOB->ODR &= ~GPIO_ODR_ODR8;
		GPIOB->ODR &= ~GPIO_ODR_ODR9;
		GPIOB->ODR &= ~GPIO_ODR_ODR10;
	} else if ((batVal >= 500) && (batVal < 1500)) {
		GPIOB->ODR |= GPIO_ODR_ODR6;
		GPIOB->ODR |= GPIO_ODR_ODR7;
		GPIOB->ODR &= ~GPIO_ODR_ODR8;
		GPIOB->ODR &= ~GPIO_ODR_ODR9;
		GPIOB->ODR &= ~GPIO_ODR_ODR10;
	} else if ((batVal >= 1500) && (batVal < 2000)) {
		GPIOB->ODR |= GPIO_ODR_ODR6;
		GPIOB->ODR |= GPIO_ODR_ODR7;
		GPIOB->ODR |= GPIO_ODR_ODR8;
		GPIOB->ODR &= ~GPIO_ODR_ODR9;
		GPIOB->ODR &= ~GPIO_ODR_ODR10;
	} else if ((batVal >= 2000) && (batVal < 3000)) {
		GPIOB->ODR |= GPIO_ODR_ODR6;
		GPIOB->ODR |= GPIO_ODR_ODR7;
		GPIOB->ODR |= GPIO_ODR_ODR8;
		GPIOB->ODR |= GPIO_ODR_ODR9;
		GPIOB->ODR &= ~GPIO_ODR_ODR10;
	} else if ((batVal >= 300)) {
		GPIOB->ODR |= GPIO_ODR_ODR6;
		GPIOB->ODR |= GPIO_ODR_ODR7;
		GPIOB->ODR |= GPIO_ODR_ODR8;
		GPIOB->ODR |= GPIO_ODR_ODR9;
		GPIOB->ODR |= GPIO_ODR_ODR10;
	}
}

int main(void) {
	//setup perif
	// setup clock system
	setupRCCTo72MHz();
	setupSysTick();
	RCC->APB2ENR |= (RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN
			| RCC_APB2ENR_ADC1EN); // turn on clock on ports A, B and ADC1
	//setup PA0 to analog mode
	GPIOA->CRL &= ~(GPIO_CRL_MODE0 | GPIO_CRL_CNF0); //reset PA0 to analog
	//setup PA1 to analog mode
	GPIOA->CRL &= ~(GPIO_CRL_MODE1 | GPIO_CRL_CNF1); //reset PA1 to analog
	//setup PA4 to analog mode
	GPIOA->CRL &= ~(GPIO_CRL_MODE4 | GPIO_CRL_CNF4); //reset PA4 to analog
	// setup leds
	GPIOB->CRL &= ~(GPIO_CRL_MODE0 | GPIO_CRL_CNF0 | GPIO_CRL_MODE1
			| GPIO_CRL_CNF1 | GPIO_CRL_MODE2 | GPIO_CRL_CNF2 |
			GPIO_CRL_MODE4 | GPIO_CRL_CNF4 | GPIO_CRL_MODE5 | GPIO_CRL_CNF5
			| GPIO_CRL_MODE6 | GPIO_CRL_CNF6 |
			GPIO_CRL_MODE7 | GPIO_CRL_CNF7);
	GPIOB->CRH &= ~( GPIO_CRH_MODE8 | GPIO_CRH_CNF8 | GPIO_CRH_MODE9
			| GPIO_CRH_CNF9 |
			GPIO_CRH_MODE10 | GPIO_CRH_CNF10); //reset leds

	GPIOB->CRL |= (GPIO_CRL_MODE0_1 | GPIO_CRL_MODE1_1 | GPIO_CRL_MODE2_1
			| GPIO_CRL_MODE4_1 | GPIO_CRL_MODE5_1 |
			GPIO_CRL_MODE6_1 | GPIO_CRL_MODE7_1);
	GPIOB->CRH |= (GPIO_CRH_MODE8_1 | GPIO_CRH_MODE9_1 | GPIO_CRH_MODE10_1); //leds to output with max speed 2MHz

	// setup buttons
	GPIOB->CRH &= ~(GPIO_CRH_MODE11 | GPIO_CRH_CNF11); //reset BZ
	GPIOA->CRL &= ~(GPIO_CRL_MODE6 | GPIO_CRL_CNF6 | GPIO_CRL_MODE7
			| GPIO_CRL_CNF7); //reset Sp+, Sp-

	GPIOB->CRH |= GPIO_CRH_CNF11_1; //Bz to input pull down
	GPIOB->ODR |= GPIO_ODR_ODR11;
	GPIOA->CRL |= (GPIO_CRL_CNF6_0 | GPIO_CRL_CNF7_0); //Sp+, Sp- to input pull down
	GPIOB->ODR |= (GPIO_ODR_ODR6 | GPIO_ODR_ODR7);

	ADC_setup();

	beep(2, 500);

	while (1) {
		uint32_t jA = ADC_Read_JoystickA();
		uint32_t jB = ADC_Read_JoystickB();
		uint32_t vBat = ADC_Read_VBAT();
		printf(
				"Joystick Alpha 1: %lu, Joystick Beta 1: %lu, vBat adc data: %lu, speed: %lu",
				jA, jB, vBat, speedDisplay);

		displayBat(vBat);

		// zomer
		if ((GPIOB->IDR & GPIO_IDR_IDR11) != 0) {
			beep(1, 1000);
		}

		//Sp+
		if ((GPIOB->IDR & GPIO_IDR_IDR6) != 0) {
			if (speedDisplay < 5) {
				speedDisplay++;
			}
		}
		//Sp+
		if ((GPIOB->IDR & GPIO_IDR_IDR7) != 0) {
			if (speedDisplay > 0) {
				speedDisplay--;
			}
		}

		displaySpeed(speedDisplay);

	}
}
