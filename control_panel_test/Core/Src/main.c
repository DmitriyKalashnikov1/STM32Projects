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

int IsITMAvailable = 0;

int _write(int le, char *ptr, int len){

int DataIdx;

for(DataIdx = 0; DataIdx < len; DataIdx++){
ITM_SendChar(*ptr++);
}


return len;
}

void setupSWO(uint32_t portMask, uint32_t cpuCoreFreqHz, uint32_t baudrate)
{
	uint32_t SWOPrescaler = (cpuCoreFreqHz / baudrate) - 1u ; // baudrate in Hz, note that cpuCoreFreqHz is expected to match the CPU core clock

	CoreDebug->DEMCR = CoreDebug_DEMCR_TRCENA_Msk; 		// Debug Exception and Monitor Control Register (DEMCR): enable trace in core debug
	DBGMCU->CR	= 0x00000027u ;							// DBGMCU_CR : TRACE_IOEN DBG_STANDBY DBG_STOP 	DBG_SLEEP
	TPI->SPPR	= 0x00000002u ;							// Selected PIN Protocol Register: Select which protocol to use for trace output (2: SWO)
	TPI->ACPR	= SWOPrescaler ;						// Async Clock Prescaler Register: Scale the baud rate of the asynchronous output
	ITM->LAR	= 0xC5ACCE55u ;							// ITM Lock Access Register: C5ACCE55 enables more write access to Control Register 0xE00 :: 0xFFC
	ITM->TCR	= 0x0001000Du ;							// ITM Trace Control Register
	ITM->TPR	= ITM_TPR_PRIVMASK_Msk ;				// ITM Trace Privilege Register: All stimulus ports
	ITM->TER	= portMask ;							// ITM Trace Enable Register: Enabled tracing on stimulus ports. One bit per stimulus port.
	DWT->CTRL	= 0x400003FEu ;							// Data Watchpoint and Trace Register
	TPI->FFCR	= 0x00000100u ;							// Formatter and Flush Control Register

	// ITM/SWO works only if enabled from debugger.
	// If ITM stimulus 0 is not free, don't try to send data to SWO
	if (ITM->PORT [0].u8 == 1)
	{
		IsITMAvailable = 1;
	}
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
	//ADC1->CR2 |= (ADC_CR2_CAL); // start calib
	//while (!(ADC1->SR & ADC_CR2_CAL)); //wait for end of calib
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
	// setup clock system and swo
	setupRCCTo72MHz();
	setupSysTick();
	setupSWO(0x1, SystemCoreClock, 2000000);
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

	GPIOB->CRL |= (GPIO_CRL_MODE0_0 | GPIO_CRL_MODE1_0 | GPIO_CRL_MODE2_0
			| GPIO_CRL_MODE4_0 | GPIO_CRL_MODE5_0 |
			GPIO_CRL_MODE6_0 | GPIO_CRL_MODE7_0);
	GPIOB->CRH |= (GPIO_CRH_MODE8_0 | GPIO_CRH_MODE9_0 | GPIO_CRH_MODE10_0); //leds to output with max speed 10MHz

	// setup buttons
	GPIOB->CRH &= ~(GPIO_CRH_MODE11 | GPIO_CRH_CNF11); //reset BZ
	GPIOA->CRL &= ~(GPIO_CRL_MODE6 | GPIO_CRL_CNF6 | GPIO_CRL_MODE7
			| GPIO_CRL_CNF7); //reset Sp+, Sp-

	GPIOB->CRH |= GPIO_CRH_CNF11_0; //Bz to input pull down

	GPIOA->CRL |= (GPIO_CRL_CNF6_0 | GPIO_CRL_CNF7_0); //Sp+, Sp- to input pull down
	ADC_setup();

	beep(2, 500);

	while (1) {
		uint32_t jA = ADC_Read_JoystickA();
		uint32_t jB = ADC_Read_JoystickB();
		uint32_t vBat = ADC_Read_VBAT();
/*		if (IsITMAvailable != 0){
			printf(
					"Joystick Alpha 1: %lu, Joystick Beta 1: %lu, vBat adc data: %lu, speed: %lu \n",
					jA, jB, vBat, speedDisplay);
		}*/

		// zomer
		if ((GPIOB->IDR & GPIO_IDR_IDR11) != 0) {
			beep(1, 1000);
		}

		//Sp+
		if ((GPIOB->IDR & GPIO_IDR_IDR6) != 0) {
				speedDisplay++;
		}
		//Sp+
		if ((GPIOB->IDR & GPIO_IDR_IDR7) != 0) {
				speedDisplay--;
		}

		if (speedDisplay > 4) {
			speedDisplay = 4;
		}else if (speedDisplay < 0){
			speedDisplay = 0;
		}


		displaySpeed(speedDisplay);
		displayBat(vBat);
		delay_ms(100);
	}
}
