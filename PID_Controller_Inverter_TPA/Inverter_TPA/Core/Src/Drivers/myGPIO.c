/*
 * myGPIO.c
 *
 *  Created on: Mar 28, 2025
 *      Author: slagt
 */

#include "stm32g474xx.h"
#include "stm32g4xx_hal.h"
#include "Drivers/myGPIO.h"

/*-------------- Функция активации пина --------------*/
void GPIO_Set (GPIO_TypeDef* gpio, uint16_t pin)
{
	HAL_GPIO_WritePin(gpio, pin, GPIO_PIN_SET);
}

/*-------------- Функция дизактивации пина --------------*/
void GPIO_Reset (GPIO_TypeDef* gpio, uint16_t pin)
{
	HAL_GPIO_WritePin(gpio, pin, GPIO_PIN_RESET);
}

/*-------------- Функция задержки --------------*/
void DelayMicro(__IO uint32_t micros)
{
	micros *= (SystemCoreClock / 10000000) ;
	/* Wait till done */
	while (micros--);
}

// считываем состояние пина, учитывая тип подтяжки
int pinRead(GPIO_TypeDef* gpio, uint16_t pin)
{
	return ((gpio->IDR & pin) == pin) ? 1 : 0;
}

/*-------------- Функция чтения состояния пина с антидребезгом --------------*/
int pinReadDebounce(GPIO_TypeDef* gpio, uint16_t pin, int isPullUp){
	int btnState[debounce_filter_size];

	for (int f = 0; f < debounce_filter_size; f++){
		int pinState = pinRead(gpio, pin);
		if (isPullUp) {pinState = (pinState==1) ? 0 : 1;};
		btnState[f] = pinState;
		DelayMicro(debounceMicros);
	}

	int sum = 0;

	for (int f = 0; f < debounce_filter_size; f++){
		sum += btnState[f];
	}

	if (sum < debounce_filter_size){
		return 0;
	} else {
		return 1;
	}
}
