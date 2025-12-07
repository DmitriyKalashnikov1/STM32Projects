/*
 * myGPIO.c
 *
 *  Created on: Mar 28, 2025
 *      Author: miros
 */

#include "stm32g4xx.h"
#include "stm32g4xx_hal.h"
#include "Drivers/myGPIO.h"

/*-------------- Функция активации пина --------------*/
void GPIO_Set (GPIO_TypeDef* gpio, uint8_t pin)
{
	gpio->BSRR|=(1<<pin);
}

/*-------------- Функция дизактивации пина --------------*/
void GPIO_Reset (GPIO_TypeDef* gpio, uint8_t pin)
{
	gpio->BSRR|=(1<<(pin + 16));
}

/*-------------- Функция задержки --------------*/
void DelayMicro(__IO uint32_t micros)
{
	micros *= (SystemCoreClock / 10000000) ;
	/* Wait till done */
	while (micros--);
}

// считываем состояние пина, учитывая тип подтяжки
int pinRead(GPIO_TypeDef* gpio, uint8_t pin, int isPullUp){
	if (isPullUp){
		return !(gpio->IDR & pin) ? 1 : 0;
	} else {
		return !(gpio->IDR & pin) ? 0 : 1;
	}

}

/*-------------- Функция чтения состояния пина с антидребезгом --------------*/
int pinReadDebounce(GPIO_TypeDef* gpio, uint8_t pin, int isPullUp){
	int btnState[debounce_filter_size];

	for (int f = 0; f < debounce_filter_size; f++){
		btnState[f] = pinRead(gpio, pin, isPullUp);
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
