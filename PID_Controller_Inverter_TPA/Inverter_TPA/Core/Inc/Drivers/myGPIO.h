/*
 * myGPIO.h
 *
 *  Created on: Mar 28, 2025
 *      Author: slagt
 */

#ifndef GPIO_MYGPIO_H_
#define GPIO_MYGPIO_H_

#define debounceMicros 10000
#define debounce_filter_size 3

#define IS_BUTTON_PULLED_UP 1

//void GPIO_Init (void);
void GPIO_Set (GPIO_TypeDef* gpio, uint16_t pin);
void GPIO_Reset (GPIO_TypeDef* gpio, uint16_t pin);

void DelayMicro(__IO uint32_t micros);
int pinRead(GPIO_TypeDef* gpio, uint16_t pin);
int pinReadDebounce(GPIO_TypeDef* gpio, uint16_t pin, int isPullUp);

#endif /* GPIO_MYGPIO_H_ */
