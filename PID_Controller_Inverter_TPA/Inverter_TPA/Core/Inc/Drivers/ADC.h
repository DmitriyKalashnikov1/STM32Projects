/*
 * ADC.h
 *
 *  Created on: Oct 3, 2025
 *      Author: slagt
 */

#ifndef INC_DRIVERS_ADC_H_
#define INC_DRIVERS_ADC_H_
#include "stm32g474xx.h"
#include "stm32g4xx_hal.h"

uint32_t ADC_CH1_Read(ADC_HandleTypeDef* hadc);
uint32_t ADC_CH2_Read(ADC_HandleTypeDef* hadc);


#endif /* INC_DRIVERS_ADC_H_ */
