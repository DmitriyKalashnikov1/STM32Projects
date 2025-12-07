/*
 * ADC.c
 *
 *  Created on: Oct 3, 2025
 *      Author: slagt
 */


#include "Drivers/ADC.h"

/*Функция чтения с АЦП 1-ого канала */
uint32_t ADC_CH1_Read(ADC_HandleTypeDef* hadc){
	HAL_ADCEx_InjectedStart(hadc);
	HAL_ADCEx_InjectedPollForConversion(hadc, 10);
	uint32_t res = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
	return res;
}

/*Функция чтения с АЦП 2-ого канала */
uint32_t ADC_CH2_Read(ADC_HandleTypeDef* hadc){
	HAL_ADCEx_InjectedStart(hadc);
	HAL_ADCEx_InjectedPollForConversion(hadc, 10);
	uint32_t res = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
	return res;
}
