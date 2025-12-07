/*
 * myVoltageRead.c
 *
 *  Created on: Oct 3, 2025
 *      Author: slagt
 */


#include "Drivers/myVoltageRead.h"

/*Функция чтения напряжения. Возвращает напряжение в [В] */
float myVoltageRead(ADC_HandleTypeDef* hadc){
	uint32_t adcVal = ADC_CH2_Read(hadc);
	float vadc = (adcVal/ADCMAXVAL)*VREF;
	float vout = vadc/VIN_FROM_VADC;

	return vout;
};
