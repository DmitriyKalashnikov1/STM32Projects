/*
 * myVoltageRead.c
 *
 *  Created on: Oct 3, 2025
 *      Author: slagt
 */


#include "Drivers/myVoltageRead.h"

/*Функция чтения напряжения. Возвращает напряжение в [В] */
float myVoltageRead(ADC_HandleTypeDef* hadc){
	float adcVal = ADC_CH2_Read(hadc);
	float vadc = (adcVal/(float)ADCMAXVAL)*(float)VREF;
	float vout = vadc/VIN_FROM_VADC;

	return vout;
};
