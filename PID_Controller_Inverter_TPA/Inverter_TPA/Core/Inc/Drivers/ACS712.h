/*
 * ACS712.h
 *
 *  Created on: Sep 12, 2025
 *      Author: slagt
 */

#ifndef INC_DRIVERS_ACS712_H_
#define INC_DRIVERS_ACS712_H_

#include "main.h"
#include "math.h"
#include "ADC.h"

#define ADC_COUNT           4096
#define ADC_COUNT_HALF      ADC_COUNT / 2

// do to: более точно настроить эти константы
// если вычисленные значения не будут точными
#define ADC_COUNT_ACS712_1A 37.88

#define ACS712_20A_CurrentToVoltageConstant 100

#define ACS712_20A_MAX_CURRENT 19
#define ACS712_20A_MIN_CURRENT 5
// параметры оптимизированного битовым сдвигом усреднения значений
#define ACS712_20A_SHIFT 5

#define ACS712_20A_SAMPLE_TIMES pow(2.0, ACS712_20A_SHIFT)




float ACS712_ReadCurrentDC(ADC_HandleTypeDef* hadc);
float ACS712_ReadVoltageDC(ADC_HandleTypeDef* hadc);
void  ACS712_DebugPrintf(ADC_HandleTypeDef* hadc);

#endif /* INC_DRIVERS_ACS712_H_ */
