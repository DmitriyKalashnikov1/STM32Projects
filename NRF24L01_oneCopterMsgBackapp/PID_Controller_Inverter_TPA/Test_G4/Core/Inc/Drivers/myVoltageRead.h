/*
 * myVoltageRead.h
 *
 *  Created on: Sep 12, 2025
 *      Author: slagt
 */

#ifndef INC_DRIVERS_MYVOLTAGEREAD_H_
#define INC_DRIVERS_MYVOLTAGEREAD_H_

#define MAX_VOLTAGE 20
#define MIN_VOLTAGE 10
#define VIN_FROM_VADC 0.0855
#define VREF 3.3
#define ADCMAXVAL 4096

#include "main.h"
#include "ADC.h"

float myVoltageRead(ADC_HandleTypeDef* hadc);


#endif
