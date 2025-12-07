/*
 * ACS712.c
 *
 *  Created on: Sep 12, 2025
 *      Author: slagt
 */

#include "Drivers/ACS712.h"

/*Функция чтения тока. Возвращает ток в [A] */
float ACS712_ReadCurrentDC(ADC_HandleTypeDef* hadc){
	  int sensorValue = 0;
	  float sensorCurrent = 0;
	  for (int i = 0; i < ACS712_20A_SAMPLE_TIMES; i++) {
		  sensorValue += ADC_CH1_Read(hadc);
		  //HAL_Delay(1);
	  }
	  sensorValue = sensorValue >> ACS712_20A_SHIFT;
	  sensorCurrent = (sensorValue - ADC_COUNT_HALF) / ADC_COUNT_ACS712_1A;
	  return sensorCurrent;

};

/*Функция чтения напряжения. Возвращает напряжение в [В] */
float ACS712_ReadVoltageDC(ADC_HandleTypeDef* hadc){
      float current = ACS712_ReadCurrentDC(hadc);
      float mVolt = current * ACS712_20A_CurrentToVoltageConstant;
      float voltage = mVolt / 1000;

      return voltage;
}

/*Функция вывода отладочной информации */
void  ACS712_DebugPrintf(ADC_HandleTypeDef* hadc){
	int adc_raw = ADC_CH1_Read(hadc);
	float current = ACS712_ReadCurrentDC(hadc);
	float voltage = ACS712_ReadVoltageDC(hadc);
	printf("\n\tACS712 Debug Info \n\tADC_RAW: %i, \n\tCurrent [A]: %.3f, \n\tVoltage [V]: %.3f \n\tEnd of ACS712 Debug Info", adc_raw, current, voltage);
}
