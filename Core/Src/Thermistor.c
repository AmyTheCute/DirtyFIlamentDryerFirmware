/*
 * Thermistor.c
 *
 *  Created on: Jun 19, 2024
 *      Author: Bunny
 */


#include "Thermistor.h"

uint8_t array_index = 0;
float temp_history[SAMPLES];

void Thermistor_Init() {
	array_index = 0;
	for(int i = 0; i < SAMPLES; i++){
			temp_history[i] = 25.0;
		}

}
float Thermistor_get_temp() {
	float sum = 0;
	for(int i = 0; i < SAMPLES; i++){
		sum += temp_history[i];
	}

	return sum / SAMPLES;
}

float ADC_to_Temp(uint16_t reading) {
	float result;

	result = (4095.0 / reading - 1.0);
	result = result * PULL_UP;
	float logR2 = log(result);
	float temp = (1.0 / (A_CONSTANT + (B_CONSTANT*logR2) + (C_CONSTANT*logR2*logR2*logR2)));
	temp = temp - 273.15;

	return temp;
}


void Thermistor_Process(uint16_t value){
 if(array_index >= SAMPLES) {
	 array_index = 0;
 }

	 temp_history[array_index] = ADC_to_Temp(value);
	 array_index++;
}
