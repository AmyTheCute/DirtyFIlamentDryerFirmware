/*
 * Thermistor.h
 *
 *  Created on: Jun 19, 2024
 *      Author: amy
 */

#ifndef THERMISTOR_H_
#define THERMISTOR_H_

#include <stdio.h>
#include <stdint.h>
#include <math.h>

#define PULL_UP 4700

#define SAMPLES 15

#define A_CONSTANT 0.6332632718e-03
#define B_CONSTANT 2.257307612e-04
#define C_CONSTANT 0.7990237204e-07

//Array index for rolling average
extern uint8_t array_index;

extern void Thermistor_Init();

// Converts a 12 bit ADC reading to temperature based on the Steinhart-Hart Equation
extern float ADC_to_Temp(uint16_t reading);

// Returns a rolling average of the temperature.
extern float Thermistor_get_temp();
// Adds value to sample array
extern void Thermistor_Process(uint16_t value);

extern float temp_history[SAMPLES];




#endif /* THERMISTOR_H_ */
