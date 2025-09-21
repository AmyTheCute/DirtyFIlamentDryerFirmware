/*
 * Thermistor.h
 *
 *  Created on: Jun 19, 2024
 *      Author: Bunny
 */

#ifndef THERMISTOR_H_
#define THERMISTOR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdint.h>
#include <math.h>

#define PULL_UP 4700

#define SAMPLES 15

#define A_CONSTANT 0.6332632718e-03
#define B_CONSTANT 2.257307612e-04
#define C_CONSTANT 0.7990237204e-07

void Thermistor_Init();

// Converts a 12 bit ADC reading to temperature based on the Steinhart-Hart Equation
float ADC_to_Temp(uint16_t reading);

// Returns a rolling average of the temperature.
float Thermistor_get_temp();e

// Adds value to sample array
void Thermistor_Process(uint16_t value);

#ifdef __cplusplus
 }
#endif
#endif /* THERMISTOR_H_ */
