/*
 * PID.c
 *
 *  Created on: Jun 19, 2024
 *      Author: Amy Haghighi
 */

#include "PID.h"

double PID_P = 55.0, PID_I = 0.004, PID_D = 0.002; //50Kp/0Ki/0Kd stable at 58.75/60
double integralSum = 0.0;
double porportionalValue = 0.0;
double last_input = 0.0;

double PID_GetDutyCycle(float current, float target) {
	double error = target - current;

	/* P and I control computer */
	porportionalValue = (PID_P * error);

	double derivative = PID_D * (current - last_input);
	integralSum -= derivative;

	if(porportionalValue < 100.0){
		integralSum += (PID_I * error); // ToDo: Take time elapsed into consideration.
	}

	/* Clamp results */
	if (integralSum > 100.0f) {
		integralSum = 100.0f;
	}

	if (integralSum < 0.0f) {
		integralSum = 0.0f;
	}

	double output = integralSum + porportionalValue;

	if (output > 100.0f) {
		output = 100.0f;
	}

	if (output < 0.0f) {
		output = 0.0f;
	}

	last_input = current;

	return output / 100.0f;
}
