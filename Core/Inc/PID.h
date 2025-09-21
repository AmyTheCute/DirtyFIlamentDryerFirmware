/*
 * PID.h
 *
 *  Created on: Jun 19, 2024
 *      Author: amy
 */

#ifndef INC_PID_H_
#define INC_PID_H_

// returns a duty cycle value based on the temperature;
double PID_GetDutyCycle(float current, float target);

extern double PID_P, PID_I, PID_D;
extern double integralSum;
extern double last_output;

#endif /* INC_PID_H_ */
