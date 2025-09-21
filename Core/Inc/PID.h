/*
 * PID.h
 *
 *  Created on: Jun 19, 2024
 *      Author: Bunny
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#ifdef __cplusplus
extern "C" {
#endif

// Returns a duty cycle value based on the temperature 
double PID_GetDutyCycle(float current, float target);

#ifdef __cplusplus
 }
#endif

#endif /* INC_PID_H_ */
