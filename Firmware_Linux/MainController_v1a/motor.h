/*
 * motor.h
 *
 *  Created on: Apr 8, 2016
 *      Author: Nishant Pol
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include <msp430.h>
#include "utils.h"
#include "err_warn_codes.h"

#define MOT_CW	0
#define MOT_CCW	1

void motor_pwm_setup(void);
void set_chimney_speed(uint16_t speed);
void set_compact_speed(uint16_t speed);
void set_bin_speed(uint16_t speed);
void set_chimney_direction(uint8_t dir);
void set_compact_direction(uint8_t dir);
void set_bin_direction(uint8_t dir);



#endif /* MOTOR_H_ */
