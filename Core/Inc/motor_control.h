/*
 * motor_control.h
 *
 *  Created on: Oct 4, 2022
 *      Author: rdenieport
 */

#ifndef INC_MOTOR_CONTROL_H_
#define INC_MOTOR_CONTROL_H_

void EXTI_IRQHandler(void);
void motor_control_update_hall(int hall_index);
void motor_control_pwm_update_callback(int pwm);

void motor_control_reset_rotation(void);
int motor_control_get_rotation(void);

void motor_control_break(void);
void motor_control_stop(void);

//for test
void _motor_control_update_pwm(int sector, int pwm_value);

#endif /* INC_MOTOR_CONTROL_H_ */
