/*
 * motor_control.c
 *
 *  Created on: Oct 4, 2022
 *      Author: rdenieport
 */

#include "stm32l4xx_ll_exti.h"
#include "stm32l4xx_ll_gpio.h"
#include "main.h"
#include "motor_control.h"

static int _Pwm_value = 0;
static int _Sector = 0;
static int32_t _Rotation = 0;

void _motor_control_update_sector(void);

typedef struct{
	GPIO_TypeDef * pwm_port;
	uint32_t pwm_pin;
	GPIO_TypeDef * en_port;
	uint32_t en_pin;
	volatile uint32_t * pwm_compare_register;
}motor_control_phase_drive_t;

static const motor_control_phase_drive_t motor_phases[3] = {
		{PWM_U_GPIO_Port, PWM_U_Pin, EN_U_GPIO_Port, EN_U_Pin, &(TIM1->CCR1)},
		{PWM_V_GPIO_Port, PWM_V_Pin, EN_V_GPIO_Port, EN_V_Pin, &(TIM1->CCR2)},
		{PWM_W_GPIO_Port, PWM_W_Pin, EN_W_GPIO_Port, EN_W_Pin, &(TIM1->CCR3)}
};

static inline void _motor_control_set_HZ(motor_control_phase_drive_t phase){
	*(phase.pwm_compare_register) = 0;
	LL_GPIO_ResetOutputPin(phase.en_port, phase.en_pin);
}

static inline void _motor_control_set_ground(motor_control_phase_drive_t phase){
	*(phase.pwm_compare_register) = 0;
	// LL_TIM_GenerateEvent_UPDATE(TIM1);
	LL_GPIO_SetOutputPin(phase.en_port, phase.en_pin);
}

static inline void _motor_control_set_pwm(motor_control_phase_drive_t phase, uint32_t pwm_value){
	*(phase.pwm_compare_register) = pwm_value;
	// LL_TIM_GenerateEvent_UPDATE(TIM1);
	LL_GPIO_SetOutputPin(phase.en_port, phase.en_pin);
}

void motor_control_break(void){
	_motor_control_set_ground(motor_phases[0]);
	_motor_control_set_ground(motor_phases[1]);
	_motor_control_set_ground(motor_phases[2]);
}

void motor_control_stop(void){
	_motor_control_set_HZ(motor_phases[0]);
	_motor_control_set_HZ(motor_phases[1]);
	_motor_control_set_HZ(motor_phases[2]);
}

static inline int _motor_control_get_hall(void){
	static const unsigned int hall_to_phase[6] = { 0, 2, 1, 4, 5, 3 };

	int hall_value = (LL_GPIO_IsInputPinSet(HALL_U_GPIO_Port,HALL_U_Pin) << 2) |
			(LL_GPIO_IsInputPinSet(HALL_V_GPIO_Port,HALL_V_Pin) << 1) |
			(LL_GPIO_IsInputPinSet(HALL_W_GPIO_Port,HALL_W_Pin));

	if ((hall_value >= 1) && (hall_value <= 6)) { // hall value ok
		return hall_to_phase[hall_value - 1];
	} else { // not a valid value
		return -1;
	}
}


static inline void _motor_control_update_pwm(int sector, int pwm_value){
	static const int grounded_pin[6] = {1, 1, 0, 0, 2, 2};
	static const int HZ_pin[6]       = {2, 0, 1, 2, 0, 1};
	static const int pmw_pin[6]      = {0, 2, 2, 1, 1, 0};

	static const int phase_order_direct_rotation[6]   = {1, 2, 3, 4, 5, 0};
	static const int phase_order_indirect_rotation[6] = {4, 5, 0, 1, 2, 3};
	uint32_t pwm;
	int drive_sector;

	//reindexing phases according to rotation
	if (pwm_value > 0){
		drive_sector = phase_order_direct_rotation[sector];
		pwm = (uint32_t)pwm_value;
	}
	else if (pwm_value < 0){
		drive_sector = phase_order_indirect_rotation[sector];
		pwm = (uint32_t)(- pwm_value);
	}
	else{
		motor_control_stop();
		return;
	}

	_motor_control_set_HZ(motor_phases[HZ_pin[drive_sector]]);
	_motor_control_set_ground(motor_phases[grounded_pin[drive_sector]]);
	_motor_control_set_pwm(motor_phases[pmw_pin[drive_sector]], pwm);
}

void _motor_control_update_sector(void){
	static int old_sector = 0;
	int delta;

	//should not happend
	//! \todo ADD TIMEOUT !!!!!!
	while ((_Sector = _motor_control_get_hall()) == -1);

	delta = _Sector - old_sector;
	old_sector = _Sector;

	if (delta <= -3){
		_Rotation += delta + 6;
	}
	else if (delta >= 3){
		_Rotation += delta - 6;
	}
	else{
		_Rotation += delta;
	}
};

void EXTI_IRQHandler(void){

	LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_0);
	LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_1);
	LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_2);

	_motor_control_update_sector();
	_motor_control_update_pwm(_Sector, _Pwm_value);
}

void motor_control_pwm_update_callback(int pwm){
	_Pwm_value = pwm;
	_motor_control_update_pwm(_Sector, _Pwm_value);
}

void motor_control_reset_rotation(void){
	_Rotation = 0;
}

int motor_control_get_rotation(void){
	return _Rotation;
}
