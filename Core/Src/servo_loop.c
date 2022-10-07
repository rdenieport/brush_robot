/*
 * servo_loop.c
 *
 *  Created on: Oct 7, 2022
 *      Author: rdenieport
 */

#include "motor_control.h"
#include "main.h"

#define TICK_TIME 1e-3f //systick
#define ENCODER_CALL_POST_SCALER 10
#define ENCODER_MAX_VALUE 0x3FFF
#define ENCODER_FREQ ((float)POST_SCALER * TICK_TIME)

volatile float _speed = 0.0f; //in rps
static float _speed_cons = 0.0f;
static float _max_speed = 2500.0f;
volatile float _max_acc = 15000.0f;
static int _speed_limit = 0;
static int _quad_limit= 0;

/**	\struct math_PID_t
	\brief PID filter structure definition
 */
typedef struct{
	const struct{
		const float p; //!<proportional action weight
		const float i; //!<integral action weight
		const float d; //!<derivative action weight
	}parameters; //!< PID parameters. Defined as const, modifying those parameters on the fly will result in unpredictable behavior
	const struct{
		const float max_out; //!<output clamp
		const float min_out; //!<output clamp
		const float max_slope; //!<quad-ramp, limits output slew-rate
	}limits; //!<PID bounds. Defined as const. \remarks can be changed, but this will cause integral windup if changed on the fly
	struct{
		float last_input; //!<used for derivation
		float accumulator; //!<used for integration
	}vars; //!<PID internal variables
	float in; //!<PID input, this variables is updated by user
	float out; //!<PID output, updated when math_compute_PID function is called
}math_PID_t;

/** \def MATH_PID_GET_INTEGRAL(fi, fe)
 * \brief helps computing frequency value in z domain
 * \param fi wanted cutoff frequency in Hertz
 * \param fe sampling frequency in Hertz
 * \return integral terms value (z transform)
 */
#define MATH_PID_GET_INTEGRAL(fi, fe) ((fi) / (fe))

/** \def MATH_PID_GET_DERIVATIVE(fi, fe)
 * \brief helps computing frequency value in z domain
 * \param fd wanted cutoff frequency in Hertz
 * \param fe sampling frequency in Hertz
 * \return derivative terms value (z transform)
 */
#define MATH_PID_GET_DERIVATIVE(fd, fe) ((fe) / (fd))

/** \def MATH_PID_GET_SLOPE(dxdt, fe)
 * \brief helps computing slope (slew-rate) value in z domain
 * \param dxdt wanted slope (dx/dt)
 * \param fe sampling frequency in Hertz
 * \return slope terms value (z transform)
 */
#define MATH_PID_GET_SLOPE(dxdt, fe) ((dxdt) / (fe))

/** \def MATH_PID_INIT(p,i,d,maxout,minout,maxslope)
 * 	\brief initialize a PID structure
 * 	\param p proportional
 * 	\param i integral value in z domain
 * 	\param d derivative value in z domain
 * 	\param maxout maximum output value
 * 	\param minout minimum output value
 * 	\param maxslope maximum output slew-rate iin z domain
 * 	\returns math_PID_t structure
 */
#define MATH_PID_INIT(p,i,d,maxout,minout,maxslope) {{(p),(i),(d)},{(maxout),(minout),(maxslope)},{0.0f,0.0f},0.0f,0.0f}

/**
 * compute and update a PID filter
 * \param pid math_PID_t structure pointer
 * \returns new PID output value (same as pid->out)
 */
static inline float math_compute_PID(math_PID_t * pid){
	float integral;
	float derivative;
	float out;

	//internal PID variables computation
	derivative = (pid->in - pid->vars.last_input);
	integral = pid->vars.accumulator + pid->in;
	out = pid->in * pid->parameters.p + integral * pid->parameters.i + derivative * pid->parameters.d;

	//limit the output swing. We do NOT accumulate the integral in limitation (anti-windup)
	if(out > pid->limits.max_out){
		pid->out = pid->limits.max_out;
	}
	else if(out < pid->limits.min_out){
		pid->out = pid->limits.min_out;
	}
	else {
		//limit of the output slope
		if ((out - pid->out) > pid->limits.max_slope){
			pid->out += pid->limits.max_slope;
		}
		else if ((out - pid->out) < - pid->limits.max_slope ){
			pid->out -= pid->limits.max_slope;
		}
		else{
			pid->out = out;
			pid->vars.accumulator = integral;
		}
	}

	//keep the input value for derivative action
	pid->vars.last_input = pid->in;
	return pid->out;
}

/**
 * reset a PID filter
 * \param pid math_PID_t structure pointer
 */
static inline void math_reset_PID(math_PID_t * pid){
	pid->out = 0;
	pid->vars.accumulator = 0.0f;
	pid->vars.last_input = 0.0f;
}


static math_PID_t speed_pid;



void servo_loop_set_config(float imax, float max_speed_rpm){
	_max_speed = max_speed_rpm;
	_max_acc = imax;
	return;
}

void servo_loop_set_setpoint(uint8_t enable, float rpm){
	int i;

	if (enable){
		_speed_cons = rpm;
	}
	else {
		_speed_cons = 0.0;
		motor_control_pwm_update_callback(0);
	}
	return;
}

static inline float _quadramp(float target_speed, float dt){
	static float current_speed = 0.0;
	float acc = (target_speed - current_speed) /dt;

	if (dt == 0){ //just to be shure
		return current_speed;
	}

	if ((acc > -_max_acc) && (acc < _max_acc)){//acceleration below limmit
		_quad_limit = 0;
		current_speed = target_speed;
		return target_speed;
	}
	else{ //quadramp limit
		_quad_limit = 1;
		if (acc > 0){ //increse speed
			current_speed += _max_acc * (float)dt;
		}
		else{ //decrease speed
			current_speed -= _max_acc * (float)dt;
		}
		return current_speed;
	}
	return 0.0; //never reached
}


static inline float _limit_out(float speed){
	if ((speed > -_max_speed) && (speed < _max_speed)){// in normal range
		_speed_limit = 0;
		return speed;
	}
	else {
		_speed_limit = 1;
		if (speed < 0.0){
			return -_max_speed;
		}
		else {
			return _max_speed;
		}
	}
	return 0.0; //never reached
}

volatile float integ = 0.25;
volatile float der = 0;
volatile float prop = 1;

volatile float acc = 0.0f;


void SysTick_IRQHandler(void){
	static int psc = 0;
	if (psc < ENCODER_CALL_POST_SCALER){
		psc++;
		return;
	}

	psc = 0;

	//debug
	LL_GPIO_SetOutputPin(CS2_GPIO_Port, CS2_Pin);


	//compute servo loop
	float error = _speed_cons - _speed;
	float cons;


	//_Pwm = (int32_t)(_limit_out(_quadramp(_speed_cons,dt)) * MOTOR_CONTROL_PWM_FACTOR);

	cons = acc + prop * error;

	if ((!_quad_limit) && (!_speed_limit)){
		acc += integ * error;
	}
	cons = _quadramp(cons,TICK_TIME);
	cons = _limit_out(cons);
	motor_control_pwm_update_callback((int)(cons / 10.0f));


	//LL_SPI_DisableIT_RXNE(SPI2);
	LL_SPI_TransmitData16(SPI2,0xFFFFu);
	//LL_SPI_EnableIT_TXE(SPI2);

	//debug
	LL_GPIO_ResetOutputPin(CS2_GPIO_Port, CS2_Pin);
}


void compute_speed(uint32_t data){
	static uint32_t old_rotation = 0;
	int32_t delta;

	delta = (int32_t)data - (int32_t)old_rotation;
	old_rotation = data;

	if (delta > (ENCODER_MAX_VALUE >>1)){
		delta -= ENCODER_MAX_VALUE;
	}
	if (delta < -(ENCODER_MAX_VALUE >>1)){
		delta += ENCODER_MAX_VALUE;
	}

	_speed = ((float)delta);

	/*
	new_rotation = (float)data;
	speed = (new_rotation - old_rotation) / 1.0e-3f;
	old_rotation = new_rotation;
	 */
}


void SPI2_IRQHandler(void){
	uint16_t data = 0;
	/*
	if (LL_SPI_IsEnabledIT_TXE(SPI2) && LL_SPI_IsActiveFlag_TXE(SPI2)){

		LL_GPIO_SetOutputPin(CS2_GPIO_Port,CS2_Pin);

		for (volatile int i = 0; i < 40; i++);
		LL_SPI_DisableIT_TXE(SPI2);
		LL_SPI_TransmitData16(SPI2,0x0000);

		LL_SPI_ReceiveData16(SPI2);
		NVIC_ClearPendingIRQ(SPI2_IRQn);
		LL_SPI_EnableIT_RXNE(SPI2);

		LL_GPIO_ResetOutputPin(CS2_GPIO_Port,CS2_Pin);
		return;
	}
	 */
	if (LL_SPI_IsEnabledIT_RXNE(SPI2) && LL_SPI_IsActiveFlag_RXNE(SPI2)){
		//data = LL_SPI_ReceiveData16(SPI2) & 0x3fff;
		// hack because ST is are stupid
		data = (LL_SPI_ReceiveData16(SPI2)<<1) & 0x3fff;
		compute_speed(data);
	}
}

