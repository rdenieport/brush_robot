/*
 * servo_loop.c
 *
 *  Created on: Oct 7, 2022
 *      Author: rdenieport
 */

#include "motor_control.h"
#include "main.h"

volatile float speed = 0.0f;
static float old_rotation = 0.0f;

void SysTick_IRQHandler(void){


	//debug
	LL_GPIO_SetOutputPin(CS2_GPIO_Port, CS2_Pin);

	LL_SPI_DisableIT_RXNE(SPI2);
	LL_SPI_TransmitData16(SPI2,0xFFFFu);
	LL_SPI_EnableIT_TXE(SPI2);




	//debug
	LL_GPIO_ResetOutputPin(CS2_GPIO_Port, CS2_Pin);
}

volatile uint16_t data = 0;

void compute_speed(uint32_t data){
	static uint32_t old_rotation;
	int32_t delta;

	delta = data - old_rotation;
/*
	if (delta > 0x1ffff){
		delta -= 0x
*/

/*
	new_rotation = (float)data;
	speed = (new_rotation - old_rotation) / 1.0e-3f;
	old_rotation = new_rotation;
*/
}


void SPI2_IRQHandler(void){


	if (LL_SPI_IsEnabledIT_TXE(SPI2) && LL_SPI_IsActiveFlag_TXE(SPI2)){

		LL_GPIO_SetOutputPin(CS2_GPIO_Port,CS2_Pin);

		LL_SPI_DisableIT_TXE(SPI2);
		LL_SPI_TransmitData16(SPI2,0x0000);

		NVIC_ClearPendingIRQ(SPI2_IRQn);
		LL_SPI_EnableIT_RXNE(SPI2);

		LL_GPIO_ResetOutputPin(CS2_GPIO_Port,CS2_Pin);
		return;
	}

	if (LL_SPI_IsEnabledIT_RXNE(SPI2) && LL_SPI_IsActiveFlag_RXNE(SPI2)){
		data = LL_SPI_ReceiveData16(SPI2) & 0x3fff;

	}

}
