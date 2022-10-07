/*
 * servo_loop.c
 *
 *  Created on: Oct 7, 2022
 *      Author: rdenieport
 */

#include "motor_control.h"
#include "main.h"

volatile float speed = 0.0f;


void SysTick_IRQHandler(void){


	//debug
	LL_GPIO_SetOutputPin(CS2_GPIO_Port, CS2_Pin);

	//LL_SPI_DisableIT_RXNE(SPI2);
	LL_SPI_TransmitData16(SPI2,0xFFFFu);
	//LL_SPI_EnableIT_TXE(SPI2);



	//debug
	LL_GPIO_ResetOutputPin(CS2_GPIO_Port, CS2_Pin);
}

volatile uint16_t data = 0;
volatile uint32_t old_rotation = 0;
volatile int32_t delta;

void compute_speed(uint32_t data){
	//static uint32_t old_rotation = 0;
	//int32_t delta;

	delta = (int32_t)data - (int32_t)old_rotation;
	old_rotation = data;

	if (delta > 0x3fff){
		delta -=  0x4000;
	}
	if (delta < -0x3fff){
		delta += 0x4000;
	}

	speed = ((float)delta);

/*
	new_rotation = (float)data;
	speed = (new_rotation - old_rotation) / 1.0e-3f;
	old_rotation = new_rotation;
*/
}


void SPI2_IRQHandler(void){

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
