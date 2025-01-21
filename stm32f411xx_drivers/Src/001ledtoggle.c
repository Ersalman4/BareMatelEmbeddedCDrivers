/*
 * 001ledtoggle.c
 *
 *  Created on: Dec 21, 2024
 *      Author: hp
 * @    write a program to toggle on board led
 *      case 1: use push pull configuration
 *      case 2: use open drain configuration
 */

#include "stm32f411xx.h"

void delay(void){
	for(int i=0; i<250000; i++);
}

int main(void){

	//i need to create one variable of Gpio handle type structure
	GPIO_Handle_t GpioLed;
	//i need to initialize this variable with various configuration settings
	//i need to select the port
	GpioLed.pGPIOx = GPIOA;
	//i need to select the pin number on which led is connected
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	//i need to set the mode of pin
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	//i need to select the pull up and pull down setting
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NOPUPD;
	//i need to select the speed of the pin
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;

	//enable the peripheral clock for GPIOA port
	GPIO_PeriClkControl(GPIOA, ENABLE);
	GPIO_Init(&GpioLed);
	while(1){
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
		delay();
	}


}
