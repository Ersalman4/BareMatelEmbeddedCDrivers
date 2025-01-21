/*
 * 002ledtoggle.c
 *
 *  Created on: Dec 21, 2024
 *      Author: hp
 *@		write a prgram to toggle on board led whenever the button is pressed
 *@
 */

#include "stm32f411xx.h"

#define LOW				0
#define BTTN_PRESSED	LOW

void delay(void){
	for(uint32_t i = 0; i < 500000; i++);
}

int main(){

	//i need two variables of GPIO handle structure type
	GPIO_Handle_t GpioLed, GpioBttn;
	//first needs to configure the input side of this application for that
	//needs to configure the GpioBttn first
	//needs to select the port
	GpioBttn.pGPIOx = GPIOC;
	//select the pin number which user button is connected
	GpioBttn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	//select the mode of the pin
	GpioBttn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	//configure the pupd setting
	GpioBttn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NOPUPD;
	//configure the speed of the pin
	GpioBttn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	//enable the peripheral clock for the portC
	GPIO_PeriClkControl(GPIOC, ENABLE);

	GPIO_Init(&GpioBttn);

	//select the output configuration for this
	//select the port
	GpioLed.pGPIOx = GPIOA;
	//select the pin number
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	//select the mode of the pin
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	//select the op type of this pin
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	//select the pupd settings of gpio
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NOPUPD;
	//select the pin speed
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//enable the peripheral clock for both the ports portA and portC
	GPIO_PeriClkControl(GPIOA, ENABLE);

	GPIO_Init(&GpioLed);

	while(1){
		//now i need to read the status of the button pin only
		if(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) == BTTN_PRESSED){

			delay();
			//toggle the output pin
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
		}
	}
	return 0;
}


