/*
 * 010I2C_Tx_testing.c
 *
 *  Created on: Dec 31, 2024
 *      Author: hp
 */


#include "stm32f411xx.h"

void delay(void){
	for(uint32_t i=0; i<500000; i++);
}

void I2C1_GpioInits(void){
	GPIO_Handle_t I2C1Pins;
	//1. let's select the port
	I2C1Pins.pGPIOx = GPIOB;
	//2. let's select the mode of the pin
	I2C1Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	//3. let's select the op type of the pin
	I2C1Pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	//4. select the pin pu pd settings
	I2C1Pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	//5. select the alt func
	I2C1Pins.GPIO_PinConfig.GPIO_PinAltFuncMode = 4;

	//6. select the pin numbers for SCL->PB6 and for SDA->PB9
	I2C1Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&I2C1Pins);

	//7. for sda
	I2C1Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&I2C1Pins);


}

void I2C1_Inits(){

}

int main(){

	I2C1_GpioInits();
	I2C1_Inits();
	return 0;
}
