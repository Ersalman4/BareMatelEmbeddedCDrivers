/*
 * 004SPI_TX_Arduino_Testing.c
 *
 *  Created on: Dec 26, 2024
 *      Author: hp
 */


/*
 * 003SPI_Tx_Testing.c
 *
 *  Created on: Dec 24, 2024
 *      Author: hp
 */
/*
 * i am using port B for SPI PINS
 * PB12->SPI_NSS
 * PB13->SPI_SCLK
 * PB14->SPI_MISO
 * PB15->SPI_MOSI
 * Alt function type is 5 for spi
 *
 */

#include "stm32f411xx.h"
#include "string.h"

#define BTTN_PRESSED		1
#define BTTN_UNPRESSED		0

void delay(void){
	for(uint32_t i = 0; i< 500000/2; i++);
}

void SPI2_GPIOBttn(void){
	GPIO_Handle_t GpioBttn;
	//select the port
	GpioBttn.pGPIOx = GPIOA;
	//select the pin on which button is connected
	GpioBttn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	//select the mode of the pin
	GpioBttn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	//select the op type
	GpioBttn.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	//select the pu pd setting
	GpioBttn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	//select the speed of the pin
	GpioBttn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//enable the clock for GPIOA
	GPIO_PeriClkControl(GPIOA, ENABLE);
	//initialize the port A
	GPIO_Init(&GpioBttn);

}

void SPI2_Init(){
	SPI_Handle_t SPI2Handle;
	//select the spi
	SPI2Handle.pSPIx = SPI2;
	//use the full duplex mode
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	//use the spi master mode
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	//use max speed
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;
	//use 8 bit dff
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	//use cpol is low
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	//use cpha is low
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	//use software slave select disable
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_DI;

	SPI_Init(&SPI2Handle);

}

void SPI2_GPIO_Init(){
	GPIO_Handle_t SPIPins;
	//select the port
	SPIPins.pGPIOx = GPIOB;
	//select the pin mode
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFuncMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NOPUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	//nss
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);
	//serial clock
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);
	//mosi
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);
	//miso
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);



}


int main(){
	char user_data[] = "hello world";

	SPI2_GPIO_Init();

	SPI2_Init();

	SPI2_GPIOBttn();

	//this makes NSS pin internally high, to avoid the MODF error
	//SPI_SSIConfig(SPI2, ENABLE);

	/*
	* making SSOE 1 does NSS output enable.
	* The NSS pin is automatically managed by the hardware.
	* i.e when SPE=1 , NSS will be pulled to low
	* and NSS pin will be high when SPE=0
	*/

	SPI_SSOEConfig(SPI2, ENABLE);
	while(1)
	{
		//wait, untill button is pressed
		while(! (GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0)));

		//to avoid button debouncing
		delay();

		//enable the spi2
		SPI_PeripheralControl(SPI2, ENABLE);

		//before sending data, sent the length to slave
		uint8_t data_len = strlen(user_data);
		SPI_SendData(SPI2, &data_len, 1);

		//sent the data
		SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

		//lets confirm first spi is not busy
		while((SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG)))

		//then only disable the spi
		SPI_PeripheralControl(SPI2, DISABLE);
	}

	return 0;
}

