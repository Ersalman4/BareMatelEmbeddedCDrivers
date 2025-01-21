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
	//use software slave select enable
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_EN;

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


	//this makes NSS pin internally high, to avoid the MODF error
	SPI_SSIConfig(SPI2, ENABLE);

	//enable the spi2
	SPI_PeripheralControl(SPI2, ENABLE);


	//sent the data
	SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

	//lets confirm first spi is not busy
	while((SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG)))

	//then only disable the spi
	SPI_PeripheralControl(SPI2, DISABLE);

	while(1);

	return 0;
}

