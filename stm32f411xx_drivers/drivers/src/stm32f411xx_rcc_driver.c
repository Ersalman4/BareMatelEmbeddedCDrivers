/*
 * stm32f411xx_rcc_driver.c
 *
 *  Created on: Jan 7, 2025
 *      Author: hp
 */

#include "stm32f411xx_rcc_driver.h"


uint16_t AHB_PreScalar_Div_Values[8] = {2, 4, 8, 16, 64, 128, 256, 512};
uint8_t APB1_PreScalar_Div_Values[4] = {2, 4, 8, 16};
uint8_t APB2_PreScalar_Div_Values[4] = {2, 4, 8, 16};

uint32_t RCC_GetPLLOutputClock(void){
	return 0;
}

uint32_t RCC_GetPCLK2Value(void){
	uint32_t PClk2, SystemClock;
	uint8_t ClockSource, AHB_PreScalar, APB2_PreScalar, temp;

	//1. first find out the clock source, to do this we have to read the CFGR register of RCC engine
	ClockSource = (RCC->CFGR >> 2 & 0x3);
	if(ClockSource == 0){
		SystemClock = 1600000;
	}else if(ClockSource == 1){
		SystemClock = 800000;
	}else if(ClockSource == 2){
		SystemClock = RCC_GetPLLOutputClock();
	}

	//now reads the value of CFGR extreme right most bits, this is for AHB_PreScalar
	temp = ((RCC->CFGR >> 4) & 0xF);

	if(temp < 8){
		AHB_PreScalar = 1;
	}else{
		AHB_PreScalar = AHB_PreScalar_Div_Values[temp - 4];
	}

	//now reads the value of CFGR register extreme right most bits, this is for APB_PreScalar
	temp = ((RCC->CFGR >> 13) & 0x7);

	if(temp < 8){
		APB2_PreScalar = 1;
	}else{
		APB2_PreScalar = APB2_PreScalar_Div_Values[temp - 4];
	}

	//now calculate the clock
	PClk2 = ((SystemClock / AHB_PreScalar) / APB2_PreScalar);

	return PClk2;
}

uint32_t RCC_GetPCLK1Value(void){

	uint32_t PClk1, SystemClock;

	uint8_t ClockSource, temp, AHB_PreScalar, APB1_PreScalar;
	//first find out the clock source, to do this we have to read the CFGR register of RCC
	ClockSource = (RCC->CFGR >> 2 & 0x3);
	if(ClockSource == 0){
		//then system clock would be HSI, that means 16MHZ
		SystemClock = 1600000;
	}else if(ClockSource == 1){
		//then system clock would be HSE, that means 8MHZ
		SystemClock = 800000;
	}else if(ClockSource == 2){
		//then system clock would be calculated with PLL Source
		SystemClock = RCC_GetPLLOutputClock();
	}
	//now reads the value of CFGR extreme right most bits, this is for AHB_PreScalar
	temp = ((RCC->CFGR >> 4) & 0xF);

	if(temp < 8){
		AHB_PreScalar = 1;
	}else{
		AHB_PreScalar = AHB_PreScalar_Div_Values[temp - 8];
	}
	//now reads the value of CFGR extreme right most bits, this is for APB_PreScalar
	temp = ((RCC->CFGR >> 10) & 0x7);

	if(temp < 8){
		APB1_PreScalar = 1;
	}else{
		APB1_PreScalar = APB1_PreScalar_Div_Values[temp - 4];
	}
	//can calculate the PClk1 with this formula

	PClk1 = (SystemClock/AHB_PreScalar)/APB1_PreScalar;

	return PClk1;

}

