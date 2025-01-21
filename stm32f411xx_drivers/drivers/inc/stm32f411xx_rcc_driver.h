/*
 * stm32f411xx_rcc_driver.h
 *
 *  Created on: Jan 7, 2025
 *      Author: hp
 */

#ifndef INC_STM32F411XX_RCC_DRIVER_H_
#define INC_STM32F411XX_RCC_DRIVER_H_

#include "stm32f411xx.h"

/***************************************************************************************************
 * 					Prototypes for the functions to get the desired clock
 **************************************************************************************************/

uint32_t RCC_GetPLLOutputClock(void);

uint32_t RCC_GetPCLK2Value(void);

uint32_t RCC_GetPCLK1Value(void);



#endif /* INC_STM32F411XX_RCC_DRIVER_H_ */
