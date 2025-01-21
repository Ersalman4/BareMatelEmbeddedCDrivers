/*
 * stm32f411xx_gpio_driver.c
 *
 *  Created on: Dec 20, 2024
 *      Author: hp
 */

#include "stm32f411xx_gpio_driver.h"

/************************************************************************************************
 * @fn						-> GPIO_PeriClkControl
 *
 * @Brief					-> This function is use to control peripheral clock
 *
 * @param[in]				-> base address of the GPIO peripheral
 * @param[in]				-> command to enable or disable
 * @param[in]				->
 *
 * @return					-> void
 *
 * @Note					->
 *
 */

void GPIO_PeriClkControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}else if(pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
		}else if(pGPIOx == GPIOD){
			GPIOD_PCLK_EN();
		}else if(pGPIOx == GPIOE){
			GPIOE_PCLK_EN();
		}else if(pGPIOx == GPIOF){
			GPIOF_PCLK_EN();
		}else if(pGPIOx == GPIOG){
			GPIOG_PCLK_EN();
		}else if(pGPIOx == GPIOH){
			GPIOH_PCLK_EN();
		}else if(pGPIOx == GPIOI){
			GPIOI_PCLK_EN();
		}
	}else{
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_DI();
		}else if(pGPIOx == GPIOB){
			GPIOB_PCLK_DI();
		}else if(pGPIOx == GPIOC){
			GPIOC_PCLK_DI();
		}else if(pGPIOx == GPIOD){
			GPIOD_PCLK_DI();
		}else if(pGPIOx == GPIOE){
			GPIOE_PCLK_DI();
		}else if(pGPIOx == GPIOF){
			GPIOF_PCLK_DI();
		}else if(pGPIOx == GPIOG){
			GPIOG_PCLK_DI();
		}else if(pGPIOx == GPIOH){
			GPIOH_PCLK_DI();
		}else if(pGPIOx == GPIOI){
			GPIOI_PCLK_DI();
		}
	}
}


/************************************************************************************************
 * @fn						-> GPIO_Init
 *
 * @Brief					-> This function is use to initialize the GPIO peripheral
 *
 * @param[in]				-> base address of the GPIO handle structure
 * @param[in]				->
 * @param[in]				->
 *
 * @return					-> void
 *
 * @Note					->
 *
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle){

	uint32_t temp = 0;
	GPIO_PeriClkControl(pGPIOHandle->pGPIOx, ENABLE);
	//1. configure the mode of the GPIO pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){
		//non-interrupt mode
		//configuration of the mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << 2 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER |= temp;

		temp = 0;
		//2. configure the pin pupd settings
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << 2 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->PUPDR |= temp;

		temp = 0;
		//3. configure the pin speed
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << 2 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->OSPEEDR |= temp;

		temp = 0;

		//4. configure the op type
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->OTYPER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->OTYPER |= temp;
		temp = 0;

		//5. configure the pin alt function type
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){
			//then only configure the alternate functionality of the pin
			uint8_t temp1 = 0, temp2 = 0;
			temp1 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8);
			temp2 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8);
			pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << 4* temp2);
			pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFuncMode << (4 * temp2));
		}
	}else{
		//interrupt mode
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT){

			//configure the FTSR register
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//clear the corresponding bits in RTSR register
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){
			//configure the RTSR register
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//clear the corresponding bit fields of FTSR
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT){
			//configure the rising falling edge trigger
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			//clear the corresponding bits of both the registers
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		//configure the GPIO port selection in SYSCFG EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG->EXTICR[temp1] = (portcode << (temp2 * 4));

		//enable the exti interrupt delivery using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	}

}


/************************************************************************************************
 * @fn						-> GPIO_DeInit
 *
 * @Brief					-> This function is use to de-initialize the GPIO peripheral
 *
 * @param[in]				-> base address of the GPIO peripherals
 * @param[in]				->
 * @param[in]				->
 *
 * @return					-> void
 *
 * @Note					->
 *
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){
	if(pGPIOx == GPIOA){
		GPIOA_REG_RESET();
	}else if(pGPIOx == GPIOB){
		GPIOB_REG_RESET();
	}else if(pGPIOx == GPIOC){
		GPIOC_REG_RESET();
	}else if(pGPIOx == GPIOD){
		GPIOD_REG_RESET();
	}else if(pGPIOx == GPIOE){
		GPIOE_REG_RESET();
	}else if(pGPIOx == GPIOF){
		GPIOF_REG_RESET();
	}else if(pGPIOx == GPIOG){
		GPIOG_REG_RESET();
	}else if(pGPIOx == GPIOH){
		GPIOH_REG_RESET();
	}else if(pGPIOx == GPIOI){
		GPIOI_REG_RESET();
	}
}

/************************************************************************************************
 * @fn						-> GPIO_ReadFromInputPort
 *
 * @Brief					-> This function is use to read from the input port
 *
 * @param[in]				-> base address of the GPIO peripheral
 * @param[in]				->
 * @param[in]				->
 *
 * @return					-> 0 or 1
 *
 * @Note					->
 *
 */

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){
	uint16_t value;
	value = (pGPIOx->IDR);
	return value;
}

/************************************************************************************************
 * @fn						-> GPIO_ReadFromInputPin
 *
 * @Brief					-> This function is use to read from the input pin
 *
 * @param[in]				-> base address of the GPIO peripheral
 * @param[in]				-> pin number
 * @param[in]				->
 *
 * @return					-> 0 or 1
 *
 * @Note					->
 *
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	uint8_t value;
	value = ((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}

/************************************************************************************************
 * @fn						-> GPIO_ToggleOutputPin
 *
 * @Brief					-> This function is use to toggle the output pin
 *
 * @param[in]				-> base address of the GPIO peripheral
 * @param[in]				-> pin number
 * @param[in]				->
 *
 * @return					-> void
 *
 * @Note					->
 *
 */


void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){

	pGPIOx->ODR ^= (1 << PinNumber);

}

/************************************************************************************************
 * @fn						-> GPIO_WriteToOutputPort
 *
 * @Brief					-> This function is use to write on the output port
 *
 * @param[in]				-> base address of the GPIO peripheral
 * @param[in]				-> value to write on the port
 * @param[in]				->
 *
 * @return					-> void
 *
 * @Note					->
 *
 */

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value){
	pGPIOx->ODR = value;
}

/************************************************************************************************
 * @fn						-> GPIO_WriteToOutputPin
 *
 * @Brief					-> This function is use to write on the output pin
 *
 * @param[in]				-> base address of the GPIO peripheral
 * @param[in]				-> pin number
 * @param[in]				-> value to write
 *
 * @return					-> void
 *
 * @Note					->
 *
 */


void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value){
	if(value == GPIO_PIN_SET){
		//write 1 to the bit field of ODR corresponding to the pin number
		pGPIOx->ODR |= (1 << PinNumber);
	}else{
		//write 0 to the bit field of ODR corresponding to the pin number
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

/************************************************************************************************
 * @fn						-> GPIO_IRQInterruptConfig
 *
 * @Brief					-> This function is use to configure the interrupt
 *
 * @param[in]				-> irq number
 * @param[in]				->
 * @param[in]				-> command to enable or disable interrupt
 *
 * @return					-> void
 *
 * @Note					->
 *
 */

void GPIO_IRQInterruptConfig(uint8_t IRQ_Number, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(IRQ_Number <= 31){
			//set the ISER0 register
			*NVIC_ISER0 |= (1 << IRQ_Number);
		}else if(IRQ_Number > 31 && IRQ_Number < 64){
			//set the ISER1 register
			*NVIC_ISER1 |= (1 << (IRQ_Number % 32));
		}else if(IRQ_Number > 64 && IRQ_Number < 96){
			//set the ISER2 register
			*NVIC_ISER2 |= (1 << (IRQ_Number % 64));
		}
	}else{
		if(IRQ_Number <= 31){
			//clear the ICER0 register
			*NVIC_ICER0 |= (1 << IRQ_Number);
		}else if(IRQ_Number > 31 && IRQ_Number < 64){
			//clears the ICER1 register
			*NVIC_ICER1 |= (1 << (IRQ_Number % 32));
		}else if(IRQ_Number > 64 && IRQ_Number < 96){
			//clears the ICER2 register
			*NVIC_ICER2 |= (1 << (IRQ_Number % 64));
		}
	}
}

/************************************************************************************************
 * @fn						-> GPIO_IRQPriorityConfig
 *
 * @Brief					-> This function is use to set the priority of the interrupt
 *
 * @param[in]				-> irq priority
 * @param[in]				-> irq number
 * @param[in]				->
 *
 * @return					-> void
 *
 * @Note					->
 *
 */

void GPIO_IRQPriorityConfig(uint8_t IRQ_Number, uint32_t IRQ_Priority){
	//1 first lets find out the priority register for this
	uint8_t iprx = IRQ_Number/4;
	uint8_t iprx_section = IRQ_Number % 4;
	uint8_t shift_amount = ((8 * iprx_section) + (8 - NO_OF_PR_BITS_IMPLEMENTED));

	//add this iprx value to pr base addr to get correct register
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQ_Priority << shift_amount);

}

/************************************************************************************************
 * @fn						-> GPIO_IRQHandling
 *
 * @Brief					-> This function is use to handling the interrupt
 *
 * @param[in]				-> irq number
 * @param[in]				-> irq priority
 * @param[in]				-> command to enable or disable
 *
 * @return					-> void
 *
 * @Note					->
 *
 */


void GPIO_IRQHandling(uint8_t PinNumber){
	//clears the exti pr register corresponding to the pin number
	if(EXTI->PR & (1 << PinNumber)){
		//to clear set the corresponding bit
		EXTI->PR |= (1 << PinNumber);
	}
}
