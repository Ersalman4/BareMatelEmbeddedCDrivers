/*
 * stm32f411xx_spi_driver.c
 *
 *  Created on: Dec 22, 2024
 *      Author: hp
 */

#include "stm32f411xx_spi_driver.h"

static void spi_txe_interrupt_handler(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handler(SPI_Handle_t *pSPIHndle);
static void spi_ovr_err_interrupt_handler(SPI_Handle_t *pSPIHandle);

/************************************************************************************************
 *																								*
 * 				Some private helper function implementation spi interrupt handlers				*
 * 																								*
 ************************************************************************************************/

static void spi_txe_interrupt_handler(SPI_Handle_t *pSPIHandle){
	//check the dff bit to check the size of the data
	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)){
		//16 bit data so load the data into the data register
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		//decrement the length by 2 times
		pSPIHandle->TxLen -= 2;
		//update the TxBuffer pointer to point to the next address
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}else{
		//8 bit data so load the data in this case
		pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer);
		//decrement the length 1 times only bcz 8 bit data
		pSPIHandle->TxLen--;
		//update the TxBuffer pointer to point to the next address
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}
	//now checks if length becomes 0

	if(! pSPIHandle->TxLen){
		//so close the spi transmission and inform the application that
		//transmission is over
		//this prevents interrupts from setting up of  txe flag
		SPI_CloseTransmission(pSPIHandle);
		//inform the application that transmission is complete
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);

	}
}

static void spi_rxne_interrupt_handler(SPI_Handle_t *pSPIHandle){
	//check the dff bit in data register
	if(pSPIHandle->pSPIx->DR & (1 << SPI_CR1_DFF)){
		//16 bit data, just needs to read the data from DR register
		*((uint16_t*)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
		//decrements the length 2 times
		pSPIHandle->RxLen -= 2;
		//update the RxBuffer address to the next address
		(uint16_t*)pSPIHandle->pRxBuffer++;
	}else{
		//it is 8 bit data, so just reads the data register in the rxbuffer
		*(pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
		//decrements the length 1 time only, since it is 8 bit data
		pSPIHandle->RxLen--;
		//update the rxBuffer address points to the next data address
		pSPIHandle->pRxBuffer++;
	}

	//now again checks if reception is complete
	if(! pSPIHandle->RxLen){
		//if length becomes 0, reception is over and inform the application that reception is over
		//so close the spi reception, to do this just clears out the rxneie control bit in cr2
		//this prevents interrupts from setting up of rxne flag
		SPI_CloseReception(pSPIHandle);
		//inform the application
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}

static void spi_ovr_err_interrupt_handler(SPI_Handle_t *pSPIHandle){
	uint8_t temp = 0;
	//we just needs to do 2 things
	//checks the spi state first
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX){
		//1. clear the ovr flag, to clear ovr flag just reads the DR register followed by read SR reg
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	//2. inform the application that reception is complete
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}


/************************************************************************************************
 * @fn						-> SPI_PeriClkControl
 *
 * @Brief					-> This function is use to control SPI peripheral clock
 *
 * @param[in]				-> base address of the SPI peripheral
 * @param[in]				-> command to enable or disable
 * @param[in]				->
 *
 * @return					-> void
 *
 * @Note					->
 *
 */



void SPI_PeriClkControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(pSPIx == SPI1){
			SPI1_PCLK_EN();
		}else if(pSPIx == SPI2){
			SPI2_PCLK_EN();
		}else if(pSPIx == SPI3){
			SPI3_PCLK_EN();
		}
	}else{
		if(pSPIx == SPI1){
			SPI1_PCLK_DI();
		}else if(pSPIx == SPI2){
			SPI2_PCLK_DI();
		}else if(pSPIx == SPI3){
			SPI3_PCLK_DI();
		}
	}
}

/************************************************************************************************
 * @fn						-> SPI_Init
 *
 * @Brief					-> This function is use to Initialize SPI peripheral
 *
 * @param[in]				-> base address of the SPI handle structure
 * @param[in]				->
 * @param[in]				->
 *
 * @return					-> void
 *
 * @Note					->
 *
 */

void SPI_Init(SPI_Handle_t *pSPIHandle){
	SPI_PeriClkControl(pSPIHandle->pSPIx, ENABLE);

	//first lets configure the CR1 register
	uint32_t tempreg = 0;
	//1. configure the device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	//2. Configure the bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD){
		//bidi mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD){
		//bidi mode should be enabled
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY){
		//bidi mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		//RXONLY bit should be set
		tempreg |= (1 << SPI_CR1_RXONLY);
	}
	//3. configure the spi sclk speed
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	//4. configure the DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	//5. configure the CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;
	//6. configure the CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA <<  SPI_CR1_CPHA;

	pSPIHandle->pSPIx->CR1 = tempreg;


}

/************************************************************************************************
 * @fn						-> SPI_DeInit
 *
 * @Brief					-> This function is use to DeInitialize SPI peripheral
 *
 * @param[in]				-> base address of the SPI peripheral
 * @param[in]				->
 * @param[in]				->
 *
 * @return					-> void
 *
 * @Note					->
 *
 */

void SPI_DeInit(SPI_RegDef_t *pSPIx){
	if(pSPIx == SPI1){
		SPI1_REG_RESET();
	}else if(pSPIx == SPI2){
		SPI2_REG_RESET();
	}else if(pSPIx == SPI3){
		SPI3_REG_RESET();
	}
}

/************************************************************************************************
 * @fn						-> SPI_GetFlagStatus
 *
 * @Brief					-> This function is use to sent data over SPI peripheral
 *
 * @param[in]				-> base address of the SPI peripheral
 * @param[in]				-> flag name
 * @param[in]				->
 *
 * @return					-> flag status
 *
 * @Note					-> this is just an helping function
 *
 */

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName){
	if(pSPIx->SR & FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}


/************************************************************************************************
 * @fn						-> SPI_SendData
 *
 * @Brief					-> This function is use to sent data over SPI peripheral
 *
 * @param[in]				-> base address of the SPI peripheral
 * @param[in]				-> pointer to the tx buffer
 * @param[in]				-> len of the data
 *
 * @return					-> void
 *
 * @Note					->
 *
 */


void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len){
	//loop untill len becomes 0
	while(Len > 0){
		//wait untill txe flag  is set
		while(!(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG)) == FLAG_RESET);
		//check for DFF bit if it is set means 16 bit data
		if(pSPIx->CR1 & SPI_CR1_DFF){
			//16 bit data load data in the DR
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}else{
			//8 bit data, load data in the DR
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
}


/************************************************************************************************
 * @fn						-> SPI_ReceiveData
 *
 * @Brief					-> This function is use to Receive the data over SPI peripheral
 *
 * @param[in]				-> base address of the SPI peripheral
 * @param[in]				-> pointer to rx buffer
 * @param[in]				-> len of the data
 *
 * @return					-> void
 *
 * @Note					->
 *
 */

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len){
	//wait untill length becomes 0
	while(Len > 0){
		//checks and get the rxne flag of spi
		while(!(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG)) == FLAG_RESET);
		//checks the dff bit of CR1 for data is 16 bit or 8 bit
		if(pSPIx->SR & SPI_CR1_DFF){
			//16 bit data, reads the data
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			Len--;Len--;
			(uint16_t*)pRxBuffer++;
		}else{
			//8 bit data
			*(pRxBuffer) = pSPIx->DR;
			Len--;
			pRxBuffer++;
		}
	}
}

/************************************************************************************************
 * @fn						-> SPI_SendDataIT
 *
 * @Brief					-> This function is use to sent the data using isr
 *
 * @param[in]				-> base address of the spi peripheral handle structue
 * @param[in]				-> pointer to txbuffer
 * @param[in]				-> lenghth of the data
 *
 * @return					-> state
 *
 * @Note					->
 *
 */

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len){
	//declare one variable of state to set the state of spi peripheral
	uint8_t state = pSPIHandle->TxState;
	//check the state of the SPI peripheral

	if(state != SPI_BUSY_IN_RX){
		//1. store the pTxBuffer address in global variable defined in handle structure of spi
		pSPIHandle->pTxBuffer = pTxBuffer;
		//2. store the lenght of the data in global variable defined in handle structure of spi
		pSPIHandle->TxLen = Len;
		//3. mark the spi state as busy in transmission, so that no other code take over the spi bus
		pSPIHandle->TxState = state;
		//4. enable the TXEIE control bit in CR2 to get interrupt when TXE flag is set in SR register
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);

	}
	//return the state
	return state;
}

/************************************************************************************************
 * @fn						-> SPI_ReceiveDataIT
 *
 * @Brief					-> This function is use to receive the data using isr
 *
 * @param[in]				-> base address of the spi peripheral handle structue
 * @param[in]				-> pointer to rxbuffer
 * @param[in]				-> lenghth of the data
 *
 * @return					-> state
 *
 * @Note					->
 *
 */

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len){
	//declare one variable to first save the spi peripheral state
	uint8_t state = pSPIHandle->RxState;
	//check the state of spi peripheral
	if(state != SPI_BUSY_IN_TX){
		//1. store the RxBuffer address in global variable defined in handle structure
		pSPIHandle->pRxBuffer = pRxBuffer;
		//2. store the length of the data in global variable defined in handle structure
		pSPIHandle->RxLen = Len;
		//3. mark the spi state busy in reception
		pSPIHandle->RxState = SPI_BUSY_IN_RX;
		//4. enable the RXNEIE control bit in CR2 to get the interrupt when the RXNE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
	}
	//just return the state
	return state;
}


/************************************************************************************************
 * @fn						-> SPI_PeripheralControl
 *
 * @Brief					-> This function is use to enable or disable SPI peripheral
 *
 * @param[in]				-> base address of the SPI reg def structure
 * @param[in]				-> command to enable or disable
 * @param[in]				->
 *
 * @return					-> void
 *
 * @Note					->
 *
 */

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		//set the SPE bit in CR1
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}else{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

/************************************************************************************************
 * @fn						-> SPI_SSIConfig
 *
 * @Brief					-> This function is use to enable or disable SPI SSI bit
 *
 * @param[in]				-> base address of the SPI reg def structure
 * @param[in]				-> command to enable or disable
 * @param[in]				->
 *
 * @return					-> void
 *
 * @Note					->
 *
 */

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		//set the SSI bit in CR1
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}else{
		//clears the SSI bit
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}


/************************************************************************************************
 * @fn						-> SPI_SSOEConfig
 *
 * @Brief					-> This function is use to control output enable control
 *
 * @param[in]				-> base address of the SPI reg def structure
 * @param[in]				-> command to enable or disable
 * @param[in]				->
 *
 * @return					-> void
 *
 * @Note					->
 *
 */

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		//enable the SSOE in CR2
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}else{
		//disble the SSOE
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}

}

/************************************************************************************************
 * @fn						-> SPI_IRQInterruptConfig
 *
 * @Brief					-> This function is use to configure the interrupt for spi events
 *
 * @param[in]				-> irq number
 * @param[in]				-> command to enable or disable
 * @param[in]				->
 *
 * @return					-> void
 *
 * @Note					->
 *
 */

void SPI_IRQInterrutConfig(uint8_t IRQ_Number, uint8_t EnorDi){
	//checks if the command is enable
	if(EnorDi == ENABLE){
		//check again for irq number for which intrrupt needs to be configured
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
 * @fn						-> SPI_IRQPrioriyConfig
 *
 * @Brief					-> This function is use to configure the interrupt for spi events
 *
 * @param[in]				-> irq number
 * @param[in]				-> irq priority
 * @param[in]				->
 *
 * @return					-> void
 *
 * @Note					->
 *
 */

void SPI_IRQPriorityConfig(uint8_t IRQ_Number, uint32_t IRQPriority){
	//1 first lets find out the priority register for this
	uint8_t iprx = IRQ_Number/4;
	uint8_t iprx_section = IRQ_Number % 4;
	uint8_t shift_amount = ((8 * iprx_section) + (8 - NO_OF_PR_BITS_IMPLEMENTED));

	//add this iprx value to pr base addr to get correct register
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}

/************************************************************************************************
 * @fn						-> SPI_IRQHandling
 *
 * @Brief					-> This function is use to configure the interrupt for spi events
 *
 * @param[in]				-> irq number
 * @param[in]				-> irq priority
 * @param[in]				->
 *
 * @return					-> void
 *
 * @Note					->
 *
 */

void SPI_IRQHandling(SPI_Handle_t *pSPIHandle){
	uint8_t temp1 = 0, temp2 = 0;

	//first let's check for the txe flag in SR
	temp1 = (pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE));
	//let's chekc for the txeie bit in cR2
	temp2 = (pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE));

	//if both variables are true then this interrupt is transmission interrupt,
	if(temp1 && temp2){
		//handle the txe interrupt
		spi_txe_interrupt_handler(pSPIHandle);
	}
	temp1 = 0, temp2 = 0;
	//similarly for other events
	temp1 = (pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE));
	//check for rxneie bit in cr2
	temp2 = (pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE));

	//if both are set, then this interrupt is due to rx event
	if(temp1 && temp2){
		//handle the rxne event interrupt
		spi_rxne_interrupt_handler(pSPIHandle);
	}
	temp1 = 0, temp2 = 0;

	//check for over flag
	temp1 = (pSPIHandle->pSPIx->SR & (1 << SPI_SR_OVR));
	//check errie bit in cr2
	temp2 = (pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE));

	//if both are true then interrupt is because of over flag
	if(temp1 && temp2){
		//handle the over error interrupt
		spi_ovr_err_interrupt_handler(pSPIHandle);

	}

}


/************************************************************************************************
 * @fn						-> SPI_IRQHandling
 *
 * @Brief					-> This function is use to configure the interrupt for spi events
 *
 * @param[in]				-> irq number
 * @param[in]				-> irq priority
 * @param[in]				->
 *
 * @return					-> void
 *
 * @Note					->
 *
 */

void SPI_ClearOvrFlag(SPI_RegDef_t *pSPIx){
	uint8_t temp;
	//to clear the over flag just needs to reads the data register followed by read to SR register
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}

/************************************************************************************************
 * @fn						-> SPI_CloseTransmission
 *
 * @Brief					-> This function is use to close spi transmission
 *
 * @param[in]				-> pointer to the handle structure
 * @param[in]				->
 * @param[in]				->
 *
 * @return					-> void
 *
 * @Note					->
 *
 */

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle){
	//clears the txeie control bit in cr2
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	//assign tx buffer to null
	pSPIHandle->pTxBuffer = NULL;
	//so the length becomes 0
	pSPIHandle->TxLen = 0;
	//mark the spi state as ready
	pSPIHandle->TxState = SPI_READY;
}

/************************************************************************************************
 * @fn						-> SPI_CloseReception
 *
 * @Brief					-> This function is use to close the spi reception
 *
 * @param[in]				-> pointer to handle structure
 * @param[in]				->
 * @param[in]				->
 *
 * @return					-> void
 *
 * @Note					->
 *
 */

void SPI_CloseReception(SPI_Handle_t *pSPIHandle){
	//clears the rxneie control bit in cr2
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	//points rxBuffer to null
	pSPIHandle->pRxBuffer = NULL;
	//make rxlen is 0
	pSPIHandle->RxLen = 0;
	//mark the spi state as ready
	pSPIHandle->RxState = SPI_READY;
}


/*
 * weak implementation of application event callback
 */

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv){

	//this is the weak implementation of application event callback, application may override this

}
