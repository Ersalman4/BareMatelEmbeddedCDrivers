/*
 * stm32411xx_usart_driver.c
 *
 *  Created on: Jan 5, 2025
 *      Author: hp
 */

#include "stm32f411xx_usart_driver.h"

/************************************************************************************************
 * @fn						-> USART_PeriClkControl
 *
 * @Brief					-> This function is use to control USART peripheral clock
 *
 * @param[in]				-> base address of the USART peripheral
 * @param[in]				-> command to enable or disable
 * @param[in]				->
 *
 * @return					-> void
 *
 * @Note					->
 *
 */

void USART_PeriClkControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(pUSARTx == USART1){
			//call macro to enable usart1 clock
			USART1_PCLK_EN();
		}else if(pUSARTx == USART2){
			USART2_PCLK_EN();
		}else if(pUSARTx == USART3){
			USART3_PCLK_EN();
		}else if(pUSARTx == UART4){
			UART4_PCLK_EN();
		}else if(pUSARTx == UART5){
			UART5_PCLK_EN();
		}else if(pUSARTx == USART6){
			USART6_PCLK_EN();
		}
	}else{
		if(pUSARTx == USART1){
			USART1_PCLK_DI();
		}else if(pUSARTx == USART2){
			USART2_PCLK_DI();
		}else if(pUSARTx == USART3){
			USART3_PCLK_DI();
		}else if(pUSARTx == UART4){
			UART4_PCLK_DI();
		}else if(pUSARTx == UART5){
			UART5_PCLK_DI();
		}else if(pUSARTx == USART6){
			USART6_PCLK_DI();
		}
	}
}


/************************************************************************************************
 * @fn						-> USART_SetBaudRate
 *
 * @Brief					-> This function is use to set the baud rate of the usart peripheral
 *
 * @param[in]				-> base address of the USART peripheral
 * @param[in]				-> desired baud rate
 * @param[in]				->
 *
 * @return					-> void
 *
 * @Note					->
 *
 */

void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate){
	//variable to hold the apb clock
	uint32_t PCLKx;
	//variable to hold the usart division factor
	uint32_t usartdiv;
	//variable to hold mantissa and fraction values
	uint32_t M_part, F_part;
	uint32_t tempreg = 0;

	//Calculate the value of clock
	if(pUSARTx == USART1 || pUSARTx == USART6){
		//these two usarts are connected to APB2 bus
		PCLKx = RCC_GetPCLK2Value();
	}else{
		//rest of all the usarts are connected to APB1 bus so get get the clock from apb1 bus
		PCLKx = RCC_GetPCLK1Value();
	}
	//check for over8 configuration, to check whether the oversampling by 8 is activated or
	//oversampling by 16 is used
	if(pUSARTx->DR & (1 << USART_CR1_OVER8)){
		//if over8 bit is set, over sampling by 8 is used
		usartdiv = ((25 * PCLKx) / (2 * BaudRate));
	}else{
		//oversampling by 16 is used
		usartdiv = ((25 * PCLKx) / (4 * BaudRate));
	}
	//Calculate the Mantissa Part
	M_part = usartdiv/100;
	//store the value in tempreg correct bit positon so that later stored in the usart brr register
	tempreg |= M_part << 4;

	//Extract the fractional part
	F_part = (usartdiv - (M_part * 100));

	//Calculate the final fractional part
	if(pUSARTx->CR1 & (1 << USART_CR1_OVER8)){
		//over8 is set, means over sampling by 8 is used
		F_part = (((F_part * 8) + 50) / 100) & ((uint8_t)0x07);
	}else{
		//oversampling by 16 is used
		F_part = (((F_part * 16) + 50) / 100) & ((uint8_t)0x0F);
	}
	//place the fractional part in tempreg appropriate position
	tempreg |= F_part;
	//loads the actual BRR register of usart peripheral with tempreg value
	pUSARTx->BRR = tempreg;
}

/************************************************************************************************
 * @fn						-> USART_Init
 *
 * @Brief					-> This function is use to initialize USART peripheral
 *
 * @param[in]				-> base address of the USART peripheral
 * @param[in]				->
 * @param[in]				->
 *
 * @return					-> void
 *
 * @Note					->
 *
 */

void USART_Init(USART_Handle_t *pUSARTHandle){

	uint32_t tempreg = 0;
	//enable the peripheral clock
	USART_PeriClkControl(pUSARTHandle->pUSARTx, ENABLE);
	/********************************Configuration of CR1 register*****************************/
	//1. configure the mode of usart peripheral
	//check whether the mode is, receiver mode
	if(pUSARTHandle->USARTConfig.USART_Mode == USART_MODE_ONLY_RX){
		//then enable the receiver bit field of cR1
		tempreg |= (1 << USART_CR1_RE);
	}else if(pUSARTHandle->USARTConfig.USART_Mode == USART_MODE_ONLY_TX){
		//then enable the transmitter bit field of cR1
		tempreg |= (1 << USART_CR1_TE);
	}else if(pUSARTHandle->USARTConfig.USART_Mode == USART_MODE_TXRX){
		//then enable the transmitter and receiver both bit fields
		tempreg |= (1 << USART_CR1_TE);
		tempreg |= (1 << USART_CR1_RE);
	}
	//2. configure the word length
	tempreg |= (pUSARTHandle->USARTConfig.USART_WordLength << USART_CR1_M);

	//3.  configure the parity control bit in cr1
	if(pUSARTHandle->USARTConfig.USART_ParityControl == USART_PARITY_EN_EVEN){
		//then enable the parity control
		tempreg |= (pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_PCE));
		//then select the even parity
		tempreg |= (pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_PS));
	}else if(pUSARTHandle->USARTConfig.USART_ParityControl == USART_PARITY_EN_ODD){
		//then enable the parity control
		tempreg |= (pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_PCE));
		//then select the odd parity
		tempreg |= (pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_PS));
	}
	//loads the values in CR1
	pUSARTHandle->pUSARTx->CR1 |= tempreg;
	/**************************Configuration of CR2****************************************/
	tempreg = 0;
	//1. configure the number of stop bits inserted in usart transmission
	tempreg |= (pUSARTHandle->USARTConfig.USART_NoOfStopBits << USART_CR2_STOP);

	//loads the values in cr2
	pUSARTHandle->pUSARTx->CR2 |= tempreg;

	/***************************Configuration of CR3****************************************/
	tempreg = 0;
	//1. configure the usat hardware flow control
	if(pUSARTHandle->USARTConfig.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS){
		//enable the cts control bit in cr3
		tempreg |= (1 << USART_CR3_CTSE);
	}else if(pUSARTHandle->USARTConfig.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS){
		//enable the rts control bit in cr3
		tempreg |= (1 << USART_CR3_RTSE);
	}else if(pUSARTHandle->USARTConfig.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS){
		//then enable both cts and rts bit field
		tempreg |= ((1 << USART_CR3_CTSE) | (1 << USART_CR3_RTSE));
	}
	//loads the values in cr3
	pUSARTHandle->pUSARTx->CR3 |= tempreg;
	/****************************Configuration of BRR register******************************/
	USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USARTConfig.USART_Baud);


}

/************************************************************************************************
 * @fn						-> USART_DeInit
 *
 * @Brief					-> This function is use to Deinitialize USART peripheral
 *
 * @param[in]				-> base address of the USART peripheral
 * @param[in]				->
 * @param[in]				->
 *
 * @return					-> void
 *
 * @Note					->
 *
 */

void USART_DeInit(USART_RegDef_t *pUSARTx){

	if(pUSARTx == USART1){
		USART1_REG_RESET();
	}else if(pUSARTx == USART2){
		USART2_REG_RESET();
	}else if(pUSARTx == USART3){
		USART3_REG_RESET();
	}else if(pUSARTx == UART4){
		UART4_REG_RESET();
	}else if(pUSARTx == UART5){
		UART5_REG_RESET();
	}else if(pUSARTx == USART6){
		USART6_REG_RESET();
	}
}

/************************************************************************************************
 * @fn						-> USART_PeripheralControl
 *
 * @Brief					-> This function is use to enable or disable the usart peripheral
 *
 * @param[in]				-> base address of the USART peripheral
 * @param[in]				->
 * @param[in]				->
 *
 * @return					-> void
 *
 * @Note					->
 *
 */

void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		pUSARTx->CR1 |= (1 << USART_CR1_UE);
	}else{
		pUSARTx->CR1 &= ~(1 << USART_CR1_UE);
	}
}

/************************************************************************************************
 * @fn						-> USART_GetFlagStatus
 *
 * @Brief					-> This function is use to check status of various flags of usart
 *
 * @param[in]				-> base address of the USART peripheral
 * @param[in]				-> flag name
 * @param[in]				->
 *
 * @return					-> void
 *
 * @Note					->
 *
 */

uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t FlagName){

	if(pUSARTx->SR & FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/************************************************************************************************
 * @fn						-> USART_SendData
 *
 * @Brief					-> This function is use to send the data from USART peripheral
 *
 * @param[in]				-> base address of the USART peripheral
 * @param[in]				->
 * @param[in]				->
 *
 * @return					-> void
 *
 * @Note					->
 *
 */

void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len){
	//2 bytes pointer to the data
	uint16_t *pdata;
	//loop over length of the data untill all the bytes are transferred
	for(uint32_t i = 0; i<Len; i++){
		//wait untill txe flag is set, indicates all the bytes are transferred
		while(!(USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TXE)));
		//check the word length, whether 9 bits or 8 bits
		if(pUSARTHandle->USARTConfig.USART_WordLength == USART_WORDLEN_9BITS){
			//9 bit data, load the data register with 2 bytes data, mask out other bits except 9 bits
			pdata = (uint16_t*)pTxBuffer;
			//store the remaining data, after masking with
			pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);
			//check for parity control,
			if(pUSARTHandle->USARTConfig.USART_ParityControl == USART_PARITY_DISABLE){
				//1. when parity is disabled, so all the 9 bits are valid data
				pTxBuffer++;
				pTxBuffer++;
			}else{
				//1 bit of parity, in this case means odd parity and that is managed by hardware
				pTxBuffer++;
			}
		}else{
			//word length is 8 bits, mask out 1 byte, except 8 bits means 1 byte
			pUSARTHandle->pUSARTx->DR |= (*pTxBuffer & (uint8_t)0xFF);
			pTxBuffer++;
		}
	}
	//wait untill TC flag is set, indicates all the bytes are transferred
	while(!(USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TC)));

}

/************************************************************************************************
 * @fn						-> USART_ReceiveData
 *
 * @Brief					-> This function is use to send the data from USART peripheral
 *
 * @param[in]				-> base address of the USART peripheral
 * @param[in]				->
 * @param[in]				->
 *
 * @return					-> void
 *
 * @Note					->
 *
 */

void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len){

	//loop over length of the data, all the bytes are received
	for(uint32_t i = 0; i < Len; i++){
		//wait untill rxne is set, indicates 1 byte is received in rx buffer
		while(!(USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_RXNE)));
		//check word length to know the 9 bit data frame, or 8 bit data frame
		if(pUSARTHandle->USARTConfig.USART_WordLength == USART_WORDLEN_9BITS){
			//9 bit data
			//check for parity control
			if(pUSARTHandle->USARTConfig.USART_ParityControl == USART_PARITY_DISABLE){
				//no parity is used

				*(uint16_t*)pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint16_t)0x01FF);
				pRxBuffer++;
				pRxBuffer++;
			}else{
				//8 bit data
				*pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t) 0xFF);
				pRxBuffer++;
			}

		}else{
			//8 bit data, check for parity
			if(pUSARTHandle->USARTConfig.USART_ParityControl == USART_PARITY_DISABLE){
				//no parity, 8 bits of valid data
				*pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t) 0xFF);
				pRxBuffer++;
			}else{
				//parity is used, 7 bits of data and 1 bit of parity
				*pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t)0x7F);
				pRxBuffer++;
			}
		}

	}

}


/************************************************************************************************
 * @fn						-> USART_SendDataIT
 *
 * @Brief					-> This function is use to send the data from USART peripheral
 *
 * @param[in]				-> base address of the USART peripheral
 * @param[in]				->
 * @param[in]				->
 *
 * @return					-> void
 *
 * @Note					->
 *
 */

uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len){
	uint8_t Usart_State;
	Usart_State = pUSARTHandle->TxState;
	//check the usart application state
	if(Usart_State != USART_BUSY_IN_TX){
		//store the buffer address to global variable
		pUSARTHandle->pTxBuffer = pTxBuffer;
		//store the length into global variable
		pUSARTHandle->TxLen = Len;
		//mark the usart state as busy in tx
		pUSARTHandle->TxState = USART_BUSY_IN_TX;

		//enable the interrupt control bit of TXE and also TC in CR1
		pUSARTHandle->pUSARTx->CR1 |= 1 << USART_CR1_TXEIE;
		pUSARTHandle->pUSARTx->CR1 |= 1 << USART_CR1_TCEIE;

	}
	return Usart_State;
}


/************************************************************************************************
 * @fn						-> USART_ReceiveDataIT
 *
 * @Brief					-> This function is use to send the data from USART peripheral
 *
 * @param[in]				-> base address of the USART peripheral
 * @param[in]				->
 * @param[in]				->
 *
 * @return					-> void
 *
 * @Note					->
 *
 */

uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len){
	uint8_t Usart_State;
	Usart_State = pUSARTHandle->RxState;

	//check the usart state
	if(Usart_State != USART_BUSY_IN_RX){
		//store the buffer into global variable
		pUSARTHandle->pRxBuffer = pRxBuffer;
		//store the lengh into global variable
		pUSARTHandle->RxLen = Len;
		//mark the usart application state as busy in reception
		pUSARTHandle->RxState = USART_BUSY_IN_RX;

		//enable the rxneie bit
		pUSARTHandle->pUSARTx->CR1 |= 1 << USART_CR1_RXNEIE;

	}
	return Usart_State;
}


/************************************************************************************************
 * @fn						-> USART_IRQInterruptConfig
 *
 * @Brief					-> This function is use to configure the interrupt for USART peripheral
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

void USART_IRQInterruptConfig(uint8_t IRQ_Number, uint8_t EnorDi){
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
 * @fn						-> USART_IRQPriorityConfig
 *
 * @Brief					-> This function is use to configure the interrupt priorityfor USART
 * 								peripheral
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

void USART_IRQPriorityConfig(uint8_t IRQ_Number, uint32_t IRQ_Priority){
	//1 first lets find out the priority register for this
	uint8_t iprx = IRQ_Number/4;
	uint8_t iprx_section = IRQ_Number % 4;
	uint8_t shift_amount = ((8 * iprx_section) + (8 - NO_OF_PR_BITS_IMPLEMENTED));

	//add this iprx value to pr base addr to get correct register
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQ_Priority << shift_amount);
}


/************************************************************************************************
 * @fn						-> USART_IRQInterruptHandling
 *
 * @Brief					-> This function is use to handle the interrupt generated by various
 * 								usart related events and errors
 *
 * @param[in]				-> pointer to handle structure
 * @param[in]				->
 * @param[in]				->
 *
 * @return					-> void
 *
 * @Note					->
 */


void USART_IRQInterruptHandling(USART_Handle_t *pUSARTHandle){

	uint32_t temp1, temp2, temp3;
	uint16_t *pdata;
	/****************************1. Check for TC Flag******************************/

	//first save the status of TC flag in SR register
	temp1 = (pUSARTHandle->pUSARTx->SR & (1 << USART_SR_TC));
	//that means tc flag is set now check the status of TCEIE bit in CR1
	temp2 = (pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_TCEIE));

	//since both are set, hence
	if(temp1 && temp2){
		//this interrupt is cause because of transmission complete interrupt
		//close the transmission and inform the application if txlen = 0
		if(pUSARTHandle->TxState == USART_BUSY_IN_TX){
			//now check TXLen if it is 0, then close the data transmission
			if(!pUSARTHandle->TxLen){
				//1. clear the TC flag in SR
				pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_TC);
				//2. clear the TCEIE bit in CR1
				pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_TCEIE);
				//3. mark the usart application  as ready
				pUSARTHandle->TxState = USART_READY;
				//4. reset buffer address to null
				pUSARTHandle->pTxBuffer = NULL;
				//5. assign the length to 0
				pUSARTHandle->TxLen = 0;
				//6. call the application callback about this event
				USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_TX_CMPLT);

			}
		}

	}
	/******************************2. Check for TXE flag****************************************/
	//check the status of TXE flag in SR
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_TXE);
	//check the TXEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_TXEIE);
	//since both are set
	if(temp1 && temp2){
		//this interrut is because of TXE event
		if(pUSARTHandle->TxState == USART_BUSY_IN_TX){
			//keep sending data here untill len becomes 0
			if(pUSARTHandle->TxLen > 0){
				//check here word length
				if(pUSARTHandle->USARTConfig.USART_WordLength == USART_WORDLEN_9BITS){
					//9 bit data, so load the DR with 2 bytes, mask out other bytes except 9 bits
					pdata = (uint16_t*)pUSARTHandle->pTxBuffer;
					pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);
					//check for parity control
					if(pUSARTHandle->USARTConfig.USART_ParityControl == USART_PARITY_DISABLE){
						//no parity is used, so all 9 bits are valid data
						//increment buffer address twice
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->pTxBuffer++;
						//decrement the length twice
						pUSARTHandle->TxLen-=2;

					}else{
						//parity is used in this transfer, 8 bit data, 1 bit of parity replaced by
						//usart hardware
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->TxLen--;
					}
				}else{
					//word length is 8 bits
					pUSARTHandle->pUSARTx->DR = (*pUSARTHandle->pTxBuffer & (uint8_t)0xFF);
					//increment the buffer
					pUSARTHandle->pTxBuffer++;
					//decrement the length
					pUSARTHandle->TxLen--;
				}
			}
			if(pUSARTHandle->TxLen == 0){
				//all the bytes are transferred
				//clear the txeie bit in cr1
				pUSARTHandle->pUSARTx->CR1 &= (1 << USART_CR1_TXEIE);
				//clears the txe flag in sr
				pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_TXE);
			}
		}
	}
	/******************************3. Check for RXNE flag********************************/
	//check status of rxne flag in sr
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_RXNE);
	//check for rxneie bit in cr1
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_RXNEIE);
	//since both are set,
	if(temp1 && temp2){
		//this interrupt is because of rxne flag
		//check the usart state
		if(pUSARTHandle->RxState == USART_BUSY_IN_RX){
			//receive untill length becomes 0
			if(pUSARTHandle->RxLen > 0){
				//check for word length
				if(pUSARTHandle->USARTConfig.USART_WordLength == USART_WORDLEN_9BITS){
					//9 bit data
					//check for parity control
					if(pUSARTHandle->USARTConfig.USART_ParityControl == USART_PARITY_DISABLE){
						//no parity used, so all 9 bits are valid data
						*((uint16_t*)pUSARTHandle->pRxBuffer) = ((uint16_t)pUSARTHandle->pUSARTx->DR & (uint16_t)0x01FF);
						//increment the buffer twice
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->pRxBuffer++;
						//decrement the lenght twice
						pUSARTHandle->RxLen -= 2;

					}else{
						//parity is used so mask out the bits except first 9 bits
						*((uint16_t*)pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);
						//increment the buffer
						pUSARTHandle->pRxBuffer++;
						//decrement the length
						pUSARTHandle->RxLen--;
					}
				}else{
					//word length is 8 bits
					//check for parity control
					if(pUSARTHandle->USARTConfig.USART_ParityControl == USART_PARITY_DISABLE){
						//no parity is used, so all 8 bits are valid data
						*(pUSARTHandle->pRxBuffer) = (*pUSARTHandle->pRxBuffer & (uint8_t)0xFF);
						//increment the buffer
						pUSARTHandle->pRxBuffer++;
						//decrement the length
						pUSARTHandle->RxLen--;
					}else{
						//parity is used, 7 bit data and 1 bit parity
						*(pUSARTHandle->pRxBuffer) = (*pUSARTHandle->pRxBuffer & (uint8_t)0x7F);
						//increment the buffer
						pUSARTHandle->pRxBuffer++;
						//decrement the length
						pUSARTHandle->RxLen--;
					}
				}
				//check if length becomes 0
				if(pUSARTHandle->RxLen == 0){
					//close the reception, and reset all the member elements and inform the applicaion
					//clears the rxneie bit
					pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_RXNEIE);
					//clears the rxne flag in sr
					pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_RXNE);
					//mark the usart state as ready
					pUSARTHandle->RxState = USART_READY;
					//assign buffer address to null
					pUSARTHandle->pRxBuffer = NULL;
					//assign rxlen to 0
					pUSARTHandle->RxLen = 0;
					//inform the application
					USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_RX_CMPLT);
				}
			}
		}
	}
	/***************************4. Check for CTS flag************************************/
	//not applicable for uart4 and uart5
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_CTS);
	//store the status of ctse bit in cr3
	temp2 = pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_CTSE);
	//store the status of ctseie bit in cr3
	temp3 = pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_CTSIE);

	if(temp1 && temp2 && temp3){
		//this interrupt is because of cts flag
		//just needs to clear out cts flag and ctsie bit
		pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_CTS);
		pUSARTHandle->pUSARTx->CR3 &= ~(1 << USART_CR3_CTSE);
		pUSARTHandle->pUSARTx->CR3 &= ~(1 << USART_CR3_CTSIE);
		//inform the application
		USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_CTS);
	}
	/***************************5. Check for IDLE flag***********************************/
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_IDLE);
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_IDLEIE);
	if(temp1 && temp2){
		//this interrupt is because of idle flag
		//clear the idle flag
		pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_IDLEIE);
		pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_IDLE);
		//inform the application
		USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_IDLE);
	}
	/****************************6. OverRun Error***************************************/
	//store status of ore bit
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_ORE);
	//store status of rxneie bit
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_RXNEIE);

	if(temp1 && temp2){
		//this is over run error needs to handle this error
		USART_ClearOREFlag(pUSARTHandle);
		//inform the application about this event
		USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_ORE);
	}
	/*******************************7. Check for Error flag********************************/
	//noise error, framing error in multibuffer communication
	//this piece of code is executed only in multibuffer communication
	//store status of eie bit cr3
	temp2 = pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_EIE);
	if(temp2){
		//store status of SR reg
		temp1 = pUSARTHandle->pUSARTx->SR;
		if(temp1 & (1 << USART_SR_FE)){
			//this is framing error,
			/*
			This bit is set by hardware when a de-synchronization, excessive noise or a break
			character is detected. It is cleared by a software sequence (a read to the USART_SR
			register followed by a read to the USART_DR register).
			*/
			uint8_t dummyRead;
			dummyRead = pUSARTHandle->pUSARTx->SR;
			dummyRead = pUSARTHandle->pUSARTx->DR;
			(void)dummyRead;
			//inform the application
			USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_FE);
		}
		if(temp1 & (1 << USART_SR_NE)){
			//this is noise error
			/*
			This bit is set by hardware when noise is detected on a received frame. It is cleared
			by a software sequence (an read to the USART_SR register followed by a read to the
			USART_DR register).
			*/
			uint8_t dummyRead;
			dummyRead = pUSARTHandle->pUSARTx->SR;
			dummyRead = pUSARTHandle->pUSARTx->DR;
			(void)dummyRead;
			//inform the application
			USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_NE);

		}
	}


}


/************************************************************************************************
 * @fn						-> USART_ClearOREFlag
 *
 * @Brief					-> This function is use to clear the over run error flag
 *
 *
 * @param[in]				-> pointer to handle structure
 * @param[in]				->
 * @param[in]				->
 *
 * @return					-> void
 *
 * @Note					->
 */

void USART_ClearOREFlag(USART_Handle_t *pUSARTHandle){
	//just needs to clear the ore flag in sr register
	pUSARTHandle->pUSARTx->SR &= ~(1  << USART_SR_ORE);
	//needs to clear the error bit in
	pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_RXNEIE);
}

/************************************************************************************************
 * @fn						-> USART_ApplicationEventCallback
 *
 * @Brief					-> This function is use to handle the usart application callback
 *
 *
 * @param[in]				-> pointer to handle structure
 * @param[in]				-> application events
 * @param[in]				->
 *
 * @return					-> void
 *
 * @Note					->
 */

__weak void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t AppEv){

}
