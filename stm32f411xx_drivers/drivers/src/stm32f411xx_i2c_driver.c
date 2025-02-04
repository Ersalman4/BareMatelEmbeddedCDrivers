/*
 * stm32f411xx_i2c_driver.c
 *
 *  Created on: Dec 29, 2024
 *      Author: hp
 */

#include "stm32f411xx_i2c_driver.h"



static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_Handle_t *pI2CHandle, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_Handle_t *pI2CHandle, uint8_t SlaveAddr);
static void I2C_ClearAddrFlag(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle);



/*************************************************************************************************
 	 	 	 	 	 	 	 	 	 |Private Helper functions|
**************************************************************************************************/

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx){
	//set the stop bit in cr1
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName){
	//test i2c related flags generated in sr1 with flagname
	if(pI2Cx->SR1 & FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}

static void I2C_ExecuteAddressPhaseWrite(I2C_Handle_t *pI2CHandle, uint8_t SlaveAddr){
	//first left shift the slave address by 1, because 0th bit is meant for R/W bit
	SlaveAddr = SlaveAddr << 1;
	//now clears the 0th bit of SlaveAddr
	SlaveAddr &= ~(1);
	//now just loads the slave addr in DR register of I2C
	pI2CHandle->pI2Cx->DR = SlaveAddr;
}

static void I2C_ExecuteAddressPhaseRead(I2C_Handle_t *pI2CHandle, uint8_t SlaveAddr){
	//first left shift the slave address by 1, because 0th bit is meant for R/W bit
	SlaveAddr = SlaveAddr << 1;
	//now clears the 0th bit of SlaveAddr
	SlaveAddr |= 1;
	//now just loads the slave addr in DR register of I2C
	pI2CHandle->pI2Cx->DR = SlaveAddr;
}

static void I2C_ClearAddrFlag(I2C_Handle_t *pI2CHandle){
	uint8_t dummyRead;
	//check for device mode
	if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)){
		//device is in master mode
		//check for the i2c state
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX){
			//check the rx size
			if(pI2CHandle->RxSize == 1){
				//disable the acking
				I2C_ManageAcking(pI2CHandle, I2C_ACK_DISABLE);
				//clear the addr flag, to clear read SR1, followed by read SR2
				dummyRead = pI2CHandle->pI2Cx->SR1;
				dummyRead = pI2CHandle->pI2Cx->SR2;
				(void)dummyRead;
			}
		}else{
			//clear the addr flag, to clear read SR1, followed by read SR2
			dummyRead = pI2CHandle->pI2Cx->SR1;
			dummyRead = pI2CHandle->pI2Cx->SR2;
			(void)dummyRead;
		}
	}else{

		//device is in slave mode
		//clear the addr flag, to clear read SR1, followed by read SR2
		dummyRead = pI2CHandle->pI2Cx->SR1;
		dummyRead = pI2CHandle->pI2Cx->SR2;
		(void)dummyRead;
	}

}

static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle){
	//first check the length of the data
	if(pI2CHandle->TxLen > 0){
		//1. load the data in to data register
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);
		//2. decrement the length
		pI2CHandle->TxLen--;
		//3. update the txbuffer pointer to next data location
		pI2CHandle->pTxBuffer++;
	}
}


static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle){
	//we have to do the data reception
	//1. checks the length
	if(pI2CHandle->RxSize == 1){
		//read data in rx buffer
		*(pI2CHandle->pRxBuffer) = pI2CHandle->pI2Cx->DR;
		//decrement the length
		pI2CHandle->RxLen--;
	}
	if(pI2CHandle->RxSize > 1){
		//when last two bytes of data remains
		if(pI2CHandle->RxLen == 2){
			//clear the ack bit
			I2C_ManageAcking(pI2CHandle, I2C_ACK_DISABLE);
		}
		//otherwise just reads the data into the rx buffer
		*(pI2CHandle->pRxBuffer) = pI2CHandle->pI2Cx->DR;
		//decrements the length
		pI2CHandle->RxLen--;
		//update the rxbuffer pointer to next
		pI2CHandle->pRxBuffer++;

	}
	if(pI2CHandle->RxLen == 0){
		//close the reception, and notify application about the event
		//1. generate the stop condition
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		//2. close reception
		I2C_CloseReception(pI2CHandle);
		//3. notify application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
	}
}


/************************************************************************************************
 * @fn						-> I2C_PeriClkControl
 *
 * @Brief					-> This function is use to control I2C peripheral clock
 *
 * @param[in]				-> base address of the I2C peripheral
 * @param[in]				-> command to enable or disable
 * @param[in]				->
 *
 * @return					-> void
 *
 * @Note					->
 *
 */


void I2C_PeriClkControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi){
	//first checks the commands if it is enable
	if(EnorDi == ENABLE){
		//again checks the which i2c peripheral it is
		if(pI2Cx == I2C1){
			//enable the clock for i2c1
			I2C1_PCLK_EN();
		}else if(pI2Cx == I2C2){
			//enable the clock for i2c2
			I2C2_PCLK_EN();
		}else if(pI2Cx == I2C3){
			//enable the clock for i2c3
			I2C3_PCLK_EN();
		}
	}else{
		//checks which i2c peripheral needs to be closed
		if(pI2Cx == I2C1){
			I2C1_PCLK_DI();
		}else if(pI2Cx == I2C2){
			I2C2_PCLK_DI();
		}else if(pI2Cx == I2C3){
			I2C3_PCLK_DI();
		}
	}
}

/************************************************************************************************
 * @fn						-> I2C_Init
 *
 * @Brief					-> This function is use to initialize the i2c peripheral
 *
 * @param[in]				-> base address of the I2C peripheral
 * @param[in]				->
 * @param[in]				->
 *
 * @return					-> void
 *
 * @Note					->
 *
 */


void I2C_Init(I2C_Handle_t *pI2CHandle){
	I2C_PeriClkControl(pI2CHandle->pI2Cx, ENABLE);

	uint32_t tempreg = 0;
	//1. first configure the ack control bit in cr1
	tempreg |= pI2CHandle->I2C_Config.I2C_AckControl << I2C_CR1_ACK;	//10th bit cr1
	//store the tempreg value in CR1 register
	pI2CHandle->pI2Cx->CR1 |= tempreg;
	//2. to configure the seriel clock speed needs to calculate the apb1 bus clock value
	//2. to do this i need to create seperate function for that now we have value of clock so that
	//configure FREQ field of CR2 register
	tempreg = 0;
	//get the clock value from above function and divide it 100000 to get the value 16
	tempreg = RCC_GetPCLK1Value()/100000U;
	//store the value in CR2 Freq field
	pI2CHandle->pI2Cx->CR2 |= (tempreg & 0x3F); //bit 0 to 5 is for freq field
	//configure the OAR register for this
	tempreg = 0;
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1; //because 0th bit is reserve for R/W bit
	//3. this is important to do as per reference manual
	tempreg |= (1 << 14);
	pI2CHandle->pI2Cx->OAR1 |= tempreg;

	//4. CCR register
	uint16_t ccr_value = 0;
	tempreg = 0;
	//checks the speed of the i2c serial
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM){
		//speed mode is standard mode means 100khz = 100000
		ccr_value = (RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		tempreg |= (ccr_value & 0xFFF);

	}else{
		//for fast mode, i have to set the 15 bit field, so the fast mode is activated
		//then program the duty cycle bit field according to reference manual
		tempreg |= (1 << 15);
		tempreg |= pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14;

		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2){
			ccr_value = (RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}else{
			ccr_value = (RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		tempreg |= (ccr_value & 0xFFF);
	}

	pI2CHandle->pI2Cx->CCR |= tempreg;
	//6. TRISE calculation
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM){
		//speed mode is standard mode means 100khz = 100000
		tempreg = 0;
		tempreg = (RCC_GetPCLK1Value() / 1000000U) + 1;


	}else{
		//fast mode
		tempreg = ((RCC_GetPCLK1Value() * 300) / 100000000U) + 1;

	}
	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);
}

/************************************************************************************************
 * @fn						-> I2C_DeInit
 *
 * @Brief					-> This function is use to de-initialize the i2c peripheral
 *
 * @param[in]				-> base address of the I2C peripheral
 * @param[in]				->
 * @param[in]				->
 *
 * @return					-> void
 *
 * @Note					->
 *
 */


void I2C_DeInit(I2C_RegDef_t *pI2Cx){
	if(pI2Cx == I2C1){
		I2C1_REG_RESET();
	}else if(pI2Cx == I2C2){
		I2C2_REG_RESET();
	}else if(pI2Cx == I2C3){
		I2C3_REG_RESET();
	}
}

/************************************************************************************************
 * @fn						-> I2C_MasterSendData
 *
 * @Brief					-> This function is use to send the data
 *
 * @param[in]				-> pointer to the handle structure
 * @param[in]				-> pointer to the txBuffer
 * @param[in]				-> Length of the data
 * @param[in]				-> Slave Address
 *
 * @return					-> void
 *
 * @Note					->
 *
 */

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr){
	//1. Generate the Start Condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. Confirm that start generation is completed by checking the SB flag in the SR1
	//	Note: Untill SB is cleared, clock is stretched(pulled to low)
	//so wait untill SB bit is cleared
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB)));

	//3. Execute the Address phase with R/W bit (0th bit) of slave addr
	I2C_ExecuteAddressPhaseWrite(pI2CHandle, SlaveAddr);

	//4. Confirms that address phase is completed, by checking the addr flag in SR1
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR)));

	//5. clears the addr flag according to it's sequence
	//Note: untill addr is cleared, scl is stretched(pulled to low)
	I2C_ClearAddrFlag(pI2CHandle);

	//6. send the data untill length becomes 0
	while(Len > 0){
		//wait untill txe is cleared
		while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE)));
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		Len--;
		pTxBuffer++;

	}

	//7. when length becomes 0, close the transmsission, when length becomes 0, you have to wait
	//untill the TXE flag and BTF flag is set after that only generate stop condition
	//Note: TXE = 1, BTF = 1 means SR and DR is empty, and the next transmission should begin
	//when BTF = 1, SCL will be stretched to low
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1_TXE)));
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1_BTF)));

	//8. Generate the stop condition
	if(Sr == I2C_DISABLE_SR)
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
}

/************************************************************************************************
 * @fn						-> I2C_MasterReceiveData
 *
 * @Brief					-> This function is use to receive the data
 *
 * @param[in]				-> pointer to the handle structure
 * @param[in]				-> pointer to the txBuffer
 * @param[in]				-> Length of the data
 * @param[in]				-> Slave Address
 *
 * @return					-> void
 *
 * @Note					->
 *
 */

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr){

	//1. Generate the start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. Confirm start condition by checking the sb flag, untill sb is cleared clock is stretched
	//to low, wait untill Sb flag is cleared
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB)));

	//3. Execute the slave address phase with r/w bit as 1 because it is read operation
	I2C_ExecuteAddressPhaseRead(pI2CHandle, SlaveAddr);

	//4. Confirm that address phase is successfully executed by checking the addr flag in sr1
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR)));

	//5. start reading the data
	//procedure to read only 1 byte from slave
	if(Len == 1){

		//1. Disable Acking
		I2C_ManageAcking(pI2CHandle, I2C_ACK_DISABLE);

		//2. clear the addr flag
		I2C_ClearAddrFlag(pI2CHandle);

		//3. wait untill rxne becomes 1
		while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE)));

		//4. generate stop condition
		if(Sr == I2C_DISABLE_SR)
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);


		//5. read data in to buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;
	}

	if(Len > 1){

		//1. clear the ADDR flag
		I2C_ClearAddrFlag(pI2CHandle);

		//2. read the data untill len becomes 0
		for(uint32_t i = Len; i > 0; i--){

			//3. wait untill rxne becomes 1
			I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE);

			//4. when last 2 bytes are remaining
			if(i == 2){

				//5. clear the ack bit
				I2C_ManageAcking(pI2CHandle, I2C_ACK_DISABLE);

				//6. generate the stop condition
				if(Sr == I2C_DISABLE_SR)
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}
			//7. read the data from data register in to buffer
			*pRxBuffer = pI2CHandle->pI2Cx->DR;

			//8. increment the buffer address
			pRxBuffer++;
		}
	}

	//9. re-enable acking
	if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE){
		I2C_ManageAcking(pI2CHandle, I2C_ACK_ENABLE);
	}

}


/************************************************************************************************
 * @fn						-> I2C_MasterReceiveDataIT
 *
 * @Brief					-> This function is use to sent the data with interrupt capability
 *
 * @param[in]				-> pointer to the handle structure
 * @param[in]				-> pointer to the txBuffer
 * @param[in]				-> Length of the data
 * @param[in]				-> Slave Address
 *
 * @return					-> state of peripheral
 *
 * @Note					->
 *
 */

uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr){

	//1. declare a local variable to get the state of i2c
	uint8_t busystate = pI2CHandle->TxRxState;

	//2. checks the state of i2c peripheral
	if(busystate != I2C_BUSY_IN_TX && busystate != I2C_BUSY_IN_RX){
		//3. store the Rx buffer address in a global variable
		pI2CHandle->pRxBuffer = pRxBuffer;
		//4. store the len information in global variable
		pI2CHandle->RxLen = Len;
		//5. mark the i2c state as busy in rx
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		//6. save the slave address in global variable dev addr
		pI2CHandle->DevAddr = SlaveAddr;
		//7. store repeated start in global variable sr
		pI2CHandle->Sr = Sr;
		//8. generate the start condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
		//9. enable the itbufen bit in cr2 to get the interrupt when sb is set
		(pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN));
		//10. enable the it ev en bit in cr2
		(pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN));
		//11. enable the it er en bit in cr2
		(pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN));
	}
	return busystate;
}

/************************************************************************************************
 * @fn						-> I2C_MasterSendDataIT
 *
 * @Brief					-> This function is use to receive the data with interrupt capability
 *
 * @param[in]				-> pointer to the handle structure
 * @param[in]				-> pointer to the txBuffer
 * @param[in]				-> Length of the data
 * @param[in]				-> Slave Address
 *
 * @return					-> void
 *
 * @Note					->
 *
 */

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr){
	//1. declaring a local variable to know the state of I2C bus
	uint8_t busystate = pI2CHandle->TxRxState;

	//2. compare the state with i2c states
	if(busystate != I2C_BUSY_IN_TX && busystate != I2C_BUSY_IN_RX){
		//1. store the TxBuffer address in a global variable
		pI2CHandle->pTxBuffer = pTxBuffer;
		//2. store the Len in the Len variable
		pI2CHandle->TxLen = Len;
		//3. mark the i2c state as busy in tx
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		//4. store the slave in the global variable dev addr
		pI2CHandle->DevAddr = SlaveAddr;
		//5. store the repeated start in global variable sr
		pI2CHandle->Sr = Sr;
		//6. generate the start condition now
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
		//7. enable the Interrupt buffer enable ITBUFEN bit in cr2
		(pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN));
		//8. enable the interrupt event enable bit in cr2
		(pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN));
		//9. enable the interrupt error enable bit in cr2
		(pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN));

	}
	return busystate;
}



/************************************************************************************************
 * @fn						-> I2C_ManageAcking
 *
 * @Brief					-> This function is use to control the acking
 *
 * @param[in]				-> pointer to handle structure
 * @param[in]				-> command to enable or disable
 * @param[in]				->
 *
 * @return					-> void
 *
 * @Note					->
 *
 */

void I2C_ManageAcking(I2C_Handle_t *pI2CHandle, uint8_t EnorDi){

	if(EnorDi == I2C_ACK_DISABLE){
		//enable the ack
		pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}else{
		//disable the ack
		pI2CHandle->pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}

}


/************************************************************************************************
 * @fn						-> I2C_IRQInterruptConfig
 *
 * @Brief					-> This function is use to configure the interrupts
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

void I2C_IRQInterruptConfig(uint8_t IRQ_Number, uint8_t EnorDi){
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
 * @fn						-> I2C_IRQPriorityConfig
 *
 * @Brief					-> This function is use to set the priority of the irq numbers
 *
 * @param[in]				-> irq numbers
 * @param[in]				-> irq priority
 * @param[in]				->
 *
 * @return					-> void
 *
 * @Note					->
 *
 */

void I2C_IRQPriorityConfig(uint8_t IRQ_Number, uint32_t IRQ_Priority){
	//1 first lets find out the priority register for this
	uint8_t iprx = IRQ_Number/4;
	uint8_t iprx_section = IRQ_Number % 4;
	uint8_t shift_amount = ((8 * iprx_section) + (8 - NO_OF_PR_BITS_IMPLEMENTED));

	//add this iprx value to pr base addr to get correct register
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQ_Priority << shift_amount);
}

/************************************************************************************************
 * @fn						-> I2C_EV_IRQHandling
 *
 * @Brief					-> This function is use handle the interrupts related to i2c events
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

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle){
	//this api is for common for both master and slave mode of i2c peripheral

	uint32_t temp1, temp2, temp3;
	//1. read the status of IT EV enable bit in cr2
	temp1 = (pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN));
	//2. read the status of IT BUF enable bit in cr2
	temp2 = (pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN));
	//3. now read the status of SB bit in SR1 to make sure the SB bit is set or not
	temp3 = (pI2CHandle->pI2Cx->CR2 & (1 << I2C_SR1_SB));

	//4. handle for interrupt generated by SB event
	//NOTE: SB flag is only applicable in master mode
	if(temp1 && temp3){
		//SB is set otherwise SB is not set
		//interrupt is generated by sb event
		//this block will not be executed in slave mode because for slave sb is always 0
		//in this block let's execute the address phase
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX){
		//call the execute address write api
			I2C_ExecuteAddressPhaseWrite(pI2CHandle, pI2CHandle->DevAddr);
		}else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX){
			//call the execute address phase read api
			I2C_ExecuteAddressPhaseRead(pI2CHandle, pI2CHandle->DevAddr);
		}
	}
	//check the ADDR bit in SR1
	temp3 |= (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR));
	//5. handle for interrupt generated by ADDR event
	//NOTE: when master mode: Address is sent
	//		when slave mode: Address is matched with own address

	if(temp1 && temp3){
		//interrupt is generated by addr event
		//clear the addr flag
		I2C_ClearAddrFlag(pI2CHandle);
	}

	//check the btf bit in sr1
	temp3 |= (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF));
	//6. handle for interrupt generated by byte transfer finished BTF event

	if(temp1 && temp3){
		//BTF flag is set
		//check i2c states
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX){
			//make sure that txe is also set, this indicates transmission is complete and
			//we can close the transmission
			if(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE)){
				if(pI2CHandle->TxLen == 0){
					//btf and txe both are set
					//generate the stop condition to close the transmission
					if(pI2CHandle->Sr == I2C_DISABLE_SR)
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

					//reset all the member element of handle structure
					I2C_CloseTransmission(pI2CHandle);

					//notify the application about this event
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);

				}
			}
		}else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX){
			;
		}

	}
	//check for Stopf bit in SR1
	temp3 |= (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF));

	//7. handle for interrupt generated by stopF event
	//NOTE: stop detection flag is applicable only in slave mode, for master this flag is never set
	//this block is never executed for the master mode
	if(temp1 && temp3){
		//stop is detected
		//to clear it, read the SR1 register followed write to CR1
		pI2CHandle->pI2Cx->CR1 |= 0x0000;
		//notify the application about it
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
	}

	//check for txe bit in sr1
	temp3 |= (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE));
	//8. handle for interrupt generated by txe event

	if(temp1 && temp2 && temp3){
		//txe flag is set
		//confirm the device is in master mode
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)){
		//we have to do the data transmission
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX){
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
			}
		}else{
			//device is in slave mode
			//make sure that slave is in sending mode
			if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA)){
				//slave send data
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
			}
		}
	}
	//check for rxne bit in sr1
	temp3 |= (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE));

	//9. handle for interrupt generated by rxne event

	if(temp1 && temp2 && temp3){
		//confirm the device is in master mode
		if(pI2CHandle->pI2Cx->SR2 & I2C_SR2_MSL){
			//make sure that device is in receive mode
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX){
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);
			}
		}else{
			//make sure that slave is in receiver mode
			if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA)){
				//receive the data
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
			}
		}
	}
}



void I2C_CloseReception(I2C_Handle_t *pI2CHandle){
	//disable all the interrupts, starts with ITBUFEN bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
	//disable the interrupt generated by ITEVEN bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);
	//mark the i2c state as ready
	pI2CHandle->TxRxState = I2C_READY;
	//point the rxbuffer address to null
	pI2CHandle->pRxBuffer = NULL;
	//make the rxlen is 0
	pI2CHandle->RxLen = 0;
	//make the rxsize is 0
	pI2CHandle->RxSize = 0;
	//enable the acking
	if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE)
		I2C_ManageAcking(pI2CHandle, I2C_ACK_ENABLE);
}

void I2C_CloseTransmission(I2C_Handle_t *pI2CHandle){
	//disable all the interrupts, starts with ITBUFEN bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
	//disable the interrupt generated by ITEVEN bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	//mark the i2c state as ready
	pI2CHandle->TxRxState = I2C_READY;
	//point the txbuffer address to null
	pI2CHandle->pTxBuffer = NULL;
	//make the txlen is 0
	pI2CHandle->TxLen = 0;
}

/************************************************************************************************
 * @fn						-> I2C_ER_IRQHandling
 *
 * @Brief					-> This function is use to handle the interrupts related to errors
 *
 * @param[in]				-> pointer to the handle structure of i2c
 * @param[in]				->
 * @param[in]				->
 *
 * @return					-> void
 *
 * @Note					->
 *
 */


void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle){
	uint32_t temp1, temp2;
	//check the status of interrupt error enable bit in cr2
	temp1 = (pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITERREN));

	/******************************1. check for bus error********************************/
	temp2 = (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BERR));
	if(temp1 && temp2){
		//confirmed that bus error happens,
		//1. clears the bus error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_BERR);
		//2. notify the application that bus error is cleared
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ER_BERR);
	}
	/*****************************2. check for arbitration lost******************************/
	temp2 = (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ARLO));
	if(temp1 && temp2){
		//confirmed that arbitration lost error occured
		//1. clears the arbitration lost flag in SR1
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_ARLO);
		//2. notify the application that arbitration lost flag is cleared
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ER_ARL0);
	}
	/*****************************3. check for ack failure***********************************/
	temp2 = (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_AF));
	if(temp1 && temp2){
		//ack failure happens
		//1. clears the ack failure flag in sR1
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_AF);
		//2. notify the application that ack failure resolved
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ER_AF);
	}
	/****************************4. Check for Overrun/Underrun********************************/
	temp2 = (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_OVR));
	if(temp1 && temp2){
		//over run or under run error occured
		//1. clears the ovr flag in sr1
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_OVR);
		//2. notify the application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ER_OVR);
	}
	/****************************5. Check for timeout error***********************************/
	temp2 = (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TIMEOUT));
	if(temp1 && temp2){
		//timeout error occured
		//1. clears the timeout flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_TIMEOUT);
		//2. notify the application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ER_TIMEOUT);
	}
}

/************************************************************************************************
 * @fn						-> I2C_PeripheralControl
 *
 * @Brief					-> This function is use to control I2C peripheral
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

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		//enable the PE bit in CR1 register
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	}else{
		//disable the PE bit in CR1 register
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}


/************************************************************************************************
 * @fn						-> I2C_GeneratesStopCondition
 *
 * @Brief					-> This function is use to generates the stop condition
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

void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx){
	//set the stop bit in CR1
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

/************************************************************************************************
 * @fn						-> I2C_SlaveSendData
 *
 * @Brief					-> This function is use to sent the data in slave mode
 *
 * @param[in]				-> base address of the SPI peripheral
 * @param[in]				-> data
 * @param[in]				->
 *
 * @return					-> void
 *
 * @Note					->
 *
 */

void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data){
	//loads the data in data register
	pI2Cx->DR = data;
}

/************************************************************************************************
 * @fn						-> I2C_SlaveReceiveData
 *
 * @Brief					-> This function is use to receive the data in slave mode
 *
 * @param[in]				-> base address of the SPI peripheral
 * @param[in]				->
 * @param[in]				->
 *
 * @return					-> data
 *
 * @Note					->
 *
 */

uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx){

	//return the data in the data register
	return (uint8_t)pI2Cx->DR;

}

/************************************************************************************************
 * @fn						-> I2C_SlaveEnableDisableCallbackEvents
 *
 * @Brief					-> This function is use to enable or disable the interrupts
 * 							   control bits in slave mode
 *
 * @param[in]				-> base address of the SPI peripheral
 * @param[in]				-> command to enable or disable
 * @param[in]				->
 *
 * @return					->
 *
 * @Note					->
 *
 */
void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t EnorDi){

	if(EnorDi == ENABLE){
		pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);
		pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);
		pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);

	}else{
		pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
		pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);
		pI2Cx->CR2 &= ~(1 << I2C_CR2_ITERREN);
	}
}
