/*
 * stm32f411xx_i2c_driver.h
 *
 *  Created on: Dec 29, 2024
 *      Author: hp
 */

#ifndef INC_STM32F411XX_I2C_DRIVER_H_
#define INC_STM32F411XX_I2C_DRIVER_H_

#include "stm32f411xx.h"

/*
 *  configuration structure for i2c peripheral
 */

typedef struct{

	uint32_t I2C_SCLSpeed;
	uint8_t  I2C_DeviceAddress;
	uint8_t  I2C_AckControl;
	uint16_t I2C_FMDutyCycle;


}I2C_Config_t;


/*
 * handle structure of i2c peripheral
 */

typedef struct{

	I2C_RegDef_t *pI2Cx;		/* pointer to base address of i2c peripherals 	*/
	I2C_Config_t I2C_Config;	/* variable to hold i2c configurations 			*/
	uint8_t 	 *pTxBuffer;	/* pointer to the Tx buffer						*/
	uint8_t 	 *pRxBuffer;	/* pointer to the Rx buffer 					*/
	uint32_t 	 TxLen;			/* Transmission length 							*/
	uint32_t	 RxLen;			/* Reception length								*/
	uint8_t 	 TxRxState;		/* variable to hold the i2c states				*/
	uint8_t 	 DevAddr;		/* variable to hold the slave address			*/
	uint32_t 	 RxSize;		/* variable to hold the Rx size 				*/
	uint8_t 	 Sr;			/* variable to hold the repeated start setting	*/

}I2C_Handle_t;


/*
 * I2C Application States
 */

#define I2C_READY			0
#define I2C_BUSY_IN_RX		1
#define I2C_BUSY_IN_TX		2


/*
 * I2C repeated starts
 */

#define I2C_DISABLE_SR	RESET
#define I2C_ENABLE_SR	SET


/*
 * user configurable macros
 * @I2C_SCLSpeed
 */

#define I2C_SCL_SPEED_SM	100000
#define I2C_SCL_SPEED_FM4K	400000
#define I2C_SCL_SPEED_FM2K	200000

/*
 * @I2C_ACKControl
 */

#define I2C_ACK_DISABLE		0
#define I2C_ACK_ENABLE		1

/*
 * @I2C_FMDutyCycle
 */

#define I2C_FM_DUTY_2		0
#define I2C_FM_DUTY_16_9	1

/*
 * i2c application events
 */

#define I2C_EV_TX_CMPLT		0
#define I2C_EV_RX_CMPLT		1
#define I2C_EV_STOP			2

#define I2C_ER_BERR			3
#define I2C_ER_ARL0			4
#define I2C_ER_AF			5
#define I2C_ER_OVR			6
#define I2C_ER_TIMEOUT		7
#define I2C_EV_DATA_REQ		8
#define I2C_EV_DATA_RCV		9




/*
 * various flags related to i2c peripheral
 */

#define I2C_FLAG_SB			(1 << I2C_SR1_SB)
#define I2C_FLAG_ADDR		(1 << I2C_SR1_ADDR)
#define I2C_FLAG_BTF		(1 << I2C_SR1_BTF)
#define I2C_FLAG_STOPF		(1 << I2C_SR1_STOPF)

#define I2C_FLAG_RXNE		(1 << I2C_SR1_RXNE)
#define I2C_FLAG_TXE		(1 << I2C_SR1_TXE)
#define I2C_FLAG_BERR		(1 << I2C_SR1_BERR)
#define I2C_FLAG_AF			(1 << I2C_SR1_AF)
#define I2C_FLAG_OVR		(1 << I2C_SR1_OVR)
#define I2C_FLAG_PECERR		(1 << I2C_SR1_PECERR)
#define I2C_FLAG_TIMEOUT	(1 << I2C_SR1_TIMEOUT)
#define I2C_FLAG_SMBALERT	(1 << I2C_SR1_SMBALERT)


/*************************************************************************************************
*							API'S Supported by this driver for I2C Peripheral
*@ 					for more information check functions description and comments
*************************************************************************************************/

/*
 * peripheral clock setup
 */

void I2C_PeriClkControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/*
 * i2c Init and deinit api's
 */

void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/*
 * Data send and receive polling based
 */

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);

/*
 * Data send and receive interrupt based
 */

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);

void I2C_CloseReception(I2C_Handle_t *pI2CHandle);
void I2C_CloseTransmission(I2C_Handle_t *pI2CHandle);


/*
 * Data send and receive slave mode api's
 */

void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx);

void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);


/*
 * IRQ Configuration and ISR handling
 */

void I2C_IRQInterrutConfig(uint8_t IRQ_Number, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQ_Number, uint32_t IRQ_Priority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);



/*
 * other i2c peripheral control api's
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);

void I2C_ManageAcking(I2C_Handle_t *pI2CHandle, uint8_t EnorDi);

void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);

/*
 * I2C Application event callback
 */

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv);

#endif /* INC_STM32F411XX_I2C_DRIVER_H_ */
