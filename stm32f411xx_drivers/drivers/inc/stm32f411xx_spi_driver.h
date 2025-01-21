/*
 * stm32f411xx_spi_driver.h
 *
 *  Created on: Dec 22, 2024
 *      Author: hp
 */

#ifndef INC_STM32F411XX_SPI_DRIVER_H_
#define INC_STM32F411XX_SPI_DRIVER_H_

#include "stm32f411xx.h"

/*
 * Configuration structure of SPI Peripheral to hold the various configurable settings
 */

typedef struct{

	uint8_t SPI_DeviceMode;		/*|-> this variable to select the spi device mode, master or slave  */
	uint8_t SPI_BusConfig;		/*|-> this variable to select the spi bus configuration				*/
	uint8_t SPI_SclkSpeed;		/*|-> this variable to hold the SPI clock speed 					*/
	uint8_t SPI_DFF;			/*|-> this variable to configure the data frame format 				*/
	uint8_t SPI_CPOL;			/*|-> this is to select the CPOL value 								*/
	uint8_t SPI_CPHA;			/*|-> this is to select the CPHA value								*/
	uint8_t SPI_SSM;			/*|-> this is to select the software slave select management		*/

}SPI_Config_t;

/*
 * handle structure of SPI Peripheral to hold the various registers and configuration settings
 */

typedef struct{

	SPI_RegDef_t *pSPIx;		/*|-> base addresses of spi peripheral 							*/
	SPI_Config_t  SPIConfig;	/*|-> spi configuration settings 								*/
	uint8_t 	*pTxBuffer;				/*|-> This holds address of TX buffer of SPI			*/
	uint8_t 	*pRxBuffer;				/*|-> This holds address of Rx buffer of SPI 			*/
	uint8_t 	TxLen;					/*|-> store the length of transmitted data 				*/
	uint8_t 	RxLen;					/*|-> variable to store the length of received data 	*/
	uint8_t 	TxState;				/*|-> variable to store the state of transmission 		*/
	uint8_t		RxState;				/*|-> variable to store the state of reception			*/

}SPI_Handle_t;


/*
 * @SPI_DeviceMode
 */

#define SPI_DEVICE_MODE_SLAVE		0
#define SPI_DEVICE_MODE_MASTER		1


/*
 * @SPI_BusConfig
 */

#define SPI_BUS_CONFIG_FD				1
#define SPI_BUS_CONFIG_HD				2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY	3

/*
 * @SPI_SclkSpeed
 */

#define SPI_SCLK_SPEED_DIV2				0
#define SPI_SCLK_SPEED_DIV4				1
#define SPI_SCLK_SPEED_DIV8				2
#define SPI_SCLK_SPEED_DIV16			3
#define SPI_SCLK_SPEED_DIV32			4
#define SPI_SCLK_SPEED_DIV64			5
#define SPI_SCLK_SPEED_DIV128			6
#define SPI_SCLK_SPEED_DIV256			7



/*
 * @SPI_DFF
 */

#define SPI_DFF_8BITS					0
#define SPI_DFF_16BITS					1


/*
 * @SPI_CPOL
 */

#define SPI_CPOL_LOW			0
#define SPI_CPOL_HIGH			1

/*
 * @SPI_CPHA
 */

#define SPI_CPHA_LOW			0
#define SPI_CPHA_HIGH			1


/*
 * @SPI_SSM
 */

#define SPI_SSM_DI				0
#define SPI_SSM_EN				1


/*
 * SPI related status flags definition
 */

#define SPI_TXE_FLAG 	(1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG 	(1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG 	(1 << SPI_SR_BSY)

/*
 * spi application possible events
 */

#define SPI_EVENT_TX_CMPLT		1
#define SPI_EVENT_RX_CMPLT		2
#define SPI_EVENT_OVR_ERR		3
#define SPI_EVENT_CRC_ERR		4


/*
 * SPI application states
 */

#define SPI_READY			0
#define SPI_BUSY_IN_TX		1
#define SPI_BUSY_IN_RX		2



/*************************************************************************************************
*							API'S Supported by this driver for SPI Peripheral
*@ 					for more information check functions description and comments
*************************************************************************************************/

/*
 * peripheral clock setup
 */

void SPI_PeriClkControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*
 * Spi Init and deinit api's
 */

void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Data send and receive polling based
 */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

/*
 * Data send and receive interrupt based
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);


/*
 * IRQ Configuration and ISR handling
 */

void SPI_IRQInterrutConfig(uint8_t IRQ_Number, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQ_Number, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

/*
 * other spi peripheral control api's
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);

void SPI_ClearOvrFlag(SPI_RegDef_t *pSPIx);

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);

void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

/*
 * SPI Application event callback
 */

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv);

#endif /* INC_STM32F411XX_SPI_DRIVER_H_ */
