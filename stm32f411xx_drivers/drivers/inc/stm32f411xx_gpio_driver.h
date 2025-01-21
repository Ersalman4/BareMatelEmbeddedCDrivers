/*
 * stm32f411xx_gpio_driver.h
 *
 *  Created on: Dec 20, 2024
 *      Author: hp
 */

#ifndef INC_STM32F411XX_GPIO_DRIVER_H_
#define INC_STM32F411XX_GPIO_DRIVER_H_

#include "stm32f411xx.h"

/*
 * Configuration structure for GPIO Peripheral
 */

typedef struct{

	uint8_t GPIO_PinNumber;			/*|-> possible values from @GPIO_PIN_NUMBERS					*/
	uint8_t GPIO_PinMode;			/*|-> possible values from @GPIO_PIN_MODES						*/
	uint8_t GPIO_PinSpeed;			/*|-> possible values from @GPIO_PIN_SPEED						*/
	uint8_t GPIO_PinPuPdControl;	/*|-> possible values from @GPIO_PIN_PUPD						*/
	uint8_t GPIO_PinOPType;			/*|-> possible values from @GPIO_PIN_OP_TYPE					*/
	uint8_t GPIO_PinAltFuncMode;	/*|-> possible values from @GPIO_PIN_ALTFN						*/


}GPIO_PinConfig_t;


/*
 * Handle structure for GPIO Peripheral
 */

typedef struct{

	GPIO_RegDef_t *pGPIOx;				/*|-> this holds the base address of GPIO peripheral */

	GPIO_PinConfig_t GPIO_PinConfig;	/*|-> this holds the configuration settings of GPIO peripheral */

}GPIO_Handle_t;


/*
 *  @GPIO_PIN_MODES
 *  Peripheral specific macros
 *  this is for GPIO mode settings
 */

#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FT		4
#define GPIO_MODE_IT_RT		5
#define GPIO_MODE_IT_RFT	6

/*
 * @GPIO_PIN_SPEED
 * GPIO Pin possible output types
 */

#define GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OD		1


/*
 * @GPIO_PIN_SPEED
 * GPIO Pin speed settings
 */

#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3



/*
 * @GPIO_PIN_PUPD
 * GPIO Pin pull up and pull down settings
 */

#define GPIO_PIN_NOPUPD		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2

/*
 * @GPIO_PIN_NUMBERS
 * possible pin numbers
 */

#define GPIO_PIN_NO_0		0
#define GPIO_PIN_NO_1		1
#define GPIO_PIN_NO_2		2
#define GPIO_PIN_NO_3		3
#define GPIO_PIN_NO_4		4
#define GPIO_PIN_NO_5		5
#define GPIO_PIN_NO_6		6
#define GPIO_PIN_NO_7		7
#define GPIO_PIN_NO_8		8
#define GPIO_PIN_NO_9		9
#define GPIO_PIN_NO_10		10
#define GPIO_PIN_NO_11		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15


/*************************************************************************************************
 * 							API'S Supported By this Driver
 * 				For more information check the function description and comments
 *************************************************************************************************/

/*
 * Peripheral clock setup
 */
void GPIO_PeriClkControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * GPIO initialization and deinitialization api's
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Read the entire port API and pin api
 */

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);

uint8_t  GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);


/*
 * toggle output pin api
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);


/*
 * write to output port and pin api
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value);

void GPIO_IRQInterruptConfig(uint8_t IRQ_Number, uint8_t EnorDi);

void GPIO_IRQPriorityConfig(uint8_t IRQ_Number, uint32_t IRQ_Priority);

void GPIO_IRQHandling(uint8_t PinNumber);



#endif /* INC_STM32F411XX_GPIO_DRIVER_H_ */
