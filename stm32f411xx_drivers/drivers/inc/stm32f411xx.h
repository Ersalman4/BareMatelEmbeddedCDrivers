/*
 * stm32f411xx.h
 *
 *  Created on: Dec 19, 2024
 *      Author: Salman Ahmad
 */

#ifndef INC_STM32F411XX_H_
#define INC_STM32F411XX_H_

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#define __vo						volatile
#define __weak						__attribute__((weak))

/****************************STARTS PROCESSOR SPECIFIC DETAIlS************************************
 *
 * 				ARM Cortex Mx Processor NVIC ISERx registers base addresses
 *
 ************************************************************************************************/

#define NVIC_ISER0					((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1					((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2					((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3					((__vo uint32_t*)0xE000E10C)
#define NVIC_ISER4					((__vo uint32_t*)0xE000E110)
#define NVIC_ISER5					((__vo uint32_t*)0xE000E114)
#define NVIC_ISER6					((__vo uint32_t*)0xE000E118)
#define NVIC_ISER7					((__vo uint32_t*)0xE000E11C)

/******************ARM Cortex Mx Processor NVIC ISERx registers base addresses*******************/

#define NVIC_ICER0					((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1					((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2					((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3					((__vo uint32_t*)0xE000E18C)
#define NVIC_ICER4					((__vo uint32_t*)0xE000E190)
#define NVIC_ICER5					((__vo uint32_t*)0xE000E194)
#define NVIC_ICER6					((__vo uint32_t*)0xE000E198)
#define NVIC_ICER7					((__vo uint32_t*)0xE000E19C)


/********************ARM Cortex Mx processors priority registers base address********************/

#define NVIC_PR_BASE_ADDR			((__vo uint32_t*)0xE000E400)

/*****************ARM Cortex Mx processors number of priority bits implemented*******************/

#define NO_OF_PR_BITS_IMPLEMENTED	4

/*
 * macros for all the possible priority levels
 */


#define NVIC_IRQ_PRI0				0
#define NVIC_IRQ_PRI1				1
#define NVIC_IRQ_PRI2				2
#define NVIC_IRQ_PRI3				3
#define NVIC_IRQ_PRI4				4
#define NVIC_IRQ_PRI5				5
#define NVIC_IRQ_PRI6				6
#define NVIC_IRQ_PRI7				7
#define NVIC_IRQ_PRI8				8
#define NVIC_IRQ_PRI9				9
#define NVIC_IRQ_PRI10				10
#define NVIC_IRQ_PRI11				11
#define NVIC_IRQ_PRI12				12
#define NVIC_IRQ_PRI13				13
#define NVIC_IRQ_PRI14				14
#define NVIC_IRQ_PRI15				15


/*
 * Base addresses of various peripherals of mcu
 */

#define FLASH_BASEADDR				0x08000000U				/*|-> address of flash memory 	*/
#define SRAM1_BASEADDR				0x20000000U				/*|-> address of sram1 			*/
#define SRAM2_BASEADDR				0x2001C000U				/*|-> address of sram2 			*/
#define SRAM_BASEADDR				SRAM1_BASEADDR			/*|-> address of sram 			*/
#define ROM_BASEADDR				0x1FFF0000U				/*|-> Rom address 				*/

/*
 * Base addresses of various bus of mcu
 */

#define PERIPH_BASEADDR				0x40000000U				/*|-> base address of bus starts */
#define AHB1_BASEADDR				0x40020000U				/*|-> base address of AHB1 bus 	 */
#define AHB2_BASEADDR				0x50000000U				/*|-> base address of AHB2 bus 	 */
#define APB1_BASEADDR				PERIPH_BASEADDR			/*|-> same as periph bus 		 */
#define APB2_BASEADDR				0x40007C00U				/*|-> base address of APB2 bus 	 */


/*
 * Base addresses of  peripherals which is hanging on AHB1 bus
 */

#define GPIOA_BASEADDR				(AHB1_BASEADDR + 0x0000)
#define GPIOB_BASEADDR				(AHB1_BASEADDR + 0x0400)
#define GPIOC_BASEADDR				(AHB1_BASEADDR + 0x0800)
#define GPIOD_BASEADDR				(AHB1_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR				(AHB1_BASEADDR + 0x1000)
#define GPIOF_BASEADDR				(AHB1_BASEADDR + 0x1400)
#define GPIOG_BASEADDR 				(AHB1_BASEADDR + 0x1800)
#define GPIOH_BASEADDR				(AHB1_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR 				(AHB1_BASEADDR + 0x2000)

#define RCC_BASEADDR				(AHB1_BASEADDR + 0x3800)

/*
 * Base addresses of peripherals which is hanging on APB1 bus
 */

#define SPI2_BASEADDR				(APB1_BASEADDR + 0x3800)
#define SPI3_BASEADDR				(APB1_BASEADDR + 0x3C00)
#define I2C1_BASEADDR				(APB1_BASEADDR + 0x5400)
#define I2C2_BASEADDR				(APB1_BASEADDR + 0x5800)
#define I2C3_BASEADDR				(APB1_BASEADDR + 0x5C00)
#define USART2_BASEADDR				(APB1_BASEADDR + 0x4400)
#define USART3_BASEADDR				(APB1_BASEADDR + 0x4800)
#define UART4_BASEADDR				(APB1_BASEADDR + 0x4C00)
#define UART5_BASEADDR				(APB1_BASEADDR + 0x5000)

/*
 * Base addresses of peripherals which is hanging on APB2 bus
 */

#define USART1_BASEADDR				(APB2_BASEADDR + 0x1000)
#define USART6_BASEADDR				(APB2_BASEADDR + 0x1400)
#define SPI1_BASEADDR				(APB2_BASEADDR + 0x3000)
#define EXTI_BASEADDR				(APB2_BASEADDR + 0x3C00)
#define SYSCFG_BASEADDR				(APB2_BASEADDR + 0x3800)

/***************************Peripheral register definition structures****************************/
/*
 * structuring GPIO periphal register
 */

typedef struct{

	__vo uint32_t  MODER;	/*|-> GPIO Port Mode register		  		Address Offset: 0x0000	*/
	__vo uint32_t  OTYPER;	/*|-> GPIO Port Output Type Register 		Address Offset: 0x0400	*/
	__vo uint32_t  OSPEEDR;	/*|-> GPIO Port Output Speed Register 		Address Offset: 0x0800	*/
	__vo uint32_t  PUPDR;	/*|-> GPIO Port Pull Pull down Register 	Address Offset: 0x0C00	*/
	__vo uint32_t  IDR;		/*|-> GPIO Port Input data register 		Address Offset: 0x1000	*/
	__vo uint32_t  ODR;		/*|-> GPIO Port Output data register 		Address Offset: 0x1400	*/
	__vo uint32_t  BSRR;	/*|-> GPIO Port Bit Set Reset Register 		Address Offset: 0x1800	*/
	__vo uint32_t  LCKR;	/*|-> GPIO Port Configuration Lock register Address Offset: 0x1C00	*/
	__vo uint32_t  AFR[2];	/*|-> GPIO Port Alternate function register Address Offset: 0x2000	*/

}GPIO_RegDef_t;


typedef struct{

	__vo uint32_t  CR;			/*|-> RCC Clock Control Register			Address Offset: 0x0000	*/
	__vo uint32_t  PLLCFGR; 	/*|-> PLL Configuration Register			Address Offset: 0x0400	*/
	__vo uint32_t  CFGR;		/*|-> Clock Configuration Register			Address Offset: 0x0800	*/
	__vo uint32_t  CIR;			/*|-> Clock Interrupt Register				Address Offset: 0x0C00	*/
	__vo uint32_t  AHB1RSTR;	/*|-> Reset register for AHB1 peripherals   Address Offset: 0x1000  */
	__vo uint32_t  AHB2RSTR;	/*|-> Reset Register for AHB2 peripherals   Address Offset: 0x1400	*/
	__vo uint32_t  AHB3RSTR;	/*|-> Reset Register for AHB3 peripherals   Address Offset: 0x1800  */
		 uint32_t  RESERVED0;	/*|-> Reserved Space						Address Offset: 0x1C00	*/
	__vo uint32_t  APB1RSTR;	/*|-> Reset Register for APB1 peripherals	Address Offset: 0x2000	*/
	__vo uint32_t  APB2RSTR;	/*|-> Reset Register for APB2 peripheral	Address Offset: 0x2400	*/
		 uint32_t  RESERVED1[2]; /*|-> Reserved Space 						Address Offset: 0x28-2C */
	__vo uint32_t  AHB1ENR;		/*|-> AHB1 peripheral enable register   	Address Offset: 0x3000	*/
	__vo uint32_t  AHB2ENR;		/*|-> AHB2 peripheral enable register		Address Offset: 0x3400	*/
	__vo uint32_t  AHB3ENR;		/*|-> AHB3 peripheral enable register 	 	Address Offset: 0x3800	*/
		 uint32_t  RESERVED2;	/*|-> Reserved Space						Address Offset: 0x3C00	*/
	__vo uint32_t  APB1ENR;		/*|-> APB1 peripheral enable register		Address Offset: 0x4000	*/
	__vo uint32_t  APB2ENR;		/*|-> APB2 peripheral enable register		Address Offset: 0x4400	*/
		 uint32_t  RESERVED3[2]; /*|-> Reserved Space						Address Offset: 0x48-4C */
	__vo uint32_t  AHB1LPENR;	/*|-> AHB1 Peripheral low power enable		Address Offset: 0x5000  */
	__vo uint32_t  AHB2LPENR;	/*|-> AHB2 Peripheral low power enable 		Address Offset: 0x5400  */
	__vo uint32_t  AHB3LPENR;	/*|-> AHB3 Peripheral low power enable 		Address Offset: 0x5800	*/
		 uint32_t  RESERVED4;	/*|-> Reserved Space						Address Offset: 0x5C00	*/
	__vo uint32_t  APB1LPENR;	/*|-> APB1 low power enable register 		Address Offset: 0x6000	*/
	__vo uint32_t  APB2LPENR;	/*|-> APB2 low power enable register		Address Offset: 0x6400	*/
		 uint32_t  RESERVED5[2];	/*|-> Reserved Space						Address Offset: 0x68-6C */
	__vo uint32_t  BDCR;		/*|-> Backup domain control register		Address Offset: 0x7000	*/
	__vo uint32_t  CSR;			/*|-> Clock & Status Register				Address Offset: 0x7400	*/
		 uint32_t  RESERVED6[2]; /*|-> Reserved Space						Address Offset: 0x78-7C */
	__vo uint32_t  SSCGR;		/*|-> Spread Spectrum Clock Generation reg	Address Offset: 0x8000	*/
	__vo uint32_t  PLLI2SCFGR;	/*|-> PLL I2S Configuration register		Address Offset: 0x8400	*/

}RCC_RegDef_t;


/*
 * EXTI register definitions
 */

typedef struct{

	__vo uint32_t IMR;			/*|-> Interrupt mask register				Address Offset: 0x00 	*/
	__vo uint32_t EMR;			/*|-> Event mask register					Address Offset: 0x04	*/
	__vo uint32_t RTSR;			/*|-> Rising trigger selection register		Address Offset: 0x08	*/
	__vo uint32_t FTSR;			/*|-> Falling trigger selection register	Address Offset: 0x0C	*/
	__vo uint32_t SWIER;		/*|-> Software interrupt event register		Address Offset: 0x10	*/
	__vo uint32_t PR;			/*|-> pending register						Address Offset: 0x14	*/

}EXTI_RegDef_t;


/*
 * register definitions for syscfg peripheral
 */

typedef struct{

	__vo uint32_t MEMRMP;		/*|-> SysCfg memory remap register			Address Offset: 0x00	*/
	__vo uint32_t PMC;			/*|-> SysCfg peripheral mode configuration  Address Offset: 0x04	*/
	__vo uint32_t EXTICR[4];	/*|-> EXTI line control register			Address Offset: 0x08-14 */
		 uint32_t RESERVED0[2]; /*|-> Reserved Space						Address Offset: 0x18-1C */
	__vo uint32_t CMPCR;		/*|-> compensation cell control register	Address Offset: 0x20	*/
		 uint32_t RESERVED1[2]; /*|-> Reserved Space						Address Offset: 0x24-28 */
	__vo uint32_t CFGR;			/*|-> Configuration register				Address Offset: 0x2C	*/

}SYSCFG_RegDef_t;

/*
 * SPI register definition structure
 */

typedef struct{

	__vo uint32_t CR1;		/*|-> SPI CR1 register							Address Offset: 0x0000	*/
	__vo uint32_t CR2;		/*|-> SPI CR2 register							Address Offset: 0x0400	*/
	__vo uint32_t SR;		/*|-> SPI Status register						Address Offset: 0x0800	*/
	__vo uint32_t DR;		/*|-> SPI Data register							Address Offset: 0x0C00	*/
	__vo uint32_t CRCPR;	/*|-> SPI CRC pending register					Address Offset: 0x1000	*/
	__vo uint32_t RXCRCR;	/*|-> SPI RX CRC register						Address Offset: 0x1400	*/
	__vo uint32_t TXCRCR;	/*|-> SPI TX CRC register						Address Offset: 0x1800	*/
	__vo uint32_t I2SCFGR;	/*|-> SPI I2S configuration register			Address Offset: 0x1C00	*/
	__vo uint32_t I2SPR;	/*|-> SPI I2S pending register					Address Offset: 0x2000	*/

}SPI_RegDef_t;


/************************peripheral register definition structures USART peripheral**************
 * Note: registers of a peripheral are specific to MCU
 *
 ************************************************************************************************/

typedef struct{

	__vo uint32_t SR;				/*|-> USART status register						offset: 0x00		*/
	__vo uint32_t DR;				/*|-> USART data register						offset: 0x04		*/
	__vo uint32_t BRR;				/*|-> USART Baud rate register					offset: 0x08		*/
	__vo uint32_t CR1;				/*|-> USART Control register 1 					offset: 0x0C		*/
	__vo uint32_t CR2;				/*|-> USART Control register 2					offset: 0x10		*/
	__vo uint32_t CR3;				/*|-> USART Control register 3					offset: 0x14		*/
	__vo uint32_t GTPR;				/*|-> USART General time and prescalar register offset: 0x18		*/


}USART_RegDef_t;


/************************peripheral register definition structures I2C peripheral********************************
 * Note: registers of a peripheral are specific to MCU
 *
 ************************************************************************************************/

typedef struct{
	__vo uint32_t CR1;				/*|-> I2Cx peripheral control register1	Address offset: 0x00        */
	__vo uint32_t CR2;				/*|-> I2Cx peripheral control register2	Address offset: 0x04        */
	__vo uint32_t OAR1;				/*|-> I2Cx peripheral own address register1	    offset: 0x08        */
	__vo uint32_t OAR2;				/*|-> I2Cx peripheral own address register2     offset: 0x0C        */
	__vo uint32_t DR;				/*|-> I2Cx peripheral data register			    offset: 0x10        */
	__vo uint32_t SR1;				/*|-> I2Cx peripheral status register1		    offset: 0x14        */
	__vo uint32_t SR2;				/*|-> I2Cx peripheral status register 2 	    offset: 0x18		*/
	__vo uint32_t CCR;				/*|-> I2Cx peripheral control register			offset: 0x1C		*/
	__vo uint32_t TRISE;			/*|-> I2Cx peripheral T rise register 			offset: 0x20		*/
	__vo uint32_t FLTR;	 			/*|-> I2Cx peripheral FLTR register				offset: 0x24		*/



}I2C_RegDef_t;

/*
 * peripheral register definition typecasted to it's pointer based
 */

#define GPIOA							((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB							((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC							((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD							((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE							((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF							((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG							((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH							((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI							((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC								((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI							((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG							((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1							((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2							((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3							((SPI_RegDef_t*)SPI3_BASEADDR)

#define I2C1							((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2							((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3							((I2C_RegDef_t*)I2C3_BASEADDR)

#define USART1							((USART_RegDef_t*)USART1_BASEADDR)
#define USART2							((USART_RegDef_t*)USART2_BASEADDR)
#define USART3							((USART_RegDef_t*)USART3_BASEADDR)
#define USART6							((USART_RegDef_t*)USART6_BASEADDR)

#define UART4							((USART_RegDef_t*)UART4_BASEADDR)
#define UART5							((USART_RegDef_t*)UART5_BASEADDR)
/*
 * Clock enable and disable macros
 */

/**************************Clock Enable Macros for GPIO Peripherals *****************************/

#define GPIOA_PCLK_EN()					(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()					(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()					(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()					(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()					(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()					(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()					(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()					(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()					(RCC->AHB1ENR |= (1 << 8))

/***************************Clock Enable Macros for USART Peripherals****************************/

#define USART1_PCLK_EN()				(RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()				(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()				(RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()					(RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()					(RCC->APB1ENR |= (1 << 20))
#define USART6_PCLK_EN()				(RCC->APB2ENR |= (1 << 5))

/***************************Clock Enable Macros for SPI Peripherals******************************/

#define SPI1_PCLK_EN()					(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()					(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()					(RCC->APB1ENR |= (1 << 15))


/******************************Clock Enable Macros for I2C Peripheral***************************/

#define I2C1_PCLK_EN()					(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()					(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()					(RCC->APB1ENR |= (1 << 23))

/*****************************Clock Enable Macros for SYSCFG Peripheral*************************/

#define SYSCFG_PCLK_EN()				(RCC->APB2ENR |= (1 << 14))


/******************************Clock Disable Macros for GPIO Peripheral**************************/

#define GPIOA_PCLK_DI()					(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()					(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()					(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()					(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()					(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()					(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()					(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()					(RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI()					(RCC->AHB1ENR &= ~(1 << 8))


/******************************Clock Disable Macros for GPIO Peripheral**************************/

#define SPI1_PCLK_DI()					(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()					(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()					(RCC->APB1ENR &= ~(1 << 15))

/******************************Clock Disable Macros for GPIO Peripheral**************************/

#define I2C1_PCLK_DI()					(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()					(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()					(RCC->APB1ENR &= ~(1 << 23))


/***************************Clock Enable Macros for USART Peripherals****************************/

#define USART1_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI()					(RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI()					(RCC->APB1ENR &= ~(1 << 20))
#define USART6_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 5))


/****************************Clock Disable Macros for SysCFG Peripheral**************************/

#define SYSCFG_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 14))

/*
 * macros to reset GPIO registers in one short
 */

#define GPIOA_REG_RESET()				do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~ (1 << 0));}while(0)
#define GPIOB_REG_RESET()				do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~ (1 << 0));}while(0)
#define GPIOC_REG_RESET()				do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~ (1 << 0));}while(0)
#define GPIOD_REG_RESET()				do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~ (1 << 0));}while(0)
#define GPIOE_REG_RESET()				do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~ (1 << 0));}while(0)
#define GPIOF_REG_RESET()				do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~ (1 << 0));}while(0)
#define GPIOG_REG_RESET()				do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~ (1 << 0));}while(0)
#define GPIOH_REG_RESET()				do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~ (1 << 0));}while(0)
#define GPIOI_REG_RESET()				do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~ (1 << 0));}while(0)

/*
 * macros to reset SPI peripheral registers in one short
 */

#define SPI1_REG_RESET()				do{ (RCC->APB2RSTR |= (1 << 12)); (RCC->APB1RSTR &= ~(1 << 12));}while(0)
#define SPI2_REG_RESET()				do{ (RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14));}while(0)
#define SPI3_REG_RESET()				do{ (RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15));}while(0)

/*
 * macros to reset I2C peripheral registers in one short
 */

#define I2C1_REG_RESET()				do{ (RCC->APB1RSTR |= (1 << 21)); (RCC->APB1RSTR &= ~(1 << 21));}while(0)
#define I2C2_REG_RESET()				do{ (RCC->APB1RSTR |= (1 << 22)); (RCC->APB1RSTR &= ~(1 << 22));}while(0)
#define I2C3_REG_RESET()				do{ (RCC->APB1RSTR |= (1 << 23)); (RCC->APB1RSTR &= ~(1 << 23));}while(0)


/*
 * macros to reset USARTx peripheral registers in one short
 */

#define USART1_REG_RESET()				do{ (RCC->APB1RSTR |= (1 << 4)); (RCC->APB1RSTR &= ~(1 << 4));}while(0)
#define USART2_REG_RESET()				do{ (RCC->APB1RSTR |= (1 << 17)); (RCC->APB1RSTR &= ~(1 << 17));}while(0)
#define USART3_REG_RESET()				do{ (RCC->APB1RSTR |= (1 << 18)); (RCC->APB1RSTR &= ~(1 << 18));}while(0)
#define UART4_REG_RESET()				do{ (RCC->APB1RSTR |= (1 << 19)); (RCC->APB1RSTR &= ~(1 << 19));}while(0)
#define UART5_REG_RESET()				do{ (RCC->APB1RSTR |= (1 << 20)); (RCC->APB1RSTR &= ~(1 << 20));}while(0)
#define USART6_REG_RESET()				do{ (RCC->APB1RSTR |= (1 << 5)); (RCC->APB1RSTR &= ~(1 << 5));}while(0)




/*
 * GPIO peripheral base address to port code
 */

#define GPIO_BASEADDR_TO_CODE(x)		((x == GPIOA) ? 0:\
										 (x == GPIOB) ? 1:\
										 (x == GPIOC) ? 2:\
										 (x == GPIOD) ? 3:\
										 (x == GPIOE) ? 4:\
										 (x == GPIOF) ? 5:\
										 (x == GPIOG) ? 6:\
										 (x == GPIOH) ? 7:0)

/*******************************************************************************************************
 ** Bit definition macros for spi peripheral configuration registers
 ******************************************************************************************************/

/*
 * bit position definition of SPI_CR1 register
 */

#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR			3
#define SPI_CR1_SPE			6
#define SPI_CR1_LSBFIRST	7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RXONLY		10
#define SPI_CR1_DFF			11
#define SPI_CR1_CRCNEXT		12
#define SPI_CR1_CRCEN		13
#define SPI_CR1_BIDIOE		14
#define SPI_CR1_BIDIMODE	15

/*
 * Bit position definition of SPI_CR2
 */

#define SPI_CR2_RXDMAEN		0
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_SSOE		2
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE		7

/*
 * bit position definition of SPI_SR register
 */

#define SPI_SR_RXNE			0
#define SPI_SR_TXE			1
#define SPI_SR_CHSIDE		2
#define SPI_SR_UDR			3
#define SPI_SR_CRCERR		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR			6
#define SPI_SR_BSY			7
#define SPI_SR_FRE			8


/***************************************************************************************************
 * 							BIT positions definitions of I2C peripheral I2C_CR1
 * *************************************************************************************************/

#define I2C_CR1_PE			0
#define I2C_CR1_NOSTRETCH	7
#define I2C_CR1_START		8
#define I2C_CR1_STOP		9
#define I2C_CR1_ACK			10
#define I2C_CR1_SWRST		15

/***************************************************************************************************
 * 							BIT positions definitions of I2C peripheral I2C_CR2
 * *************************************************************************************************/

#define I2C_CR2_FREQ				 	0
#define I2C_CR2_ITERREN				 	8
#define I2C_CR2_ITEVTEN				 	9
#define I2C_CR2_ITBUFEN 			    10

/***************************************************************************************************
 * 					BIT positions definitions of I2C peripheral I2C_OAR1
 * *************************************************************************************************/

#define I2C_OAR1_ADD0    				 0
#define I2C_OAR1_ADD71 				 	 1
#define I2C_OAR1_ADD98  			 	 8
#define I2C_OAR1_ADDMODE   			 	 15

/***************************************************************************************************
 * 								BIT positions definitions of I2C peripheral I2C_SR1
 * *************************************************************************************************/

#define I2C_SR1_SB 					 	0
#define I2C_SR1_ADDR 				 	1
#define I2C_SR1_BTF 					2
#define I2C_SR1_ADD10 					3
#define I2C_SR1_STOPF 					4
#define I2C_SR1_RXNE 					6
#define I2C_SR1_TXE 					7
#define I2C_SR1_BERR 					8
#define I2C_SR1_ARLO 					9
#define I2C_SR1_AF 					 	10
#define I2C_SR1_OVR 					11
#define I2C_SR1_PECERR					12
#define I2C_SR1_TIMEOUT 				14
#define I2C_SR1_SMBALERT				15



/***************************************************************************************************
 * 						BIT positions definitions of I2C peripheral I2C_SR2
 * *************************************************************************************************/

#define I2C_SR2_MSL						0
#define I2C_SR2_BUSY 					1
#define I2C_SR2_TRA 					2
#define I2C_SR2_GENCALL 				4
#define I2C_SR2_DUALF 					7

/***************************************************************************************************
 * 						BIT positions definitions of I2C peripheral I2C_CCR
 * *************************************************************************************************/

#define I2C_CCR_CCR 					 0
#define I2C_CCR_DUTY 					 14
#define I2C_CCR_FS  				 	 15

/***************************************************************************************************
 * 										BIT positions definitions of USART_CR1
 * *************************************************************************************************/
#define USART_CR1_SBK					0
#define USART_CR1_RWU 					1
#define USART_CR1_RE  					2
#define USART_CR1_TE 					3
#define USART_CR1_IDLEIE 				4
#define USART_CR1_RXNEIE  				5
#define USART_CR1_TCEIE					6
#define USART_CR1_TXEIE					7
#define USART_CR1_PEIE 					8
#define USART_CR1_PS 					9
#define USART_CR1_PCE 					10
#define USART_CR1_WAKE  				11
#define USART_CR1_M 					12
#define USART_CR1_UE 					13
#define USART_CR1_OVER8  				15

/***************************************************************************************************
 * 							BIT positions definitions of USART_CR2
 * *************************************************************************************************/

#define USART_CR2_ADD   				0
#define USART_CR2_LBDL   				5
#define USART_CR2_LBDIE  				6
#define USART_CR2_LBCL   				8
#define USART_CR2_CPHA   				9
#define USART_CR2_CPOL   				10
#define USART_CR2_STOP   				12
#define USART_CR2_LINEN   				14

/***************************************************************************************************
 * 							BIT positions definitions of USART_CR3
 ***************************************************************************************************/

#define USART_CR3_EIE   				0
#define USART_CR3_IREN   				1
#define USART_CR3_IRLP  				2
#define USART_CR3_HDSEL   				3
#define USART_CR3_NACK   				4
#define USART_CR3_SCEN   				5
#define USART_CR3_DMAR  				6
#define USART_CR3_DMAT   				7
#define USART_CR3_RTSE   				8
#define USART_CR3_CTSE   				9
#define USART_CR3_CTSIE   				10
#define USART_CR3_ONEBIT   				11

/***************************************************************************************************
 * 					BIT positions definitions of I2C peripheral USART_SR
***************************************************************************************************/

#define USART_SR_PE 					0
#define USART_SR_FE        				1
#define USART_SR_NE        				2
#define USART_SR_ORE       				3
#define USART_SR_IDLE       			4
#define USART_SR_RXNE        			5
#define USART_SR_TC        				6
#define USART_SR_TXE        			7
#define USART_SR_LBD        			8
#define USART_SR_CTS        			9


/*
 * IRQ (Interrupt Request Numbers) of STM32F411RE Nucleo board MCU for the GPIO peripherals
 * NOTE: this information is specific to mcu family same for arm cortex m4 mcu family
 */

#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI5_10		40

#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2			36
#define IRQ_NO_SPI3			51
#define IRQ_NO_I2C1_EV		31
#define IRQ_NO_I2C1_ER		32
#define IRQ_NO_I2C2_EV		33
#define IRQ_NO_I2C2_ER		34
#define IRQ_NO_I2C3_EV		72
#define IRQ_NO_I2C3_ER		73
#define IRQ_NO_USART1		37
#define IRQ_NO_USART2		38
#define IRQ_NO_USART3		39
#define IRQ_NO_UART4		52
#define IRQ_NO_UART5		53
#define IRQ_NO_USART6		71





/*
 * Some generic macros
 */

#define ENABLE 			1
#define DISABLE			0
#define SET				ENABLE
#define RESET			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET
#define FLAG_SET		SET
#define FLAG_RESET		RESET



#include "stm32f411xx_gpio_driver.h"
#include "stm32f411xx_spi_driver.h"
#include "stm32f411xx_i2c_driver.h"
#include <stm32f411xx_usart_driver.h>

#endif /* INC_STM32F411XX_H_ */
