/*
 * stm32f407xx.h
 *
 *  Created on: Dec 12, 2019
 *      Author: Eren
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>
#include <stddef.h>

#define __vo volatile
#define __weak __attribute__((weak))

/*
 * ARM Cortex M4 Processor NVIC ISERx Registers Addresses
 */

#define NVIC_ISER0						((__vo uint32_t*) 0xE000E100)
#define NVIC_ISER1						((__vo uint32_t*) 0xE000E104)
#define NVIC_ISER2						((__vo uint32_t*) 0xE000E108)
#define NVIC_ISER3						((__vo uint32_t*) 0xE000E10C)
#define NVIC_ISER4						((__vo uint32_t*) 0xE000E110)
#define NVIC_ISER5						((__vo uint32_t*) 0xE000E114)
#define NVIC_ISER6						((__vo uint32_t*) 0xE000E118)
#define NVIC_ISER7						((__vo uint32_t*) 0xE000E11C)

/*
 * ARM Cortex M4 Processor NVIC ICERx Registers Addresses
 */

#define NVIC_ICER0						((__vo uint32_t*) 0XE000E180)
#define NVIC_ICER1						((__vo uint32_t*) 0XE000E184)
#define NVIC_ICER2						((__vo uint32_t*) 0XE000E188)
#define NVIC_ICER3						((__vo uint32_t*) 0XE000E18C)
#define NVIC_ICER4						((__vo uint32_t*) 0XE000E190)
#define NVIC_ICER5						((__vo uint32_t*) 0XE000E194)
#define NVIC_ICER6						((__vo uint32_t*) 0XE000E198)
#define NVIC_ICER7						((__vo uint32_t*) 0XE000E19C)

/*
 * ARM Cortex M4 Processor NVIC IPR Register Base Address
 */

#define NVIC_IPR0_BASE					((__vo uint32_t *)0xE000E400)

/*
 * ARM Cortex M4 Processor number of priority bits implemented in Priority Register
 */

#define NO_PR_BITS_IMPLEMENTED			4

/*
 *  Base addresses of Flash and SRAM memories
 */

#define FLASH_BASE						0x08000000UL
#define SRAM1_BASE						0x20000000UL
#define SRAM2_BASE						0x2001C000UL
#define ROM_BASE						0x1FFF0000UL
#define SRAM 							SRAM1_BASE

/*
 * Peripheral Memory Map
 */

#define PERIPH_BASE						0X40000000UL
#define APB1PERIPH_BASE					PERIPH_BASE
#define APB2PERIPH_BASE					(PERIPH_BASE + 0X00010000UL)
#define AHB1PERIPH_BASE					(PERIPH_BASE + 0X00020000UL)
#define AHB2PERIPH_BASE					(PERIPH_BASE + 0X10000000UL)



/*
 * Base addresses of AHB1 bus Peripherals
 */

#define GPIOA_BASE						(AHB1PERIPH_BASE + 0X0000UL)
#define GPIOB_BASE						(AHB1PERIPH_BASE + 0X0400UL)
#define GPIOC_BASE						(AHB1PERIPH_BASE + 0X0800UL)
#define GPIOD_BASE						(AHB1PERIPH_BASE + 0X0C00UL)
#define GPIOE_BASE						(AHB1PERIPH_BASE + 0X1000UL)
#define GPIOF_BASE						(AHB1PERIPH_BASE + 0X1400UL)
#define GPIOG_BASE						(AHB1PERIPH_BASE + 0X1800UL)
#define GPIOH_BASE						(AHB1PERIPH_BASE + 0X1C00UL)
#define GPIOI_BASE						(AHB1PERIPH_BASE + 0X2000UL)
#define RCC_BASE						(AHB1PERIPH_BASE + 0X3800UL)

/*
 * Base addresses of APB1 bus Peripherals
 */

#define SPI2_BASE             			(APB1PERIPH_BASE + 0x3800UL)
#define SPI3_BASE            			(APB1PERIPH_BASE + 0x3C00UL)
#define USART2_BASE           			(APB1PERIPH_BASE + 0x4400UL)
#define USART3_BASE           			(APB1PERIPH_BASE + 0x4800UL)
#define UART4_BASE            			(APB1PERIPH_BASE + 0x4C00UL)
#define UART5_BASE           			(APB1PERIPH_BASE + 0x5000UL)
#define I2C1_BASE             			(APB1PERIPH_BASE + 0x5400UL)
#define I2C2_BASE             			(APB1PERIPH_BASE + 0x5800UL)
#define I2C3_BASE             			(APB1PERIPH_BASE + 0x5C00UL)

/*
 * Base addresses of APB2 bus Peripherals
 */

#define USART1_BASE           			(APB2PERIPH_BASE + 0x1000UL)
#define USART6_BASE           			(APB2PERIPH_BASE + 0x1400UL)
#define SPI1_BASE             			(APB2PERIPH_BASE + 0x3000UL)
#define SYSCFG_BASE           			(APB2PERIPH_BASE + 0x3800UL)
#define EXTI_BASE             			(APB2PERIPH_BASE + 0x3C00UL)


/*
 * Peripheral registers structures
 */


/*
 * GPIO
 */

typedef struct
{
	__vo uint32_t MODER;    /*!< GPIO port mode register,               Address offset: 0x00      */
	__vo uint32_t OTYPER;   /*!< GPIO port output type register,        Address offset: 0x04      */
	__vo uint32_t OSPEEDR;  /*!< GPIO port output speed register,       Address offset: 0x08      */
	__vo uint32_t PUPDR;    /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
	__vo uint32_t IDR;      /*!< GPIO port input data register,         Address offset: 0x10      */
	__vo uint32_t ODR;      /*!< GPIO port output data register,        Address offset: 0x14      */
	__vo uint32_t BSRR;     /*!< GPIO port bit set/reset register,      Address offset: 0x18      */
	__vo uint32_t LCKR;     /*!< GPIO port configuration lock register, Address offset: 0x1C      */
	__vo uint32_t AFR[2];   /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */

}GPIO_TypeDef;

/*
 * SPI
 */
typedef struct
{
	__vo uint32_t SPI_CR1; 			/*<	SPI control register 1,			Address offset: 0x00 		*/
	__vo uint32_t SPI_CR2;			/*<	SPI control register 2,			Address offset: 0x04 		*/
	__vo uint32_t SPI_SR;			/*<	SPI status register,			Address offset: 0x08 		*/
	__vo uint32_t SPI_DR;			/*<	SPI data register,				Address offset: 0x0C 		*/
	__vo uint32_t SPI_CRCPR;		/*<	SPI CRC polynomial register,	Address offset: 0x10 		*/
	__vo uint32_t SPI_RXCRCR;		/*<	SPI RX CRC register,			Address offset: 0x14 		*/
	__vo uint32_t SPI_TXCRCR;		/*<	SPI TX CRC register,			Address offset: 0x18 		*/
	__vo uint32_t SPI_I2SCFGR;		/*<	SPI_I2S configuration register,	Address offset: 0x00 		*/
	__vo uint32_t SPI_I2SPR;		/*<	SPI_I2S prescaler register,		Address offset: 0x00 		*/
}SPI_TypeDef;


/*
 * I2C
 */

typedef struct
{
	__vo uint32_t I2C_CR1;			/*<	I2C control register 1,			Address offset: 0x00 		*/
	__vo uint32_t I2C_CR2;			/*<	I2C control register 2,			Address offset: 0x04 		*/
	__vo uint32_t I2C_OAR1;			/*<	I2C Own address register 1,		Address offset: 0x08 		*/
	__vo uint32_t I2C_OAR2;			/*<	I2C Own address register 2,		Address offset: 0x0C 		*/
	__vo uint32_t I2C_DR;			/*<	I2C Data register,				Address offset: 0x10 		*/
	__vo uint32_t I2C_SR1;			/*<	I2C Status register 1,			Address offset: 0x14 		*/
	__vo uint32_t I2C_SR2;			/*<	I2C Status register 2,			Address offset: 0x18		*/
	__vo uint32_t I2C_CCR;			/*<	I2C Clock control register,		Address offset: 0x1C 		*/
	__vo uint32_t I2C_TRISE;		/*<	I2C TRISE register,				Address offset: 0x20 		*/
}I2C_TypeDef;

/*
 * RCC
 */

typedef struct
{
	__vo uint32_t CR;            /*!< RCC clock control register,                                  Address offset: 0x00 */
	__vo uint32_t PLLCFGR;       /*!< RCC PLL configuration register,                              Address offset: 0x04 */
	__vo uint32_t CFGR;          /*!< RCC clock configuration register,                            Address offset: 0x08 */
	__vo uint32_t CIR;           /*!< RCC clock interrupt register,                                Address offset: 0x0C */
	__vo uint32_t AHB1RSTR;      /*!< RCC AHB1 peripheral reset register,                          Address offset: 0x10 */
	__vo uint32_t AHB2RSTR;      /*!< RCC AHB2 peripheral reset register,                          Address offset: 0x14 */
	__vo uint32_t AHB3RSTR;      /*!< RCC AHB3 peripheral reset register,                          Address offset: 0x18 */
	uint32_t      RESERVED0;     /*!< Reserved, 0x1C                                                                    */
	__vo uint32_t APB1RSTR;      /*!< RCC APB1 peripheral reset register,                          Address offset: 0x20 */
	__vo uint32_t APB2RSTR;      /*!< RCC APB2 peripheral reset register,                          Address offset: 0x24 */
	uint32_t      RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                               */
	__vo uint32_t AHB1ENR;       /*!< RCC AHB1 peripheral clock register,                          Address offset: 0x30 */
	__vo uint32_t AHB2ENR;       /*!< RCC AHB2 peripheral clock register,                          Address offset: 0x34 */
	__vo uint32_t AHB3ENR;       /*!< RCC AHB3 peripheral clock register,                          Address offset: 0x38 */
	uint32_t      RESERVED2;     /*!< Reserved, 0x3C                                                                    */
	__vo uint32_t APB1ENR;       /*!< RCC APB1 peripheral clock enable register,                   Address offset: 0x40 */
	__vo uint32_t APB2ENR;       /*!< RCC APB2 peripheral clock enable register,                   Address offset: 0x44 */
	uint32_t      RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                               */
	__vo uint32_t AHB1LPENR;     /*!< RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x50 */
	__vo uint32_t AHB2LPENR;     /*!< RCC AHB2 peripheral clock enable in low power mode register, Address offset: 0x54 */
	__vo uint32_t AHB3LPENR;     /*!< RCC AHB3 peripheral clock enable in low power mode register, Address offset: 0x58 */
	uint32_t      RESERVED4;     /*!< Reserved, 0x5C                                                                    */
	__vo uint32_t APB1LPENR;     /*!< RCC APB1 peripheral clock enable in low power mode register, Address offset: 0x60 */
	__vo uint32_t APB2LPENR;     /*!< RCC APB2 peripheral clock enable in low power mode register, Address offset: 0x64 */
	uint32_t      RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                               */
	__vo uint32_t BDCR;          /*!< RCC Backup domain control register,                          Address offset: 0x70 */
	__vo uint32_t CSR;           /*!< RCC clock control & status register,                         Address offset: 0x74 */
	uint32_t      RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                               */
	__vo uint32_t SSCGR;         /*!< RCC spread spectrum clock generation register,               Address offset: 0x80 */
	__vo uint32_t PLLI2SCFGR;    /*!< RCC PLLI2S configuration register,                           Address offset: 0x84 */
}RCC_TypeDef;


/*
 * EXTI
 */

typedef struct
{
	__vo uint32_t IMR;			/*!< EXTI Interrupt mask register,          	 Address offset: 0x00 */
	__vo uint32_t EMR;			/*!< EXTI Event mask register,                 	 Address offset: 0x04 */
	__vo uint32_t RTSR;			/*!< EXTI Rising trigger selection register, 	 Address offset: 0x08 */
	__vo uint32_t FTSR;			/*!< EXTI Falling trigger selection register, 	 Address offset: 0x0C */
	__vo uint32_t SWIER;		/*!< EXTI Software interrupt event register,  	 Address offset: 0x10 */
	__vo uint32_t PR;			/*!< EXTI Pending register,                   	 Address offset: 0x14 */
}EXTI_TypeDef;

/*
 * SYSCFG
 */

typedef struct
{
	__vo uint32_t MEMRMP;		/*!< SYSCFG memory remap register,                   		    Address offset: 0x00      */
	__vo uint32_t PMC;			/*!< SYSCFG peripheral mode configuration register,  		    Address offset: 0x04      */
	__vo uint32_t EXTICR[4];    /*!< SYSCFG external interrupt configuration registers,			Address offset: 0x08-0x14 */
	uint32_t      RESERVED[2];  /*!< Reserved, 0x18-0x1C                                        		                  */
	__vo uint32_t CMPCR;        /*!< SYSCFG Compensation cell control register,         		Address offset: 0x20      */
}SYSCFG_TypeDef;


/*
 * Peripheral definitions	(Peripheral base addresses typecasted to xx_TypeDef)
 */


#define GPIOA			((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB			((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC			((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD			((GPIO_TypeDef *) GPIOD_BASE)
#define GPIOE			((GPIO_TypeDef *) GPIOE_BASE)
#define GPIOF			((GPIO_TypeDef *) GPIOF_BASE)
#define GPIOG			((GPIO_TypeDef *) GPIOG_BASE)
#define GPIOH			((GPIO_TypeDef *) GPIOH_BASE)
#define GPIOI			((GPIO_TypeDef *) GPIOI_BASE)
#define SPI1			((SPI_TypeDef *) SPI1_BASE)
#define SPI2			((SPI_TypeDef *) SPI2_BASE)
#define SPI3			((SPI_TypeDef *) SPI3_BASE)
#define I2C1			((I2C_TypeDef *) I2C1_BASE)
#define I2C2			((I2C_TypeDef *) I2C2_BASE)
#define I2C3			((I2C_TypeDef *) I2C3_BASE)
#define RCC				((RCC_TypeDef *) RCC_BASE)
#define EXTI			((EXTI_TypeDef *) EXTI_BASE)
#define SYSCFG			((SYSCFG_TypeDef *) SYSCFG_BASE)

/*
 * Clock enable/disable macros for GPIOx Peripheral
 */

//Enable
#define GPIOA_PCLK_EN()		(RCC->AHB1ENR |=(1 << 0))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |=(1 << 1))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |=(1 << 2))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |=(1 << 3))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |=(1 << 4))
#define GPIOF_PCLK_EN()		(RCC->AHB1ENR |=(1 << 5))
#define GPIOG_PCLK_EN()		(RCC->AHB1ENR |=(1 << 6))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |=(1 << 7))
#define GPIOI_PCLK_EN()		(RCC->AHB1ENR |=(1 << 8))

//Disable
#define GPIOA_PCLK_DI()		(RCC->AHB1ENR &=~(1 << 0))
#define GPIOB_PCLK_DI()		(RCC->AHB1ENR &=~(1 << 1))
#define GPIOC_PCLK_DI()		(RCC->AHB1ENR &=~(1 << 2))
#define GPIOD_PCLK_DI()		(RCC->AHB1ENR &=~(1 << 3))
#define GPIOE_PCLK_DI()		(RCC->AHB1ENR &=~(1 << 4))
#define GPIOF_PCLK_DI()		(RCC->AHB1ENR &=~(1 << 5))
#define GPIOG_PCLK_DI()		(RCC->AHB1ENR &=~(1 << 6))
#define GPIOH_PCLK_DI()		(RCC->AHB1ENR &=~(1 << 7))
#define GPIOI_PCLK_DI()		(RCC->AHB1ENR &=~(1 << 8))




/*
 * Clock enable/disable macros for I2Cx Peripheral
 */


//Enable
#define I2C1_PCLK_EN()			(RCC->APB1ENR |=(1 << 21))
#define I2C2_PCLK_EN()			(RCC->APB1ENR |=(1 << 22))
#define I2C3_PCLK_EN()			(RCC->APB1ENR |=(1 << 23))

//Disable
#define I2C1_PCLK_DI()			(RCC->APB1ENR &=~(1 << 21))
#define I2C2_PCLK_DI()			(RCC->APB1ENR &=~(1 << 22))
#define I2C3_PCLK_DI()			(RCC->APB1ENR &=~(1 << 23))

/*
 * Clock enable/disable macros for SPIx Peripheral
 */

//Enable
#define SPI1_PCLK_EN()			(RCC->APB2ENR |=(1 << 12))
#define SPI2_PCLK_EN()			(RCC->APB1ENR |=(1 << 14))
#define SPI3_PCLK_EN()			(RCC->APB1ENR |=(1 << 15))

//Disable
#define SPI1_PCLK_DI()			(RCC->APB2ENR &=~(1 << 12))
#define SPI2_PCLK_DI()			(RCC->APB1ENR &=~(1 << 14))
#define SPI3_PCLK_DI()			(RCC->APB1ENR &=~(1 << 15))

/*
 * Clock enable/disable macros for USARTx Peripheral
 */

//Enable
#define USART1_PCLK_EN()		(RCC->APB2ENR |=(1 << 4))
#define USART2_PCLK_EN()		(RCC->APB1ENR |=(1 << 17))
#define USART3_PCLK_EN()		(RCC->APB1ENR |=(1 << 18))
#define UART4_PCLK_EN()			(RCC->APB1ENR |=(1 << 19))
#define UART5_PCLK_EN()			(RCC->APB1ENR |=(1 << 20))
#define USART6_PCLK_EN()		(RCC->APB2ENR |=(1 << 5))

//Disable
#define USART1_PCLK_DI()		(RCC->APB2ENR &=~(1 << 4))
#define USART2_PCLK_DI()		(RCC->APB1ENR &=~(1 << 17))
#define USART3_PCLK_DI()		(RCC->APB1ENR &=~(1 << 18))
#define UART4_PCLK_DI()			(RCC->APB1ENR &=~(1 << 19))
#define UART5_PCLK_DI()			(RCC->APB1ENR &=~(1 << 20))
#define USART6_PCLK_DI()		(RCC->APB2ENR &=~(1 << 5))


/*
 * Clock enable/disable macro for SYSCFG Peripheral
 */

//Enable
#define SYSCFG_PCLK_EN()		(RCC->APB2ENR |=(1 << 14))

//Disable
#define SYSCFG_PCLK_DI()		(RCC->APB2ENR &=~(1 << 14))


/*
 * Clock reset macros for GPIOx Peripheral
 */

#define GPIOA_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 0));	(RCC->AHB1RSTR &= ~(1 << 0));}while(0)
#define GPIOB_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 1));	(RCC->AHB1RSTR &= ~(1 << 1));}while(0)
#define GPIOC_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 2));	(RCC->AHB1RSTR &= ~(1 << 2));}while(0)
#define GPIOD_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 3));	(RCC->AHB1RSTR &= ~(1 << 3));}while(0)
#define GPIOE_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 4));	(RCC->AHB1RSTR &= ~(1 << 4));}while(0)
#define GPIOF_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 5));	(RCC->AHB1RSTR &= ~(1 << 5));}while(0)
#define GPIOG_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 6));	(RCC->AHB1RSTR &= ~(1 << 6));}while(0)
#define GPIOH_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 7));	(RCC->AHB1RSTR &= ~(1 << 7));}while(0)
#define GPIOI_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 8));	(RCC->AHB1RSTR &= ~(1 << 8));}while(0)

/*
 * Clock reset macros for SPIx Peripheral
 */

#define SPI1_REG_RESET()		do{(RCC->APB2RSTR |= (1 << 12));	(RCC->APB2RSTR &= ~(1 << 12));}while(0)
#define SPI2_REG_RESET()		do{(RCC->APB1RSTR |= (1 << 14));	(RCC->APB1RSTR &= ~(1 << 14));}while(0)
#define SPI3_REG_RESET()		do{(RCC->APB1RSTR |= (1 << 15));	(RCC->APB1RSTR &= ~(1 << 15));}while(0)

/*
 * Clock reset macros for I2Cx Peripheral
 */

#define I2C1_REG_RESET()		do{(RCC->APB1RSTR |= (1 << 21));	(RCC->APB1RSTR &= ~(1 << 21));}while(0)
#define I2C2_REG_RESET()		do{(RCC->APB1RSTR |= (1 << 22));	(RCC->APB1RSTR &= ~(1 << 22));}while(0)
#define I2C3_REG_RESET()		do{(RCC->APB1RSTR |= (1 << 23));	(RCC->APB1RSTR &= ~(1 << 23));}while(0)

/*
 * Returns port codes for given GPIOx base address
 */

#define GPIO_BASE_TO_PORTCODE(x)		((x == GPIOA) ? 0 :\
										 (x == GPIOB) ? 1 :\
										 (x == GPIOC) ? 2 :\
										 (x == GPIOD) ? 3 :\
										 (x == GPIOE) ? 4 :\
										 (x == GPIOF) ? 5 :\
										 (x == GPIOG) ? 6 :\
										 (x == GPIOI) ? 7 :0)


/*
 * Interrupt Request Numbers of STM32F407xx MCU
 */

typedef enum
{
	EXTI0_IRQn                  = 6,      /*!< EXTI Line0 Interrupt                                              */
	EXTI1_IRQn                  = 7,      /*!< EXTI Line1 Interrupt                                              */
	EXTI2_IRQn                  = 8,      /*!< EXTI Line2 Interrupt                                              */
	EXTI3_IRQn                  = 9,      /*!< EXTI Line3 Interrupt                                              */
	EXTI4_IRQn                  = 10,     /*!< EXTI Line4 Interrupt                                              */
	EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                                     */
	I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt                                              */
	I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                              */
	I2C2_EV_IRQn                = 33,     /*!< I2C2 Event Interrupt                                              */
	I2C2_ER_IRQn                = 34,     /*!< I2C2 Error Interrupt                                              */
	SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                             */
	SPI2_IRQn                   = 36,     /*!< SPI2 global Interrupt                                             */
	USART1_IRQn                 = 37,     /*!< USART1 global Interrupt                                           */
	USART2_IRQn                 = 38,     /*!< USART2 global Interrupt                                           */
	USART3_IRQn                 = 39,     /*!< USART3 global Interrupt                                           */
	EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                                   */
	SPI3_IRQn                   = 51,     /*!< SPI3 global Interrupt                                             */
	UART4_IRQn                  = 52,     /*!< UART4 global Interrupt                                            */
	UART5_IRQn                  = 53,     /*!< UART5 global Interrupt                                            */
	I2C3_EV_IRQn                = 72,     /*!< I2C3 event interrupt                                              */
	I2C3_ER_IRQn                = 73,     /*!< I2C3 error interrupt                                              */
}IRQNumberDef;


/*
 *
 */

#define NVIC_IRQ_PRI0		0
#define NVIC_IRQ_PRI1		1
#define NVIC_IRQ_PRI2		2
#define NVIC_IRQ_PRI3		3
#define NVIC_IRQ_PRI4		4
#define NVIC_IRQ_PRI5		5
#define NVIC_IRQ_PRI6		6
#define NVIC_IRQ_PRI7		7
#define NVIC_IRQ_PRI8		8
#define NVIC_IRQ_PRI9		9
#define NVIC_IRQ_PRI10		10
#define NVIC_IRQ_PRI11		11
#define NVIC_IRQ_PRI12		12
#define NVIC_IRQ_PRI13		13
#define NVIC_IRQ_PRI14		14
#define NVIC_IRQ_PRI15		15

/**********************************************************************************************************
 * BIT POSITION DEFINITION MACROS FOR SPI PERIPHERAL
***********************************************************************************************************/

#define	SPI_CR1_CPHA		0
#define	SPI_CR1_CPOL		1
#define	SPI_CR1_MSTR		2
#define	SPI_CR1_BR			3
#define SPI_CR1_SPE			6
#define	SPI_CR1_LSBFIRST	7
#define	SPI_CR1_SSI			8
#define	SPI_CR1_SSM			9
#define	SPI_CR1_RXONLY		10
#define	SPI_CR1_DFF			11
#define	SPI_CR1_CRCNEXT		12
#define	SPI_CR1_CRCEN		13
#define	SPI_CR1_BIDIOE		14
#define SPI_CR1_BIDIMODE	15

#define	SPI_CR2_RXDMAEN		0
#define	SPI_CR2_TXDMAEN		1
#define	SPI_CR2_SSOE		2
#define	SPI_CR2_FRF			4
#define SPI_CR2_ERRIE		5
#define	SPI_CR2_RXNEIE		6
#define	SPI_CR2_TXEIE		7

#define	SPI_SR_RXNE			0
#define	SPI_SR_TXE			1
#define	SPI_SR_CHSIDE		2
#define	SPI_SR_UDR			3
#define	SPI_SR_CRCERR		4
#define SPI_SR_MODF			5
#define	SPI_SR_OVR			6
#define	SPI_SR_BSY			7
#define	SPI_SR_FRE			8

/**********************************************************************************************************
 * BIT POSITION DEFINITION MACROS FOR I2C PERIPHERAL
***********************************************************************************************************/
#define	I2C_CR1_PE				0
#define	I2C_CR1_SMBUS			1
#define	I2C_CR1_SMBTYPE			3
#define	I2C_CR1_ENARP			4
#define	I2C_CR1_ENPEC			5
#define	I2C_CR1_ENGC			6
#define	I2C_CR1_NOSTRETCH		7
#define	I2C_CR1_START			8
#define	I2C_CR1_STOP			9
#define	I2C_CR1_ACK				10
#define	I2C_CR1_POS				11
#define	I2C_CR1_PEC				12
#define	I2C_CR1_ALERT			13
#define	I2C_CR1_SWRST			15

#define I2C_CR2_FREQ			0
#define I2C_CR2_ITERREN			8
#define I2C_CR2_ITEVTEN			9
#define I2C_CR2_ITBUFEN			10
#define I2C_CR2_DMAEN			11
#define I2C_CR2_LAST			12

#define I2C_SR1_SB				0
#define I2C_SR1_ADDR			1
#define I2C_SR1_BTF				2
#define I2C_SR1_ADD10			3
#define I2C_SR1_STOPF			4
#define I2C_SR1_RxNE			6
#define I2C_SR1_TxE				7
#define I2C_SR1_BERR			8
#define I2C_SR1_ARLO			9
#define I2C_SR1_AF				10
#define I2C_SR1_OVR				11
#define I2C_SR1_PECERR			12
#define I2C_SR1_TIMEOUT			14
#define I2C_SR1_SMBALERT		15

#define I2C_SR2_MSL				0
#define I2C_SR2_BUSY			1
#define I2C_SR2_TRA				2
#define I2C_SR2_GENCALL			4
#define I2C_SR2_SMBDEFAULT		5
#define I2C_SR2_SMBHOST			6
#define I2C_SR2_DUALF			7
#define I2C_SR2_PEC				8

#define I2C_CCR_CCR				0
#define I2C_CCR_DUTY			14
#define I2C_CCR_FS				15

/*
 * Some Generic Macros
 */


#define ENABLE 			1
#define DISABLE			0
#define	SET				ENABLE
#define RESET			DISABLE
#define	GPIO_PIN_SET	SET
#define	GPIO_PIN_RESET	RESET
#define FLAG_RESET		RESET
#define FLAG_SET		SET



#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"

#endif /* INC_STM32F407XX_H_ */
