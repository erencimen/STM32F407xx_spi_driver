/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Dec 12, 2019
 *      Author: Eren
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

/*
 * This is Configuration Structure for GPIOx Peripheral
 */

typedef struct
{
	uint8_t GPIO_PinNumber;			// <Possible Values From @GPIO_PIN_NUMBERS	*>
	uint8_t GPIO_PinMode;			// <Possible Values From @GPIO_PIN_MODES	*>
	uint8_t GPIO_PinSpeed;			// <Possible Values From @GPIO_SPEEDS		*>
	uint8_t GPIO_PuPdControl;		// <Possible Values From @GPIO_PIN_PUPD		*>
	uint8_t	GPIO_PinOPType;			// <Possible Values From @GPIO_OP_TYPES		*>
	uint8_t	GPIO_PinAltFunMode;

}GPIO_PinConfig_t;



/*
 * This is a Handle structure for GPIO pin
 */

typedef struct
{
	GPIO_TypeDef *pGPIOx;							//*pGPIOx holds the base address of the GPIO port to which the pin belongs
	GPIO_PinConfig_t GPIO_PinConfig;


}GPIO_Handle_t;


/*
 * @GPIO_PIN_NUMBERS
 * Possible GPIO pin numbers
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
/*
 * @GPIO_PIN_MODES
 * Possible GPIO modes
 */
#define GPIO_MODE_IN 		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_AF		2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FT		4
#define GPIO_MODE_IT_RT		5
#define GPIO_MODE_IT_RFT	6

/*
 * @GPIO_OP_TYPES
 * Possible GPIO output types
 */
#define GPIO_OP_TYPE_PP		0
#define	GPIO_OP_TYPE_OD		1

/*
 * @GPIO_SPEEDS
 * Possible GPIO output speeds
 */
#define GPIO_OP_SPEED_LOW		0
#define GPIO_OP_SPEED_MEDIUM	1
#define GPIO_OP_SPEED_HIGH		2
#define GPIO_OP_SPEED_VERYHIGH	3

/*
 * @GPIO_PIN_PUPD
 * Possible GPIO Pull-up Pull-down modes
 */

#define GPIO_NO_PUPD	0
#define GPIO_PIN_PU		1
#define GPIO_PIN_PD		2


/*******************************************************************************************************************************
 * 										APIs supported by this driver
 * 							For more information about the APIs check the function definitions
 *******************************************************************************************************************************/
/*
 * GPIO Peripheral Clock Setup
 */

void GPIO_PCLK_Control(GPIO_TypeDef *pGPIOx,uint8_t EnorDi);

/*
 * GPIO Peripheral Init and De-init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_TypeDef *pGPIOx);

/*
 * GPIO Peripheral Data Read and Write
 */
uint8_t GPIO_Read_Pin(GPIO_TypeDef *pGPIOx,uint8_t GPIO_PinNumber);
uint16_t GPIO_Read_Port(GPIO_TypeDef *pGPIOx);
void GPIO_Write_Pin(GPIO_TypeDef *pGPIOx,uint8_t GPIO_PinNumber,uint8_t PinState);
void GPIO_Write_Port(GPIO_TypeDef *pGPIOx,uint16_t PortState);
void GPIO_Toggle_Pin(GPIO_TypeDef *pGPIOx,uint8_t GPIO_PinNumber);

/*
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber ,uint8_t IRQPriority);
void GPIO_IRQ_Handling(uint8_t GPIO_PinNumber);


#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
