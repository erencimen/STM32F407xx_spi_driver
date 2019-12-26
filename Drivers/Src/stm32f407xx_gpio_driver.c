/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Dec 12, 2019
 *      Author: Eren
 */


#include "stm32f407xx_gpio_driver.h"


/*
 * GPIO Peripheral Clock Setup
 */

/*************************************************************************************
 * @fn				- GPIO_PCLK_Control
 *
 * @brief			- This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]		- Base address of the GPIO peripheral
 * @param[in]		- ENABLE or DISABLE Macros
 * @param[in]		-
 *
 * @return			- None
 *
 * @Note			- None
 */
void GPIO_PCLK_Control(GPIO_TypeDef *pGPIOx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx ==GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx ==GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx ==GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx ==GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx ==GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if(pGPIOx ==GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if(pGPIOx ==GPIOH)
		{
			GPIOH_PCLK_EN();
		}
		else if(pGPIOx ==GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx ==GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx ==GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx ==GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx ==GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if(pGPIOx ==GPIOF)
		{
			GPIOF_PCLK_DI();
		}
		else if(pGPIOx ==GPIOG)
		{
			GPIOG_PCLK_DI();
		}
		else if(pGPIOx ==GPIOH)
		{
			GPIOH_PCLK_DI();
		}
		else if(pGPIOx ==GPIOI)
		{
			GPIOI_PCLK_DI();
		}

	}
}

/*
 * GPIO Peripheral Init and De-init
 */

/*************************************************************************************
 * @fn				- GPIO_Init
 *
 * @brief			- This function configure the GPIO registers
 *
 * @param[in]		- pointer of GPIO handle structure
 * @param[in]		-
 * @param[in]		-
 *
 * @return			- None
 *
 * @Note			- None
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{

	uint32_t temp =0;

	//Enable the peripheral clock

	GPIO_PCLK_Control(pGPIOHandle->pGPIOx, ENABLE);

	//GPIO pin mode configuration
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <<(2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );	//Clearing
		pGPIOHandle->pGPIOx->MODER |= temp;		//Setting
		temp=0;
	}
	else
	{
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//Set corresponding FTSR bit
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear corresponding RTSR bit
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//Set corresponding RTSR bit
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear corresponding FTSR bit
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			//Set corresponding FTSR and RTSR bits
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		uint32_t temp1,temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;

		//Get port code for given GPIOx
		uint8_t PORTCODE =GPIO_BASE_TO_PORTCODE(pGPIOHandle->pGPIOx);

		//Enable the Clock for SYSCFG
		SYSCFG_PCLK_EN();

		//Set source input for the EXTIx
		SYSCFG->EXTICR[temp1] = PORTCODE << (temp2 * 4);

		//Enable the EXTI Interrupt delivery using IMR
		EXTI->IMR |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;


	}

	//GPIO pin output speed configuration
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed <<(2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );	//Clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;	//Setting
	temp =0;

	//GPIO pull-up pull-down configuration
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PuPdControl <<(2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );	//Clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp;		//Setting
	temp =0;

	//GPIO output type configuration
	//Output Type Mode register set only when GPIO Mode is out
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_OUT)
	{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );	//Clearing
		pGPIOHandle->pGPIOx->OTYPER |= temp;	//Setting
		temp =0;
	}

	//GPIO alternate function mode configuration
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_AF)
	{
		uint8_t temp1, temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;

		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4*temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~((0xF) << 4*temp2);
		pGPIOHandle->pGPIOx->AFR[temp1] |= temp;
		temp =0;
	}

}

/*************************************************************************************
 * @fn				- GPIO_DeInit
 *
 * @brief			- De-initializes the GPIOx peripheral registers to their default reset values.
 *
 * @param[in]		- Base address of the GPIO port
 * @param[in]		-
 * @param[in]		-
 *
 * @return			- None
 *
 * @Note			- None
 */
void GPIO_DeInit(GPIO_TypeDef *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if(pGPIOx ==GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if(pGPIOx ==GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if(pGPIOx ==GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if(pGPIOx ==GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if(pGPIOx ==GPIOF)
	{
		GPIOF_REG_RESET();
	}
	else if(pGPIOx ==GPIOG)
	{
		GPIOG_REG_RESET();
	}
	else if(pGPIOx ==GPIOH)
	{
		GPIOH_REG_RESET();
	}
	else if(pGPIOx ==GPIOI)
	{
		GPIOI_REG_RESET();
	}
}

/*
 * GPIO Peripheral Data Read and Write
 */

/*************************************************************************************
 * @fn				- GPIO_Read_Pin
 *
 * @brief			- This function reads specified input port pin
 *
 * @param[in]		- Base address of the GPIO port
 * @param[in]		- GPIO_PinNumber specifies the port bit to read
 * @param[in]		-
 *
 * @return			- The input port pin value
 *
 * @Note			- None
 */
uint8_t GPIO_Read_Pin(GPIO_TypeDef *pGPIOx,uint8_t GPIO_PinNumber)
{
	uint8_t value;
	value = (uint8_t) (pGPIOx->IDR >> GPIO_PinNumber) & (0x00000001);

	return value;
}

/*************************************************************************************
 * @fn				- GPIO_Read_Port
 *
 * @brief			- This function reads specified input port
 *
 * @param[in]		- Base address of the GPIO port
 * @param[in]		-
 * @param[in]		-
 *
 * @return			- The input port all pins value
 *
 * @Note			- None
 */
uint16_t GPIO_Read_Port(GPIO_TypeDef *pGPIOx)
{
	uint16_t value;
	value = (uint16_t) (pGPIOx->IDR);

	return value;

}

/*************************************************************************************
 * @fn				- GPIO_Write_Pin
 *
 * @brief			- This function writes specified output port pin
 *
 * @param[in]		- Base address of the GPIO port
 * @param[in]		- Specified output port pin number
 * @param[in]		- The value to be written to the selected bit
 *
 * @return			- None
 *
 * @Note			- None
 */
void GPIO_Write_Pin(GPIO_TypeDef *pGPIOx,uint8_t GPIO_PinNumber,uint8_t PinState)
{
	if(PinState == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (0x1 << GPIO_PinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~(0x1 << GPIO_PinNumber);
	}
}

/*************************************************************************************
 * @fn				- GPIO_Write_Port
 *
 * @brief			- This function writes specified output port
 *
 * @param[in]		- Base address of the GPIO port
 * @param[in]		- The value to be written to the selected port
 * @param[in]		-
 *
 * @return			- None
 *
 * @Note			- None
 */
void GPIO_Write_Port(GPIO_TypeDef *pGPIOx,uint16_t PortState)
{
	pGPIOx->ODR = PortState;

}

/*************************************************************************************
 * @fn				- GPIO_Toggle_Pin
 *
 * @brief			- This function toggle specified output port pin
 *
 * @param[in]		- Base address of the GPIO port
 * @param[in]		- Specified output port pin number
 * @param[in]		-
 *
 * @return			- None
 *
 * @Note			- None
 */
void GPIO_Toggle_Pin(GPIO_TypeDef *pGPIOx,uint8_t GPIO_PinNumber)
{
	pGPIOx->ODR ^= (0x1 << GPIO_PinNumber);
}

/*
 * IRQ Configuration and ISR handling
 */

/*************************************************************************************
 * @fn				- GPIO_IRQITConfig
 *
 * @brief			- This function used for enable or disable interrupt for given IRQ Number
 *
 * @param[in]		- IRQ Number
 * @param[in]		- Enable or Disable
 * @param[in]		-
 *
 * @return			- None
 *
 * @Note			- None
 */
void GPIO_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber>31 && IRQNumber <= 63)
		{
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber>63 && IRQNumber <96)
		{
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}
	else
	{
		if(IRQNumber <= 31)
		{
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber>31 && IRQNumber <= 63)
		{
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber>63 && IRQNumber <96)
		{
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}
}

/*************************************************************************************
 * @fn				- GPIO_IRQPriorityConfig
 *
 * @brief			- This function used for set interrupt priority for given IRQ Number
 *
 * @param[in]		- IRQ Number
 * @param[in]		- IRQ Priority wanted to set
 * @param[in]		-
 *
 * @return			- None
 *
 * @Note			- None
 */

void GPIO_IRQPriorityConfig(uint8_t IRQNumber ,uint8_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_IPR0_BASE + iprx) = (IRQPriority << shift_amount);
}
/*************************************************************************************
 * @fn				- GPIO_IRQ_Handling
 *
 * @brief			- This function clear EXTI PR Register
 *
 * @param[in]		- Pin number interrupt occur
 * @param[in]		-
 * @param[in]		-
 *
 * @return			- None
 *
 * @Note			- None
 */
void GPIO_IRQ_Handling(uint8_t GPIO_PinNumber)
{
	//Clear the EXTI PR Register corresponding to the pin number
	if(EXTI->PR & (1 << GPIO_PinNumber))
	{
		EXTI->PR |= (1 << GPIO_PinNumber); //This bit is cleared by programming it to ‘1’
	}
}
