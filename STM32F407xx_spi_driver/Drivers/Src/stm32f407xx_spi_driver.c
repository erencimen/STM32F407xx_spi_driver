/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Dec 23, 2019
 *      Author: erenc
 */

#include "stm32f407xx_spi_driver.h"


static void SPI_TXE_Interrupt_Handle(SPI_Handle_t *pSPIHandle);
static void SPI_RXNE_Interrupt_Handle(SPI_Handle_t *pSPIHandle);
static void SPI_OVR_Error_Interrupt_Handle(SPI_Handle_t *pSPIHandle);

/*
 * SPI Peripheral Clock Setup
 */

/*************************************************************************************
 * @fn				- SPI_PCLK_Control
 *
 * @brief			- This function enables or disables peripheral clock for the given SPI
 *
 * @param[in]		- Base address of the SPI peripheral
 * @param[in]		- ENABLE or DISABLE Macros
 * @param[in]		-
 *
 * @return			- None
 *
 * @Note			- None
 */
void SPI_PCLK_Control(SPI_TypeDef *pSPIx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
	}
	else
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
	}
}

/*
 * SPI Peripheral Init and De-init
 */

/*************************************************************************************
 * @fn				- SPI_Init
 *
 * @brief			- This function configure the SPI registers
 *
 * @param[in]		- SPI Handle structure pointer
 * @param[in]		-
 * @param[in]		-
 *
 * @return			- None
 *
 * @Note			- None
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{

	uint32_t temp = 0;

	//Enable the peripheral clock

	SPI_PCLK_Control(pSPIHandle->pSPIx, ENABLE);

	//SPI Mode configuration

	temp |= pSPIHandle->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR;

	//SPI BUS configuration
	if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CNFG_FD)
	{
		temp &= ~(0x1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CNFG_HD)
	{
		temp |= (0x1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CNFG_SIMPLEX_RXONLY)
	{
		temp &= ~(0x1 << SPI_CR1_BIDIMODE);
		temp |= (0x1 << SPI_CR1_RXONLY);
	}

	//SPI Serial Clock Speed Configuration

	temp |= (pSPIHandle->SPI_Config.SPI_SclkSpeed << SPI_CR1_BR);

	//SPI Data Frame Format Configuration

	temp |= (pSPIHandle->SPI_Config.SPI_DFF << SPI_CR1_DFF);

	//SPI Serial Clock Polarity configuration

	temp |= (pSPIHandle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL);

	//SPI Serial Clock Phase configuration

	temp |= (pSPIHandle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA);

	//SPI Software Slave Select Management configuration

	temp |= (pSPIHandle->SPI_Config.SPI_SSM << SPI_CR1_SSM);

	//SPI Control Register 1 configuration
	pSPIHandle->pSPIx->SPI_CR1 = temp;
}

/*************************************************************************************
 * @fn				- SPI_DeInit
 *
 * @brief			- De-initializes the SPIx peripheral registers to their default reset values.
 *
 * @param[in]		- Base address of the SPIx
 * @param[in]		-
 * @param[in]		-
 *
 * @return			- None
 *
 * @Note			- None
 */
void SPI_DeInit(SPI_TypeDef *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}
	if(pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}
	if(pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}
}

/*
 * SPI Peripheral Data Send
 */

/*************************************************************************************
 * @fn				- SPI_SendData
 *
 * @brief			- This function send data
 *
 * @param[in]		- Base address of the SPIx
 * @param[in]		- Data pointer which will be send
 * @param[in]		- Length of the data will be send
 *
 * @return			- None
 *
 * @Note			- This is Blocking Call
 */

void SPI_SendData(SPI_TypeDef *pSPIx, uint8_t *pTXBuffer, uint32_t Length)
{
	while(Length > 0)
	{
		//Wait until TXE is set (TXE will be set when TX buffer is empty)
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		if(pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF))
		{
			//16 Bit DFF
			pSPIx->SPI_DR = *((uint16_t*)pTXBuffer);
			Length --;
			Length --;
			(uint16_t*)pTXBuffer++;
		}
		else
		{
			//8 Bit DFF
			pSPIx->SPI_DR = *(pTXBuffer);
			Length--;
			pTXBuffer++;
		}
	}
}

/*************************************************************************************
 * @fn				- SPI_SendDataIT
 *
 * @brief			- This function send data
 *
 * @param[in]		- SPI Handle structure pointer
 * @param[in]		- Data pointer which will be send
 * @param[in]		- Length of the data will be send
 *
 * @return			- None
 *
 * @Note			- This is NonBlocking Call
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTXBuffer, uint32_t Length)
{
	uint8_t state = pSPIHandle->TXState;
	if(state != SPI_BUSY_IN_TX)
	{
		//Save the TX buffer address and Length
			pSPIHandle->pTXBuffer = pTXBuffer;
			pSPIHandle->TXLength = Length;

			//Mark the SPI state busy
			pSPIHandle->TXState = SPI_BUSY_IN_TX;

			//Enable the TXEIE
			pSPIHandle->pSPIx->SPI_CR2 |= (0x1 << SPI_CR2_TXEIE);
	}

	return state;
}


/*
 * SPI Peripheral Data Receive
 */

/*************************************************************************************
 * @fn				- SPI_ReceiveData
 *
 * @brief			- This function receive data
 *
 * @param[in]		- Base address of the SPIx
 * @param[in]		- Data pointer which will be receive
 * @param[in]		- Length of the data will be receive
 *
 * @return			- None
 *
 * @Note			- This is Blocking Call
 */

void SPI_ReceiveData(SPI_TypeDef *pSPIx, uint8_t *pRXBuffer, uint32_t Length)
{
	while(Length > 0)
	{
		while(SPI_GetFlagStatus(pSPIx, SPI_RXE_FLAG) == FLAG_RESET);

		if(pSPIx->SPI_CR1 & (0x1 << SPI_CR1_DFF))
		{
			*(uint16_t*)pRXBuffer = pSPIx->SPI_DR;
			Length --;
			Length --;
			(uint16_t*)pRXBuffer ++;
		}
		else
		{
			*pRXBuffer = (uint8_t)pSPIx->SPI_DR;
			Length --;
			pRXBuffer ++;
		}
	}
}

/*************************************************************************************
 * @fn				- SPI_ReceiveDataIT
 *
 * @brief			- This function receive data
 *
 * @param[in]		- SPI Handle structure pointer
 * @param[in]		- Data pointer which will be receive
 * @param[in]		- Length of the data will be receive
 *
 * @return			- None
 *
 * @Note			- This is Blocking Call
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRXBuffer, uint32_t Length)
{
	uint8_t state = pSPIHandle->RXState;
	if(state != SPI_BUSY_IN_RX)
	{
		//Save the TX buffer address and Length
			pSPIHandle->pRXBuffer = pRXBuffer;
			pSPIHandle->RXLength = Length;

			//Mark the SPI state busy
			pSPIHandle->RXState = SPI_BUSY_IN_RX;

			//Enable the TXEIE
			pSPIHandle->pSPIx->SPI_CR2 |= (0x1 << SPI_CR2_RXNEIE);
	}

	return state;
}
/*
 * IRQ Configuration and ISR handling
 */

/*************************************************************************************
 * @fn				- SPI_IRQITConfig
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
void SPI_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
 * @fn				- SPI_IRQPriorityConfig
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
void SPI_IRQPriorityConfig(uint8_t IRQNumber ,uint8_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_IPR0_BASE + iprx) = (IRQPriority << shift_amount);
}

/*************************************************************************************
 * @fn				- SPI_IRQ_Handling
 *
 * @brief			- This function check the which event triggered interrupt
 *
 * @param[in]		- SPI Handle structure pointer
 * @param[in]		-
 * @param[in]		-
 *
 * @return			- None
 *
 * @Note			- None
 */
void SPI_IRQ_Handling(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp1, temp2;

	//Check TXE
	temp1 = pSPIHandle->pSPIx->SPI_SR << SPI_SR_TXE;
	temp2 = pSPIHandle->pSPIx->SPI_CR2 << SPI_CR2_TXEIE;

	if(temp1 && temp2)
	{
		//Handle TXE
		SPI_TXE_Interrupt_Handle(pSPIHandle);
	}

	//Check RXNE
	temp1 = pSPIHandle->pSPIx->SPI_SR << SPI_SR_RXNE;
	temp2 = pSPIHandle->pSPIx->SPI_CR2 << SPI_CR2_RXNEIE;

	if(temp1 && temp2)
	{
		//Handle RXNE
		SPI_RXNE_Interrupt_Handle(pSPIHandle);
	}

	//Check OVR
	temp1 = pSPIHandle->pSPIx->SPI_SR << SPI_SR_OVR;
	temp2 = pSPIHandle->pSPIx->SPI_CR2 << SPI_CR2_ERRIE;

	if(temp1 && temp2)
	{
		//Handle OVR
		SPI_OVR_Error_Interrupt_Handle(pSPIHandle);
	}
}

/*************************************************************************************
 * @fn				- SPI_PeripheralControl
 *
 * @brief			- This function used for Enable or Disable the SPI Peripheral
 *
 * @param[in]		- Base address of the SPIx
 * @param[in]		- ENABLE or DISABLE
 * @param[in]		-
 *
 * @return			- None
 *
 * @Note			- None
 */
void SPI_PeripheralControl(SPI_TypeDef *pSPIx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->SPI_CR1 |= (0X1 << SPI_CR1_SPE);
	}
	else
	{
		pSPIx->SPI_CR1 &= ~(0X1 << SPI_CR1_SPE);
	}
}

/*************************************************************************************
 * @fn				- SPI_SSI_Config
 *
 * @brief			- This function used for Enable or Disable the SSI
 *
 * @param[in]		- Base address of the SPIx
 * @param[in]		- ENABLE or DISABLE
 * @param[in]		-
 *
 * @return			- None
 *
 * @Note			- None
 */
void SPI_SSI_Config(SPI_TypeDef *pSPIx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			pSPIx->SPI_CR1 |= (0X1 << SPI_CR1_SSI);
		}
		else
		{
			pSPIx->SPI_CR1 &= ~(0X1 << SPI_CR1_SSI);
		}
}

/*************************************************************************************
 * @fn				- SPI_SSOE_Config
 *
 * @brief			- This function used for Enable or Disable the SSI
 *
 * @param[in]		- Base address of the SPIx
 * @param[in]		- ENABLE or DISABLE
 * @param[in]		-
 *
 * @return			- None
 *
 * @Note			- None
 */
void SPI_SSOE_Config(SPI_TypeDef *pSPIx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->SPI_CR2 |= (0X1 << SPI_CR2_SSOE);
	}
	else
	{
		pSPIx->SPI_CR2 &= ~(0X1 << SPI_CR2_SSOE);
	}
}
/*************************************************************************************
 * @fn				- SPI_GetFlagStatus
 *
 * @brief			- This function check the SPI_SR
 *
 * @param[in]		- Base address of the SPIx
 * @param[in]		- Flag name which will be check
 * @param[in]		-
 *
 * @return			- This function returns the flag status
 *
 * @Note			- None
 */

uint8_t SPI_GetFlagStatus(SPI_TypeDef *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SPI_SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*************************************************************************************
 * @fn				- SPI_CloseTransmission
 *
 * @brief			- This function ends the transmission
 *
 * @param[in]		- SPI Handle structure pointer
 * @param[in]		-
 * @param[in]		-
 *
 * @return			- None
 *
 * @Note			- None
 */
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->TXState = SPI_READY;
	pSPIHandle->TXLength = 0;
	pSPIHandle->pTXBuffer = NULL;
}

/*************************************************************************************
 * @fn				- SPI_CloseReception
 *
 * @brief			- This function ends the reception
 *
 * @param[in]		- SPI Handle structure pointer
 * @param[in]		-
 * @param[in]		-
 *
 * @return			- None
 *
 * @Note			- None
 */

void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->RXLength = 0;
	pSPIHandle->pRXBuffer = NULL;
	pSPIHandle->RXState = SPI_READY;
}

/*************************************************************************************
 * @fn				- SPI_ClearOVRError
 *
 * @brief			- This function clear OVR error
 *
 * @param[in]		- Base address of the SPIx
 * @param[in]		-
 * @param[in]		-
 *
 * @return			- None
 *
 * @Note			- None
 */
void SPI_ClearOVRError(SPI_TypeDef *pSPIx)
{
	uint8_t temp;

	temp = pSPIx->SPI_DR;
	temp = pSPIx->SPI_SR;

	(void)temp;
}

/*
 * Some helper functions implementations
 */

/*************************************************************************************
 * @fn				- SPI_TXE_Interrupt_Handle
 *
 * @brief			- This is TXE interrupt handler
 *
 * @param[in]		- SPI Handle structure pointer
 * @param[in]		-
 * @param[in]		-
 *
 * @return			- None
 *
 * @Note			- This is a private function
 */
static void SPI_TXE_Interrupt_Handle(SPI_Handle_t *pSPIHandle)
{
	if(pSPIHandle->pSPIx->SPI_CR1 & (0x1 << SPI_CR1_DFF))
	{
		pSPIHandle->pSPIx->SPI_DR =*((uint16_t*)pSPIHandle->pTXBuffer);
		pSPIHandle->TXLength --;
		pSPIHandle->TXLength --;
		(uint16_t*)pSPIHandle->pTXBuffer ++;
	}
	else
	{
		pSPIHandle->pSPIx->SPI_DR =*(pSPIHandle->pTXBuffer);
		pSPIHandle->TXLength --;
		pSPIHandle->pTXBuffer ++;
	}
	if(! pSPIHandle->TXLength)
	{
		SPI_CloseTransmission(pSPIHandle);

		SPI_AplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);
	}
}

/*************************************************************************************
 * @fn				- SPI_RXE_Interrupt_Handle
 *
 * @brief			- This is RXNE interrupt handler
 *
 * @param[in]		- SPI Handle structure pointer
 * @param[in]		-
 * @param[in]		-
 *
 * @return			- None
 *
 * @Note			- This is a private function
 */
static void SPI_RXNE_Interrupt_Handle(SPI_Handle_t *pSPIHandle)
{
	if(pSPIHandle->pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF))
	{
		*((uint16_t*)pSPIHandle->pRXBuffer) = pSPIHandle->pSPIx->SPI_DR;
		pSPIHandle->RXLength --;
		pSPIHandle->RXLength --;
		(uint16_t*)pSPIHandle->pRXBuffer ++;
	}
	else
	{
		*(pSPIHandle->pRXBuffer) = pSPIHandle->pSPIx->SPI_DR;
		pSPIHandle->RXLength --;
		pSPIHandle->pRXBuffer ++;
	}
	if(! pSPIHandle->RXLength)
	{
		SPI_CloseReception(pSPIHandle);

		SPI_AplicationEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);
	}
}

/*************************************************************************************
 * @fn				- SPI_OVR_Error_Interrupt_Handle
 *
 * @brief			- This function clear OVR error
 *
 * @param[in]		- SPI Handle structure pointer
 * @param[in]		-
 * @param[in]		-
 *
 * @return			- None
 *
 * @Note			- This is a private function
 */

static void SPI_OVR_Error_Interrupt_Handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;

	if(pSPIHandle->TXState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->SPI_DR;
		temp = pSPIHandle->pSPIx->SPI_SR;
	}
	(void)temp;

	SPI_AplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);
}


__weak void SPI_AplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEvent)
{
	//This is a weak function. The application may override this function.
}
