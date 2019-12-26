/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: Dec 23, 2019
 *      Author: erenc
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"


/*
 * This is Configuration Structure for GPIOx Peripheral
 */

typedef struct
{
	uint8_t SPI_DeviceMode;			// <Possible Values From @SPI_DeviceMode	*>
	uint8_t SPI_BusConfig;			// <Possible Values From @SPI_BusConfig		*>
	uint8_t SPI_SclkSpeed;			// <Possible Values From @SPI_SclkSpeed		*>
	uint8_t SPI_DFF;				// <Possible Values From @SPI_DFF			*>
	uint8_t	SPI_CPOL;				// <Possible Values From @SPI_CPOH			*>
	uint8_t	SPI_CPHA;				// <Possible Values From @SPI_CPHA			*>
	uint8_t SPI_SSM;				// <Possible Values From @SPI_SSM			*>

}SPI_Config_t;



/*
 * This is a Handle structure for GPIO pin
 */

typedef struct
{
	SPI_TypeDef *pSPIx;								//*pSPIx holds the base address of the SPIx pripheral
	SPI_Config_t SPI_Config;
	uint8_t	*pTXBuffer;								//To store the app TX buffer address
	uint8_t	*pRXBuffer;								//To store the app RX buffer address
	uint32_t TXLength;								//To store TX length
	uint32_t RXLength;								//To store RX length
	uint8_t TXState;								//To store TX state
	uint8_t RXState;								//To store RX state

}SPI_Handle_t;

/*
 * @SPI_DeviceMode
 * Possible device modes
 */

#define	SPI_DEVICE_MODE_MASTER		1
#define SPI_DEVICE_MODE_SLAVE		0

/*
 * @SPI_BusConfig
 * Possible bus configurations
 */

#define SPI_BUS_CNFG_FD					1
#define	SPI_BUS_CNFG_HD					2
#define	SPI_BUS_CNFG_SIMPLEX_RXONLY		3


/*
 * @SPI_SclkSpeed
 * Possible Baud Rate configurations (BR)
 */
#define	SPI_SCLK_SPEED_DIV2		0
#define	SPI_SCLK_SPEED_DIV4		1
#define SPI_SCLK_SPEED_DIV8		2
#define	SPI_SCLK_SPEED_DIV16	3
#define	SPI_SCLK_SPEED_DIV32	4
#define	SPI_SCLK_SPEED_DIV64	5
#define	SPI_SCLK_SPEED_DIV128	6
#define	SPI_SCLK_SPEED_DIV2256	7

/*
 * @SPI_DFF
 * Possible data frame formats
 */
#define SPI_DFF_8BIT		0
#define SPI_DFF_16BIT		1

/*
 * @SPI_CPOL
 * Possible Serial Clock Polarity configurations
 */
#define SPI_CPOL_LOW		0
#define SPI_CPOL_HIGH		1

/*
 * @SPI_CPHA
 * Possible Serial Clock Phase
 */

#define	SPI_CPHA_LOW		0
#define	SPI_CPHA_HIGH		1

/*
 * @SPI_SSM
 * Possible Slave Select Management Modes
 */
#define SPI_SSM_DI		0
#define SPI_SSM_EN		1

/*
 * Possible SPI Application State
 */
#define SPI_READY		0
#define SPI_BUSY_IN_TX	1
#define SPI_BUSY_IN_RX	2

/*
 * Possible Applications Events
 */
#define SPI_EVENT_TX_CMPLT		1
#define SPI_EVENT_RX_CMPLT		2
#define SPI_EVENT_OVR_ERR		3

/*
 * SPI Related Status Flag Definitions
 */

#define SPI_TXE_FLAG 		(1 << SPI_SR_TXE)
#define SPI_RXE_FLAG		(1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG		(1 << SPI_SR_BSY)
#define SPI_CHSIDE_FLAG 	(1 << SPI_SR_CHSIDE)
#define SPI_CRCERR_FLAG 	(1 << SPI_SR_CRCERR)
#define SPI_FRE_FLAG 		(1 << SPI_SR_FRE)
#define SPI_MODF_FLAG 		(1 << SPI_SR_MODF)
#define SPI_OVR_FLAG 		(1 << SPI_SR_OVR)
#define SPI_UDR_FLAG 		(1 << SPI_SR_UDR)

/*******************************************************************************************************************************
 * 										APIs supported by this driver
 * 							For more information about the APIs check the function definitions
 *******************************************************************************************************************************/
/*
 * SPI Peripheral Clock Setup
 */

void SPI_PCLK_Control(SPI_TypeDef *pSPIx,uint8_t EnorDi);

/*
 * SPI Peripheral Init and De-init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_TypeDef *pSPIx);

/*
 * Data Send and Receive
 */

void SPI_SendData(SPI_TypeDef *pSPIx, uint8_t *pTXBuffer, uint32_t Length);
void SPI_ReceiveData(SPI_TypeDef *pSPIx, uint8_t *pRXBuffer, uint32_t Length);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTXBuffer, uint32_t Length);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRXBuffer, uint32_t Length);

/*
 * IRQ Configuration and ISR handling
 */

void SPI_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber ,uint8_t IRQPriority);
void SPI_IRQ_Handling(SPI_Handle_t *pSPIHandle);

/*
 * Application Callback
 */

void SPI_AplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEvent);

/*
 * Other Peripheral Controls
 */

void SPI_PeripheralControl(SPI_TypeDef *pSPIx,uint8_t EnorDi);
uint8_t SPI_GetFlagStatus(SPI_TypeDef *pSPIx, uint32_t FlagName);
void SPI_SSI_Config(SPI_TypeDef *pSPIx,uint8_t EnorDi);
void SPI_SSOE_Config(SPI_TypeDef *pSPIx,uint8_t EnorDi);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);
void SPI_ClearOVRError(SPI_TypeDef *pSPIx);



#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
