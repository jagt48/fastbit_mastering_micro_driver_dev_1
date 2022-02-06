/*
 * stm32f407xx_spi_drv.h
 *
 *  Created on: Jan 6, 2022
 *      Author: joseph
 */

#ifndef INC_STM32F407XX_SPI_DRV_H_
#define INC_STM32F407XX_SPI_DRV_H_

#include "stm32f407xx.h"

/*
 * Configuration structure for SPIx peripheral.
 */
typedef struct
{
	uint8_t SPI_DeviceMode;		// Device mode. @SPI_DeviceMode
	uint8_t SPI_BusConfig;		// Communication bus type. @SPI_BusConfig
	uint8_t SPI_SclkSpeed;		// Serial clock speed. @SPI_SclkSpeed
	uint8_t SPI_DFF;			// Data frame format. @SPI_DFF
	uint8_t SPI_CPOL;			// Clock idle polarity. @SPI_CPOL
	uint8_t SPI_CPHA;			// Clock phase. @SPI_CPHA
	uint8_t SPI_SSM;			// Slave select. @SPI_SSM
}SPI_Config_t;

/*
 * Handle structure for SPIx peripheral.
 */
typedef struct
{
	SPI_RegDef_t *pSPIx;	// Holds base address of peripheral used. (x:0, 1, 2)
	SPI_Config_t SPIConfig;	// All peripheral configuration data set by user.
	uint8_t *pTxBuffer;		// App TX buffer address.
	uint8_t *pRxBuffer;		// App RX buffer address.
	uint32_t TxLen;			// Length of TX buffer.
	uint32_t RxLen;			// Length of RX buffer.
	uint8_t TxState;		// State of TX.	@SPI_STATES
	uint8_t RxState;		// State of RX. @SPI_STATES
}SPI_Handle_t;

/*
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_SLAVE	0		// Configure device in slave mode.
#define SPI_DEVICE_MODE_MASTER	1		// Condifure device in master mode.

/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD			0	// Full duplex
#define SPI_BUS_CONFIG_HD			1	// Half duplex
#define SPI_BUS_CONFIG_SIM_RX		2	// Simplex RX mode only

/*
 * @SPI_SclkSpeed
 */
#define SPI_SCLK_DIV2		0		// Fclk / 2
#define SPI_SCLK_DIV4		1		// Fclk / 4
#define SPI_SCLK_DIV8		2		// Fclk / 8
#define SPI_SCLK_DIV16		3		// Fclk / 16
#define SPI_SCLK_DIV32		4		// Fclk / 32
#define SPI_SCLK_DIV64		5		// Fclk / 64
#define SPI_SCLK_DIV128		6		// Fclk / 128
#define SPI_SCLK_DIV256		7		// Fclk / 256

/*
 * @SPI_DFF
 */
#define SPI_DFF_8BITS		0		// 8-bit data frame format.
#define SPI_DFF_16BITS		1		// 16-bit data frame format.

/*
 * @SPI_CPOL
 */
#define SPI_CPOL_IDLE_0		0		// Clock 0 when idle.
#define SPI_CPOL_IDLE_1		1		// Clock 1 when idle.

/*
 * @SPI_CPHA
 */
#define SPI_CPHA_FIRST		0		// Data captured on first edge.
#define SPI_CPHA_SECOND		1		// Data captured on second edge.

/*
 * @SPI_SSM
 */
#define SPI_SSM_DIS			0		// Slave selection managed by hardware.
#define SPI_SSM_EN			1		// Slave selection managed by software.

/*
 * @SPI_STATES
 */
#define SPI_READY			0		// SPI peripheral is ready for TX/RX.
#define SPI_BUSY_IN_RX		1		// SPI peripheral is busy performing a TX.
#define SPI_BUSY_IN_TX		2		// SPI peripheral is busy performing an RX.

/*
 * @SPI_APP_EVNTS
 */
#define SPI_EVENT_TX_COMPLETE 	1	/**<SPI TX complete*/
#define SPI_EVENT_RX_COMPLETE 	2	/**<SPI RX complete*/
#define SPI_EVENT_ERR_OVR		3	/**<SPI overrun error*/
#define SPI_EVENT_ERR_CRC		4	/**<SPI CRC error*/



/********************************************************************
 * SPIx API
 *******************************************************************/

/*
 * Flags
 */
#define SPI_RXNE_FLAG 	(1 << SPI_SR_RXNE)	// Receive buffer not empty.
#define SPI_TXE_FLAG 	(1 << SPI_SR_TXE)	// Transmit buffer empty.
#define SPI_BUSY_FLAG	(1 << SPI_SR_BSY)	// SPI peripheral busy.

/*
 * Clock Control
 */
void SPI_ClkCtrl(SPI_RegDef_t *pSPIx, uint8_t En);

/*
 * Initialization/Deinitialization
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Data Send/Receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ Configuration and ISR Handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t En);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

/*
 * Helper Functions
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t En);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t En);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t En);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIX);
void SPI_CloseTx(SPI_Handle_t *pSPIHandle);
void SPI_CloseRx(SPI_Handle_t *pSPIHandle);

/*
 * Application Callback
 */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv);

#endif /* INC_STM32F407XX_SPI_DRV_H_ */
