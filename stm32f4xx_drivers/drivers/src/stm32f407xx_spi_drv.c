/*
 * stm32f407xx_spi_drv.c
 *
 *  Created on: Jan 6, 2022
 *      Author: Joseph Gillispie
 */

/**
 * @file 	stm32f407xx_spi_drv.c
 *
 * @author	Joseph Gillispie <jagt48@gmail.com>
 *
 * @brief	This driver sets up a basic I/O interface for the STM32F407xx SPI peripheral.
 */
#include "stm32f407xx_spi_drv.h"

/*******************************************************************
 * PRIVATE HELPER FUNCTIONS
 *******************************************************************/
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_interrupt_handle(SPI_Handle_t *pSPIHandle);


/*******************************************************************
 * SPI FUNCTIONS
 *******************************************************************/

/**
 * @fn				void SPI_ClkCtrl(SPI_RegDef_t *pSPIx, uint8_t En)
 *
 * @brief			Enable/disable clock for SPI peripheral.
 *
 * @param[in]		pSPIx	SPI peripheral used.
 * @param[in]		En		Enable bit.
 *
 * @return			none
 *
 * @Note
 */
void SPI_ClkCtrl(SPI_RegDef_t *pSPIx, uint8_t En)
{
	if (En == ENABLE)
	{
		if (pSPIx == SPI1)
		{
			SPI1_CLK_EN();
		} else if (pSPIx == SPI2)
		{
			SPI2_CLK_EN();
		} else if (pSPIx == SPI3)
		{
			SPI3_CLK_EN();
		}
	} else
	{
		if (pSPIx == SPI1)
		{
			SPI1_CLK_DIS();
		} else if (pSPIx == SPI2)
		{
			SPI2_CLK_DIS();
		} else if (pSPIx == SPI3)
		{
			SPI3_CLK_DIS();
		}

	}
}

/**
 * @fn				void SPI_Init(SPI_Handle_t *pSPIHandle)
 *
 * @brief			Initialize the SPI peripheral using user-defined parameters.
 *
 * @param[in]		pSPIHandle Pointer to the structure containing user-defined parameters.
 *
 * @return			none
 *
 * @Note
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	uint32_t temp = 0;

	// 1. Enable clock.
	SPI_ClkCtrl(pSPIHandle->pSPIx, ENABLE);

	//SPI_DeInit(pSPIHandle->pSPIx);

	// 2. Configure SPI_CR1.
	// a. Device mode
	temp |= (pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR);

	// b. Bus configuration
	if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		// BIDI mode should be cleared.
		temp &= ~(1 << SPI_CR1_BIDIMODE);

	} else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		// BIDI mode should be set.
		temp |= (1 << SPI_CR1_BIDIMODE);

	} else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIM_RX)
	{
		// BIDI mode should be cleared.
		temp &= ~(1 << SPI_CR1_BIDIMODE);
		// RXONLY bit should be set.
		temp |= (1 << SPI_CR1_RXONLY);
	}

	// c. SPI serial clock (baud rate).
	temp |= (pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR);

	// d. Data frame size.
	temp |= (pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF);

	// e. Clock polarity.
	temp |= (pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);

	// f. Clock phase.
	temp |= (pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA);

	// g. SSM
	temp |= (pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM);

	pSPIHandle->pSPIx->CR1 = temp;
}

/********************************************************************
 * @fun				SPI_DeInit
 *
 * @brief			Reset the specified SPIx peripheral.
 *
 * @param[in]		*pSPIx SPIx peripheral to be reset.
 *
 * @return			none
 *
 * @Note
 *******************************************************************/
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_REG_RST();
	} else if (pSPIx == SPI2)
	{
		SPI2_REG_RST();
	} else if (pSPIx == SPI3)
	{
		SPI3_REG_RST();
	}
}

/********************************************************************
 * @fun				SPI_SendData
 *
 * @brief			Send data from buffer.
 *
 * @param[in]
 * @param[in]
 *
 * @return			none
 *
 * @Note			This is a polling/blocking call.
 *******************************************************************/
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	// 1. Check if Len != 0, otherwise exit.
	while (Len > 0)
	{
		// 2. Wait until Tx buffer empty (TXE is set).
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		// 3. If DFF cleared, load DR with 8 bits and decrement length by one.
		// If DFF set, load DR with 16 bits and decrement length by two.
		if (pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			/*16-bit*/
			pSPIx->DR = *((uint16_t*)pTxBuffer);	// Cast to 16-bit to capture two bytes.
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		} else // 8-bit
		{
			/*8-bit*/
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}

}

/********************************************************************
 * @fun				SPI_SendDataIT
 *
 * @brief			Send data from buffer.
 *
 * @param[in]		*pSPIHandle	Pointer to SPI handle.
 * @param[in]		*pTxBuffer	Pointer to TX buffer.
 *
 * @return			State	Current state of TX.
 *
 * @Note			This is an interrupt-based call. This function only enables
 * 					the interrupt. The data is then ready for TX, to be
 * 					executed in the interrupt function itself.
 *
 *******************************************************************/

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t State = pSPIHandle->TxState;

	if (State != SPI_BUSY_IN_TX)
	{
		// 1. Save TX buffer address and Len into handle structure.
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;


		// 2. Mark SPI state as "busy" so that no other code can take over SPI until TX is complete.
		pSPIHandle->TxState = SPI_BUSY_IN_TX;


		// 3. Enable TXEIE control bit to get interrupt when TXE flag is set is SR.
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);

	}

	// 4. Data TX will be handled by ISR code.

	return State;

}

/********************************************************************
 * @fun				SPI_ReceiveDataIT
 *
 * @brief			Send data from buffer.
 *
 * @param[in]		*pSPIHandle	Pointer to SPI handle.
 * @param[in]		*pRxBuffer	Pointer to RX buffer.
 *
 * @return			State	Current state of RX.
 *
 * @Note			This is an interrupt-based call. This function only enables
 * 					the interrupt. The data is then ready for RX, to be
 * 					executed in the interrupt function itself.
  *******************************************************************/

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t State = pSPIHandle->RxState;

	if (State != SPI_BUSY_IN_RX)
	{
		// 1. Save RX buffer address and Len into handle structure.
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;


		// 2. Mark SPI state as "busy" so that no other code can take over SPI until RX is complete.
		pSPIHandle->RxState = SPI_BUSY_IN_RX;


		// 3. Enable TXEIE control bit to get interrupt when RXNE flag is set is SR.
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);

	}

	// 4. Data TX will be handled by ISR code.

	return State;

}


/********************************************************************
 * @fun				SPI_ReceiveData
 *
 * @brief
 *
 * @param[in]		pSPIx		Pointer to SPI peripheral to read from.
 * @param[in]		pRxBuffer	Pointer to buffer to store received data.
 * @param[in]		Len			Length of data to be received.
 *
 * @return			none
 *
 * @Note
 *******************************************************************/
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	// 1. Check if Len != 0, otherwise exit.
	while (Len > 0)
	{
		// 2. Wait until RX buffer not empty (RXNE is set).
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

		// 3. If DFF cleared, load buffer with DR contents and decrement length by one.
		// If DFF set, load DR with 16 bits and decrement length by two.
		if (pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			/*16-bit*/
			*((uint16_t*)pRxBuffer) = pSPIx->DR;	// Cast to 16-bit to capture two bytes.
			Len--;
			Len--;
			(uint16_t*)pRxBuffer++;
		} else
		{
			/*8-bit*/
			*pRxBuffer = pSPIx->DR;
			Len--;
			pRxBuffer++;
		}
	}
}

/********************************************************************
 * @fun				SPI_IRQInterruptConfig
 *
 * @brief
 *
 * @param[in]
 * @param[in]
 *
 * @return			none
 *
 * @Note
 *******************************************************************/
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t En)
{
	if (En == ENABLE)
		{
			if(IRQNumber <= 31)
			{
				// ISER0
				*NVIC_ISER0 |= (1 << IRQNumber);

			} else if(IRQNumber > 31 && IRQNumber < 64)
			{
				// ISER1
				*NVIC_ISER1 |= (1 << (IRQNumber % 32));
			}
			else if(IRQNumber >= 64 && IRQNumber < 96)
			{
				// ISER2
				*NVIC_ISER2 |= (1 << (IRQNumber % 64));
			}
		} else
		{
			if(IRQNumber <= 31)
			{
				// ISER0
				*NVIC_ICER0 |= (1 << IRQNumber);

			} else if(IRQNumber > 31 && IRQNumber < 64)
			{
				// ISER1
				*NVIC_ICER1 |= (1 << (IRQNumber % 32));
			}
			else if(IRQNumber >= 64 && IRQNumber < 96)
			{
				// ISER2
				*NVIC_ICER2 |= (1 << (IRQNumber % 64));
			}

		}
}

/********************************************************************
 * @fun				SPI_IRQPriorityConfig
 *
 * @brief
 *
 * @param[in]
 * @param[in]
 *
 * @return			none
 *
 * @Note
 *******************************************************************/
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	/*
	 * NVIC Priority register bitfields are 8 bits wide, with 4 available per register.
	 * Ex: [32...0] ->
	 * [PRI239, PRI238, PRI237, PRI236 = PRI59
	 * .
	 * .
	 * .
	 * [PRI7, PRI6, PRI5, PRI4] = PRI1
	 * [PRI3, PRI2, PRI1, PRI0] = PRI0
	 *
	 * To determine which register to use, first calculate the required Priority register
	 * using the Position as defined in the vector table of the reference manual..
	 * Ex: Position 6 -> 6/4 = 1, so Position 6 is found in PRI1.
	 *
	 * To determine which bitfield to use, calculate the modulus of the
	 * desired Position with the possible number of Positions in a register.
	 * Note that in the STM32F4xx family, only the upper nibble of the Priority regsiter
	 * is used (16 possible priority levels). This requires an additional shift by 4 bits
	 * to move the calculated priority into the correct position of the bitfeld.
	 * Ex: Position 6 -> 6%4 = 2, so Positon 6 bitfield is 8*2 = lshift by 16.
	 * Additional shift required to move priority into the correct bitfield = 4, so total
	 * lshift = 16 + 4 = 20 bits.
	 */

	// 1. Calculate IPR register and section.
	uint8_t iprx = IRQNumber / 4;			// Priority register.
	uint8_t iprx_section = IRQNumber % 4;	// Priority section within register.
	uint8_t shift;

	// Note that lower nibble not implemented in Priority register. A lshift is required.
	// Shift by amout defined by 8 bits * iprx section, then add addition shifts required
	// due to Priority register settings using the upper nibble only.
	shift = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_IPR_BASEADDR + iprx) |= (IRQPriority << shift);

}

/********************************************************************
 * @fun				SPI_IRQHandling
 *
 * @brief
 *
 * @param[in]		pSPIx
 * @param[in]		En
 *
 * @return			none
 *
 * @Note
 *******************************************************************/
void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	uint8_t Temp1;
	uint8_t Temp2;

	// Determine which event triggered ISR.
	Temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	Temp2 = pHandle->pSPIx->SR & (1 << SPI_CR2_TXEIE);

	// TXE
	if (Temp1 && Temp2)
	{
		spi_txe_interrupt_handle(pHandle);
	}


	Temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	Temp2 = pHandle->pSPIx->SR & (1 << SPI_CR2_RXNEIE);

	// RXNE
	if (Temp1 && Temp2)
	{
		spi_rxe_interrupt_handle(pHandle);
	}

	/*
	 * An overrun condition occurs if the master/slave completes
	 * the reception of the next data frame while the read
	 * operation of the previous frame from RX buffer has not completed.
	 * All newly arrived data will be lost until OVR bit is read.
	 */

	// OVR
	Temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	Temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_EERIE);
	if (Temp1 && Temp2)
	{
		spi_ovr_interrupt_handle(pHandle);
	}

	// ERROR
	// TODO: Implement MODF, CRC, etc. error checks in SPI ISR handler.
}

/********************************************************************
 * @fun				SPI_GetFlagStatus
 *
 * @brief			Get the status of an individual flag.
 *
 * @param[in]		pSPIx
 * @param[in]		En
 *
 * @return			Flag status
 *
 * @Note
 *******************************************************************/
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if (pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/********************************************************************
 * @fun				SPI_PeripheralControl
 *
 * @brief			Enable/disable the SPI peripheral.
 *
 * @param[in]		pSPIx
 * @param[in]		En
 *
 * @return			none
 *
 * @Note
 *******************************************************************/
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t En)
{
	if (En == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	} else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

/********************************************************************
 * @fun				SPI_SSIConfig
 *
 * @brief			Enable/disable the SSI bit to prevent MODF error.
 *
 * @param[in]		pSPIx
 * @param[in]		En
 *
 * @return			none
 *
 * @Note
 *******************************************************************/
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t En)
{
	if (En == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	} else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

/********************************************************************
 * @fun				SPI_SSOEConfig
 *
 * @brief			Enable/disable the SSOE bit.
 *
 * @param[in]		pSPIx
 * @param[in]		En
 *
 * @return			none
 *
 * @Note
 *******************************************************************/
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t En)
{
	if (En == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	} else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}


static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	// If DFF cleared, load DR with 8 bits and decrement length by one.
	// If DFF set, load DR with 16 bits and decrement length by two.
	if (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
	{
		/*16-bit*/
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);	// Cast to 16-bit to capture two bytes.
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	} else // 8-bit
	{
		/*8-bit*/
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);	// Cast to 16-bit to capture two bytes.
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}

	if (!pSPIHandle->TxLen)
	{
		// TxLen == 0, so close SPI TX and inform the application host.
		SPI_CloseTx(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_COMPLETE);
	}
}

static void spi_rxe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	// If DFF cleared, load DR with 8 bits and decrement length by one.
	// If DFF set, load DR with 16 bits and decrement length by two.
	if (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
	{
		/*16-bit*/
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pRxBuffer);	// Cast to 16-bit to capture two bytes.
		pSPIHandle->RxLen--;
		pSPIHandle->RxLen--;
		(uint16_t*)pSPIHandle->pRxBuffer++;
	} else // 8-bit
	{
		/*8-bit*/
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pRxBuffer);	// Cast to 16-bit to capture two bytes.
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}

	if (!pSPIHandle->RxLen)
	{
		// TxLen == 0, so close SPI TX and inform the application host.
		SPI_CloseRx(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_COMPLETE);
	}

}

static void spi_ovr_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;

	if (pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		// Clear OVR flag.
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;

	// Inform application.
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_ERR_OVR);
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIX)
{

}

void SPI_CloseTx(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);	// Disable interrupt.
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}

void SPI_CloseRx(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);	// Disable interrupt.
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}

/**
 * @fun				SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
 *
 * @brief			Callback for IRQ.
 *
 * @param[in]		pSPIHandle		Handle for I2Cx
 * @param[in]		AppEv
 *
 * @return			none
 *
 * @Note			Can be overridden by user application
 */
__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{
	// This is a weak implementation. The application may override this function.
}

