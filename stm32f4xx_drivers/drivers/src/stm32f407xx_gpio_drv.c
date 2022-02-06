/*
 * stm32f407xx_gpio_drv.c
 *
 *  Created on: Jan 2, 2022
 *      Author: joseph
 */

#include "stm32f407xx_gpio_drv.h"

/********************************************************************
 * GPIOx API
 *******************************************************************/

/**
 * @fn			GPIO_ClkCtrl
 *
 * @brief 		Enables or disables the GPIOx peripheral clock.
 *
 * @param[in]	*pGPIOx	Pointer to the base address of the GPIOx register to be written to.
 * @param[in]	En		Bit to ENABLE (1) or DISABLE (0) selected GPIOx clock.
 *
 * @retvalue 	none
 *
 * @Note		This function should be called before writing to any GPIOx registers.
 */
void GPIO_ClkCtrl(GPIO_RegDef_t *pGPIOx, uint8_t En)
{
	if (En == ENABLE)
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_CLK_EN();
		} else if (pGPIOx == GPIOB)
		{
			GPIOB_CLK_EN();
		} else if (pGPIOx == GPIOC)
		{
			GPIOC_CLK_EN();
		} else if (pGPIOx == GPIOD)
		{
			GPIOD_CLK_EN();
		} else if (pGPIOx == GPIOE)
		{
			GPIOE_CLK_EN();
		} else if (pGPIOx == GPIOF)
		{
			GPIOF_CLK_EN();
		} else if (pGPIOx == GPIOG)
		{
			GPIOG_CLK_EN();
		} else if (pGPIOx == GPIOH)
		{
			GPIOH_CLK_EN();
		} else if (pGPIOx == GPIOI)
		{
			GPIOI_CLK_EN();
		}
	} else
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_CLK_DIS();
		} else if (pGPIOx == GPIOB)
		{
			GPIOB_CLK_DIS();
		} else if (pGPIOx == GPIOC)
		{
			GPIOC_CLK_DIS();
		} else if (pGPIOx == GPIOD)
		{
			GPIOD_CLK_DIS();
		} else if (pGPIOx == GPIOE)
		{
			GPIOE_CLK_DIS();
		} else if (pGPIOx == GPIOF)
		{
			GPIOF_CLK_DIS();
		} else if (pGPIOx == GPIOG)
		{
			GPIOG_CLK_DIS();
		} else if (pGPIOx == GPIOH)
		{
			GPIOH_CLK_DIS();
		} else if (pGPIOx == GPIOI)
		{
			GPIOI_CLK_DIS();
		}
	}
}

/*Initialization/Deinitialization*/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0; 	// Temporary value to store each register bitfield calculation.
	uint8_t TempReg;		// Alternate Function register selection (Low = AF[0], High = AF[1]).
	uint8_t TempShift;	// Number of bits to shift AF pin selection by.

	// 0. Enable clock.
	GPIO_ClkCtrl(pGPIOHandle->pGPIOx, ENABLE);

	// 1. Configure mode.
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{// Non-IRQ mode.
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x03 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));	// Clear.
		pGPIOHandle->pGPIOx->MODER |= temp;															// Set.
	} else
	{// IRQ mode
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			// 1. Configure FTSR.
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);	// Set for falling edge.
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);	// Clear for corresponding rising edge.
		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			// 1. Configure RTSR.
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);	// Set for rising edge.
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			// 1. Configure both FTSR and RTSR.
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);	// Set for rising edge.
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);	// Set for falling edge.

		}
		// 2. Configure GPIO port selection in SYSCFG_EXTICR.
		TempReg = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;		// EXTI configuration register selection.
		TempShift = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;		// EXTI bitfield selection.
		uint8_t PortCode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_CLK_EN();
		SYSCFG->EXTICR[TempReg] = PortCode << (4 * TempShift);

		// 3. Enable the EXTI interrupt delivery using IMR.
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);			// Unmask IRQ bit.
	}

	// 2. Configure speed.
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x03 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	// 3. Configure pull-up/pull-down settings.
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdCtrl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x03 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	// 4. Configure output type.
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x01 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	// 5. Configure alternate functionality.
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_AF)
	{
		/*
		 * Alternate Function register bitfields are 4 bits wide. AFLow and AFHigh are split
		 * between two registers.
		 *
		 * To determine which register to use, calculate if the selected
		 * pin is in the Low (0..7) or High (8..15) range.
		 * Ex: Pin 9 -> 9/8 = 1, so Pin 9 is in High range, and AF[1] will be used.
		 *
		 * To determine which bitfield to use, calculate the modulus of the
		 * selected pin with the possible number of pin selections in a register.
		 * Ex: Pin 6 -> 6%8 = 6, so Pin 6 bitfield is 4*6 = lshift by 24.
		 */
		TempReg = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;	// 8 possible pin selections per register.
		TempShift = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFnMode << (4 * TempShift);
		pGPIOHandle->pGPIOx->AFR[TempReg] &= ~(0x0F << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->AFR[TempReg] |= temp;
	}
}
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if (pGPIOx == GPIOA)
	{
		GPIOA_REG_RST();
	} else if (pGPIOx == GPIOB)
	{
		GPIOB_REG_RST();
	} else if (pGPIOx == GPIOC)
	{
		GPIOC_REG_RST();
	} else if (pGPIOx == GPIOD)
	{
		GPIOD_REG_RST();
	} else if (pGPIOx == GPIOE)
	{
		GPIOE_REG_RST();
	} else if (pGPIOx == GPIOF)
	{
		GPIOF_REG_RST();
	} else if (pGPIOx == GPIOG)
	{
		GPIOG_REG_RST();
	} else if (pGPIOx == GPIOH)
	{
		GPIOH_REG_RST();
	} else if (pGPIOx == GPIOI)
	{
		GPIOI_REG_RST();
	}

}

/*GPIO Input */
uint8_t GPIO_ReadPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;

	value = (uint8_t)(pGPIOx->IDR >> PinNumber) & 0x00000001;

	return value;
}
uint16_t GPIO_ReadPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;

	value = (uint16_t)pGPIOx->IDR;

	return value;
}

/*GPIO Output*/
void GPIO_WritePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if (Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1 << PinNumber);
	} else
	{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

void GPIO_WritePort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}

void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}

/*GPIO Interrupt Configuration*/
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t En)
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

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
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

void GPIO_IRQHandler(uint8_t PinNumber)
{
	// Clear the EXTI PR register corresponding to the pin number.
	if(EXTI->PR & (1 << PinNumber))
	{
		// Clear by writing 1.
		EXTI->PR |= (1 << PinNumber);
	}
}
