/*
 * stm32f407xx_i2c_drv.c
 *
 *  Created on: Jan 20, 2022
 *      Author: joseph
 */

/**
 * @file 	stm32f407xx_i2c_drv.c
 *
 * @author	Joseph Gillispie <jagt48@gmail.com>
 *
 * @brief	This driver sets up a basic I/O interface for the STM32F407xx I2C peripheral.
 */

#include "stm32f407xx_i2c_drv.h"

static uint16_t ahbPrescaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};
static uint8_t apbPrescaler[4] = {2, 4, 8, 16};

/*
 * Helper Function Prototypes
 */
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);

/**
 * @fn				I2C_ClkCtrl(I2C_RegDef_t *pI2Cx, uint8_t En)
 *
 * @brief			Enable/disable the I2Cx peripheral clock.
 *
 * @param[in]		pI2Cx	I2Cx peripheral
 * @param[in]		En		Peripheral clock setting
 *
 * @return			none
 *
 * @note			none
 */
void I2C_ClkCtrl(I2C_RegDef_t *pI2Cx, uint8_t En)
{
	if(En == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_CLK_EN();
		}
		else if (pI2Cx == I2C2)
		{
			I2C2_CLK_EN();
		}
		else if (pI2Cx == I2C3)
		{
			I2C3_CLK_EN();
		}
	}
	else
	{
		if(pI2Cx == I2C1)
		{
			I2C1_CLK_DIS();
		}
		else if (pI2Cx == I2C2)
		{
			I2C2_CLK_DIS();
		}
		else if (pI2Cx == I2C3)
		{
			I2C3_CLK_DIS();
		}
	}
}

/**
 * @fn				I2C_Init(I2C_Handle_t *pI2CHandle)
 *
 * @brief			Initialize the I2Cx peripheral.
 *
 * @param[in]		pI2CHandle	I2Cx initialization parameters.
 *
 * @return			none
 *
 * @note			none
 */
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempReg;
	uint16_t ccrValue;

	// CR1
	// 1. Configure ACK
	tempReg = (pI2CHandle->I2C_Config.ACKControl << I2C_CR1_ACK);
	pI2CHandle->pI2Cx->CR1 = tempReg;

	// 2. Configure bus clock frequency.
	tempReg = RCC_GetPCLK1Value() / 1000000;
	pI2CHandle->pI2Cx->CR2 = (tempReg & 0x3F);

	// 3. Configure device's own address.
	tempReg = (pI2CHandle->I2C_Config.DeviceAddress << I2C_OAR1_ADD_7_1);	// 7-bit
	tempReg |= (1 << 14);	// Should always be kept as 1 per reference manual.
	pI2CHandle->pI2Cx->OAR1 = (tempReg & 0xFE);

	// 4. Configure I2Cx clock register
	tempReg = 0;
	if (pI2CHandle->I2C_Config.SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		// Standard mode

		/*
		 * CCR Calculation - Standard mode
		 * Uses 50% DC.
		 * T_high(scl) = T_low(scl) = CCR * T_pclk1 -> CCR = F_pclk1 / (2 * F_sclk)
		 * CCR = T_scl / (2 * T_pclk1) -> F_pclk1 / (2 * F_scl)
		 */
		ccrValue = RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.SCLSpeed);
		tempReg |= (ccrValue & 0xFFF);
	}
	else
	{
		// Fast mode
		tempReg |= (1 << I2C_CCR_FS);
		tempReg |= (pI2CHandle->I2C_Config.FMDutyCycle << I2C_CCR_DUTY);

		/*
		 * CCR Calculation - Fast mode
		 * If DUTY = 0, T_high = CCR * T_pclk1, T_low = 2 * CCR * T_pclk1 -> CCR = F_pclk1 / (3 * F_sclk)
		 * If DUTY = 1, T_high = 9 * CCR * T_pclk1, T_low = 16 * CCR * T_pclk1 -> CCR = F_pclk1 / (25 * F_sclk)
		 *
		 */
		if (pI2CHandle->I2C_Config.FMDutyCycle == I2C_SCL_SPEED_FM2K)
		{
			ccrValue = RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.SCLSpeed);
		}
		else
		{
			ccrValue = RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.SCLSpeed);
		}
		tempReg |= (ccrValue & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempReg;

	// TRise configuration.
	if (pI2CHandle->I2C_Config.SCLSpeed <= I2C_SCL_SPEED_FM2K)
	{
		// Standard Mode
		tempReg = RCC_GetPCLK1Value() / 1000000 + 1;	// 1000 ns
	}
	else
	{
		// Fast Mode
		tempReg = (RCC_GetPCLK1Value() * 300) / 1000000000 + 1;	// 300 ns
	}

	pI2CHandle->pI2Cx->TRISE = (tempReg & 0x3F);

}

/**
 * @fn				I2C_DeInit(I2C_RegDef_t *pI2Cx)
 *
 * @brief			Deinitialize the I2Cx peripheral.
 *
 * @param[in]		pI2Cx	I2Cx peripheral
 *
 * @return			none
 *
 * @note			none
 */
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	if(pI2Cx == I2C1)
	{
		I2C1_REG_RESET();
	}
	else if (pI2Cx == I2C2)
	{
		I2C2_REG_RESET();
	}
	else if (pI2Cx == I2C3)
	{
		I2C3_REG_RESET();
	}
}

/*
 * Data Send/Receive
 */

/**
 * @fn				I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t pTxBuffer, uint32_t Len, uint8_t SlaveAddr)
 *
 * @brief			Send data in master mode
 *
 * @param[in]		pI2CHandle	Pointer to handle of I2C peripheral.
 * @param[in]		pTxBuffer	Pointer to buffer to send.
 * @param[in]		Len			Length of the buffer to send.
 * @param[in]		SlaveAddr	Address of the slave to send data to.
 *
 * @return			none
 *
 * @note			none
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr)
{
	// 1. Generate start condition.
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// 2. Confirm start condition completed.
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SB_FLAG)));

	// 3. Send address of slave with R/W bit set to 0.
	I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, SlaveAddr);

	// 4. Confirm that address phase is completed.
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_ADDR_FLAG)));

	// 5. Clear ADDR flag. (Until cleared, SCL will be stretched LOW).
	I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

	// 6. Send data.
	while (Len > 0)
	{
		while (!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_TXE_FLAG)));
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		Len--;
	}

	// 7. When Len == 0, wait for TXE=1, BTF=1 before generating STOP condition.
	// Note: This means SR and DR are empty and next TX should begin.
	// When BTF=1, SCL will be stretched LOW.
	while (!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_TXE_FLAG)));
	while (!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_BTF_FLAG)));

	// 8. Generate STOP condition.
	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
}

/**
 * @fn				I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t En)
 *
 * @brief			Configure the SPIx IRQ interrupt.
 *
 * @param[in]		IRQNumber	IRQ number of interrupt.
 * @param[in]		En			Mode setting
 *
 * @return			none
 *
 * @note			none
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t En)
{

}

/**
 * @fn				I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
 *
 * @brief			Set I2Cx IRQ priority.
 *
 * @param[in]		IRQNumber		I2Cx IRQ number
 * @param[in]		IRQPriority		I2Cx IRQ priority
 *
 * @return			none
 *
 * @note			none
 */
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{

}


/**
 * @fn				I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
 *
 * @brief			Enable/disable the I2Cx peripheral clock.
 *
 * @param[in]		pI2Cx		I2Cx peripheral
 * @param[in]		FlagName	Name of flag to check
 *
 * @return			none
 *
 * @note			none
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{

	if (pI2Cx->SR1 & FlagName)
	{
		return FLAG_SET;
	}

	return FLAG_RESET;
}

/**
 * @fn				I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t En)
 *
 * @brief			Enable/disable the I2Cx peripheral.
 *
 * @param[in]		pI2Cx	I2Cx peripheral
 * @param[in]		En		Mode setting
 *
 * @return			none
 *
 * @note			none
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t En)
{

}


/**
 * @fn				I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv)
 *
 * @brief			Callback for IRQ.
 *
 * @param[in]		pI2CHandle	Handle for I2Cx
 * @param[in]		AppEv
 *
 * @return			none
 *
 * @note			Can be overridden by user application
 */
__weak void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv)
{

}

/**
 * @fn				RCC_GetPLLOutputClk(void)
 *
 * @brief			Get PLL clock frequency.
 *
 * @param[in]		none
 *
 * @return			PLL clock frequency
 *
 * @note			none
 */
uint32_t RCC_GetPLLOutputClk(void)
{

}

/**
 * @fn				RCC_GetPCLK1Value(void)
 *
 * @brief			Get peripheral bus clock frequency.
 *
 * @param[in]		none
 *
 * @return			Peripheral bus clock frequency.
 *
 * @note			none
 */
uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t systemClk;
	uint32_t pClk1;
	uint8_t clkSrc;
	uint8_t temp;
	uint8_t ahbp;			// AHB prescaler
	uint8_t apb1p;			// APB1 prescaler

	// 1. Determine which clock source is used as the system clock.
	clkSrc = (RCC->CFGR >> 2) & 0x03;

	if (clkSrc == 0)
	{
		// HSI
		systemClk = 16000000;
	}
	else if (clkSrc == 1)
	{
		// HSE
		systemClk = 8000000;
	}
	else if (clkSrc == 2)
	{
		// PLL
		systemClk = RCC_GetPLLOutputClk(); // !!!NOT YET IMPLEMENTED!!!
	}

	// 2. Determine AHB prescaler.
	temp = (RCC->CFGR >> 4) & 0x0F;

	if (temp < 8){
		// System clock not divided.
		ahbp = 1;
	}
	else
	{
		ahbp = ahbPrescaler[temp - 8];
	}

	// 3. Determine the APB1 prescaler
	temp = (RCC->CFGR >> 10) & 0x07;

	if (temp < 4){
		// System clock not divided.
		apb1p = 1;
	}
	else
	{
		apb1p = apbPrescaler[temp - 4];
	}

	pClk1 = (systemClk / ahbp) / apb1p;

	return pClk1;
}

/**
 * @page	module_name Private Helper Functions
 *
 * @subpage	I2C
 */

/**
 * @subpage I2C
 *
 * @fn		I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
 *
 * @brief	Generates a start condition.
 */
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

/**
 * @subpage I2C
 *
 * @fn		I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
 *
 * @brief	Combine slave address with R/W bit.
 */
static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr <<= 1;		// Shift by 1 to make room for R/W bit.
	SlaveAddr &= ~(0x01);	// 0 = write.
	pI2Cx->DR = SlaveAddr;
}

/**
 * @subpage I2C
 *
 * @fn		I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx)
 *
 * @brief	Clears the ADDR flag.
 */
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx)
{
	uint32_t temp;

	// Read SR1 followd by SR2 to clear ADDR flag.
	temp = pI2Cx->SR1;
	temp = pI2Cx->SR2;

	(void)temp;	// Prevent compiler "unused variable" error.
}

/**
 * @subpage I2C
 *
 * @fn		I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
 *
 * @brief	Clears the ADDR flag.
 */
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}
