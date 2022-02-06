/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include <stdint.h>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif


#define RCC_BASE_ADDR		0x40023800UL
#define RCC_CFGR_ENR_OFFSET	0x08UL
#define RCC_CGFR_ENR_ADDR	(RCC_BASE_ADDR + RCC_CFGR_ENR_OFFSET)
#define RCC_AHB1ENR_OFFSET	0x30UL
#define RCC_AHB1ENR_ADDR	(RCC_BASE_ADDR + RCC_AHB1ENR_OFFSET)
#define GPIOA_BASE_ADDR		0x40020000UL
#define GPIOA_MODER_OFFSET	0x00UL
#define GPIOA_MODER_ADDR	(GPIOA_BASE_ADDR + GPIOA_MODER_OFFSET)
#define GPIOA_AFRH_OFFSET	0x24UL
#define GPIOA_AFRH_ADDR		(GPIOA_BASE_ADDR + GPIOA_AFRH_OFFSET)

int main(void)
{
	// 1. Configure RCC_CFGR register.
	uint32_t *pRccCfgrReg = (uint32_t *)RCC_CGFR_ENR_ADDR;
	uint32_t *pRccAhb1Enr = (uint32_t *)RCC_AHB1ENR_ADDR;
	uint32_t *pGpioAModeReg	= (uint32_t *)GPIOA_MODER_ADDR;
	uint32_t *pGpioAAfrhReg	= (uint32_t *)GPIOA_AFRH_ADDR;

	// Configure RCC_CFGR MCO1 to select HSI as clock source.
	*pRccCfgrReg &= ~(0x03 << 21); // Clear.
	// Set MCO1 prescaler to divide by 4.
	*pRccCfgrReg &= ~(0x07 << 24); 	// Clear.
	*pRccCfgrReg |= (0x06 << 24);	// Set.

	// 2. Configure PA8 and AF0 mode to have as MC01 signal.
	// a. Enable the peripheral clock for GPIOA.
	*pRccAhb1Enr |= (1 << 0);

	// b. Configure the mode of GPIOA pin 8 as alternate function mode.
	*pGpioAModeReg &= ~(0x03 << 16); // Clear.
	*pGpioAModeReg |= (0x02 << 16); // Set.

	// c. Configure the alternate function register to AF0 for PA8.
	*pGpioAAfrhReg &= ~(0x0F << 0);

	for(;;);
}