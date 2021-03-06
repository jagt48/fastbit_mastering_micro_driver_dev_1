/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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

#define SRAM_ADDRESS1	0x20000004UL

int main(void)
{
    uint32_t value = 0;
    // Changing to volatile will not allow the compiler to optimize out.
    uint32_t volatile *p = (uint32_t *)SRAM_ADDRESS1;	// Store address into pointer by type casting.

    while (1)
    {
    	value = *p;	// Dereference pointer to store data at pointer into value.
    	if(value)
    	{
    		break;
    	}
    }
	for(;;);

	return 0;
}
