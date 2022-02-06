/*
 * 003_isr_implemenation.c
 *
 *  Created on: Jan 5, 2022
 *      Author: joseph
 */

#include "stm32f407xx.h"
#include "stm32f407xx_gpio_drv.h"
#include "string.h"

GPIO_Handle_t GpioLed;
GPIO_Handle_t GpioBtn;


int main (void)
{
	// Set all elements of the variables to 0 (memset part of string.h).
	memset(&GpioLed,0,sizeof(GpioLed));
	memset(&GpioLed,0,sizeof(GpioLed));

	/*On-board LED @ PD12*/
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HI;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_PUPD_NONE;

	GPIO_ClkCtrl(GpioLed.pGPIOx, ENABLE);
	GPIO_Init(&GpioLed);

	/*On-board button @ PA0*/
	GpioBtn.pGPIOx = GPIOA;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN0;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_RT;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HI;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_PUPD_NONE;

	GPIO_ClkCtrl(GpioBtn.pGPIOx, ENABLE);
	GPIO_Init(&GpioBtn);

	// Configure button interrupt.
	GPIO_IRQPriorityConfig(IRQ_EXTI0, NVIC_IRQ_PRIO15);
	GPIO_IRQInterruptConfig(IRQ_EXTI0, ENABLE);

	while(1)
	{

	}

	return 0;
}

void EXTI0_IRQHandler(void)
{
	// Handle interrupt.
	GPIO_IRQHandler(GpioBtn.GPIO_PinConfig.GPIO_PinNumber);

	// Toogle pin and add small delay to debounc (!!!BAD PRACTICE IN ISR!!!)
	GPIO_TogglePin(GpioLed.pGPIOx, GpioLed.GPIO_PinConfig.GPIO_PinNumber);
	for(uint32_t i = 0; i < 5000000; i++);
}
