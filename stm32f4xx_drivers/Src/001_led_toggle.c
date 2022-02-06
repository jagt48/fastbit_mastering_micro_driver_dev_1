/*
 * 001_led_toggle.c
 *
 *  Created on: Jan 4, 2022
 *      Author: joseph
 */


#include "stm32f407xx.h"
#include "stm32f407xx_gpio_drv.h"

void delay(void){
	for(uint32_t i = 0; i < 250000; i++);
}

int main(void)
{
	GPIO_Handle_t GpioLed;

	GpioLed.pGPIOx = GPIOD;

	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HI;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_PUPD_NONE;

	GPIO_ClkCtrl(GPIOD, ENABLE);
	GPIO_Init(&GpioLed);

	while(1)
	{
		GPIO_TogglePin(GPIOD, GPIO_PIN12);
		delay();
	}

	return 0;
}
