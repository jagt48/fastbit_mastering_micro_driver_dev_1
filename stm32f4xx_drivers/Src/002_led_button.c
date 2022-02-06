/*
 * 002_led_toggle.c
 *
 *  Created on: Jan 4, 2022
 *      Author: joseph
 */


#include "stm32f407xx.h"
#include "stm32f407xx_gpio_drv.h"
#include "string.h"

#define BTN_PRESSED 1

void delay(void){
	for(uint32_t i = 0; i < 250000; i++);
}

int main(void)
{
	GPIO_Handle_t GpioLed;
	GPIO_Handle_t GpioBtn;

	// Set all elements of the variables to 0 (memset part of string.h).
	memset(&GpioLed,0,sizeof(GpioLed));
	memset(&GpioLed,0,sizeof(GpioLed));

	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HI;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_PUPD_NONE;

	GPIO_ClkCtrl(GPIOD, ENABLE);
	GPIO_Init(&GpioLed);

	GpioBtn.pGPIOx = GPIOA;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN0;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HI;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_PUPD_NONE;

	GPIO_ClkCtrl(GPIOA, ENABLE);
	GPIO_Init(&GpioBtn);

	while(1)
	{
		if(GPIO_ReadPin(GPIOA, GPIO_PIN0) == BTN_PRESSED)
		{
			delay();	// Debounce keypress.
			GPIO_TogglePin(GPIOD, GPIO_PIN12);
		}
	}

	return 0;
}
