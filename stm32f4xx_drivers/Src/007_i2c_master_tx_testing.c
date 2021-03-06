/*
 * 007_i2c_master_tx_testing.c
 *
 *  Created on: Feb 4, 2022
 *      Author: joseph
 */

#include "stm32f407xx_i2c_drv.h"
#include "stm32f407xx_gpio_drv.h"
#include "string.h"


/*
 * PB6 -> SCL
 * PB0 -> SDA
 */

void I2C1_GPIOInit(void)
{
	GPIO_Handle_t I2CPins;

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFnMode
	I2CPins.GPIO_PinConfig.GPIO_PinMode
	I2CPins.GPIO_PinConfig.GPIO_PinNumber
	I2CPins.GPIO_PinConfig.GPIO_PinOPType
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdCtrl
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed

}

void I2C1_I2CInit(void)
{

}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GPIOBtn;
	GPIO_Handle_t GpioLed;

	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinAltFnMode = GPIO_AF0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_PUPD_NONE;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	GPIO_Init(&GPIOBtn);

	GpioLed.pGPIOx = GPIOD;

	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HI;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_PUPD_NONE;

	GPIO_ClkCtrl(GPIOD, ENABLE);
	GPIO_Init(&GpioLed);
}

int main(void)
{

	return 0;
}

