/*
 * 006_spi_cmd_handling.c
 *
 *  Created on: Jan 13, 2022
 *      Author: joseph
 */

#include "stm32f407xx_spi_drv.h"
#include "stm32f407xx_gpio_drv.h"
#include "string.h"

/*
 * Command Codes
 */
#define COMMAND_LED_CTRL		0x50
#define COMMAND_SENSOR_READ		0x51
#define COMMAND_LED_READ		0x52
#define COMMAND_PRINT			0x53
#define COMMAND_ID_READ			0x54
#define COMMAND_ACK				0xF5
#define LED_ON					0x01
#define LED_OFF					0x00

/*
 * Arduino Analog Pins
 */
#define ANALOG_PIN0				0
#define ANALOG_PIN1				1
#define ANALOG_PIN2				2
#define ANALOG_PIN3				3
#define ANALOG_PIN4				4

/*
 * Arduino LED
 */
#define LED_PIN					9

/*
 * AF mode --> 5
 * PB14 --> SPI2_MISO
 * PB15 --> SPI2_MOSI
 * PB13 --> SPI2_SCLK
 * PB12 -->	SPI2_NSS
 */

void SPI2_GPIOInit(void)
{
	GPIO_Handle_t SPIPins;


	/*SPI2*/
	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFnMode = GPIO_AF5;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_AF;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_PUPD_NONE;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_VHI;

	/*SPI2_SCLK*/
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN13;
	GPIO_Init(&SPIPins);

	/*SPI2_MOSI*/
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN15;
	GPIO_Init(&SPIPins);

	/*SPI2_MIS0*/
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN14;
	GPIO_Init(&SPIPins);

	/*SPI2_NSS*/
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN12;
	GPIO_Init(&SPIPins);
}

void SPI2_SPIInit(void)
{
	SPI_Handle_t SPI2Handle;

	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_FIRST;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_IDLE_1;
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_DIS;
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_DIV8;
	SPI_Init(&SPI2Handle);
}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GPIOBtn;

	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinAltFnMode = GPIO_AF0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_PUPD_NONE;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	GPIO_Init(&GPIOBtn);
}

void delay(void)
{
	for (uint32_t i = 0; i < 250000; i++);
}

uint8_t SPI_VerifyResponse(uint8_t AckByte)
{
	if (AckByte == COMMAND_ACK)
	{
		return 1;
	}

	return 0;
}

int main(void)
{
	uint8_t CommandCode;
	uint8_t DummyByte = 0xFF;
	uint8_t RxBuff;
	uint8_t args[2];

	SPI2_GPIOInit();
	SPI2_SPIInit();
	GPIO_ButtonInit();
	SPI_SSOEConfig(SPI2, ENABLE);

	while(1)
	{
		/*
		 * Turn on LED.
		 */
		while (!(GPIO_ReadPin(GPIOA, GPIO_PIN0)));				// Wait for button press.
		delay();												// Debounce.

		SPI_PeripheralControl(SPI2, ENABLE);					// Begin data send.

		CommandCode = COMMAND_LED_CTRL;
		SPI_SendData(SPI2, &CommandCode, sizeof(CommandCode));
		SPI_ReceiveData(SPI2, &RxBuff, sizeof(RxBuff));			// Dummy read to clear RXNE flag.
		SPI_SendData(SPI2, &DummyByte, sizeof(DummyByte));		// Send dummy byte for slave clock.
		SPI_ReceiveData(SPI2, &RxBuff, sizeof(RxBuff));			// Receive data.

		/*
		 * !!!Force an ACK due to no HW target connected to target!!!
		 */
		RxBuff = COMMAND_ACK;

		if (SPI_VerifyResponse(RxBuff))							// If "ack" received, send arguments.
		{
			args[0] = LED_PIN;
			args[1] = LED_ON;
			SPI_SendData(SPI2, (uint8_t*)args, sizeof(args));
		}

		/*
		 * Read sensor.
		 */
		while (!(GPIO_ReadPin(GPIOA, GPIO_PIN0)));				// Wait for button press.
		delay();												// Debounce.

		SPI_PeripheralControl(SPI2, ENABLE);					// Begin data send.

		CommandCode = COMMAND_SENSOR_READ;
		SPI_SendData(SPI2, &CommandCode, sizeof(CommandCode));
		SPI_ReceiveData(SPI2, &RxBuff, sizeof(RxBuff));			// Dummy read to clear RXNE flag.
		SPI_SendData(SPI2, &DummyByte, sizeof(DummyByte));		// Send dummy byte for slave clock.
		SPI_ReceiveData(SPI2, &RxBuff, sizeof(RxBuff));			// Receive data.

		/*
		 * !!!Force an ACK due to no HW target connected to target!!!
		 */
		RxBuff = COMMAND_ACK;

		if (SPI_VerifyResponse(RxBuff))							// If "ack" received, send arguments.
		{
			args[0] = ANALOG_PIN0;
			SPI_SendData(SPI2, (uint8_t*)args, sizeof(args[0]));

			SPI_ReceiveData(SPI2, &RxBuff, sizeof(RxBuff));			// Dummy read to clear RXNE flag.
			delay();												// Allow time for slave conversion.
			SPI_SendData(SPI2, &DummyByte, sizeof(DummyByte));		// Send dummy byte for slave clock.
			SPI_ReceiveData(SPI2, &RxBuff, sizeof(RxBuff));			// Receive data.
		}


		SPI_PeripheralControl(SPI2, DISABLE);
	}


	return 0;
}
