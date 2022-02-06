/*
 * 005_spi_txonly_arduino.c
 *
 *  Created on: Jan 13, 2022
 *      Author: joseph
 */

#include "stm32f407xx_spi_drv.h"
#include "stm32f407xx_gpio_drv.h"
#include "string.h"

/*
 * AF mode --> 5
 * PB14 --> SPI2_MISO --> Arduino P12
 * PB15 --> SPI2_MOSI --> Arduino P11
 * PB13 --> SPI2_SCLK --> Arduino P13
 * PB12 -->	SPI2_NSS  --> Arduino P10
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
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_PUPD_PU;
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN12;
	GPIO_Init(&SPIPins);
}

void SPI2_SPIInit(void)
{
	SPI_Handle_t SPI2Handle;

	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_FIRST;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_IDLE_0;
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_DIS;
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_DIV32;
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

int main(void)
{
	char msg[] = "Hello World!\n";
	uint8_t dataLen;

	// 1. Set up GPIO pins for SPI2.
	SPI2_GPIOInit();

	// 2. Set up SPI2 and enable.
	SPI2_SPIInit();

	// 3. Set up button.
	GPIO_ButtonInit();

	/*
	 * NOTE
	 * NSS Output Enable (SSM=0, SSOE=1)
	 * NSS pin managed by HW. NSS driven low as soon as SPI is enabled (SPE=1),
	 * and kept low until the SPI is disabled (SPE=0).
	 */
	SPI_SSOEConfig(SPI2, ENABLE);

	while(1)
	{
		// Wait for button press
		while (!(GPIO_ReadPin(GPIOA, GPIO_PIN0)));
		delay();	// Debounce.

		// 4. Enable SPIx peripheral to drive NSS line low.
		SPI_PeripheralControl(SPI2, ENABLE);

		// 5. Transmit message.
		dataLen = strlen(msg);
		SPI_SendData(SPI2, &dataLen, sizeof(dataLen));
		SPI_SendData(SPI2, (uint8_t*)msg, strlen(msg));

		// Wait until last byte in transmitted.
		while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

		// 6. Disable SPIx peripheral to drive NSS line high.
		SPI_PeripheralControl(SPI2, DISABLE);
	}

	return 0;
}
