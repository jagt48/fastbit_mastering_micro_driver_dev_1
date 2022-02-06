/*
 * 004_spi_tx_test.c
 *
 *  Created on: Jan 8, 2022
 *      Author: joseph
 */

#include "stm32f407xx_spi_drv.h"
#include "stm32f407xx_gpio_drv.h"
#include "string.h"

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
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_IDLE_0;
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_EN;
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_DIV16;
	SPI_Init(&SPI2Handle);
}

int main(void)
{
	char msg[] = "Hello World!\n";

	// 1. Set up GPIO pins for SPI2.
	SPI2_GPIOInit();

	// 2. Set up SPI2 and enable.
	SPI2_SPIInit();

	SPI_SSIConfig(SPI2, ENABLE);

	// 3. Enable SPIx peripheral to drive NSS line low.
	SPI_PeripheralControl(SPI2, ENABLE);
	//GPIO_WritePin(GPIOB, GPIO_PIN12, RESET);

	// 4. Transmit message.
	SPI_SendData(SPI2, (uint8_t*)msg, strlen(msg));
	while(SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG) );

	// 5. Disable SPIx peripheral to drive NSS line high.
	SPI_PeripheralControl(SPI2, DISABLE);
	//GPIO_WritePin(GPIOB, GPIO_PIN12, SET);

	while(1)
	{
		// Do nothing.
	}

	return 0;
}
