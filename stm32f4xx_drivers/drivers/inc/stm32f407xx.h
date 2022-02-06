/*
 * stm32f07xx.h
 *
 *  Created on: Jan 1, 2022
 *      Author: Joseph Gillispie
 */

#include <stdint.h>
#include <stddef.h>

#ifndef INC_STM32F07XX_H_
#define INC_STM32F07XX_H_


#define __weak	__attribute__((weak))
/********************************************************************
 * ARM Cortex M4 NVIC Register Addresses
 *******************************************************************/

/*NVIC*/
/*ISER/ICER*/
#define NVIC_ISER0			((volatile uint32_t*)0xE000E100)
#define NVIC_ISER1			((volatile uint32_t*)(NVIC_ISER0 + 0x04))
#define NVIC_ISER2			((volatile uint32_t*)(NVIC_ISER1 + 0x04))
#define NVIC_ISER3			((volatile uint32_t*)(NVIC_ISER2 + 0x04))
#define NVIC_ICER0			((volatile uint32_t*)0XE000E180)
#define NVIC_ICER1			((volatile uint32_t*)(NVIC_ICER0 + 0x04))
#define NVIC_ICER2			((volatile uint32_t*)(NVIC_ICER1 + 0x04))
#define NVIC_ICER3			((volatile uint32_t*)(NVIC_ICER2 + 0x04))

/*IPR*/
#define NVIC_IPR_BASEADDR		((volatile uint32_t*)0xE000E400)
#define NO_PR_BITS_IMPLEMENTED	4

/*
 * Global Flags
 */
#define ENABLE				1			// Global enable bit.
#define DISABLE				0			// Global disable bit.
#define	SET					ENABLE
#define RESET				DISABLE
#define GPIO_PIN_SET		ENABLE
#define GPIO_PIN_RESET		DISABLE
#define FLAG_RESET			RESET
#define FLAG_SET			SET

#define FLASH_MEM_BASEADDR	0x08000000U
#define SRAM1_BASEADDR		0x20000000U
#define SRAM1_SIZE			0x1C000U						// 112k
#define SRAM2_BASEADDR		(SRAM1_BASEADDR + SRAM1_SIZE)
#define SRAM2_SIZE			0x4000U							// 16k
#define ROM_BASEADDR		0x1FFF0000U						// System Memory
#define ROM_SIZE			0x7800U							// 30.72k
#define SRAM				SRAM1_BASEADDR

/********************************************************************
 * AHBx and APBx Bus Peripheral base addresses
 *******************************************************************/
#define PERIPH_BASE			0x40000000U
#define APB1PERIPH_BASE		PERIPH_BASE
#define APB2PERIPH_BASE		0x40010000U
#define AHB1PERIPH_BASE		0x40020000U
#define AHB2PERIPH_BASE		0x50000000U

/********************************************************************
 * AHB1 Bus Peripheral base addresses
 *******************************************************************/
/*GPIOx*/
#define GPIOA_OFFSET		0x0000U
#define GPIOA_BASEADDR		(AHB1PERIPH_BASE + GPIOA_OFFSET)
#define GPIOB_OFFSET		0x0400U
#define GPIOB_BASEADDR		(AHB1PERIPH_BASE + GPIOB_OFFSET)
#define GPIOC_OFFSET		0x0800U
#define GPIOC_BASEADDR		(AHB1PERIPH_BASE + GPIOC_OFFSET)
#define GPIOD_OFFSET		0x0C00U
#define GPIOD_BASEADDR		(AHB1PERIPH_BASE + GPIOD_OFFSET)
#define GPIOE_OFFSET		0x1000U
#define GPIOE_BASEADDR		(AHB1PERIPH_BASE + GPIOE_OFFSET)
#define GPIOF_OFFSET		0x1400U
#define GPIOF_BASEADDR		(AHB1PERIPH_BASE + GPIOF_OFFSET)
#define GPIOG_OFFSET		0x1800U
#define GPIOG_BASEADDR		(AHB1PERIPH_BASE + GPIOG_OFFSET)
#define GPIOH_OFFSET		0x1C00U
#define GPIOH_BASEADDR		(AHB1PERIPH_BASE + GPIOH_OFFSET)
#define GPIOI_OFFSET		0x2000U
#define GPIOI_BASEADDR		(AHB1PERIPH_BASE + GPIOI_OFFSET)
#define GPIOJ_OFFSET		0x2400U
#define GPIOJ_BASEADDR		(AHB1PERIPH_BASE + GPIOJ_OFFSET)
#define GPIOK_OFFSET		0x2800U
#define GPIOK_BASEADDR		(AHB1PERIPH_BASE + GPIOK_OFFSET)

/*CRC*/
#define CRC_OFFSET			0x3000U
#define CRC_BASEADDR		(AHB1PERIPH_BASE + CRC_OFFSET)

/*RCC*/
#define RCC_OFFSET			0x3800U
#define RCC_BASEADDR		(AHB1PERIPH_BASE + RCC_OFFSET)

/*FLASH*/
#define FLASH_OFFSET		0x3C00U
#define FLASH_BASEADDR		(AHB1PERIPH_BASE + FLASH_OFFSET)

/*BKPSRAM*/
#define BKPSRAM_OFFSET		0x4000U
#define BKPSRAM_BASEADDR	(AHB1PERIPH_BASE + BKPSRAM_OFFSET)

/*DMA1*/
#define DMA1_OFFSET			0x6000U
#define DMA1_BASEADDR		(AHB1PERIPH_BASE + DMA1_OFFSET)

/*DMA2*/
#define DMA2_OFFSET			0x6400U
#define DMA2_BASEADDR		(AHB1PERIPH_BASE + DMA2_OFFSET)

/*ETHERNET_MAC*/
#define ETHERNET_MAC_OFFSET		0x8000U
#define ETHERNET_MAC_BASEADDR	(AHB1PERIPH_BASE + ETHERNET_MAC_OFFSET)

/*DMA2D*/
#define DMA2D_OFFSET		0xB000U
#define DMA2D_BASEADDR		(AHB1PERIPH_BASE + DMA2D_OFFSET)

/*USB_OTC_HS*/
#define USB_OTC_HS_OFFSET	0x40000U
#define USB_OTC_HS_BASEADDR	(AHB1PERIPH_BASE + USB_OTC_HS_OFFSET)


/********************************************************************
 * APB1 Bus Peripheral base addresses
 *******************************************************************/

/*TIM2*/
#define TIM2_OFFSET			0x0000U
#define TIM2_BASEADDR		(APB1PERIPH_BASE + TIM2_OFFSET)

/*TIM3*/
#define TIM3_OFFSET			0x0400U
#define TIM3_BASEADDR		(APB1PERIPH_BASE + TIM3_OFFSET)

/*TIM4*/
#define TIM4_OFFSET			0x0800U
#define TIM4_BASEADDR		(APB1PERIPH_BASE + TIM4_OFFSET)

/*TIM5*/
#define TIM5_OFFSET			0x0C00U
#define TIM5_BASEADDR		(APB1PERIPH_BASE + TIM5_OFFSET)

/*TIM6*/
#define TIM6_OFFSET			0x1000U
#define TIM6_BASEADDR		(APB1PERIPH_BASE + TIM6_OFFSET)

/*TIM7*/
#define TIM7_OFFSET			0x1400U
#define TIM7_BASEADDR		(APB1PERIPH_BASE + TIM7_OFFSET)

/*TIM12*/
#define TIM12_OFFSET		0x1800U
#define TIM12_BASEADDR		(APB1PERIPH_BASE + TIM12_OFFSET)

/*TIM13*/
#define TIM13_OFFSET		0x1C00U
#define TIM13_BASEADDR		(APB1PERIPH_BASE + TIM13_OFFSET)

/*TIM14*/
#define TIM14_OFFSET		0x2000U
#define TIM14_BASEADDR		(APB1PERIPH_BASE + TIM14_OFFSET)

/*RTC*/
#define RTC_OFFSET			0x2800U
#define RTC_BASEADDR		(APB1PERIPH_BASE + RTC_OFFSET)

/*WWDG*/
#define WWDG_OFFSET			0x2C00U
#define WWDG_BASEADDR		(APB1PERIPH_BASE + WWDG_OFFSET)

/*IWDG*/
#define IWDG_OFFSET			0x3000U
#define IWDG_BASEADDR		(APB1PERIPH_BASE + IWDG_OFFSET)

/*I2S2ext*/
#define I2S2ext_OFFSET		0x3400U
#define I2S2ext_BASEADDR	(APB1PERIPH_BASE + I2S2ext_OFFSET)

/*SPI2*/
#define SPI2_OFFSET			0x3800U
#define SPI2_BASEADDR		(APB1PERIPH_BASE + SPI2_OFFSET)

/*SPI3*/
#define SPI3_OFFSET			0x3C00U
#define SPI3_BASEADDR		(APB1PERIPH_BASE + SPI3_OFFSET)

/*I2S3ext*/
#define I2S3ext_OFFSET		0x4000U
#define I2S3ext_BASEADDR	(APB1PERIPH_BASE + I2S3ext_OFFSET)

/*USART2*/
#define USART2_OFFSET		0x4400U
#define USART2_BASEADDR		(APB1PERIPH_BASE + USART2_OFFSET)

/*USART3*/
#define USART3_OFFSET		0x4800U
#define USART3_BASEADDR		(APB1PERIPH_BASE + USART3_OFFSET)

/*UART4*/
#define UART4_OFFSET		0x4C00U
#define UART4_BASEADDR		(APB1PERIPH_BASE + UART4_OFFSET)

/*UART5*/
#define UART5_OFFSET		0x5000U
#define UART5_BASEADDR		(APB1PERIPH_BASE + UART5_OFFSET)

/*I2C1*/
#define I2C1_OFFSET			0x5400U
#define I2C1_BASEADDR		(APB1PERIPH_BASE + I2C1_OFFSET)

/*I2C2*/
#define I2C2_OFFSET			0x5800U
#define I2C2_BASEADDR		(APB1PERIPH_BASE + I2C2_OFFSET)

/*I2C3*/
#define I2C3_OFFSET			0x5C00U
#define I2C3_BASEADDR		(APB1PERIPH_BASE + I2C3_OFFSET)

/*CAN1*/
#define CAN1_OFFSET			0x6400U
#define CAN1_BASEADDR		(APB1PERIPH_BASE + CAN1_OFFSET)

/*CAN2*/
#define CAN2_OFFSET			0x6800U
#define CAN2_BASEADDR		(APB1PERIPH_BASE + CAN2_OFFSET)

/*PWR*/
#define PWR_OFFSET			0x7000U
#define PWR_BASEADDR		(APB1PERIPH_BASE + PWR_OFFSET)

/*DAC*/
#define DAC_OFFSET			0x7400U
#define DAC_BASEADDR		(APB1PERIPH_BASE + DAC_OFFSET)

/*UART7*/
#define UART7_OFFSET		0x7800U
#define UART7_BASEADDR		(APB1PERIPH_BASE + UART7_OFFSET)

/*UART8*/
#define UART8_OFFSET		0x7C00U
#define UART8_BASEADDR		(APB1PERIPH_BASE + UART8_OFFSET)

/********************************************************************
 * APB2 Bus Peripheral base addresses
 *******************************************************************/

/*TIM1*/
#define TIM1_OFFSET			0x0000U
#define TIM1_BASEADDR		(APB2PERIPH_BASE + TIM1_OFFSET)

/*TIM8*/
#define TIM8_OFFSET			0x0400U
#define TIM8_BASEADDR		(APB2PERIPH_BASE + TIM8_OFFSET)

/*USART1*/
#define USART1_OFFSET		0x1000U
#define USART1_BASEADDR		(APB2PERIPH_BASE + USART1_OFFSET)

/*USART6*/
#define USART6_OFFSET		0x1400U
#define USART6_BASEADDR		(APB2PERIPH_BASE + USART6_OFFSET)

/*ADC*/
#define ADC_OFFSET			0x2000U
#define ADC_BASEADDR		(APB2PERIPH_BASE + ADC_OFFSET)

/*SDIO*/
#define SDIO_OFFSET			0x2C00U
#define SDIO_BASEADDR		(APB2PERIPH_BASE + SDIO_OFFSET)

/*SPI1*/
#define SPI1_OFFSET			0x3000U
#define SPI1_BASEADDR		(APB2PERIPH_BASE + SPI1_OFFSET)

/*SPI4*/
#define SPI4_OFFSET			0x3400U
#define SPI4_BASEADDR		(APB2PERIPH_BASE + SPI4_OFFSET)

/*SYSCFG*/
#define SYSCFG_OFFSET		0x3800U
#define SYSCFG_BASEADDR		(APB2PERIPH_BASE + SYSCFG_OFFSET)

/*EXTI*/
#define EXTI_OFFSET			0x3C00U
#define EXTI_BASEADDR		(APB2PERIPH_BASE + EXTI_OFFSET)

/*TIM9*/
#define TIM9_OFFSET			0x4000U
#define TIM9_BASEADDR		(APB2PERIPH_BASE + TIM9_OFFSET)

/*TIM10*/
#define TIM10_OFFSET		0x4400U
#define TIM10_BASEADDR		(APB2PERIPH_BASE + TIM10_OFFSET)

/*TIM11*/
#define TIM11_OFFSET		0x4800U
#define TIM11_BASEADDR		(APB2PERIPH_BASE + TIM11_OFFSET)

/*SPI5*/
#define SPI5_OFFSET			0x5000U
#define SPI5_BASEADDR		(APB2PERIPH_BASE + SPI5_OFFSET)

/*SPI6*/
#define SPI6_OFFSET			0x5400U
#define SPI6_BASEADDR		(APB2PERIPH_BASE + SPI6_OFFSET)

/*SAI1*/
#define SAI1_OFFSET			0x5800U
#define SAI1_BASEADDR		(APB2PERIPH_BASE + SAI1_OFFSET)

/*LTDC*/
#define LTDC_OFFSET			0x6800U
#define LTDC_BASEADDR		(APB2PERIPH_BASE + LTDC_OFFSET)

/********************************************************************
 * APB2 Bus Peripheral base addresses
 *******************************************************************/

/*USB_OTC_FS*/
#define USB_OTC_FS_OFFSET	0x0000U
#define USB_OTC_FS_BASEADDR	(AHB2PERIPH_BASE + USB_OTC_FS_OFFSET)

/*DCMI*/
#define DCMI_OFFSET			0x50000U
#define DCMI_BASEADDR		(AHB2PERIPH_BASE + DCMI_OFFSET)

/*CRYP*/
#define CRYP_OFFSET			0x60000U
#define CRYP_BASEADDR		(AHB2PERIPH_BASE + CRYP_OFFSET)

/*HASH*/
#define HASH_OFFSET			0x60400U
#define HASH_BASEADDR		(AHB2PERIPH_BASE + HASH_OFFSET)

/*RNG*/
#define RNG_OFFSET			0x60800U
#define RNG_BASEADDR		(AHB2PERIPH_BASE + RNG_OFFSET)

/********************************************************************
 * Peripheral Register Definitions
 *******************************************************************/
/*GPIOx*/
typedef struct
{
	uint32_t MODER;		// Mode
	uint32_t OTYPER;	// Output type
	uint32_t OSPEEDR;	// Output speed
	uint32_t PUPDR;		// Pull-up/pull-down
	uint32_t IDR;		// Input data
	uint32_t ODR;		// Output data
	uint32_t BSRR;		// Bit set/reset
	uint32_t LCKR;		// Configuration lock
	uint32_t AFR[2];	// Alternate function (high = [0], low = [1])
}volatile GPIO_RegDef_t;

/*RCC*/
typedef struct
{
	uint32_t CR;			// Clock control
	uint32_t PLLCFGR;		// PLL congifuration
	uint32_t CFGR;			// Clock configuration
	uint32_t CIR;			// Clock interrupt
	uint32_t AHB1RSTR;		// AHB1 peripheral reset
	uint32_t AHB2RSTR;		// AHB2 peripheral reset
	uint32_t AHB3RSTR;		// AHB3 peripheral reset
	uint32_t RESERVED0;		// RESERVED (0x1C)
	uint32_t APB1RSTR;		// APB1 peripheral reset
	uint32_t APB2RSTR;		// APB2 peripheral reset
	uint32_t RESERVED1[2];	// RESERVED (0x28..0x2C)
	uint32_t AHB1ENR;		// AHB1 peripheral clock enable
	uint32_t AHB2ENR;		// AHB2 peripheral clock enable
	uint32_t AHB3ENR;		// AHB3 peripheral clock enable
	uint32_t RESERVED2;		// RESERVED (0x3C)
	uint32_t APB1ENR;		// APB1 peripheral clock enable
	uint32_t APB2ENR;		// APB2 peripheral clock enable
	uint32_t RESERVED3[2];	// RESERVED (0x48..0x4C)
	uint32_t AHB1LPENR;		// AHB1 low power clock enable
	uint32_t AHB2LPENR;		// AHB2 low power clock enable
	uint32_t AHB3LPENR;		// AHB3 low power clock enable
	uint32_t RESERVED4;		// RESERVED (0x5C)
	uint32_t APB1LPENR;		// APB1 low power clock enable
	uint32_t APB2LPENR;		// APB2 low power clock enable
	uint32_t RESERVED5[2];	// RESERVED (0x68..0x6C)
	uint32_t BDCR;			// Backup domain control
	uint32_t CSR;			// Clock control and status
	uint32_t RESERVED6[2];	// RESERVED (0x78..0x7C)
	uint32_t SSCGR;			// Spread spectrum clock generator
	uint32_t PLLI2SCFGR;	// PLLI2S configuration
}volatile RCC_RegDef_t;

/*EXTI*/
typedef struct
{
	uint32_t IMR;			// Interrupt mask register (0x00)
	uint32_t EMR;			// Event mask register (0x04)
	uint32_t RTSR;			// Rising trigger selection (0x08)
	uint32_t FTSR;			// Falling trigger selection (0x0C)
	uint32_t SWIER;			// Software interrupt event (0x10)
	uint32_t PR;			// Pending register (0x14)
}volatile EXTI_RegDef_t;

/*SYSCFG*/
typedef struct
{
	uint32_t MEMRMP;		// Memory remap (0x00)
	uint32_t PMC;			// Peripheral mode configuration (0x04)
	uint32_t EXTICR[4];		// External interrupt configuration (0x08..0x14)
	uint32_t CMPCR;			// Compensation cell control (0x20)
}volatile SYSCFG_RegDef_t;

/*SPIx*/
typedef struct
{
	uint32_t CR1;			// Control 1 (0x00)
	uint32_t CR2;			// Control 2 (0x04)
	uint32_t SR;			// Status (0x08)
	uint32_t DR;			// Data (0x0C)
	uint32_t CRCPR;			// CRC polynomial (0x10)
	uint32_t RXCRCR;		// RX CRC (0x14)
	uint32_t TXCRCR;		// TX CRC (0x18)
	uint32_t I2SCFGR;		// SPI_I2S configuration (0x1C)
	uint32_t I2SPR;			// SPI_I2S prescaler
}volatile SPI_RegDef_t;

/*I2C*/
typedef struct
{
	uint32_t CR1;			//<Control 1 (0x00)
	uint32_t CR2;			//<Control 2 (0x04)
	uint32_t OAR1;			//<Own address 1 (0x08)
	uint32_t OAR2;			//<Own address 2 (0x0C)
	uint32_t DR;			//<Data (0x10)
	uint32_t SR1;			//<Status 1 (0x14)
	uint32_t SR2;			//<Status 2 (0x18)
	uint32_t CCR;			//<Clock control register (0x1C)
	uint32_t TRISE;			//<Rise time (0x20)
	uint32_t FLTR;			//<Filter (0x24)
}volatile I2C_RegDef_t;

/********************************************************************
 * Peripheral Bitfield Definitions
 *******************************************************************/

/*
 * SPIx/I2Sx
 */

// CR1
#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR			3
#define SPI_CR1_SPE			6
#define SPI_CR1_LSBFIRST	7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RXONLY		10
#define SPI_CR1_DFF			11
#define SPI_CR1_CRCNEXT		12
#define SPI_CR1_CRCEN		13
#define SPI_CR1_BIDIOE		14
#define SPI_CR1_BIDIMODE	15

// CR2
#define SPI_CR2_RXDMAEN		0
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_SSOE		2
#define SPI_CR2_FRF			4
#define SPI_CR2_EERIE		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE		7

// SR
#define SPI_SR_RXNE			0
#define SPI_SR_TXE			1
#define SPI_SR_CHSIDE		2
#define SPI_SR_UDR			3
#define SPI_SR_CRCERR		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR			6
#define SPI_SR_BSY			7
#define SPI_SR_FRE			8

// I2SCFGR
#define SPI_I2SCFGR_CHLEN		0
#define SPI_I2SCFGR_DATLEN		1
#define SPI_I2SCFGR_CKPOL		3
#define SPI_I2SCFGR_I2SSTD		4
#define SPI_I2SCFGR_PSMSYNC		7
#define SPI_I2SCFGR_I2SCFG		8
#define SPI_I2SCFGR_I2SE		10
#define SPI_I2SCFGR_I2SMOD		11

// I2SPR
#define SPI_I2SPR_II2SDIV		0
#define SPI_I2SPR_ODD			8
#define SPI_I2SPR_MCKOE			9

/*
 * I2Cx
 */
/*CR1*/
#define I2C_CR1_PE				0	//<Peripheral enable
#define I2C_CR1_SMBUS			1	//<SMBus mode
#define I2C_CR1_SMBTYPE			3	//<SMBus type
#define I2C_CR1_ENART			4	//<ARP enable
#define I2C_CR1_ENPEC			5	//<PEC enable
#define I2C_CR1_ENGC			6	//<General call enable
#define I2C_CR1_NOSTRETCH		7	//<Clock stretching disable (slave mode)
#define I2C_CR1_START			8	//<Start generation
#define I2C_CR1_STOP			9	//<Stop generation
#define I2C_CR1_ACK				10	//<Acknowledge enable
#define I2C_CR1_POS				11	//<Acknowledge/PEC position
#define I2C_CR1_PEC				12	//<Packet error checking
#define I2C_CR1_ALERT			13	//<SMBus alert
#define I2C_CR1_SWRST			15	//<Software reset

/*CR2*/
#define I2C_CR2_FREQ			0	//<Peripheral clock frequency
#define I2C_CR2_ITERREN			8	//<Error interrupt enable
#define I2C_CR2_ITEVTEN			9	//<Event interrupt enable
#define I2C_CR2_ITBUFEN			10	//<Buffer interrupt enable
#define I2C_CR2_DMAEN			11	//<DMA requests enable
#define I2C_CR2_LAST			12	//<DMA last transfer

/*OAR1*/
#define I2C_OAR1_ADD0			0	//<Interface address
#define I2C_OAR1_ADD_7_1		1	//<Bits [7..1] of address
#define I2C_OAR1_ADD_9_8		8	//<Bits [9..8] of address
#define I2C_OAR1_ADDMODE		15	//<Addressing mode (slave mode)

/*SR1*/
#define I2C_SR1_SB				0	//<Start bit (master mode)
#define I2C_SR1_ADDR			1	//<Address sent (master mode)/matched (slave mode)
#define I2C_SR1_BTF				2	//<Byte transfer finished
#define I2C_SR1_ADD10			3	//<10-bit header sent (master mode)
#define I2C_SR1_STOPF			4	//<Stop detection
#define I2C_SR1_RXNE			6	//<Receive data register not empty
#define I2C_SR1_TXE				7	//<Transmit data register empty
#define I2C_SR1_BERR			8	//<Bus error
#define I2C_SR1_ARLO			9	//<Arbitration lost (master mode)
#define I2C_SR1_AF				10	//<Acknowledge failure
#define I2C_SR1_OVR				11	//<Overrun/underrun
#define I2C_SR1_PECERR			12	//<PEC error in reception
#define I2C_SR1_TIMEOUT			14	//<Timeout or Tlow error
#define I2C_SR1_SMBALERT		15	//<SMBus alert

/*SR2*/
#define I2C_SR2_MSL				0	//<Master/slave
#define I2C_SR2_BUSY			1	//<Bus busy
#define I2C_SR2_TRA				2	//<Transmitter/receiver
#define I2C_SR2_GENCALL			4	//<General call address
#define I2C_SR2_SMBDEFAULT		5	//<SMBus device default address (slave mode)
#define I2C_SR2_SMBHOST			6	//<SMBus host header (slave mode)
#define I2C_SR2_DUALF			7	//<Dual flag (slave mode)
#define I2C_SR2_PEC				8	//<Packet error checking

/*CCR*/
#define I2C_CCR_CCR				0	//<Clock control register in Fm/Sm mode (master mode)
#define I2C_CCR_DUTY			14	//<Fm mode duty cyle
#define I2C_CCR_FS				15	//<I2C master mode selection


/********************************************************************
 * Peripheral Definitions
 *******************************************************************/
/*GPIOx*/
#define GPIOA	((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB	((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC	((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD	((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE	((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF	((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG	((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH	((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI	((GPIO_RegDef_t*)GPIOI_BASEADDR)

/*RCC*/
#define RCC		((RCC_RegDef_t*)RCC_BASEADDR)

/*EXTI*/
#define EXTI	((EXTI_RegDef_t*)EXTI_BASEADDR)

/*SYSCFG*/
#define SYSCFG	((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

/*SPIx*/
#define SPI1	((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2	((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3	((SPI_RegDef_t*)SPI3_BASEADDR)

/*I2Cx*/
#define I2C1	((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2	((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3	((I2C_RegDef_t*)I2C3_BASEADDR)

/********************************************************************
 * GPIOx Clock Enable/Disable Macros
 *******************************************************************/
#define GPIOA_CLK_EN()	(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_CLK_EN()	(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_CLK_EN()	(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_CLK_EN()	(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_CLK_EN()	(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_CLK_EN()	(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_CLK_EN()	(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_CLK_EN()	(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_CLK_EN()	(RCC->AHB1ENR |= (1 << 8))

#define GPIOA_CLK_DIS()	(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_CLK_DIS()	(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_CLK_DIS()	(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_CLK_DIS()	(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_CLK_DIS()	(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_CLK_DIS()	(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_CLK_DIS()	(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_CLK_DIS()	(RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_CLK_DIS()	(RCC->AHB1ENR &= ~(1 << 8))

/********************************************************************
 * GPIOx Reset
 *******************************************************************/
// Set to clear, reset to allow enabling of GPIOx again.
#define GPIOA_REG_RST()		do{(RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~ (1 << 0)); } while(0)
#define GPIOB_REG_RST()		do{(RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~ (1 << 1)); } while(0)
#define GPIOC_REG_RST()		do{(RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~ (1 << 2)); } while(0)
#define GPIOD_REG_RST()		do{(RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~ (1 << 3)); } while(0)
#define GPIOE_REG_RST()		do{(RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~ (1 << 4)); } while(0)
#define GPIOF_REG_RST()		do{(RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~ (1 << 5)); } while(0)
#define GPIOG_REG_RST()		do{(RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~ (1 << 6)); } while(0)
#define GPIOH_REG_RST()		do{(RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~ (1 << 7)); } while(0)
#define GPIOI_REG_RST()		do{(RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~ (1 << 8)); } while(0)

/********************************************************************
 * GPIOx Port Code Selection
 *******************************************************************/
#define GPIO_BASEADDR_TO_CODE(x)	((x == GPIOA) ? 0 :\
									(x == GPIOB) ? 1 :\
									(x == GPIOC) ? 2 :\
									(x == GPIOD) ? 3 :\
									(x == GPIOE) ? 4 :\
									(x == GPIOF) ? 5 :\
									(x == GPIOG) ? 6 :\
									(x == GPIOH) ? 7 :\
									(x == GPIOI) ? 8 : 0)

/********************************************************************
 * IRQ
 *******************************************************************/

/*Positions*/
#define IRQ_WWDG		0		// Window watchdog
#define IRQ_PVD			1		// PVD through EXTI line
#define IRQ_TAMP_STAMP	2		// Tamper and TimeStamp through EXTI line
#define IRQ_RTC_WKUP	3		// RC wakeup through EXTI line
#define IRQ_FLASH		4		// Flash
#define IRQ_RCC			5		// RCC
#define IRQ_EXTI0		6		// EXTI0
#define IRQ_EXTI1		7		// EXTI1
#define IRQ_EXTI2		8		// EXTI2
#define IRQ_EXTI3		9		// EXTI3
#define IRQ_EXTI4		10		// EXTI4
#define IRQ_ADC			18		// ADC
#define IRQ_EXTI9_5		23		// EXTI(9:5)
#define IRQ_TIM2		28		// TIM2
#define IRQ_TIM3		29		// TIM3
#define IRQ_TIM4		30		// TIM4
#define IRQ_I2C1_EV		31		// I2C1 event
#define IRQ_I2C1_ER		32		// I2C1 error
#define IRQ_I2C2_EV		33		// I2C2 event
#define IRQ_I2C2_ER		34		// I2C2 error
#define IRQ_SPI1		35		// SPI1
#define IRQ_SPI2		36		// SPI2
#define IRQ_USART1		37		// USART1
#define IRQ_USART2		38		// USART2
#define IRQ_USART3		39		// USART3
#define IRQ_EXTI5_10	40		// EXTI(15:10]
#define IRQ_RTC			41		// RTC
#define IRQ_TIM5		50		// TIM5
#define IRQ_SPI3		51		// SPI3
#define IRQ_UART4		52		// UART4
#define IRQ_UART5		53		// UART5
#define IRQ_TIM7		55		// TIM7
#define IRQ_USART6		71		// USART6
#define IRQ_I2C3_EV		72		// I2C3 event
#define IRQ_I2C3_ER		73		// I2C3 error
#define IRQ_FPU			81		// FPU

/*Priorities*/
#define NVIC_IRQ_PRIO0 		0
#define NVIC_IRQ_PRIO1	 	1
#define NVIC_IRQ_PRIO2	 	2
#define NVIC_IRQ_PRIO3		3
#define NVIC_IRQ_PRIO4	 	4
#define NVIC_IRQ_PRIO5	 	5
#define NVIC_IRQ_PRIO6	 	6
#define NVIC_IRQ_PRIO7	 	7
#define NVIC_IRQ_PRIO8	 	8
#define NVIC_IRQ_PRIO9	 	9
#define NVIC_IRQ_PRIO10 	10
#define NVIC_IRQ_PRIO11 	11
#define NVIC_IRQ_PRIO12 	12
#define NVIC_IRQ_PRIO13 	13
#define NVIC_IRQ_PRIO14 	14
#define NVIC_IRQ_PRIO15 	15

/********************************************************************
 * I2Cx Clock Enable/Disable Macros
 *******************************************************************/
#define I2C1_CLK_EN()	(RCC->APB1ENR |= (1 << 21))
#define I2C2_CLK_EN()	(RCC->APB1ENR |= (1 << 22))
#define I2C3_CLK_EN()	(RCC->APB1ENR |= (1 << 23))

#define I2C1_CLK_DIS()	(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_CLK_DIS()	(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_CLK_DIS()	(RCC->APB1ENR &= ~(1 << 23))

/********************************************************************
 * SPIx Clock Enable/Disable Macros
 *******************************************************************/
#define SPI1_CLK_EN()	(RCC->APB2ENR |= (1 << 12))
#define SPI2_CLK_EN()	(RCC->APB1ENR |= (1 << 14))
#define SPI3_CLK_EN()	(RCC->APB1ENR |= (1 << 15))

#define SPI1_CLK_DIS()	(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_CLK_DIS()	(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_CLK_DIS()	(RCC->APB1ENR &= ~(1 << 15))

/********************************************************************
 * SPIx Reset
 *******************************************************************/
// Set to clear, reset to allow enabling of GPIOx again.
#define SPI1_REG_RST()		do{(RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~ (1 << 12)); } while(0)
#define SPI2_REG_RST()		do{(RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~ (1 << 14)); } while(0)
#define SPI3_REG_RST()		do{(RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~ (1 << 15)); } while(0)

/**
 * I2Cx Clock Enable/Disable Macros
 */
#define I2C1_CLK_EN()	(RCC->APB1ENR |= (1 << 21))
#define I2C2_CLK_EN()	(RCC->APB1ENR |= (1 << 22))
#define I2C3_CLK_EN()	(RCC->APB1ENR |= (1 << 23))

#define I2C1_CLK_DIS()	(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_CLK_DIS()	(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_CLK_DIS()	(RCC->APB1ENR &= ~(1 << 23))

/**
 * I2Cx Reset
 */
#define I2C1_REG_RESET()	do{(RCC->APB1RSTR |= (1 << 21)); (RCC->APB1RSTR &= ~(1 << 21)); } while(0)
#define I2C2_REG_RESET()	do{(RCC->APB1RSTR |= (1 << 22)); (RCC->APB1RSTR &= ~(1 << 22)); } while(0)
#define I2C3_REG_RESET()	do{(RCC->APB1RSTR |= (1 << 23)); (RCC->APB1RSTR &= ~(1 << 23)); } while(0)

/********************************************************************
 * USARTx/UARTx Clock Enable/Disable Macros
 *******************************************************************/
#define USART1_CLK_EN()		(RCC->APB2ENR |= (1 << 4))
#define USART2_CLK_EN()		(RCC->APB1ENR |= (1 << 17))
#define USART3_CLK_EN()		(RCC->APB1ENR |= (1 << 18))
#define UART4_CLK_EN()		(RCC->APB1ENR |= (1 << 19))
#define UART5_CLK_EN()		(RCC->APB1ENR |= (1 << 20))
#define USART6_CLK_EN()		(RCC->APB2ENR |= (1 << 5))


#define USART1_CLK_DIS()	(RCC->APB2ENR &= ~(1 << 4))
#define USART2_CLK_DIS()	(RCC->APB1ENR &= ~(1 << 17))
#define USART3_CLK_DIS()	(RCC->APB1ENR &= ~(1 << 18))
#define UART4_CLK_DIS()		(RCC->APB1ENR &= ~(1 << 19))
#define UART5_CLK_DIS()		(RCC->APB1ENR &= ~(1 << 20))
#define USART6_CLK_DIS()	(RCC->APB2ENR &= ~(1 << 5))

/********************************************************************
 * SYSCFG Clock Enable/Disable Macros
 *******************************************************************/
#define SYSCFG_CLK_EN()		(RCC->APB2ENR |= (1 << 14))

#define SYSCFG_CLK_DIS()	(RCC->APB2ENR &= ~(1 << 14))

#endif /* INC_STM32F07XX_H_ */
