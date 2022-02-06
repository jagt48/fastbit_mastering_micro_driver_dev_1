/*
 * stm32f407xx_gpio_drv.h
 *
 *  Created on: Jan 2, 2022
 *      Author: joseph
 */

#ifndef INC_STM32F407XX_GPIO_DRV_H_
#define INC_STM32F407XX_GPIO_DRV_H_

#include "stm32f407xx.h"

typedef struct
{
	uint8_t GPIO_PinNumber;		// @GPIO_PIN_NUMS
	uint8_t GPIO_PinMode;		// @GPIO_PIN_MODES
	uint8_t GPIO_PinSpeed;		// @GPIO_PIN_SPEEDS
	uint8_t GPIO_PinPuPdCtrl;	// @GPIO_PIN_PUPDS
	uint8_t GPIO_PinOPType;		// @GPIO_OP_TYPES
	uint8_t GPIO_PinAltFnMode;	// @GPIO_ALT_FNS
}GPIO_PinConfig_t;

typedef struct
{
	GPIO_RegDef_t *pGPIOx;	/*Pointer to base address of GPIO port to which the pin belongs.*/
	GPIO_PinConfig_t GPIO_PinConfig;	/*Holds GPIO pin configuration settings.*/
}GPIO_Handle_t;

/********************************************************************
 * GPIOx Macros
 *******************************************************************/

/*
 * @GPIO_PIN_NUMS
 */
#define GPIO_PIN0		0	// Pin 0
#define GPIO_PIN1		1	// Pin 1
#define GPIO_PIN2		2	// Pin 2
#define GPIO_PIN3		3	// Pin 3
#define GPIO_PIN4		4	// Pin 4
#define GPIO_PIN5		5	// Pin 5
#define GPIO_PIN6		6	// Pin 6
#define GPIO_PIN7		7	// Pin 7
#define GPIO_PIN8		8	// Pin 8
#define GPIO_PIN9		9	// Pin 9
#define GPIO_PIN10		10	// Pin 10
#define GPIO_PIN11		11	// Pin 11
#define GPIO_PIN12		12	// Pin 12
#define GPIO_PIN13		13	// Pin 13
#define GPIO_PIN14		14	// Pin 14
#define GPIO_PIN15		15	// Pin 15

/*
 * @GPIO_PIN_MODES
 */
#define GPIO_MODE_INPUT		0	// Input
#define GPIO_MODE_OUTPUT	1	// Output
#define GPIO_MODE_AF		2	// Alternate function
#define GPIO_MODE_ANALOG	3	// Analog
#define GPIO_MODE_IT_FT		4	// Interrupt falling-edge trigger
#define GPIO_MODE_IT_RT		5	// Interrupt rising-edge trigger
#define GPIO_MODE_IT_RFT	6	// Interrupt rising- and falling-edge triggers

/*
 * @GPIO_PIN_SPEEDS
 */
#define GPIO_SPEED_LOW		0	// Low speed
#define GPIO_SPEED_MED		1	// Medium speed
#define GPIO_SPEED_HI		2	// High speed
#define GPIO_SPEED_VHI		3	// Very high speed

/*
 * @GPIO_PIN_PUPDS
 */
#define GPIO_PUPD_NONE		0	// No pull-up or pull-down
#define GPIO_PUPD_PU		1	// Pull-up
#define GPIO_PUPD_PD		2	// Pull-down

/*
 * @GPIO_OP_TYPES
 */
#define GPIO_OP_TYPE_PP		0	// Push-pull
#define GPIO_OP_TYPE_OD		1	// Open-drain

/*
 * @GPIO_ALT_FNS
 */
#define GPIO_AF0		0	// Alternate function 0
#define GPIO_AF1		1	// Alternate function 1
#define GPIO_AF2		2	// Alternate function 2
#define GPIO_AF3		3	// Alternate function 3
#define GPIO_AF4		4	// Alternate function 4
#define GPIO_AF5		5	// Alternate function 5
#define GPIO_AF6		6	// Alternate function 6
#define GPIO_AF7		7	// Alternate function 7
#define GPIO_AF8		8	// Alternate function 8
#define GPIO_AF9		9	// Alternate function 9
#define GPIO_AF10		10	// Alternate function 10
#define GPIO_AF11		11	// Alternate function 11
#define GPIO_AF12		12	// Alternate function 12
#define GPIO_AF13		13	// Alternate function 13
#define GPIO_AF14		14	// Alternate function 14
#define GPIO_AF15		15	// Alternate function 15

/********************************************************************
 * GPIOx API
 *******************************************************************/

/*Clock Control*/
void GPIO_ClkCtrl(GPIO_RegDef_t *pGPIOx, uint8_t En);

/*Initialization/Deinitialization*/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*GPIO Input */
uint8_t GPIO_ReadPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadPort(GPIO_RegDef_t *pGPIOx);

/*GPIO Output*/
void GPIO_WritePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WritePort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*GPIO Interrupt*/
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t En);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandler(uint8_t PinNumber);


#endif /* INC_STM32F407XX_GPIO_DRV_H_ */
