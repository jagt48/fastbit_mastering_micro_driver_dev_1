/*
 * stm32f407xx_i2c_drv.h
 *
 *  Created on: Jan 20, 2022
 *      Author: joseph
 */

#ifndef INC_STM32F407XX_I2C_DRV_H_
#define INC_STM32F407XX_I2C_DRV_H_

#include "stm32f407xx.h"

typedef struct
{
	uint32_t SCLSpeed;			//!<@I2C_SCLSpeed
	uint8_t DeviceAddress;		//!<Provided by user
	uint8_t ACKControl;			//!<@I2C_ACKControl
	uint16_t FMDutyCycle;		//!<@I2C_FMDutyCycle
}I2C_Config_t;


typedef struct
{
	I2C_RegDef_t *pI2Cx;		//!<Pointer to base address of I2Cx port to which the configuration belongs.
	I2C_Config_t I2C_Config;	//!<Holds I2Cx pin configuration settings.
}I2C_Handle_t;

/*
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM	100000		//!<Standard mode - 100kbps
#define I2C_SCL_SPEED_FM4K	400000		//!<Fast mode - 400kbps
#define I2C_SCL_SPEED_FM2K	200000		//!<Fast mode - 200kbps


/*
 * @I2C_ACKControl
 */
#define I2C_ACK_DIS			0		//!<No ACK returned
#define I2C_ACK_EN			1		//!<ACK returned

/*
 * @I2C_FMDutyCycle
 */
#define I2C_DUTY_2			0		//!<t_low/t_high = 2
#define I2C_DUTY_16_9		1		//!<t_low/t_high = 16/9

/********************************************************************
 * I2Cx API
 *******************************************************************/

/*
 * Flags
 */
#define I2C_SB_FLAG			(1 << I2C_SR1_SB)		//!<Start bit
#define I2C_ADDR_FLAG		(1 << I2C_SR1_ADDR)		//!<Address sent
#define I2C_BTF_FLAG		(1 << I2C_SR1_BTF)		//!<Byte transfer finished
#define I2C_ADD10_FLAG		(1 << I2C_SR1_ADD10)	//!<10-bit header sent (master mode)
#define I2C_STOPF_FLAG		(1 << I2C_SR1_STOPF)	//!<Stop detection (slave mode)
#define I2C_RXNE_FLAG 		(1 << I2C_SR1_RXNE)		//!<Buffer not empty (receiver).
#define I2C_TXE_FLAG 		(1 << I2C_SR1_TXE)		//!<Buffer empty (transmitter).
#define I2C_BERR_FLAG		(1 << I2C_SR1_BERR)		//!<Bus error
#define I2C_ARLO_FLAG		(1 << I2C_SR1_ARLO)		//!<Arbitration lost
#define I2C_AF_FLAG			(1 << I2C_SR1_AF)		//!<Acknowledge failure
#define I2C_OVR_FLAG		(1 << I2C_SR1_OVR)		//!<Overrun/underrun
#define I2C_PECERR_FLAG		(1 << I2C_SR1_PECERR)	//!<PEC error in reception
#define I2C_TIMEOUT_FLAG	(1 << I2C_SR1_TIMEOUT)	//!<Timeout error
#define I2C_SMBALERT_FLAG	(1 << I2C_SR1_SMBALERT)	//!<SMBus alert



/*
 * Clock Control
 */
void I2C_ClkCtrl(I2C_RegDef_t *pI2Cx, uint8_t En);

/*
 * Initialization/Deinitialization
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/*
 * Data Send/Receive
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr);

/*
 * IRQ Configuration and ISR Handling
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t En);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);


/*
 * Helper Functions
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t En);
uint32_t RCC_GetPCLK1Value(void);

/*
 * Application Callback
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv);

#endif /* INC_STM32F407XX_I2C_DRV_H_ */
