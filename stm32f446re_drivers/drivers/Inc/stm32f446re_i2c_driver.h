#ifndef INC_STM32F446RE_I2C_DRIVER_H_
#define INC_STM32F446RE_I2C_DRIVER_H_

#include "stm32f446re.h"

/******************************************************************************************
 *					I2C handle structures supported by this driver
 ******************************************************************************************/

typedef struct
{
	uint32_t I2C_SCLSpeed;								// possible values from @I2C_SCL_SPEED_x
	uint8_t  I2C_DeviceAddress;							// To store device address
	uint8_t  I2C_ACKControl;							// possible values from @I2C_ACK_x
	uint16_t I2C_FMDutyCycle;							// possible values from @I2C_FM_DUTY_x
} I2C_Config_t;

typedef struct
{
	I2C_RegDef_t *pI2Cx;								// possible values from @I2Cx
	I2C_Config_t I2CConfig;								// holds the I2C pin configuration settings
	uint8_t 	*pTxBuffer;								// To store Tx buffer address of the application
	uint8_t 	*pRxBuffer;								// To store Rx buffer address of the application
	uint32_t 	TxLen;									// To store Tx length
	uint32_t 	RxLen;									// To store Rx length
	uint8_t 	TxRxState;								// To store Tx-Rx state
	uint8_t 	DevAddr;								// To store device address
	uint32_t 	RxSize;									// To store Rx size
	uint8_t  	Sr;										// To store Sr value
} I2C_Handle_t;


/******************************************************************************************
 *							I2C Macros supported by this driver
 ******************************************************************************************/

// I2C application states :- @I2C_state
#define I2C_READY 										0
#define I2C_BUSY_IN_RX 									1
#define I2C_BUSY_IN_TX 									2

// I2C serial clock speeds:- @I2C_SCL_SPEED_x
#define I2C_SCL_SPEED_SM 								100000
#define I2C_SCL_SPEED_FM4K 								400000
#define I2C_SCL_SPEED_FM2K 								200000

// I2C AckControl :- @I2C_ACK_x
#define I2C_ACK_DISABLE 								0
#define I2C_ACK_ENABLE 									1

// I2C  FM duty cycle :- @I2C_FM_DUTY_x
#define I2C_FM_DUTY_2        							0
#define I2C_FM_DUTY_16_9     							1

// I2C related status flag definitions :- @I2C_FLAG_x
#define I2C_FLAG_TXE 									(1 << I2C_SR1_TXE)
#define I2C_FLAG_RXNE 									(1 << I2C_SR1_RXNE)
#define I2C_FLAG_SB 									(1 << I2C_SR1_SB)
#define I2C_FLAG_ADDR 									(1 << I2C_SR1_ADDR)
#define I2C_FLAG_BTF 									(1 << I2C_SR1_BTF)


/******************************************************************************************
 *							APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/

// Peripheral Clock setup
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

// Init
void I2C_Init(I2C_Handle_t *pI2CHandle);

// Data Send and Receive
void I2C_MasterSendData(I2C_Handle_t *pI2cHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint32_t Len, uint8_t SlaveAddr);

// IRQ Configuration and ISR handling
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

// Miscellaneous APIs
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);
__weak void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv);

#endif /* INC_STM32F446RE_I2C_DRIVER_H_ */
