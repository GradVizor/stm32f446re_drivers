#ifndef INC_STM32F446RE_SPI_DRIVER_H_
#define INC_STM32F446RE_SPI_DRIVER_H_

#include "stm32f446re.h"

/******************************************************************************************
 *					SPI handle structures supported by this driver
 ******************************************************************************************/

typedef struct
{
	uint8_t SPI_DeviceMode;								// possible values from @SPI_DEVICE_MODE_x
	uint8_t SPI_BusConfig;								// possible values from @SPI_BUS_CONFIG_x
	uint8_t SPI_SclkSpeed;								// possible values from @SPI_SCLK_SPEED_DIVx
	uint8_t SPI_DFF;									// possible values from @SPI_DFF_x
	uint8_t SPI_CPOL;									// possible values from @SPI_CPOL_x
	uint8_t SPI_CPHA;									// possible values from @SPI_CPHA_x
	uint8_t SPI_SSM;									// possible values from @SPI_SSM_x
}SPI_Config_t;

typedef struct
{
	SPI_RegDef_t *pSPIx;								// possible values from @SPIx
	SPI_Config_t SPIConfig;								// holds the SPI pin configuration settings
	uint8_t      *pTxBuffer; 							// To store Tx buffer address of the application
	uint8_t      *pRxBuffer; 							// To store Rx buffer address of the application
	uint32_t     TxLen; 								// To store Tx len
	uint32_t     RxLen; 								// To store Rx len
	uint8_t 	 TxState; 								// To store Tx state
	uint8_t 	 RxState;								// To store Rx state
}SPI_Handle_t;


/******************************************************************************************
 *							SPI Macros supported by this driver
 ******************************************************************************************/

// SPI application states :- @SPI_states_x
#define SPI_READY 										0
#define SPI_BUSY_IN_RX 									1
#define SPI_BUSY_IN_TX 									2

// Possible SPI Application events :- @SPI_EVENT_x
#define SPI_EVENT_TX_CMPLT   							1
#define SPI_EVENT_RX_CMPLT   							2
#define SPI_EVENT_OVR_ERR   			 				3
#define SPI_EVENT_CRC_ERR    							4

// SPI Device Modes :- @SPI_DEVICE_MODE_x
#define SPI_DEVICE_MODE_SLAVE							0
#define SPI_DEVICE_MODE_MASTER							1

// SPI Bus Configurations :- @SPI_BUS_CONFIG_x
#define SPI_BUS_CONFIG_FD								1
#define SPI_BUS_CONFIG_HD								2
#define SPI_BUS_CONFIG_SIMPLY_RXONLY					3

// SPI Serial Clock Speeds :- @SPI_SCLK_SPEED_DIVx
#define SPI_SCLK_SPEED_DIV2								0
#define SPI_SCLK_SPEED_DIV4								1
#define SPI_SCLK_SPEED_DIV8								2
#define SPI_SCLK_SPEED_DIV16							3
#define SPI_SCLK_SPEED_DIV32							4
#define SPI_SCLK_SPEED_DIV64							5
#define SPI_SCLK_SPEED_DIV128							6
#define SPI_SCLK_SPEED_DIV256							7

// SPI DFF values :- @SPI_DFF_x
#define SPI_DFF_8BITS									0
#define SPI_DFF_16BITS									1

// SPI CPOL values :- @SPI_CPOL_x
#define SPI_CPOL_LOW									0
#define SPI_CPOL_HIGH									1

// SPI CPHA values :- @SPI_CPHA_x
#define SPI_CPHA_LOW									0
#define SPI_CPHA_HIGH									1

// SPI SSM values :- @SPI_SSM_x
#define SPI_SSM_DI										0
#define SPI_SSM_EN										1

// SPI related status flag definitions :- @SPI_x_FLAG
#define SPI_TXE_FLAG 									(1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG 									(1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG 									(1 << SPI_SR_BSY)


/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/

// Peripheral clock setup
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

// Init
void SPI_Init(SPI_Handle_t *pSPIHandle);

// Data send and receive
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);

// IRQ configuration and ISR handling
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

// Miscellaneous APIs
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv);

#endif /* INC_STM32F446RE_SPI_DRIVER_H_ */
