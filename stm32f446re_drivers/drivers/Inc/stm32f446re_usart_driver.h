#ifndef STM32F446RE_USART_DRIVER_H_
#define STM32F446RE_USART_DRIVER_H_

#include "stm32f446re.h"

/******************************************************************************************
 *					USART handle structures supported by this driver
 ******************************************************************************************/

typedef struct
{
	uint8_t  USART_Mode;								// possible values from @USART_MODE_x
	uint32_t USART_Baud;								// possible values from @USART_STD_BAUD_x
	uint8_t  USART_NoOfStopBits;						// possible values from @USART_STOPBITS_x
	uint8_t  USART_WordLength;							// possible values from @USART_WORDLEN_x
	uint8_t  USART_ParityControl;						// possible values from @USART_PARITY_x
	uint8_t  USART_HWFlowControl;						// possible values from @USART_HW_FLOW_CTRL_x
}USART_Config_t;

typedef struct
{
	USART_RegDef_t  *pUSARTx;							// possible values from @USARTx
	USART_Config_t  USART_Config;						// holds the USART pin configuration settings
	uint8_t         *pTxBuffer;							// To store Tx buffer address of the application
	uint8_t         *pRxBuffer;							// To store Rx buffer address of the application
	uint32_t 		TxLen;								// To store Tx len
	uint32_t 		RxLen;								// To store Rx len
	uint8_t 		TxBusyState;						// To store TxBusystate
	uint8_t 		RxBusyState;						// To store RxBusystate
}USART_Handle_t;


/******************************************************************************************
 *							SPI Macros supported by this driver
 ******************************************************************************************/

// USART possible modes :- @USART_MODE_x
#define USART_MODE_ONLY_TX 								0
#define USART_MODE_ONLY_RX 								1
#define USART_MODE_TXRX  								2

// USART possible baud rates :- @USART_STD_BAUD_x
#define USART_STD_BAUD_1200								1200
#define USART_STD_BAUD_2400								400
#define USART_STD_BAUD_9600								9600
#define USART_STD_BAUD_19200 							19200
#define USART_STD_BAUD_38400 							38400
#define USART_STD_BAUD_57600 							57600
#define USART_STD_BAUD_115200 							115200
#define USART_STD_BAUD_230400 							230400
#define USART_STD_BAUD_460800 							460800
#define USART_STD_BAUD_921600 							921600
#define USART_STD_BAUD_2M 								2000000
#define USART_STD_BAUD_3M 								3000000

// USART possible options for parity controls:- @USART_PARITY_x
#define USART_PARITY_EN_ODD   							2
#define USART_PARITY_EN_EVEN  							1
#define USART_PARITY_DISABLE   							0

// USART possible options for word lengths :- @USART_WORDLEN_x
#define USART_WORDLEN_8BITS  							0
#define USART_WORDLEN_9BITS  							1

// USART possible options for stop bits :- @USART_STOPBITS_x
#define USART_STOPBITS_1     							0
#define USART_STOPBITS_0_5   							1
#define USART_STOPBITS_2     							2
#define USART_STOPBITS_1_5   							3

// USART possible values for flow control :- @USART_HW_FLOW_CTRL_x
#define USART_HW_FLOW_CTRL_NONE    						0
#define USART_HW_FLOW_CTRL_CTS    						1
#define USART_HW_FLOW_CTRL_RTS    						2
#define USART_HW_FLOW_CTRL_CTS_RTS						3

// USART possible values for flags :- @USART_FLAG_x
#define USART_FLAG_TXE 									(1 << USART_SR_TXE)
#define USART_FLAG_RXNE 								(1 << USART_SR_RXNE)
#define USART_FLAG_TC 									(1 << USART_SR_TC)

// USART application states :- @USART_states_x
#define USART_BUSY_IN_RX 								1
#define USART_BUSY_IN_TX 								2
#define USART_READY 									0

// USART possible events :- @USART_EVENT_x
#define USART_EVENT_TX_CMPLT   							0
#define	USART_EVENT_RX_CMPLT   							1
#define	USART_EVENT_IDLE      							2
#define	USART_EVENT_CTS       							3
#define	USART_EVENT_PE        							4
#define	USART_ERR_FE     								5
#define	USART_ERR_NE    	 							6
#define	USART_ERR_ORE    								7


/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/

// Peripheral clock setup
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi);

// Init
void USART_Init(USART_Handle_t *pUSARTHandle);

// Data Send and Receive
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);
void  USART_ReceiveData(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t Len);
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len);
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t Len);

// IRQ Configuration and ISR handling
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void USART_IRQHandling(USART_Handle_t *pUSARTHandle);

// Miscellaneous APIs
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t StatusFlagName);
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName);
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi);
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate);
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t ApEv);

#endif /* STM32F446RE_USART_DRIVER_H_ */
