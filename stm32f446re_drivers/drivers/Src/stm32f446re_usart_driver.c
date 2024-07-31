#include "stm32f446re_usart_driver.h"

/******************************************************************************************
 *
 *				USART APIS with their full information supported by this driver
 *
 ******************************************************************************************/


/************************ Prototypes of some helper functions used ************************/
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate);
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName);


/****************************** Peripheral clock control ***************************/

/******************************************************************************************
 * @fn      		  - USART_PeriClockControl
 *
 * @brief             - use to enable the peripheral clock
 *
 * @param[in]         - usart port that is in use :- @USART_RegDef_t
 * @param[in]         - value to enable the clock "ENABLE"
 *
 * @return            - nil
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pUSARTx == USART1)
		{
			USART1_PCCK_EN();
		}else if (pUSARTx == USART2)
		{
			USART2_PCCK_EN();
		}else if (pUSARTx == USART3)
		{
			USART3_PCCK_EN();
		}
		else if (pUSARTx == UART4)
		{
			UART4_PCCK_EN();
		}
	}
	else
	{
		//TODO
	}

}


/******************************************   Init     ************************************/

/******************************************************************************************
 * @fn      		  - USART_Init
 *
 * @brief             - use to initialize the peripheral with the necessary register sets.
 *
 * @param[in]         - necessary details to initialize a usart port :- @USART_Handle_t
 *
 * @return            - nil
 */
void USART_Init(USART_Handle_t *pUSARTHandle)
{
	uint32_t tempreg=0;


	//1. Configuring CR1 register
	USART_PeriClockControl(pUSARTHandle->pUSARTx,ENABLE);
	if ( pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
	{
		tempreg|= (1 << USART_CR1_RE);
	}else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
	{
		tempreg |= ( 1 << USART_CR1_TE );

	}else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
	{
		tempreg |= ( ( 1 << USART_CR1_RE) | ( 1 << USART_CR1_TE) );
	}
	tempreg |= pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M ;
	if ( pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
	{
		tempreg |= ( 1 << USART_CR1_PCE);
		// Implementing the code to enable EVEN parity
		// Not required because by default EVEN parity will be selected once you enable the parity control
	}else if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD )
	{
	    tempreg |= ( 1 << USART_CR1_PCE);
	    tempreg |= ( 1 << USART_CR1_PS);
	}
	pUSARTHandle->pUSARTx->CR1 = tempreg;

	//2. Configuring CR2 register
	tempreg=0;
	tempreg |= pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP;
	pUSARTHandle->pUSARTx->CR2 = tempreg;

	//3. Configuring CR3 register
	tempreg=0;
	if ( pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
	{
		tempreg |= ( 1 << USART_CR3_CTSE);
	}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
	{
		tempreg |= ( 1 << USART_CR3_RTSE);

	}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		tempreg |= ( 1 << USART_CR3_CTSE);
		tempreg |= ( 1 << USART_CR3_RTSE);
	}
	pUSARTHandle->pUSARTx->CR3 = tempreg;

	//4. Configuring BRR register
	USART_SetBaudRate(pUSARTHandle->pUSARTx,pUSARTHandle->USART_Config.USART_Baud);
}


/*********************************** Data Send and Receive ********************************/

/******************************************************************************************
 * @fn      		  - USART_SendData
 *
 * @brief             - use to send data over usart protocol
 *
 * @param[in]         - necessary details to initialize a usart port :- @USART_Handle_t
 * @param[in]      	  - value of TxBuffer
 * @param[in]         - length of data
 *
 * @return            - nil
 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint16_t *pdata;
   //Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		//Implement the code to wait until TXE flag is set in the SR
		while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TXE));

		//Check the USART_WordLength item for 9BIT or 8BIT in a frame
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//if 9BIT load the DR with 2bytes masking  the bits other than first 9 bits
			pdata = (uint16_t*) pTxBuffer;
			pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

			//check for USART_ParityControl
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used in this transfer , so 9bits of user data will be sent
				//Implement the code to increment pTxBuffer twice
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				//Parity bit is used in this transfer . so 8bits of user data will be sent
				//The 9th bit will be replaced by parity bit by the hardware
				pTxBuffer++;
			}
		}
		else
		{
			//This is 8bit data transfer
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer  & (uint8_t)0xFF);
			//Implement the code to increment the buffer address
			pTxBuffer++;
		}
	}
	//Implement the code to wait till TC flag is set in the SR
	while( ! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TC));
}

/******************************************************************************************
 * @fn      		  - USART_ReceiveData
 *
 * @brief             - use to receive data over usart protocol
 *
 * @param[in]         - necessary details to initialize a usart port :- @USART_Handle_t
 * @param[in]         - value of RxBuffer
 * @param[in]      	  - length of data
 *
 * @return            - nil
 */
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
   //Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		//Implement the code to wait until RXNE flag is set in the SR
		while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_RXNE));
		//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//We are going to receive 9bit data in a frame
			//Now, check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used , so all 9bits will be of user data

				//read only first 9 bits so mask the DR with 0x01FF
				*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);

				//Now increment the pRxBuffer two times
				pRxBuffer++;
				pRxBuffer++;
			}
			else
			{
				//Parity is used, so 8bits will be of user data and 1 bit is parity
				 *pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
				 pRxBuffer++;
			}
		}
		else
		{
			//We are going to receive 8bit data in a frame
			//Now, check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used , so all 8bits will be of user data
				//read 8 bits from DR
				 *pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
			}

			else
			{
				//Parity is used, so , 7 bits will be of user data and 1 bit is parity
				//read only 7 bits , hence mask the DR with 0X7F
				 *pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0x7F);
			}
			pRxBuffer++; //Now , increment the pRxBuffer
		}
	}
}

/******************************************************************************************
 * @fn      		  - USART_SendDataIT
 *
 * @brief             - use to send data over usart protocol in interrupt mode
 *
 * @param[in]         - necessary details to initialize a usart port :- @USART_Handle_t
 * @param[in]         - value of TxBuffer
 * @param[in]      	  - length of data
 *
 * @return            - application state :- @USART_states_x
 */
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pUSARTHandle->TxBusyState;

	if(state != USART_BUSY_IN_TX)
	{
		pUSARTHandle->TxLen = Len;
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;
		//Implement the code to enable interrupt for TXE
		pUSARTHandle->pUSARTx->CR1 |= ( 1 << USART_CR1_TXEIE);
		//Implement the code to enable interrupt for TC
		pUSARTHandle->pUSARTx->CR1 |= ( 1 << USART_CR1_TCIE);
	}
	return state;
}

/******************************************************************************************
 * @fn      		  - USART_ReceiveDataIT
 *
 * @brief             - use to receive data over usart protocol in interrupt mode
 *
 * @param[in]         - necessary details to initialize a usart port :- @USART_Handle_t
 * @param[in]         - value of RxBuffer
 * @param[in]      	  - length of data
 *
 * @return            - application state :- @USART_states_x
 */
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pUSARTHandle->RxBusyState;
	if(state != USART_BUSY_IN_RX)
	{
		pUSARTHandle->RxLen = Len;
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;
		(void)pUSARTHandle->pUSARTx->DR;
		pUSARTHandle->pUSARTx->CR1 |= ( 1 << USART_CR1_RXNEIE); // Enabling interrupt for RXNE
	}
	return state;
}


/*************************** IRQ Configuration and ISR Handling ***************************/

/******************************************************************************************
 * @fn      		  - USART_IRQInterruptConfig
 *
 * @brief             - use to configure the interrupts
 *
 * @param[in]         - IRQ number corresponding to your interrupt
 * @param[in]         - possible values "ENABLE" or "DISABLE"
 *
 * @return            - nil
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			*NVIC_ISER0 |= ( 1 << IRQNumber ); //program ISER0 register
		}else if(IRQNumber > 31 && IRQNumber < 64 ) //32 to 63
		{
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) ); //program ISER1 register
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 ) //64 to 95
		{
			*NVIC_ISER3 |= ( 1 << (IRQNumber % 64) ); //program ISER2 register
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			*NVIC_ICER0 |= ( 1 << IRQNumber ); //program ICER0 register
		}else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) ); //program ICER1 register
		}
		else if(IRQNumber >= 6 && IRQNumber < 96 )
		{
			*NVIC_ICER3 |= ( 1 << (IRQNumber % 64) ); //program ICER2 register
		}
	}
}

/******************************************************************************************
 * @fn      		  - USART_IRQPriorityConfig
 *
 * @brief             - use to configure the priority for interrupt
 *
 * @param[in]         - IRQ number corresponding to the interrupt
 * @param[in]         - value to set the priority of the interrupt
 *
 * @return            - nil
 */
void USART_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber %4 ;
	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;
	*(NVIC_PR_BASE_ADDR + iprx) |=  (IRQPriority << shift_amount);
}

/*********************************************************************
 * @fn      		  - USART_IRQHandling
 *
 * @brief             - use to configure the irq handling.
 *
 * @param[in]         - necessary details to initialize a usart port :- @USART_Handle_t
 *
 * @return            - nil
 */
void USART_IRQHandling(USART_Handle_t *pUSARTHandle)
{
	uint32_t temp1 , temp2, temp3;
	uint16_t *pdata;
	//1. Checking for TC flag
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_TC); // Checking the state of TC bit in the SR
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_TCIE); // Checking the state of TCEIE bit
	if(temp1 && temp2 )
	{
		// Closing transmission and call application callback if TxLen is zero
		if ( pUSARTHandle->TxBusyState == USART_BUSY_IN_TX)
		{
			// Checking the TxLen. (If it is zero then close the data transmission)
			if(! pUSARTHandle->TxLen )
			{
				// Clearing the TC flag
				pUSARTHandle->pUSARTx->SR &= ~( 1 << USART_SR_TC);
				// Clearing the TCIE control bit
				pUSARTHandle->TxBusyState = USART_READY; // Reset the application state
				pUSARTHandle->pTxBuffer = NULL; // Reset Buffer address to NULL
				pUSARTHandle->TxLen = 0; // Reset the length to zero
				// Call the application call back with event USART_EVENT_TX_CMPLT
				USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_TX_CMPLT);
			}
		}
	}

	//2. Checking for TXE flag
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_TXE); // Checking the state of TXE bit in the SR
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_TXEIE); // Checking the state of TXEIE bit in CR1
	if(temp1 && temp2 )
	{
		if(pUSARTHandle->TxBusyState == USART_BUSY_IN_TX)
		{
			// Keep sending data until Txlen reaches to zero
			if(pUSARTHandle->TxLen > 0)
			{
				// Check the USART_WordLength item for 9BIT or 8BIT in a frame
				if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
				{
					// if 9BIT load the DR with 2bytes masking  the bits other than first 9 bits
					pdata = (uint16_t*) pUSARTHandle->pTxBuffer;
					pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

					// Check for USART_ParityControl
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//No parity is used in this transfer , so 9bits of user data will be sent
						//Implement the code to increment pTxBuffer twice
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->TxLen-=2;
					}
					else
					{
						//Parity bit is used in this transfer . so 8bits of user data will be sent
						//The 9th bit will be replaced by parity bit by the hardware
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->TxLen-=1;
					}
				}
				else
				{
					//This is 8bit data transfer
					pUSARTHandle->pUSARTx->DR = (*pUSARTHandle->pTxBuffer  & (uint8_t)0xFF);
					//Implement the code to increment the buffer address
					pUSARTHandle->pTxBuffer++;
					pUSARTHandle->TxLen-=1;
				}
			}
			if (pUSARTHandle->TxLen == 0 )
			{
				//TxLen is zero
				//Implement the code to clear the TXEIE bit (disable interrupt for TXE flag )
				pUSARTHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_TXEIE);
			}
		}
	}

	//3. Checking for RXNE flag
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_RXNE);
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_RXNEIE);
	if(temp1 && temp2 )
	{
		if(pUSARTHandle->RxBusyState == USART_BUSY_IN_RX)
		{
			if(pUSARTHandle->RxLen > 0)
			{
				//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
				if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
				{
					//We are going to receive 9bit data in a frame

					//Now, check are we using USART_ParityControl control or not
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//No parity is used , so all 9bits will be of user data
						//read only first 9 bits so mask the DR with 0x01FF
						*((uint16_t*) pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);

						//Now increment the pRxBuffer two times
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->RxLen-=2;
					}
					else
					{
						//Parity is used, so 8bits will be of user data and 1 bit is parity
						 *pUSARTHandle->pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
						 pUSARTHandle->pRxBuffer++;
						 pUSARTHandle->RxLen-=1;
					}
				}
				else
				{
					//We are going to receive 8bit data in a frame
					//Now, check are we using USART_ParityControl control or not
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//No parity is used , so all 8bits will be of user data
						//read 8 bits from DR
						 *pUSARTHandle->pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
					}
					else
					{
						//Parity is used, so , 7 bits will be of user data and 1 bit is parity
						//read only 7 bits , hence mask the DR with 0X7F
						 *pUSARTHandle->pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0x7F);
					}
					//Now , increment the pRxBuffer
					pUSARTHandle->pRxBuffer++;
					pUSARTHandle->RxLen-=1;
				}
			}//if of >0

			if(! pUSARTHandle->RxLen)
			{
				//disable the rxne
				pUSARTHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_RXNEIE );
				pUSARTHandle->RxBusyState = USART_READY;
				USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_RX_CMPLT);
			}
		}
	}

	//4. Check for CTS flag
	//Note : CTS feature is not applicable for UART4 and UART5
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_CTS); // Checking the status of CTS bit in the SR
	temp2 = pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_CTSE); // Checking the state of CTSE bit in CR1
	temp3 = pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_CTSIE); // Checking the state of CTSIE bit in CR3 (not available for UART4 & UART5.)

	if(temp1  && temp2 )
	{
		pUSARTHandle->pUSARTx->SR &=  ~( 1 << USART_SR_CTS); // Clearing the CTS flag in SR
		USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_CTS); // This interrupt is because of cts
	}

	//5. Checking for IDLE detection flag
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_IDLE); // Checking the status of IDLE flag bit in the SR
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_IDLEIE); // Checking the state of IDLEIE bit in CR1
	if(temp1 && temp2)
	{
		temp1 = pUSARTHandle->pUSARTx->SR &= ~( 1 << USART_SR_IDLE); // Clearing the IDLE flag
		USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_IDLE); // This interrupt is because of idle
	}

	//6. Check for Overrun detection flag
	temp1 = pUSARTHandle->pUSARTx->SR & USART_SR_ORE; // Checking the status of ORE flag  in the SR
	temp2 = pUSARTHandle->pUSARTx->CR1 & USART_CR1_RXNEIE; // Checking the status of RXNEIE  bit in the CR1
	if(temp1  && temp2 )
	{
		//Need not to clear the ORE flag here, instead give an api for the application to clear the ORE flag .
		USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_ORE); // This interrupt is because of Overrun error
	}

	//7. Checking for Error Flag
	// Noise Flag, Overrun error and Framing Error in multi-buffer communication
	// We don't discuss multi-buffer communication in this course. please refer to the RM
	// The blow code will get executed in only if multi-buffer mode is used.
	temp2 =  pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_EIE) ;
	if(temp2 )
	{
		temp1 = pUSARTHandle->pUSARTx->SR;
		if(temp1 & ( 1 << USART_SR_FE))
		{
			/*
				This bit is set by hardware when a de-synchronization, excessive noise or a break character
				is detected. It is cleared by a software sequence (an read to the USART_SR register
				followed by a read to the USART_DR register).
			*/
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_FE);
		}

		if(temp1 & ( 1 << USART_SR_NE) )
		{
			/*
				This bit is set by hardware when noise is detected on a received frame. It is cleared by a
				software sequence (an read to the USART_SR register followed by a read to the
				USART_DR register).
			*/
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_NE);
		}

		if(temp1 & ( 1 << USART_SR_ORE) )
		{
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_ORE);
		}
	}
}


/************************************ Miscellaneous APIs **********************************/

/******************************************************************************************
 * @fn      		  - USART_PeripheralControl
 *
 * @brief             - use to enable the peripheral clock
 *
 * @param[in]         - usart port that is in use :- @USART_RegDef_t
 * @param[in]         - possible values "ENABLE" or "DISABLE"
 *
 * @return            - nil
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t Cmd)
{
	if(Cmd == ENABLE)
	{
		pUSARTx->CR1 |= (1 << 13);
	}else
	{
		pUSARTx->CR1 &= ~(1 << 13);
	}
}

/******************************************************************************************
 * @fn      		  - USART_GetFlagStatus
 *
 * @brief             - use to get the status of any flag
 *
 * @param[in]         - usart port that is in use :- @USART_RegDef_t
 * @param[in]         - name of the predefined flag :- @USART_FLAG_x
 *
 * @return            - current state of the flag
 */
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t StatusFlagName)
{
    if(pUSARTx->SR & StatusFlagName)
    {
    	return SET;
    }
   return RESET;
}

/******************************************************************************************
 * @fn      		  - USART_ClearFlag
 *
 * @brief             - use to enable the peripheral clock
 *
 * @param[in]         - usart port that is in use :- @USART_RegDef_t
 * @param[in]         - name of the predefined flag :- @USART_FLAG_x
 *
 * @return            - nil
 */
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName)
{
	pUSARTx->SR &= ~(StatusFlagName);
}

/******************************************************************************************
 * @fn      		  - USART_SetBaudRate
 *
 * @brief             - use to set the rate of data transfer
 *
 * @param[in]         - usart port that is in use :- @USART_RegDef_t
 * @param[in]         - rate at which data is transfered :- @USART_STD_BAUD_x
 *
 * @return            - nil
 */
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{
	uint32_t PCLKx, usartdiv, M_part, F_part;
	uint32_t tempreg=0;

	// Get the value of APB bus clock in to the variable PCLKx
	if (pUSARTx == USART1 || pUSARTx == USART6)
	{
		//USART1 and USART6 are hanging on APB2 bus
		PCLKx = RCC_GetPCLK2Value();
	} else
	{
		PCLKx = RCC_GetPCLK1Value();
	}

	// Checking for OVER8 configuration bit
	if (pUSARTx->CR1 & (1 << USART_CR1_OVER8))
	{
		usartdiv = ((25 * PCLKx) / (2 * BaudRate)); //OVER8 = 1 , over sampling by 8
	} else
	{
		usartdiv = ((25 * PCLKx) / (4 * BaudRate)); //over sampling by 16
	}

	M_part = usartdiv / 100; // Calculating the Mantissa part
	tempreg |= M_part << 4;	// Placing Mantissa part in appropriate bit position
	F_part = (usartdiv - (M_part * 100)); // Extracting the fraction part

	// Calculating the final fractional
	if (pUSARTx->CR1 & (1 << USART_CR1_OVER8))
	{
		F_part = (((F_part * 8) + 50) / 100) & ((uint8_t) 0x07); //OVER8 = 1 , over sampling by 8

	} else
	{
		F_part = (((F_part * 16) + 50) / 100) & ((uint8_t) 0x0F); //over sampling by 16
	}

	tempreg |= F_part; // Placing fractional part in appropriate bit position.
	pUSARTx->BRR = tempreg;
}

__weak void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t event)
{
	// This is a weak implementation. the user application may override this function.
}
