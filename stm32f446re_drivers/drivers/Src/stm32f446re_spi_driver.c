#include "stm32f446re_spi_driver.h"

/******************************************************************************************
 *
 *				SPI APIS with their full information supported by this driver
 *
 ******************************************************************************************/


/************************ Prototypes of some helper functions used ************************/
static void  spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void  spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void  spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);


/******************************* SPI Peripheral clock control *****************************/

/******************************************************************************************
 * @fn      		  - SPI_PeriClockControl
 *
 * @brief             - use to enable the peripheral clock
 *
 * @param[in]         - spi port that is in use :- @SPI_RegDef_t
 * @param[in]         - value to enable the clock "ENABLE"
 *
 * @return            - nil
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		} else if (pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		} else if (pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		} else if (pSPIx == SPI4)
		{
			SPI4_PCLK_EN();
		}
}


/******************************************   Init     ************************************/

/******************************************************************************************
* @fn      		  - SPI_Init
*
* @brief          - use to initialize the peripheral with the necessary register sets.
*
* @param[in]      - necessary details to initialize an spi port :- @SPI_Handle_t
*
* @return         - nil
*/
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	uint32_t tempreg = 0;

	//1. Configuring the device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << 2;

	//2. Configuring the bus-config
	if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		tempreg &= ~(1 << 15);  // BIDI mode must be cleared
	}else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		tempreg |= (1 << 15);  // BIDI mode must be set
	}else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLY_RXONLY)
	{
		tempreg &= ~(1 << 15);  // BIDI mode should be cleared
		tempreg |= (1 << 10);  // RXONLY bit must be set
	}

	//3. Configuring the SPI serial clock speed (baud rate)
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << 3;

	//4. Configuring the DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << 11;

	//5. Configuring the CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << 1;

	//6. Configuring the CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << 0;

	//7. Configuring the SPI_CR1 register
	pSPIHandle->pSPIx->CR1 = tempreg;
}


/*********************************** Data Send and Receive ********************************/

/******************************************************************************************
* @fn      		  - SPI_SendData
*
* @brief          - use to send data over spi protocol
*
* @param[in]      - spi port that is in use :- @SPI_RegDef_t
* @param[in]      - value of TxBuffer
* @param[in]      - length of data
*
* @return         - nil
*/
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	 while(Len>0)
	 {
		 // 1. Waiting until TXE is set
		 while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		 // 2. Checking the DFF bit in CR1
		 if (pSPIx->CR1 & (1 << SPI_CR1_DFF))
		 {	 // 16 bit DFF
			 //3. Load the data in to the DR
			 pSPIx->DR = *((uint16_t*)pTxBuffer);
			 Len--;
			 Len--;
			 (uint16_t*)pTxBuffer++;
		 }else
		 {	 // 8 bit DFF
			 //3. Loading the data in to the DR
			 pSPIx->DR = *pTxBuffer;
			 Len--;
			 pTxBuffer++;
		 }
	 }
}

/******************************************************************************************
* @fn      		  - SPI_ReceiveData
*
* @brief          - use to receive data over spi protocol
*
* @param[in]      - spi port that is in use :- @SPI_RegDef_t
* @param[in]      - value of RxBuffer
* @param[in]      - length of data
*
* @return         - nil
*/
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	 while(Len>0)
	 {
		 // 1. Waiting until RXE is set
		 while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

		 // 2. Checking the DFF bit in CR1
		 if (pSPIx->CR1 & (1 << SPI_CR1_DFF))
		 {	 // 16 bit DFF
			 //3. Reading the data from DR to RxBuffer
			 *((uint16_t*)pRxBuffer) = pSPIx->DR;
			 Len--;
			 Len--;
			 (uint16_t*)pRxBuffer++;
		 }else
		 {	 // 8 bit DFF
			 //3. Reading the data from DR to RxBuffer
		 }
			 *((uint16_t*)pRxBuffer) = pSPIx->DR;
			 Len--;
			 pRxBuffer++;
		 }
	 }
}

/******************************************************************************************
 * @fn      	   - SPI_SendDataIT
 *
 * @brief          - use to send data over spi protocol in interrupt mode
 *
 * @param[in]      - necessary details to initialize an spi port :- @SPI_Handle_t
 * @param[in]      - value of TxBuffer
 * @param[in]      - length of data
 *
 * @return         - application state :- @SPI_states_x
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t  state = pSPIHandle->TxState;
	if (state != SPI_BUSY_IN_RX)
	{
		//1. Saving the Tx Buffer address and Len info in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;
		//2. Marking the SPI state ad busy in transmission so that no other code can take over
		// 	 same SPI Peripheral until transmission is over.
		pSPIHandle->TxState = SPI_BUSY_IN_TX;
		//3. Enabling the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
	}
	return state;
}

/******************************************************************************************
 * @fn      		  - SPI_ReceiveDataIT
 *
 * @brief             - use to receive data over spi protocol in interrupt mode
 *
 * @param[in]         - necessary details to initialize an spi port :- @SPI_Handle_t
 * @param[in]         - value of TxBuffer
 * @param[in]      	  - length of data
 *
 * @return            - application states :- @SPI_states_x
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t  state = pSPIHandle->RxState;
	if (state != SPI_BUSY_IN_TX)
	{
		//1. Saving the Tx Buffer address and Len info in some global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;
		//2. Marking the SPI state as busy in transmission so that no other code can take over
		// 	 on same SPI Peripheral until transmission is over.
		pSPIHandle->RxState = SPI_BUSY_IN_RX;
		//3. Enabling the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
	}
	return state;
}


/*************************** IRQ Configuration and ISR Handling ***************************/

/******************************************************************************************
 * @fn      		  - SPI_IRQInterruptConfig
 *
 * @brief             - use to configure the interrupts
 *
 * @param[in]         - IRQ number corresponding to your interrupt
 * @param[in]         - possible values "ENABLE" or "DISABLE"
 *
 * @return            - nil
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (IRQNumber <= 31)
		{
			*NVIC_ISER0 |= (1<< IRQNumber);  // Programming ISER0 register
		}else if (IRQNumber > 31 && IRQNumber < 64)  // 32 to 63
		{
			*NVIC_ISER1 |= (1<< (IRQNumber%32));  // Programming ISER1 register
		}else if (IRQNumber >= 64 && IRQNumber < 96) // 64 to 95
		{
			*NVIC_ISER2 |= (1<< (IRQNumber%32));  // Programming ISER2 register
		}
	}else
	{
		if (IRQNumber <= 31)
		{
			*NVIC_ICER0 |= (1<< IRQNumber);  // Programming ICER0 register
		} else if (IRQNumber > 31 && IRQNumber < 64)  // 32 to 63
		{
			*NVIC_ICER1 |= (1<< (IRQNumber%32));  // Programming ICER1 register
		} else if (IRQNumber >= 64 && IRQNumber < 96)  // 64 to 95
		{
			*NVIC_ICER2 |= (1<< (IRQNumber%32));  // Programming ICER2 register
		}
	}
}

/******************************************************************************************
 * @fn      		  - SPI_IRQPriorityConfig
 *
 * @brief             - use to configure the priority for interrupt
 *
 * @param[in]         - IRQ number corresponding to the interrupt
 * @param[in]         - value to set the priority of the interrupt
 *
 * @return            - nil
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}

/******************************************************************************************
 * @fn      		  - SPI_IRQHandling
 *
 * @brief             - use to configure the irq handling.
 *
 * @param[in]         - necessary details to initialize an spi port :- @SPI_Handle_t
 *
 * @return            - nil
 */
void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	uint8_t temp1, temp2;

	// Checking for TXE
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	if (temp1 && temp2)
	{
		spi_txe_interrupt_handle(pHandle);  // handle TXE
	}

	// Checking for RXNE
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);
	if (temp1 && temp2)
	{
		spi_rxne_interrupt_handle(pHandle);  // handle RXNE
	}

	// Checking for OVR error
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);
	if (temp1 && temp2)
	{
		spi_ovr_err_interrupt_handle(pHandle);  // handle OVR
	}
}


/************************************ Miscellaneous APIs **********************************/

/******************************************************************************************
 * @fn      		  - SPI_PeripheralControl
 *
 * @brief             - use to enable the peripheral clock
 *
 * @param[in]         - spi port that is in use :- @SPI_RegDef_t
 * @param[in]         - value to enable the clock "ENABLE"
 *
 * @return            - nil
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

/******************************************************************************************
 * @fn      		  - SPI_SSIConfig
 *
 * @brief             - use to enable the peripheral clock
 *
 * @param[in]         - spi port that is in use :- @SPI_RegDef_t
 * @param[in]         - value to enable the clock "ENABLE"
 *
 * @return            - nil
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

/******************************************************************************************
 * @fn      		  - SPI_SSOEConfig
 *
 * @brief             - use to enable the peripheral clock
 *
 * @param[in]         - spi port that is in use :- @SPI_RegDef_t
 * @param[in]         - value to enable the clock "ENABLE"
 *
 * @return            - nil
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR2_SSOE);
	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR2_SSOE);
	}
}

/******************************************************************************************
 * @fn      		  - SPI_GetFlagStatus
 *
 * @brief             - use to get the status of any flag
 *
 * @param[in]         - spi port that is in use :- @SPI_RegDef_t
 * @param[in]         - name of the predefined flag
 *
 * @return            - current status of flag
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/******************************************************************************************
 * @fn      		  - SPI_CloseTransmission
 *
 * @brief             - use to close the transmission over spi port
 *
 * @param[in]         - necessary details to initialize an spi port :- @SPI_Handle_t
 *
 * @return            - nil
 */
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}

/******************************************************************************************
 * @fn      		  - SPI_CloseReception
 *
 * @brief             - use to close the reception over spi port
 *
 * @param[in]         - necessary details to initialize an spi port :- @SPI_Handle_t
 *
 * @return            - nil
 */
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}

/******************************************************************************************
 * @fn      		  - SPI_ClearOVRFlag
 *
 * @brief             - use to clear the OVR flag
 *
 * @param[in]         - spi port that is in use :- @SPI_RegDef_t
 *
 * @return            - nil
 */
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}

/******************************************************************************************
 * @fn      		  - spi_txe_interrupt_handle
 *
 * @brief             - use to handle TXE
 *
 * @param[in]         - necessary details to initialize an spi port :- @SPI_Handle_t
 *
 * @return            - nil
 */
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//1. Checking the DFF bit in CR1
	if (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
	{	// 16 bit DFF
		//2. Loading the data in to the DR
		pSPIHandle->pSPIx->DR = *((uint16_t*) pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*) pSPIHandle->pTxBuffer ++;
	} else
	{	// 8 bit DFF
		//2. Loading the data in to the DR
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer ++;
	}
	if (!pSPIHandle->TxLen--)
	{
		// TxLen is zero, so close the SPI transmission and inform that Tx is over.
		// This prevents interrupts from setting up of TXE flag.
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}

/******************************************************************************************
 * @fn      		  - spi_rxne_interrupt_handle
 *
 * @brief             - use to handle RXNE
 *
 * @param[in]         - necessary details to initialize an spi port :- @SPI_Handle_t
 *
 * @return            - nil
 */
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	// RXing as per the DFF
	if(pSPIHandle->pSPIx->CR1 & ( 1 << 11))
	{
		// 16 bit
		*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t) pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen -= 2;
		pSPIHandle->pRxBuffer++;
		pSPIHandle->pRxBuffer++;

	}else
	{
		// 8 bit
		*(pSPIHandle->pRxBuffer) = (uint8_t) pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}

	if(! pSPIHandle->RxLen)
	{
		// Reception is complete
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);
	}
}

/******************************************************************************************
 * @fn      		  - spi_ovr_err_interrupt_handle
 *
 * @brief             - use to handle OVR
 *
 * @param[in]         - necessary details to initialize an spi port :- @SPI_Handle_t
 *
 * @return            - nil
 */
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;
	//1. Clearing the OVR flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;

	//2. Informing the application
	SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{
	// This is a weak implementation. the user application may override this function.
}
