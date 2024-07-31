#include "stm32f446re.h"

/******************************************************************************************
 *
 *				I2C APIS with their full information supported by this driver
 *
 ******************************************************************************************/

/************************ Prototypes of some helper functions used ************************/
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);


/******************************* I2C Peripheral clock control *****************************/

/******************************************************************************************
 * @fn      		  - I2C_PeriClockControl
 *
 * @brief             - use to enable the peripheral clock
 *
 * @param[in]         - i2c port that is in use :- @I2C_RegDef_t
 * @param[in]         - value to enable the clock "ENABLE"
 *
 * @return            - nil
 */

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		} else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		} else if (pI2Cx == I2C3))
		{
			I2C3_PCLK_EN();
		}
	}
	else
	{
		//TODO
	}

}


/******************************************   Init     ************************************/

/******************************************************************************************
* @fn      		  - I2C_Init
*
* @brief          - use to initialize the peripheral with the necessary register sets.
*
* @param[in]      - necessary details to initialize an i2c port :- @I2C_Handle_t
*
* @return         - nil
*/
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg = 0;
	tempreg = pI2CHandle->I2CConfig.I2C_ACKControl << 10;
	pI2CHandle->pI2Cx->CR1 = tempreg;

	// configure the FREQ field of CR2
	tempreg = 0;
	tempreg = RCC_GetPCLK1Value() / 1000000U;
	pI2CHandle->pI2Cx->CR2 = tempreg;

	// Configure the OAR1
	tempreg |= pI2CHandle->I2CConfig.I2C_DeviceAddress << 1;
	tempreg |= (1<< 14);
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	// CCR Calculations
	uint16_t  ccr_value = 0;
	tempreg = 0;
	if (pI2CHandle->I2CConfig.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		// mode is standard mode'
		ccr_value = (RCC_GetPCLK1Value() / (2 *pI2CHandle->I2CConfig.I2C_SCLSpeed));
		tempreg |= (ccr_value & 0xFFF);
	} else
	{
		// mode is fast mode
		tempreg |= (1 << 15);
		tempreg |= (pI2CHandle->I2CConfig.I2C_FMDutyCycle << 14);
		if (pI2CHandle->I2CConfig.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = (RCC_GetPCLK1Value() / (3 *pI2CHandle->I2CConfig.I2C_SCLSpeed));
		} else
		{
			ccr_value = (RCC_GetPCLK1Value() / (25 *pI2CHandle->I2CConfig.I2C_SCLSpeed));
		}
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	// TRISE Configuration
	if (pI2CHandle->I2CConfig.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		// mode is standard mode
//		uint8_t trise;
//		trise = (RCC_GetPCLK1Value() /1000000U) + 1;
	}
	else
	{
		//mode is fast mode
		tempreg = ((RCC_GetPCLK1Value() * 300) / 1000000000U ) + 1;
	}
	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);
}


/*********************************** Data Send and Receive ********************************/

/******************************************************************************************
* @fn      		  - I2C_MasterSendData
*
* @brief          - use to send data over i2c protocol in master mode
*
* @param[in]      - necessary details to initialize an i2c port :- @I2C_Handle_t
* @param[in]      - value of TxBuffer
* @param[in]      - length of data
* @param[in]      - address of the slave connected
*
* @return         - nil
*/

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr)
{
	// 1. Generating Start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// 2. Confirming that start generation is completed by checking the SB flag in SR1
	//    Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while(I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB));

	// 3. Sending the address of the slave with r/nw bit set to w(0) (total 8 bits)
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx,SlaveAddr);

	// 4. Confirming that address phase is completed by checking the ADDR flag in the SR1
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	// 5. Clearing the ADDR flag according to its software sequence
	//    Note: Until ADDR is cleared SCL will be stretched (pulled to LOW)
	I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

	// 6. Sending the data until Len becomes 0.
	while (Len>0)
	{
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE)); // Wait till TXE is set.
		pI2CHandle->pI2Cx->DR = *pTxbuffer;
		pTxbuffer++;
		Len--;
	}

	// 7. When Len becomes zero wait for TXE=1 and BTF=1 before generating the STOP condition
	//    Note: TXE=1, BTF=1, means that both SR and DR are empty and next transmission should begin
	//    when BTF=1 SCL will be stretched (pulled to LOW).
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));

	// 8. Generating STOP condition and master need not to wait for the completion of stop condition.
	// Note: generating STOP, automatically clears the BTF
	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

}

/******************************************************************************************
* @fn      		  - I2C_MasterReceiveData
*
* @brief          - use to receive data over i2c protocol in master mode
*
* @param[in]      - necessary details to initialize an i2c port :- @I2C_Handle_t
* @param[in]      - value of RxBuffer
* @param[in]      - length of data
* @param[in]      - address of the slave connected
*
* @return         - nil
*/
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint32_t Len, uint8_t SlaveAddr)
{
	// 1. Generate the START condition.
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
	// 2. Confirm that start generation is completed by checking the SB flag in the SR1
	//    Note: Until SB is cleared SCL will be stretched (pulled to LOW).
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, SlaveAddr));
	// 3. Send the address of the slave with r/nw bit set to R(1) (total 8 bits)
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx,SlaveAddr);

	// 4. Wait until address phase is completed by checking the ADDR flag in teh SR1
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	// procedure to read only 1 byte from slave
	if (Len == 1)
	{
		// Disable Acking
		I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

		// Clear the ADDR Flag
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

		// Wait until RXNE becomes 1
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

		// Read data in to buffer
		*pRxbuffer = pI2CHandle->pI2Cx->DR;

		return;
	}

	if (Len > 1)
	{
		// Clear the ADDR Flag
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

		// read the data until Len becomes Zero
		for (uint32_t i = Len; i>0; i--)
		{
			//wait until the RXNE becomes 1
			while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));
			if(i == 2)
			{
				//Disable acking
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
				//generate STOP Condition
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}
			// read the data from data register into buffer
			*pRxbuffer = pI2CHandle->pI2Cx->DR;

			// increment the buffer address
			pRxbuffer++;
		}
	}

	//re-anable acking
	if (pI2CHandle->I2CConfig.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		return;
	}
}

/******************************************************************************************
 * @fn      		  - I2C_ManageAcking
 *
 * @brief             - use to enable or disable acking
 *
 * @param[in]         - i2c port that is in use :- @I2C_RegDef_t
 * @param[in]         - values:- "ENABLE" or "DISABLE"
 *
 * @return            - nil
 */
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == I2C_ACK_ENABLE)
	{
		// enable the ack
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}else
	{
		// disable the ack
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);

	}
}

/************************************ Miscellaneous APIs **********************************/

/******************************************************************************************
 * @fn      		  - I2C_GetFlagStatus
 *
 * @brief             - use to get the status of any flag
 *
 * @param[in]         - i2c port that is in use :- @I2C_RegDef_t
 * @param[in]         - name of the predefined flag
 *
 * @return            - current status of flag
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	if(pI2Cx->SR1 & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/******************************************************************************************
 * @fn      		  - I2C_PeripheralControl
 *
 * @brief             - use to enable the peripheral clock
 *
 * @param[in]         - i2c port that is in use :- @I2C_RegDef_t
 * @param[in]         - values :- "ENABLE" or "DISABLE"
 *
 * @return            - nil
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
		//pI2cBaseAddress->CR1 |= I2C_CR1_PE_Bit_Mask;
	}else
	{
		pI2Cx->CR1 &= ~(1 << 0);
	}
}

/******************************************************************************************
 * @fn      		  - I2C_GenerateStartCondition
 *
 * @brief             - use to generate the start condition
 *
 * @param[in]         - i2c port that is in use :- @I2C_RegDef_t
 *
 * @return            - nil
 */
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

/******************************************************************************************
 * @fn      		  - I2C_ExecuteAddressPhaseWrite
 *
 * @brief             - use to execute address phase write
 *
 * @param[in]         - i2c port that is in use :- @I2C_RegDef_t
 * @param[in]         - slave address
 *
 * @return            - nil
 */
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1);  							// SlaveAddr is Slave address +r/nw bit=0
	pI2Cx->DR = SlaveAddr;
}

/******************************************************************************************
 * @fn      		  - I2C_ExecuteAddressPhaseRead
 *
 * @brief             - use to execute address phase read
 *
 * @param[in]         - i2c port that is in use :- @I2C_RegDef_t
 * @param[in]         - slave address
 *
 * @return            - nil
 */
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= 1;  								// SlaveAddr is Slave address +r/nw bit=0
	pI2Cx->DR = SlaveAddr;
}

/******************************************************************************************
 * @fn      		  - I2C_ClearADDRFlag
 *
 * @brief             - use to clear addr flag
 *
 * @param[in]         - i2c port that is in use :- @I2C_RegDef_t
 *
 * @return            - nil
 */
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx)
{
	uint32_t dummyRead = pI2Cx->SR1;
//	uint32_t dummyRead = pI2Cx->SR2;
	(void)dummyRead;
}

/******************************************************************************************
 * @fn      		  - I2C_GenerateStopCondition
 *
 * @brief             - use to generate the stop condition
 *
 * @param[in]         - i2c port that is in use :- @I2C_RegDef_t
 *
 * @return            - nil
 */
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

