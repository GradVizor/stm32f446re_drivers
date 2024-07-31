#include "stm32f446re_gpio_driver.h"

/******************************************************************************************
 *
 *				GPIO APIS with their full information supported by this driver
 *
 ******************************************************************************************/


/****************************** GPIO Peripheral clock control *****************************/

/******************************************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - used to enable the peripheral clock
 *
 * @param[in]         - gpio port that is used :- @GPIO_RegDef_t
 * @param[in]         - value to enable the clock "ENABLE"
 *
 * @return            - nil
 *
 * @Note			  - due to compiler not allowing to add seven "else if" condition,
 * 						so I added an "else" condition instead.
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}else
		{
			GPIOH_PCLK_EN();
		}
	}
}


/************************************* Init and De-Init ***********************************/

/******************************************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             - use to initialize the peripheral with the necessary register sets.
 *
 * @param[in]         - necessary details to initialize a gpio port :- @GPIO_Handle_t
 *
 * @return            - nil
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp;
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx,ENABLE); // Enabling the peripheral clock

	//1. configure the mode of GPIO pin
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{	// Non Interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
		pGPIOHandle->pGPIOx->MODER |= temp; //setting
	} else
	{	// Interrupt mode
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//1. Configuring the FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			//2. Clearing the corresponding RTSR bit
			EXTI->RTSR &= ~(1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//1. Configuring the RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//2. Clearing the corresponding FTSR bit
			EXTI->FTSR &= ~(1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//1. Configuring the RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//2. Configuring the FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//2. Configuring the GPIO post selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BaseAddr_To_Code(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);

		//3. Enabling the EXTI Interrupt delivery using IMR
		EXTI->IMR |= 1<< pGPIOHandle-> GPIO_PinConfig.GPIO_PinNumber;
	}
	temp = 0;

	//2. Configuring the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDER &= ~(0x3<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle->pGPIOx->OSPEEDER |= temp; //setting
	temp = 0;

	//3. Configuring the PUPD settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp; //setting
	temp = 0;

	//4. Configuring the OPTYPE
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp; //setting
	temp = 0;

	//5. Configuring the alternate functionality
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint32_t temp1, temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2)); //clearing
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2)); //setting
	}
}

/******************************************************************************************
 * @fn      		  - GPIO_DeInit
 *
 * @brief             - use to de-initializes the peripheral
 *
 * @param[in]         - gpio port that is used :- @GPIO_RegDef_t
 *
 * @return            - nil
 *
 * @Note			  - due to compiler not allowing to add seven "else if" condition,
 * 						so I added an "else" condition instead.
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}else if (pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}else if (pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}else if (pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}else if (pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}else if (pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}else if (pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}else
	{
		GPIOH_REG_RESET();
	}
}


/*********************************** Data Read and Write **********************************/

/******************************************************************************************
 * @fn      		  - GPIO_ReadFromInputPin
 *
 * @brief             - use to read the data from a specific pin
 *
 * @param[in]         - gpio port that is used :- @GPIO_RegDef_t
 * @param[in]         - pin number on which the data is to be read :- @GPIO_PIN_NUMBERS
 *
 * @return            - value read by a single pin
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t) (pGPIOx->IDR >> PinNumber) & 0x00000001;
	return value;
}

/******************************************************************************************
 * @fn      		  - GPIO_ReadFromInputPort
 *
 * @brief             - use to read the data from a specific port
 *
 * @param[in]         - gpio port that is used :- @GPIO_RegDef_t
 *
 * @return            - value read by port
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t) pGPIOx->IDR;
	return value;
}

/******************************************************************************************
 * @fn      		  - GPIO_WriteToOutputPin
 *
 * @brief             - use to write data on a specific port
 *
 * @param[in]         - gpio port that is used :- @GPIO_RegDef_t
 * @param[in]         - pin number used to write data :- @GPIO_PIN_NUMBERS
 * @param[in]         - value that is to be displayed as output.
 *
 * @return            - nil
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if (Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1<<PinNumber);  // Writing 1 to output data register
	}else
	{
		pGPIOx->ODR &= ~(1<<PinNumber);  // Writing 0 to output data register
	}
}

/******************************************************************************************
 * @fn      		  - GPIO_WriteToOutputPort
 *
 * @brief             - use to write data on a specific port
 *
 * @param[in]         - gpio port that is used :- @GPIO_RegDef_t
 * @param[in]         - value that is to be displayed as output.
 *
 * @return            - nil
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}

/******************************************************************************************
 * @fn      		  - GPIO_ToggleOutputPin
 *
 * @brief             - use to toggle the output of a port
 *
 * @param[in]         - gpio port that is used :- @GPIO_RegDef_t
 * @param[in]         - pin number used to toggle output :- @GPIO_PIN_NUMBERS
 *
 * @return            - nil
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}


/*************************** IRQ Configuration and ISR Handling ***************************/

/******************************************************************************************
 * @fn      		  - GPIO_IRQInterruptConfig
 *
 * @brief             - use to configure the interrupts
 *
 * @param[in]         - IRQ number corresponding to your interrupt
 * @param[in]         - value to enable the interrupt "ENABLE"
 *
 * @return            - nil
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
 * @fn      		  - GPIO_IRQPriorityConfig
 *
 * @brief             - use to configure the priority for interrupt
 *
 * @param[in]         - IRQ number corresponding to your interrupt
 * @param[in]         - value to set the priority of the interrupt
 *
 * @return            - nil
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;  // Finding out the IPR register
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}

/******************************************************************************************
 * @fn      		  - GPIO_IRQHandling
 *
 * @brief             - used to configure the irq handling.
 *
 * @param[in]         - pin number on which the interrupt is enabled :- @GPIO_PIN_NUMBERS
 *
 * @return            - nil
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
	if (EXTI->PR & (1 << PinNumber))
	{
		EXTI->PR |= (1<<PinNumber);  // clearing
	}
}


/************************************ Miscellaneous APIs **********************************/

/******************************************************************************************
 * @fn      		  - GPIO_BaseAddr_To_Code
 *
 * @brief             - use to convert base address/ port to code
 *
 * @param[in]         - gpio port that is used :- @GPIO_RegDef_t
 *
 * @return            - code of the gpio port used
 *
 * @Note			  - due to compiler not allowing to add seven "else if" condition,
 * 						so I added an "else" condition instead.
 */
uint8_t GPIO_BaseAddr_To_Code(GPIO_RegDef_t *pGPIOx)
{
	uint8_t portcode;
	if(pGPIOx == GPIOA)
	{
		portcode=0;
	}else if (pGPIOx == GPIOB)
	{
		portcode=1;
	}else if (pGPIOx == GPIOC)
	{
		portcode=2;
	}else if (pGPIOx == GPIOD)
	{
		portcode=3;
	}else if (pGPIOx == GPIOE)
	{
		portcode=4;
	}else if (pGPIOx == GPIOF)
	{
		portcode=5;
	}else if (pGPIOx == GPIOG)
	{
		portcode=6;
	}else
	{
		portcode=7;
	}
	return portcode;
}

