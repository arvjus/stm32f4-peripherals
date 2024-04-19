
#include "stm32f407xx_usart.h"

static void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t baudRate);


void USART_ClkCtrl(USART_RegDef_t *pUSARTx, uint8_t en_di)
{
	if (en_di == ENABLE) {
		if (pUSARTx == USART1)
			USART1_CLK_EN();
		else if (pUSARTx == USART2)
			USART2_CLK_EN();
		else if (pUSARTx == USART3)
			USART3_CLK_EN();
		else if (pUSARTx == UART4)
			UART4_CLK_EN();
		else if (pUSARTx == UART5)
			UART5_CLK_EN();
		else if (pUSARTx == USART6)
			USART6_CLK_EN();
	} else {
		if (pUSARTx == USART1)
			USART1_CLK_DI();
		else if (pUSARTx == USART2)
			USART2_CLK_DI();
		else if (pUSARTx == USART3)
			USART3_CLK_DI();
		else if (pUSARTx == UART4)
			UART4_CLK_DI();
		else if (pUSARTx == UART5)
			UART5_CLK_DI();
		else if (pUSARTx == USART6)
			USART6_CLK_DI();
	}
}

void USART_Init(USART_Handle_t *pUSARTHandle)
{
	// Enable the clock
	USART_ClkCtrl(pUSARTHandle->pUSARTx, ENABLE);

	/* Configure CR1 */
	uint32_t tmpreg = 0;

	// Tx / Rx mode
	if (pUSARTHandle->config.mode == USART_MODE_TXRX || pUSARTHandle->config.mode == USART_MODE_ONLY_TX ) {
		tmpreg |= (1 << USART_CR1_TE);
	}
	if (pUSARTHandle->config.mode == USART_MODE_TXRX || pUSARTHandle->config.mode == USART_MODE_ONLY_RX ) {
		tmpreg |= (1 << USART_CR1_RE);
	}

	// Word length
	tmpreg |= (pUSARTHandle->config.wordLength << USART_CR1_M);

	// Parity control
	if (pUSARTHandle->config.parityCtrl == USART_PARITY_ODD || pUSARTHandle->config.parityCtrl == USART_PARITY_EVEN) {
		tmpreg |= (1 << USART_CR1_PCE);
		if (pUSARTHandle->config.parityCtrl == USART_PARITY_ODD)
			tmpreg |= (1 << USART_CR1_PS);
	}

	pUSARTHandle->pUSARTx->CR1 = tmpreg;

	/* Configure CR2 */
	tmpreg = 0;

	// Stop bits
	tmpreg |= (pUSARTHandle->config.stopBits << USART_CR2_STOP);

	pUSARTHandle->pUSARTx->CR2 = tmpreg;

	/* Configure CR3 */
	tmpreg = 0;

	// HW flow
	if (pUSARTHandle->config.hwFlowCtrl == USART_HW_FLOW_CTRL_CTS_RTS || pUSARTHandle->config.hwFlowCtrl == USART_HW_FLOW_CTRL_CTS ) {
		tmpreg |= (1 << USART_CR3_CTSE);
	}
	if (pUSARTHandle->config.hwFlowCtrl == USART_HW_FLOW_CTRL_CTS_RTS || pUSARTHandle->config.hwFlowCtrl == USART_HW_FLOW_CTRL_RTS ) {
		tmpreg |= (1 << USART_CR3_RTSE);
	}

	pUSARTHandle->pUSARTx->CR3 = tmpreg;

	// Baud rate
	USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->config.baudRate);
}

void USART_Reset(USART_RegDef_t *pUSARTx)
{
	USART_ClkCtrl(pUSARTx, DISABLE);
}

void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint8_t len)
{
	uint16_t *pdata;
	for (uint32_t i = 0; i < len; i++) {
		while (!USART_GetFlag(pUSARTHandle->pUSARTx, USART_SR_TXE_FLAG));

		if (pUSARTHandle->config.wordLength == USART_WORDLEN_9BITS) {
			// 9bit transfer, load the DR with 2bytes masking the bits other than first 9 bits
			pdata = (uint16_t*) pTxBuffer;
			pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t) 0x01FF);

			pTxBuffer++;
			if (pUSARTHandle->config.parityCtrl	== USART_PARITY_DI)
				pTxBuffer++;
		} else {
			// 8bit transfer
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer & (uint8_t) 0xFF);
			pTxBuffer++;
		}
	}

	while (!USART_GetFlag(pUSARTHandle->pUSARTx, USART_SR_TC_FLAG));
}

void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint8_t len) {
	for (uint32_t i = 0; i < len; i++) {
		while (!USART_GetFlag(pUSARTHandle->pUSARTx, USART_SR_RXNE_FLAG));

		if (pUSARTHandle->config.wordLength == USART_WORDLEN_9BITS) { // 9bit data transfer
			if (pUSARTHandle->config.parityCtrl	== USART_PARITY_DI) {
				// read only first 9 bits. so, mask the DR with 0x01FF
				*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->DR & (uint16_t) 0x01FF);
				pRxBuffer ++;
				pRxBuffer ++;
			} else {
				// 8bits will be of user data and 1 bit is parity
				*pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t) 0xFF);
				pRxBuffer ++;
			}
		} else {	// 8bit data transfer
			if (pUSARTHandle->config.parityCtrl	== USART_PARITY_DI) {
				// all 8bits will be of user data
				*pRxBuffer = pUSARTHandle->pUSARTx->DR;
			} else {
				// 7 bits will be of user data and 1 bit is parity
				*pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & 0X7F);
			}
			pRxBuffer++;
		}
	}

}

uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint8_t len) {
	uint8_t txstate = pUSARTHandle->txState;

	if (txstate != USART_BUSY_IN_TX) {
		pUSARTHandle->txLen = len;
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->txState = USART_BUSY_IN_TX;

		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TXEIE);
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TCIE);
	}

	return txstate;
}

uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint8_t len) {
	uint8_t rxstate = pUSARTHandle->rxState;

	if (rxstate != USART_BUSY_IN_RX) {
		pUSARTHandle->rxLen = len;
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->rxState = USART_BUSY_IN_RX;

		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_RXNEIE);
	}

	return rxstate;
}

void USART_IRQHandling(USART_Handle_t *pUSARTHandle) {
	uint32_t tmp1, tmp2, tmp3;
	uint16_t *pdata;

	tmp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_TC);
	tmp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_TCIE);
	if (tmp1 && tmp2 && pUSARTHandle->txState == USART_BUSY_IN_TX && !pUSARTHandle->txLen) {
		pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_TC);
		//pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_TXEIE);

		pUSARTHandle->txState = USART_READY;
		pUSARTHandle->pTxBuffer = NULL;
		pUSARTHandle->txLen = 0;
		USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_TX_CMPLT);
	}

	tmp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_TXE);
	tmp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_TXEIE);
	if (tmp1 && tmp2 && pUSARTHandle->txState == USART_BUSY_IN_TX) {
		if (pUSARTHandle->txLen > 0) {
			if (pUSARTHandle->config.wordLength == USART_WORDLEN_9BITS) {
				// 9BIT, load the DR with 2bytes masking the bits other than first 9 bits
				pdata = (uint16_t*) pUSARTHandle->pTxBuffer;

				//loading only first 9 bits , so we have to mask with the value 0x01FF
				pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t) 0x01FF);

				//check for USART_ParityControl
				if (pUSARTHandle->config.parityCtrl == USART_PARITY_DI) {
					pUSARTHandle->pTxBuffer++;
					pUSARTHandle->pTxBuffer++;
					pUSARTHandle->txLen -= 2;
				} else {
					pUSARTHandle->pTxBuffer++;
					pUSARTHandle->txLen++;
				}
			} else {
				pUSARTHandle->pUSARTx->DR = (*pUSARTHandle->pTxBuffer & (uint8_t) 0xFF);
				pUSARTHandle->pTxBuffer++;
				pUSARTHandle->txLen--;
			}
		}
		if (!pUSARTHandle->txLen) {
			pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_TXEIE);
		}
	}

	tmp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_RXNE);
	tmp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_RXNEIE);
	if (tmp1 && tmp2 && pUSARTHandle->rxState == USART_BUSY_IN_RX) {
		if (pUSARTHandle->rxLen > 0) {
			if (pUSARTHandle->config.wordLength == USART_WORDLEN_9BITS) {
				if (pUSARTHandle->config.parityCtrl == USART_PARITY_DI) {
					//read only first 9 bits so mask the DR with 0x01FF
					*((uint16_t*) pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR & (uint16_t) 0x01FF);
					pUSARTHandle->pRxBuffer++;
					pUSARTHandle->pRxBuffer++;
					pUSARTHandle->txLen -= 2;
				} else {
					//Parity is used. so, 8bits will be of user data and 1 bit is parity
					*pUSARTHandle->pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t) 0xFF);
					pUSARTHandle->pRxBuffer++;
					pUSARTHandle->txLen--;
				}
			} else {
				if (pUSARTHandle->config.parityCtrl == USART_PARITY_DI) {
					//No parity is used , so all 8bits will be of user data
					*pUSARTHandle->pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR	& (uint8_t) 0xFF);
				} else {
					//Parity is used, so , 7 bits will be of user data and 1 bit is parity
					*pUSARTHandle->pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR & (uint8_t) 0x7F);
				}
				pUSARTHandle->pRxBuffer++;
				pUSARTHandle->rxLen--;
			}
		}
		if (!pUSARTHandle->rxLen) {
			pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_RXNEIE);
			pUSARTHandle->rxState = USART_READY;
			USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_RX_CMPLT);
		}
	}

	if (pUSARTHandle->pUSARTx != UART4 && pUSARTHandle->pUSARTx != UART5) {
		tmp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_CTS);
		tmp2 = pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_CTSE);
		tmp3 = pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_CTSIE);
		if(tmp1 && tmp2 && tmp3) {
			pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_CTS);
			USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_CTS);
		}
	}

	tmp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_IDLE);
	tmp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_IDLEIE);
	if(tmp1 && tmp2) {
		tmp3 = pUSARTHandle->pUSARTx->DR;	// clear the IDLE flag by reading SR then DR
		USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_IDLE);
	}

	tmp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_ORE);
	tmp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_RXNEIE);
	if(tmp1 && tmp2) {
		USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_ORE);
	}

	tmp1 = pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_EIE);
	if (tmp1) {
		if(pUSARTHandle->pUSARTx->SR & (1 << USART_SR_FE))
			USART_ApplicationEventCallback(pUSARTHandle, USART_ERROR_FE);

		if (pUSARTHandle->pUSARTx->SR & (1 << USART_SR_NF))
			USART_ApplicationEventCallback(pUSARTHandle, USART_ERROR_NF);

		if (pUSARTHandle->pUSARTx->SR & (1 << USART_SR_ORE))
			USART_ApplicationEventCallback(pUSARTHandle, USART_ERROR_ORE);
	}
}

void USART_PeriphCtrl(USART_RegDef_t *pUSARTx, uint8_t en_di)
{
	if (en_di) {
		pUSARTx->CR1 |= (1 << USART_CR1_UE);
	} else {
		pUSARTx->CR1 &= ~(1 << USART_CR1_UE);
	}
}

uint16_t USART_GetFlag(USART_RegDef_t *pUSARTx, uint16_t flag)
{
	return pUSARTx->SR & flag;
}

void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t flag)
{
	pUSARTx->SR &= ~(flag);
}

static void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t baudRate) {

	uint32_t pclkx, usartdiv, m_part, f_part, tempreg = 0;

	pclkx = (pUSARTx == USART1 || pUSARTx == USART6) ? RCC_GetPCLK2Value() : RCC_GetPCLK1Value();

	if (pUSARTx->CR1 & (1 << USART_CR1_OVER8)) {	// OVER8 = 1 , over sampling by 8
		usartdiv = ((25 * pclkx) / (2 * baudRate));
	} else {	// over sampling by 16
		usartdiv = ((25 * pclkx) / (4 * baudRate));
	}

	m_part = usartdiv / 100;
	tempreg |= m_part << 4;
	f_part = (usartdiv - (m_part * 100));

	//Calculate the final fractional
	if (pUSARTx->CR1 & (1 << USART_CR1_OVER8)) {	//OVER8 = 1 , over sampling by 8
		f_part = (((f_part * 8) + 50) / 100) & ((uint8_t) 0x07);
	} else {	//over sampling by 16
		f_part = (((f_part * 16) + 50) / 100) & ((uint8_t) 0x0F);
	}
	tempreg |= f_part;
	pUSARTx->BRR = tempreg;
}

__attribute__((weak)) void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t event)
{
	// user application may override this function.
}
