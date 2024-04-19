#include "stm32f407xx_spi.h"

static void spi_txe_interrupt_handler(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handler(SPI_Handle_t *pSPIHandle);
static void spi_ovr_interrupt_handler(SPI_Handle_t *pSPIHandle);


void SPI_ClkCtrl(SPI_RegDef_t *pSPIx, uint8_t en_di)
{
	if (en_di == ENABLE) {
		if (pSPIx == SPI1)
			SPI1_CLK_EN();
		else if (pSPIx == SPI2)
			SPI2_CLK_EN();
		else if (pSPIx == SPI3)
			SPI3_CLK_EN();
	} else {
		if (pSPIx == SPI1)
			SPI1_CLK_DI();
		else if (pSPIx == SPI2)
			SPI2_CLK_DI();
		else if (pSPIx == SPI3)
			SPI3_CLK_DI();
	}
}

void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	uint32_t tmp = 0;

	// enable clock
	SPI_ClkCtrl(pSPIHandle->pSPIx, ENABLE);

	// device mode
	tmp |= pSPIHandle->config.deviceMode << SPI_CR1_MSTR;

	// bus config
	switch (pSPIHandle->config.busConfig) {
	case SPI_BUS_CONFIG_FD:
		tmp &= ~(1 << SPI_CR1_BIDI_MODE);
		break;
	case SPI_BUS_CONFIG_HD:
		tmp |= (1 << SPI_CR1_BIDI_MODE);
		break;
	case SPI_BUS_CONFIG_SIMPLEX_RX:
		tmp &= ~(1 << SPI_CR1_BIDI_MODE);
		tmp |= (1 << SPI_CR1_RX_ONLY);
		break;
	}

	// sclkSpeed
	tmp |= pSPIHandle->config.sclkSpeed << SPI_CR1_BR;

	// dff
	tmp |= pSPIHandle->config.dff << SPI_CR1_DFF;

	// cpol
	tmp |= pSPIHandle->config.cpol << SPI_CR1_CPOL;

	// cpha
	tmp |= pSPIHandle->config.cpha << SPI_CR1_CPHA;

	// ssm
	tmp |= pSPIHandle->config.ssm << SPI_CR1_SSM;
	if (pSPIHandle->config.ssm)
		tmp |= 1 << SPI_CR1_SSI;

	pSPIHandle->pSPIx->CR1 = tmp;
}

void SPI_Reset(SPI_RegDef_t *pSPIx)
{
	SPI_ClkCtrl(pSPIx, DISABLE);
}

void SPI_TransferData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint8_t *pRxBuffer, uint8_t len)
{
	uint8_t sz = pSPIx->CR1 & (1 << SPI_CR1_DFF) ? 2 : 1;
	for (int i = 0; i < len; i += sz) {
		while(!SPI_GetFlag(pSPIx, SPI_SR_TXE_FLAG));
		if (pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
			pSPIx->DR = *((uint16_t *)(pTxBuffer + i));
		} else
			pSPIx->DR = pTxBuffer[i];

		while(!SPI_GetFlag(pSPIx, SPI_SR_RXNE_FLAG));
		if (pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
			*((uint16_t *)(pRxBuffer + i)) = pSPIx->DR;
		} else
			pRxBuffer[i] = pSPIx->DR;
	}
}

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint8_t len)
{
	for (int i = 0; i < len; ) {
		while(!SPI_GetFlag(pSPIx, SPI_SR_TXE_FLAG));
		if (pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
			pSPIx->DR = *((uint16_t *)(pTxBuffer + i));
			i += 2;
		} else
			pSPIx->DR = pTxBuffer[i ++];
	}
}

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint8_t len)
{
	for (int i = 0; i < len; ) {
		while(!SPI_GetFlag(pSPIx, SPI_SR_RXNE_FLAG));
		if (pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
			*((uint16_t *)(pRxBuffer + i)) = pSPIx->DR;
			i += 2;
		} else
			pRxBuffer[i ++] = pSPIx->DR;
	}
}

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint8_t len)
{
	uint8_t state = pSPIHandle->txState;
	if (state != SPI_BUSY_IN_TX) {
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->txLen = len;
		pSPIHandle->txState = SPI_BUSY_IN_TX;
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
	}
	return state;
}

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint8_t len)
{
	uint8_t state = pSPIHandle->rxState;
	if (state != SPI_BUSY_IN_RX) {
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->rxLen = len;
		pSPIHandle->rxState = SPI_BUSY_IN_RX;
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
	}
	return state;
}

void SPI_PeriphCtrl(SPI_RegDef_t *pSPIx, uint8_t en_di)
{
	if (en_di)
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	else
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
}

void SPI_SSICtrl(SPI_RegDef_t *pSPIx, uint8_t en_di)
{
    if (en_di)
        pSPIx->CR1 |= (1 << SPI_CR1_SSI);
    else
        pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
}

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t en_di)
{
	if (en_di)
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	else
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
}

uint16_t SPI_GetFlag(SPI_RegDef_t *pSPIx, uint8_t flag)
{
	return pSPIx->SR & flag;
}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->txLen = 0;
	pSPIHandle->txState = SPI_READY;
}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->rxLen = 0;
	pSPIHandle->rxState = SPI_READY;
}

void SPI_ClearOVRFlag(SPI_Handle_t *pSPIHandle)
{
	uint8_t tmp __attribute__((unused));
	tmp = pSPIHandle->pSPIx->DR;
	tmp = pSPIHandle->pSPIx->SR;
}

void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{
	uint8_t tmp1, tmp2;

	// check for TXE
	tmp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	tmp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	if (tmp1 && tmp2)
		spi_txe_interrupt_handler(pSPIHandle);

	// check for RXNE
	tmp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	tmp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

	if (tmp1 && tmp2)
		spi_rxne_interrupt_handler(pSPIHandle);

	// check for OVR
	tmp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	tmp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

	if (tmp1 && tmp2)
		spi_ovr_interrupt_handler(pSPIHandle);
}

void spi_txe_interrupt_handler(SPI_Handle_t *pSPIHandle)
{
	// check the DFF
	if((pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))) {
		// 16 bit DFF
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->txLen -= 2;
		(uint16_t*)pSPIHandle->pTxBuffer ++;
	} else {
		// 8 bit DFF
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->txLen--;
		pSPIHandle->pTxBuffer ++;
	}

	if(!pSPIHandle->txLen) { // TX is over?
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}

void spi_rxne_interrupt_handler(SPI_Handle_t *pSPIHandle)
{
	// check the DFF
	if((pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))) {
		// 16 bit DFF
		*((uint16_t*)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
		pSPIHandle->rxLen -= 2;
		(uint16_t*)pSPIHandle->pRxBuffer ++;
	} else {
		// 8 bit DFF
		*pSPIHandle->pRxBuffer = pSPIHandle->pSPIx->DR;
		pSPIHandle->rxLen--;
		pSPIHandle->pRxBuffer ++;
	}

	if(!pSPIHandle->rxLen) { // RX is over?
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}

void spi_ovr_interrupt_handler(SPI_Handle_t *pSPIHandle)
{
	if (pSPIHandle->txState != SPI_BUSY_IN_TX) {
		SPI_ClearOVRFlag(pSPIHandle);
	}
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_ORV_ERR);
}

__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t event)
{
	// user application may override this function.
}

