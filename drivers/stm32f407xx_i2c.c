
#include "stm32f407xx_i2c.h"

static void I2C_ExecuteAddrPhase(I2C_RegDef_t *pI2Cx, uint8_t slaveAddress, uint8_t isRead);
static void I2C_ClearAddrFlag(I2C_Handle_t *pI2CHandle);
static void I2C_CloseHandle(I2C_Handle_t *pI2CHandle);

void I2C_ClkCtrl(I2C_RegDef_t *pI2Cx, uint8_t en_di)
{
	if (en_di == ENABLE) {
		if (pI2Cx == I2C1)
			I2C1_CLK_EN();
		else if (pI2Cx == I2C2)
			I2C2_CLK_EN();
		else if (pI2Cx == I2C3)
			I2C3_CLK_EN();
	} else {
		if (pI2Cx == I2C1)
			I2C1_CLK_DI();
		else if (pI2Cx == I2C2)
			I2C2_CLK_DI();
		else if (pI2Cx == I2C3)
			I2C3_CLK_DI();
	}
}

void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tmpreg;

	// Enable the clock
	I2C_ClkCtrl(pI2CHandle->pI2Cx, ENABLE);

	// ACK
	pI2CHandle->pI2Cx->CR1 = pI2CHandle->config.ackControl << I2C_CR1_ACK;

	// FREQ
	pI2CHandle->pI2Cx->CR2 = (RCC_GetPCLK1Value() / 1000000U) & 0x3f;

	// device address
	tmpreg = 0;
	tmpreg |= pI2CHandle->config.deviceAddress << 1;
	tmpreg |= (1 << 14); // stated in manual
	pI2CHandle->pI2Cx->OAR1 = tmpreg;

	// CCR calculations
	uint16_t ccr_value = 0;
	tmpreg = 0;
	if (pI2CHandle->config.sclSpeed <= I2C_SCL_SPEED_SM) {
		ccr_value = RCC_GetPCLK1Value() / (2 * pI2CHandle->config.sclSpeed);
		tmpreg |= (ccr_value & 0xfff);
	} else {
		tmpreg |= (1 << I2C_CCR_FS);
		tmpreg |= (pI2CHandle->config.fmDutyCycle << I2C_CCR_DUTY);
		if (pI2CHandle->config.fmDutyCycle == I2C_FM_DUTY_2)
			ccr_value = RCC_GetPCLK1Value() / (3 * pI2CHandle->config.sclSpeed);
		else
			ccr_value = RCC_GetPCLK1Value() / (25 * pI2CHandle->config.sclSpeed);
		tmpreg |= (ccr_value & 0xfff);
	}
	pI2CHandle->pI2Cx->CCR = tmpreg;

	// TRISE calculations
	if (pI2CHandle->config.sclSpeed <= I2C_SCL_SPEED_SM) {
		tmpreg = (RCC_GetPCLK1Value() / 1000000U) + 1;
	} else {
		tmpreg = (RCC_GetPCLK1Value() * 300 / 1000000000U) + 1;
	}
	pI2CHandle->pI2Cx->TRISE = tmpreg;
}

void I2C_Reset(I2C_RegDef_t *pI2Cx)
{
	I2C_ClkCtrl(pI2Cx, DISABLE);
}

void I2C_PeriphCtrl(I2C_RegDef_t *pI2Cx, uint8_t en_di)
{
	if (en_di)
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	else
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
}

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t slaveAddress, uint8_t sr)
{
	// Generate START condition, wait for completion
	pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_START);
	while (!I2C_GetFlag1(pI2CHandle->pI2Cx, I2C_SR1_SB_FLAG));

	// Send the address of the slave, wait for completion, clear ADDR flag
	I2C_ExecuteAddrPhase(pI2CHandle->pI2Cx, slaveAddress, 0);
	while (!I2C_GetFlag1(pI2CHandle->pI2Cx, I2C_SR1_ADDR_FLAG));
	I2C_ClearAddrFlag(pI2CHandle);

	// Send data until len == 0
	while (len > 0) {
		while (!I2C_GetFlag1(pI2CHandle->pI2Cx, I2C_SR1_TXE_FLAG));
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer ++;
		len --;
	}

	// Wait for TXE == 1 and BTF == 1
	while (!I2C_GetFlag1(pI2CHandle->pI2Cx, I2C_SR1_TXE_FLAG));
	while (!I2C_GetFlag1(pI2CHandle->pI2Cx, I2C_SR1_BTF_FLAG));

	// Generate STOP condition
	if (sr == I2C_SR_DI)
		pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t len, uint8_t slaveAddress, uint8_t sr)
{
	// Generate START condition, wait for completion
	pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_START);
	while (!I2C_GetFlag1(pI2CHandle->pI2Cx, I2C_SR1_SB_FLAG));

	// Send the address of the slave, wait for completion, clear ADDR flag
	I2C_ExecuteAddrPhase(pI2CHandle->pI2Cx, slaveAddress, 1);
	while (!I2C_GetFlag1(pI2CHandle->pI2Cx, I2C_SR1_ADDR_FLAG));

	if (len == 1) {
		// Clear ACK
		I2C_AckCtrl(pI2CHandle->pI2Cx, I2C_ACK_DI);

		// Clear ADDR flag
		I2C_ClearAddrFlag(pI2CHandle);

		// Wait for data
		while (!I2C_GetFlag1(pI2CHandle->pI2Cx, I2C_SR1_RXNE_FLAG));

		// Generate STOP condition
		if (sr == I2C_SR_DI)
			pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_STOP);

		// Read the data
		*pRxBuffer = pI2CHandle->pI2Cx->DR;
	} else if (len > 1) {
		// Clear ADDR flag
		I2C_ClearAddrFlag(pI2CHandle);

		// Loop until len = 0
		while (len > 0) {
			// Wait for data
			while (!I2C_GetFlag1(pI2CHandle->pI2Cx, I2C_SR1_RXNE_FLAG));

			if (len == 2) {
				// Clear ACK
				I2C_AckCtrl(pI2CHandle->pI2Cx, I2C_ACK_DI);

				// Generate STOP condition
				if (sr == I2C_SR_DI)
					pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
			}

			// Read the data
			*(pRxBuffer ++) = pI2CHandle->pI2Cx->DR;
			len --;
		}
	}

	// Re-enable ACK
	if (pI2CHandle->config.ackControl == I2C_ACK_EN) {
		I2C_AckCtrl(pI2CHandle->pI2Cx, I2C_ACK_EN);
	}
}

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t slaveAddress, uint8_t sr)
{
	uint8_t busystate = pI2CHandle->state;
	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->txLen = len;
		pI2CHandle->state = I2C_BUSY_IN_TX;
		pI2CHandle->devAddress = slaveAddress;
		pI2CHandle->sr = sr;

		// Enable events
		I2C_EventCtrl(pI2CHandle->pI2Cx, ENABLE);

		// Generate START condition
		pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_START);
	}

	return busystate;
}

uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t len, uint8_t slaveAddress, uint8_t sr)
{
	uint8_t busystate = pI2CHandle->state;
	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->rxLen = len;
		pI2CHandle->rxSize = len;
		pI2CHandle->state = I2C_BUSY_IN_RX;
		pI2CHandle->devAddress = slaveAddress;
		pI2CHandle->sr = sr;

		// Enable events
		I2C_EventCtrl(pI2CHandle->pI2Cx, ENABLE);

		// Generate START condition
		pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_START);
	}

	return busystate;
}

void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data)
{
	pI2Cx->DR = data;
}

uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx)
{
	return (uint8_t)pI2Cx->DR;
}

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	uint32_t tmp1, tmp2, tmp3;

	tmp1 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);
	tmp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);

	tmp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB);
	if (tmp1 && tmp3) {
		// SB is set (only in master mode)
		if (pI2CHandle->state == I2C_BUSY_IN_TX)
			I2C_ExecuteAddrPhase(pI2CHandle->pI2Cx, pI2CHandle->devAddress, 0);
		else if (pI2CHandle->state == I2C_BUSY_IN_RX)
			I2C_ExecuteAddrPhase(pI2CHandle->pI2Cx, pI2CHandle->devAddress, 1);
	}

	tmp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);
	if (tmp1 && tmp3) {
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)
				&& pI2CHandle->state == I2C_BUSY_IN_RX
				&& pI2CHandle->rxSize == 1)
			I2C_AckCtrl(pI2CHandle->pI2Cx, I2C_ACK_DI);

		I2C_ClearAddrFlag(pI2CHandle);
	}

	tmp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);
	if (tmp1 && tmp3) {
		if (pI2CHandle->state == I2C_BUSY_IN_TX && pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE) && pI2CHandle->txLen == 0) {
			if (pI2CHandle->sr == I2C_SR_DI)
				pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_STOP);

			I2C_CloseHandle(pI2CHandle);
			I2C_ApplicationEventCallback(pI2CHandle, I2C_EVENT_TX_CMPLT);
		} else if (pI2CHandle->state == I2C_BUSY_IN_RX &&  pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE)) {
			; // do nothing
		}
	}

	tmp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF);
	if (tmp1 && tmp3) {
		// STOPF set in slave mode only in receiving the data
		// To clear flag - read SR1 & write to CR1
		pI2CHandle->pI2Cx->CR1 |= 0x0;

		I2C_ApplicationEventCallback(pI2CHandle, I2C_EVENT_STOP);
	}

	tmp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE);
	if (tmp1 && tmp2 && tmp3) {
		if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)) {	// Master mode
			if (pI2CHandle->state == I2C_BUSY_IN_TX && pI2CHandle->txLen > 0) {
				pI2CHandle->pI2Cx->DR = *pI2CHandle->pTxBuffer;
				pI2CHandle->pTxBuffer ++;
				pI2CHandle->txLen --;
			}
		} else {	// Slave mode
			if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA))	// In TX mode
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EVENT_DATA_REQ);
		}
	}

	tmp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE);
	if (tmp1 && tmp2 && tmp3) {
		if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)) {	// Master mode
			if (pI2CHandle->state == I2C_BUSY_IN_RX ) {
				if (pI2CHandle->rxSize == 1) {
					// Read the data
					*(pI2CHandle->pRxBuffer ++) = pI2CHandle->pI2Cx->DR;
					pI2CHandle->rxLen --;
				} else if (pI2CHandle->rxSize > 1) {
					if (pI2CHandle->rxLen == 2)
						I2C_AckCtrl(pI2CHandle->pI2Cx, I2C_ACK_DI);

					// Read the data
					*(pI2CHandle->pRxBuffer ++) = pI2CHandle->pI2Cx->DR;
					pI2CHandle->rxLen --;
				}

				if (pI2CHandle->rxLen == 0) {
					if (pI2CHandle->sr == I2C_SR_DI)
						pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_STOP);

					I2C_CloseHandle(pI2CHandle);
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EVENT_RX_CMPLT);
				}
			}
		} else {	// Slave mode
			if (!(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA)))	// In RX mode
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EVENT_DATA_RCV);
		}
	}
}

void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	uint32_t tmp1, tmp2;

	tmp1 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITERREN);

	tmp2 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BERR);
	if (tmp1 && tmp2) {
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_BERR);
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_BERR);
	}

	tmp2 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ARLO);
	if (tmp1 && tmp2) {
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_ARLO);
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_ARLO);
	}

	tmp2 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_AF);
	if (tmp1 && tmp2) {
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_AF);
		pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_AF);
	}

	tmp2 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_OVR);
	if (tmp1 && tmp2) {
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_OVR);
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_OVR);
	}

	tmp2 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TIMEOUT);
	if (tmp1 && tmp2) {
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_TIMEOUT);
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_TIMEOUT);
	}
}

uint16_t I2C_GetFlag1(I2C_RegDef_t *pI2Cx, uint8_t flag)
{
	return pI2Cx->SR1 & flag;
}

uint16_t I2C_GetFlag2(I2C_RegDef_t *pI2Cx, uint8_t flag)
{
	return pI2Cx->SR2 & flag;
}

void I2C_AckCtrl(I2C_RegDef_t *pI2Cx, uint8_t en_di)
{
	if (en_di)
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	else
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
}

void I2C_EventCtrl(I2C_RegDef_t *pI2Cx, uint8_t en_di)
{
	if (en_di) {
		pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);
		pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);
		pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	} else {
		pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
		pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);
		pI2Cx->CR2 &= ~(1 << I2C_CR2_ITERREN);
	}
}


static void I2C_ExecuteAddrPhase(I2C_RegDef_t *pI2Cx, uint8_t slaveAddress, uint8_t isRead)
{
	slaveAddress <<= 1;
	slaveAddress |= (isRead ? 0x1 : 0x0);
	pI2Cx->DR = slaveAddress;
}

static void I2C_ClearAddrFlag(I2C_Handle_t *pI2CHandle)
{
	uint32_t dummy;
	dummy = pI2CHandle->pI2Cx->SR1;
	dummy = pI2CHandle->pI2Cx->SR2;
	(void)dummy;
}


static void I2C_CloseHandle(I2C_Handle_t *pI2CHandle)
{
	I2C_EventCtrl(pI2CHandle->pI2Cx, DISABLE);

	uint8_t tx = (pI2CHandle->state == I2C_BUSY_IN_TX);

	pI2CHandle->state = I2C_READY;
	if (tx) {
		pI2CHandle->pTxBuffer = NULL;
		pI2CHandle->txLen = 0;
	} else {
		pI2CHandle->pRxBuffer = NULL;
		pI2CHandle->rxLen = 0;
		pI2CHandle->rxSize = 0;
	}

	if (pI2CHandle->config.ackControl == I2C_ACK_EN)
		I2C_AckCtrl(pI2CHandle->pI2Cx, I2C_ACK_EN);
}

bool I2C_MasterProbe(I2C_Handle_t *pI2CHandle, uint8_t slaveAddress)
{
    // Generate START condition, wait for completion
    pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_START);
    while (!I2C_GetFlag1(pI2CHandle->pI2Cx, I2C_SR1_SB_FLAG));

    // Send the address of the slave, wait for completion, clear ADDR flag
    I2C_ExecuteAddrPhase(pI2CHandle->pI2Cx, slaveAddress, 0);
    uint32_t timout = 1000;
    while (!I2C_GetFlag1(pI2CHandle->pI2Cx, I2C_SR1_ADDR_FLAG))
        if (--timout == 0)
            return false;
    I2C_ClearAddrFlag(pI2CHandle);

    // Generate STOP condition
    pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_STOP);

    return true;
}

__attribute__((weak)) void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t event)
{
	// user application may override this function.
}

