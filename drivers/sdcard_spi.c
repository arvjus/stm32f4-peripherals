
#include <string.h>
#include "sdcard_spi.h"

void SDCARD_Init(SDCARD_Handle_t *pCDCARDHandle)
{
    // GPIO
    GPIO_Handle_t spi2Pins;
    memset(&spi2Pins, 0, sizeof(spi2Pins));

    spi2Pins.pGPIOx = pCDCARDHandle->pGPIOx;
    spi2Pins.pinConfig.mode = GPIO_PIN_MODE_ALTFN;
    spi2Pins.pinConfig.altFunMode = 5;
    spi2Pins.pinConfig.opType = GPIO_PIN_OPTYPE_PP;
    spi2Pins.pinConfig.puPdControl = GPIO_PIN_NONE_PU_PD;
    // NSS
    spi2Pins.pinConfig.number = pCDCARDHandle->config.pinNss;
    GPIO_Init(&spi2Pins);
    // SCLK
    spi2Pins.pinConfig.number = pCDCARDHandle->config.pinSclk;
    GPIO_Init(&spi2Pins);
    // MISO
    spi2Pins.pinConfig.number = pCDCARDHandle->config.pinMiso;
    GPIO_Init(&spi2Pins);
    // MOSI
    spi2Pins.pinConfig.number = pCDCARDHandle->config.pinMosi;
    GPIO_Init(&spi2Pins);

    // SPI
    pCDCARDHandle->spi.config.deviceMode = SPI_DEVICE_MODE_MASTER;
    pCDCARDHandle->spi.config.busConfig = SPI_BUS_CONFIG_FD;
    pCDCARDHandle->spi.config.sclkSpeed = SPI_SCLK_SPEED_DIV64;
    pCDCARDHandle->spi.config.dff = SPI_DFF_8BIT;					// default
    pCDCARDHandle->spi.config.cpol = SPI_CPO_0_IDLE;				// default
    pCDCARDHandle->spi.config.cpha = SPI_CPHA_1ST_CLK_TRANS;		// default
    pCDCARDHandle->spi.config.ssm = SPI_SSM_DI;
    SPI_Init(&pCDCARDHandle->spi);

    // SDCARD
}

void SDCARD_Ctrl(SDCARD_Handle_t *pCDCARDHandle, uint8_t en_di)
{
    SPI_PeriphCtrl(pCDCARDHandle->spi.pSPIx, en_di);
}
