
#include <string.h>
#include "sdcard_spi.h"

/*
 * SPI2
 * PB12 -> SPI2_NSS
 * PB13 -> SPI2_SCLK
 * PB14 -> SPI2_MISO
 * PB15 -> SPI2_MOSI
 * Alt function mode 5
 * (Alternate function mapping - datasheet p63)
 */

#define	PIN_NSS		12
#define	PIN_SCLK	13
#define	PIN_MISO	14
#define	PIN_MOSI	15

extern void stdio_usart_init(GPIO_RegDef_t *pGPIOx, uint8_t pinTx, uint8_t pinRx, USART_RegDef_t *pUSARTx);

SDCARD_Handle_t sdcardHandle;

void sdcard_init()
{
    memset(&sdcardHandle, 0, sizeof sdcardHandle);
    sdcardHandle.config.pinNss = PIN_NSS;
    sdcardHandle.config.pinSclk = PIN_SCLK;
    sdcardHandle.config.pinMiso = PIN_MISO;
    sdcardHandle.config.pinMosi = PIN_MOSI;
    sdcardHandle.pGPIOx = GPIOB;
    sdcardHandle.spi.pSPIx = SPI2;
    SDCARD_Init(&sdcardHandle);
}

void sdcard_wait_send_dummy(uint8_t count) {
    uint8_t dummy = 0xff;
    while (count --) {
        SPI_SendData(sdcardHandle.spi.pSPIx, &dummy, 1);
    }
}

int main()
{
    RCC_SysClkSwitch(RCC_SYSCLK_HSI);
    stdio_usart_init(GPIOA, 2, 3, USART2);

    sdcard_init();
    SDCARD_Ctrl(&sdcardHandle, ENABLE);

    sdcard_wait_send_dummy(10);

    while(1);
    SDCARD_Ctrl(&sdcardHandle, DISABLE);

	return 0;
}

