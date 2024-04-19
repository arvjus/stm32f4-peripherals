
#include "stm32f407xx.h"

// CPU specific
void IRQEnable(uint8_t irqNumber, uint8_t en_di)
{
	if (en_di) {
		if (irqNumber < 32) {
			*NVIC_ISER0 |= (1 << irqNumber);
		} else if (irqNumber < 64) {
			*NVIC_ISER1 |= (1 << (irqNumber % 32));
		} else if (irqNumber < 96) {
			*NVIC_ISER2 |= (1 << (irqNumber % 32));
		}
	} else {
		if (irqNumber < 32) {
			*NVIC_ICER0 |= (1 << irqNumber);
		} else if (irqNumber < 64) {
			*NVIC_ICER1 |= (1 << (irqNumber % 32));
		} else if (irqNumber < 96) {
			*NVIC_ICER2 |= (1 << (irqNumber % 32));
		}
	}
}

// CPU specific
void IRQPriority(uint8_t irqNumber, uint32_t irqPriority)
{
	uint8_t iprx = irqNumber / 4;
	uint8_t iprx_section = irqNumber % 4;
	uint8_t shift = iprx_section * 8 + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASEADDR + iprx) |= (irqPriority << shift);
}

void RCC_SysClkSwitch(uint8_t source)
{
    RCC->CFGR &= ~(0x3);
    RCC->CFGR |= (source & 0x03);
    udelay(10); // FIXIT
    switch (source) {
        case RCC_SYSCLK_HSI:
            //while (!(RCC->CR & (1 << 1)));      // wait for HSIRDY
            break;
        case RCC_SYSCLK_HSE:
            //while (!(RCC->CR & (1 << 17)));     // wait for HSERDY
            break;
        case RCC_SYSCLK_PLL:
            // FIXIT
            //while (!(RCC->CR & 0x1));           // wait for HSIRDY
            break;
    }
}


uint32_t RCC_GetSysClkValue()
{
	// system clk
	switch ((RCC->CFGR >> 2) & 0x03) { // SWS
	case RCC_SYSCLK_HSI:
		return 16000000;
	case RCC_SYSCLK_HSE:
		return 8000000;
	case RCC_SYSCLK_PLL:
    {
        uint32_t pllcfgr = RCC->PLLCFGR;
        uint32_t pll_clk_in = (pllcfgr &  (1 << 22)) ? 8000000 : 16000000; // PLLSRC
        uint32_t plln = ((pllcfgr >> 6) & 0x1ff);   // PLLN
        uint32_t pllm = (pllcfgr & 0x3f);           // PLLM;
        uint32_t pllp = ((pllcfgr >> 16) & 0x3);    // PLLP;
        return pll_clk_in * (plln / pllm) / pllp;
    }
	default:
		assert(0);
	}
}


uint32_t RCC_GetPCLK1Value()
{
	//static uint16_t ahb_prescaler[9] = {2, 4, 8, 16, 0, 64, 128, 256, 512};
	//static uint16_t apb1_prescaler[4] = {2, 4, 8, 16};

	uint32_t sysClk = RCC_GetSysClkValue();
	uint8_t tmp, ahbp, apb1;

	// ABH prescaler
	tmp = (RCC->CFGR >> 4) & 0x0f;	// HPRE
	//ahbp = tmp < 8 ? 1 : ahb_prescaler[tmp - 8];
	ahbp = tmp < 8 ? 1 : 2 ^ (tmp - 7);

	// APB1 prescaler
	tmp = (RCC->CFGR >> 10) & 0x07;	// PPRE1
	//apb1 = tmp < 4 ? 1 : apb1_prescaler[tmp - 4];
	apb1 = tmp < 4 ? 1 : 2 ^ (tmp - 3);

	return sysClk / ahbp / apb1;
}

uint32_t RCC_GetPCLK2Value()
{
	//static uint16_t ahb_prescaler[9] = {2, 4, 8, 16, 0, 64, 128, 256, 512};
	//static uint16_t apb1_prescaler[4] = {2, 4, 8, 16};

	uint32_t sysClk = RCC_GetSysClkValue();
	uint8_t tmp, ahbp, apb2;

	// ABH prescaler
	tmp = (RCC->CFGR >> 4) & 0x0f;	// HPRE
	//ahbp = tmp < 8 ? 1 : ahb_prescaler[tmp - 8];
	ahbp = tmp < 8 ? 1 : 2 ^ (tmp - 7);

	// APB1 prescaler
	tmp = (RCC->CFGR >> 13) & 0x07;	// PPRE3
	//apb1 = tmp < 4 ? 1 : apb1_prescaler[tmp - 4];
	apb2 = tmp < 4 ? 1 : 2 ^ (tmp - 3);

	return sysClk / ahbp / apb2;
}

// values are close enough only if RCC->CFGR->SW == HSI
void delay(uint32_t ms) {
    for (uint32_t i = 0; i < ms * 471; i++);
}

/*
 * Inaccuracies
 * given 1000, actual 970
 * given  100, actual 102
 * given   10, actual 17
 * given    1, actual 8
 * values are close enough only if RCC->CFGR->SW == HSI
 */
void udelay(uint32_t us) {
    for (uint32_t i = 0; i < us * 3 ; i += 7);
}
