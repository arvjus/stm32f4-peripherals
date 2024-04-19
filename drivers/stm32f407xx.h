
#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>
#include <stddef.h>
#include <assert.h>

/*
 * Generic definitions
 */
#define	ENABLE	1
#define	DISABLE	0
#define	SET		1
#define	CLEAR	0
#define	HIGH	1
#define	LOW		0

/************************* ARM Cortex-M4 CPU specific *****************************/

/*
 * NVIC register addresses
 */
#define	NVIC_ISER0		((volatile uint32_t *)0xE000E100)
#define	NVIC_ISER1		((volatile uint32_t *)0xE000E104)
#define	NVIC_ISER2		((volatile uint32_t *)0xE000E108)
#define	NVIC_ISER3		((volatile uint32_t *)0xE000E10C)

#define	NVIC_ICER0		((volatile uint32_t *)0xE000E180)
#define	NVIC_ICER1		((volatile uint32_t *)0xE000E184)
#define	NVIC_ICER2		((volatile uint32_t *)0xE000E188)
#define	NVIC_ICER3		((volatile uint32_t *)0xE000E18C)

#define	NVIC_PR_BASEADDR	((volatile uint32_t *)0xE000E400)
#define	NO_PR_BITS_IMPLEMENTED	4


#define NVIC_IRQ_PRIO_0			0
#define NVIC_IRQ_PRIO_15		15


/************************* STM32F407xx MCU specific *******************************/


/*
 * base addresses of Flash and SRAM
 */
#define	FLASH_BASEADDR			0x08000000U
#define	SRAM1_BASEADDR			0x20000000U		/* 112 Kb */
#define	SRAM2_BASEADDR			0x20001C00U		/* 16 Kb */
#define	SRAM_BASEADDR			SRAM1_BASE
#define	ROM_BASEADDR			0x1FFF0000U


/*
 * AHBx and APBx Bus Peripheral base addresses
 */
#define	PERIPH_BASEADDR			0x40000000U
#define	APB1PERIPH_BASEADDR		PERIPH_BASEADDR
#define	APB2PERIPH_BASEADDR		0x40010000U
#define	AHB1PERIPH_BASEADDR		0x40020000U
#define	AHB2PERIPH_BASEADDR		0x50000000U


/*
 * Base addresses of peripherals hanging on APB1 bus
 */
#define	I2C1_BASEADDR			(APB1PERIPH_BASEADDR + 0x5400)
#define	I2C2_BASEADDR			(APB1PERIPH_BASEADDR + 0x5800)
#define	I2C3_BASEADDR			(APB1PERIPH_BASEADDR + 0x5C00)
#define	SPI2_BASEADDR			(APB1PERIPH_BASEADDR + 0x3800)
#define	SPI3_BASEADDR			(APB1PERIPH_BASEADDR + 0x3C00)
#define	USART2_BASEADDR			(APB1PERIPH_BASEADDR + 0x4400)
#define	USART3_BASEADDR			(APB1PERIPH_BASEADDR + 0x4800)
#define	UART4_BASEADDR			(APB1PERIPH_BASEADDR + 0x4C00)
#define	UART5_BASEADDR			(APB1PERIPH_BASEADDR + 0x5000)


/*
 * Base addresses of peripherals hanging on APB2 bus
 */
#define	SPI1_BASEADDR			(APB2PERIPH_BASEADDR + 0x3000)
#define	USART1_BASEADDR			(APB2PERIPH_BASEADDR + 0x1000)
#define	USART6_BASEADDR			(APB2PERIPH_BASEADDR + 0x1400)
#define	EXTI_BASEADDR			(APB2PERIPH_BASEADDR + 0x3C00)
#define	SYSCFG_BASEADDR			(APB2PERIPH_BASEADDR + 0x3800)


/*
 * Base addresses of peripherals hanging on AHB1 bus
 */
#define	GPIOA_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0000)
#define	GPIOB_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0400)
#define	GPIOC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0800)
#define	GPIOD_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0C00)
#define	GPIOE_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1000)
#define	GPIOF_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1400)
#define	GPIOG_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1800)
#define	GPIOH_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1C00)
#define	GPIOI_BASEADDR			(AHB1PERIPH_BASEADDR + 0x2000)
#define	RCC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x3800)


/*
 * RCC_CFGR SWS values
 */
#define RCC_SYSCLK_HSI  0
#define RCC_SYSCLK_HSE  1
#define RCC_SYSCLK_PLL  2


/*
 * Peripheral register definition structures
 */
typedef struct {
	volatile uint32_t	MODER;		//GPIO port mode register
	volatile uint32_t	OTYPER;		//GPIO port output type register
	volatile uint32_t	OSPEEDR;	//GPIO port output speed register
	volatile uint32_t	PUPDR;		//GPIO port pull-up/pull-down register
	volatile uint32_t	IDR;		//GPIO port input data register
	volatile uint32_t	ODR;		//GPIO port output data register
	volatile uint32_t	BSRR;		//GPIO port bit set/reset register
	volatile uint32_t	LCKR;		//GPIO port configuration lock register
	volatile uint32_t	AFR[2];		//GPIO alternate function low + high registers
} GPIO_RegDef_t;
static_assert(sizeof(GPIO_RegDef_t) == 0x28);

typedef struct {
	volatile uint32_t	CR;			//RCC clock control register
	volatile uint32_t	PLLCFGR;	//RCC PLL configuration register
	volatile uint32_t	CFGR;		//RCC clock configuration register
	volatile uint32_t	CIR;		//RCC clock interrupt register
	volatile uint32_t	AHB1RSTR;	//RCC AHB1 peripheral reset register
	volatile uint32_t	AHB2RSTR;	//RCC AHB2 peripheral reset register
	volatile uint32_t	AHB3RSTR;	//RCC AHB3 peripheral reset register
			 uint32_t	reserved1;	//
	volatile uint32_t	APB1RSTR;	//RCC APB1 peripheral reset register
	volatile uint32_t	APB2RSTR;	//RCC APB2 peripheral reset register
			 uint32_t	reserved2[2];//
	volatile uint32_t	AHB1ENR;	//RCC AHB1 peripheral clock register
	volatile uint32_t	AHB2ENR;	//RCC AHB2 peripheral clock enable register
	volatile uint32_t	AHB3ENR;	//RCC AHB3 peripheral clock enable register
			 uint32_t	reserved3;	//
	volatile uint32_t	APB1ENR;	//RCC APB1 peripheral clock enable register
	volatile uint32_t	APB2ENR;	//RCC APB2 peripheral clock enable register
			 uint32_t	reserved4[2];//
	volatile uint32_t	AHB1LPENR;	//RCC AHB1 peripheral clock enable in low power mode register
	volatile uint32_t	AHB2LPENR;	//RCC AHB2 peripheral clock enable in low power mode register
	volatile uint32_t	AHB3LPENR;	//RCC AHB3 peripheral clock enable in low power mode register
			 uint32_t	reserved5;	//
	volatile uint32_t	APB1LPENR;	//RCC APB1 peripheral clock enable in low power mode register
	volatile uint32_t	APB2LPENR;	//RCC APB2 peripheral clock enable in low power mode register
			 uint32_t	reserved6[2];	//
	volatile uint32_t	BDCR;		//RCC Backup domain control register
	volatile uint32_t	CSR;		//RCC clock control & status register
	volatile uint32_t	SSCGR;		//RCC spread spectrum clock generation register
			 uint32_t	reserved7[2];	//
	volatile uint32_t	PLLI2SCFGR;	//RCC PLLI2S configuration register
	volatile uint32_t	PLLSAICFGR;	//RCC PLL configuration register
	volatile uint32_t	DCKCFGR;	//RCC Dedicated Clock Configuration Register
} RCC_RegDef_t;
static_assert(sizeof(RCC_RegDef_t) == 0x90);

typedef struct {
	volatile uint32_t	MEMRMP;		//SYSCFG memory remap register
	volatile uint32_t	PMC;		//SYSCFG peripheral mode configuration register
	volatile uint32_t	EXTICR[4];	//SYSCFG external interrupt configuration register 1-4
			 uint32_t	reserved[2];//
	volatile uint32_t	CMPCR;		//Compensation cell control register
} SYSCFG_RegDef_t;
static_assert(sizeof(SYSCFG_RegDef_t) == 0x24);

typedef struct {
	volatile uint32_t	IMR;		//Interrupt mask register
	volatile uint32_t	EMR;		//Event mask register
	volatile uint32_t	RTSR;		//Rising trigger selection register
	volatile uint32_t	FTSR;		//Falling trigger selection register
	volatile uint32_t	SWIER;		//Software interrupt event register
	volatile uint32_t	PR;			//Pending register
} EXTI_RegDef_t;
static_assert(sizeof(EXTI_RegDef_t) == 0x18);

typedef struct {
	volatile uint32_t	CR1;		//SPI control register 1
	volatile uint32_t	CR2;		//SPI control register 2
	volatile uint32_t	SR;			//SPI status register
	volatile uint32_t	DR;			//SPI data register
	volatile uint32_t	CRCPR;		//SPI CRC polynomial register
	volatile uint32_t	RXCRCR;		//SPI RX CRC register
	volatile uint32_t	TXCRCR;		//SPI TX CRC register
	volatile uint32_t	I2SCFGR;	//SPI_I2S configuration register
	volatile uint32_t	I2SPR;		//SPI_I2S prescaler register
} SPI_RegDef_t;
static_assert(sizeof(SPI_RegDef_t) == 0x24);

typedef struct {
	volatile uint32_t	CR1;		//I2C Control register 1 (I2C_CR1)
	volatile uint32_t	CR2;		//I2C Control register 2 (I2C_CR2)
	volatile uint32_t	OAR1;		//I2C Own address register 1 (I2C_OAR1)
	volatile uint32_t	OAR2;		//I2C Own address register 2 (I2C_OAR2)
	volatile uint32_t	DR;			//I2C Data register (I2C_DR)
	volatile uint32_t	SR1;		//I2C Status register 1 (I2C_SR1)
	volatile uint32_t	SR2;		//I2C Status register 2 (I2C_SR2)
	volatile uint32_t	CCR;		//I2C Clock control register (I2C_CCR)
	volatile uint32_t	TRISE;		//I2C TRISE register (I2C_TRISE)
	volatile uint32_t	FLTR;		//I2C FLTR register (I2C_FLTR)
} I2C_RegDef_t;
static_assert(sizeof(I2C_RegDef_t) == 0x28);

typedef struct {
	volatile uint32_t	SR;			//Status register (USART_SR)
	volatile uint32_t	DR;			//Data register (USART_DR)
	volatile uint32_t	BRR;		//Baud rate register (USART_BRR)
	volatile uint32_t	CR1;		//Control register 1 (USART_CR1)
	volatile uint32_t	CR2;		//Control register 1 (USART_CR2)
	volatile uint32_t	CR3;		//Control register 1 (USART_CR3)
	volatile uint32_t	GTPR;		//Guard time and prescaler register (USART_GTPR)
} USART_RegDef_t;
static_assert(sizeof(USART_RegDef_t) == 0x1C);

/*
 * Peripheral definitions
 */
#define	GPIOA		((GPIO_RegDef_t *)GPIOA_BASEADDR)
#define	GPIOB		((GPIO_RegDef_t *)GPIOB_BASEADDR)
#define	GPIOC		((GPIO_RegDef_t *)GPIOC_BASEADDR)
#define	GPIOD		((GPIO_RegDef_t *)GPIOD_BASEADDR)
#define	GPIOE		((GPIO_RegDef_t *)GPIOE_BASEADDR)
#define	GPIOF		((GPIO_RegDef_t *)GPIOF_BASEADDR)
#define	GPIOG		((GPIO_RegDef_t *)GPIOG_BASEADDR)
#define	GPIOH		((GPIO_RegDef_t *)GPIOH_BASEADDR)
#define	GPIOI		((GPIO_RegDef_t *)GPIOI_BASEADDR)

#define	RCC			((RCC_RegDef_t *)RCC_BASEADDR)
#define SYSCFG		((SYSCFG_RegDef_t *)SYSCFG_BASEADDR)
#define	EXTI		((EXTI_RegDef_t *)EXTI_BASEADDR)

#define	SPI1		((SPI_RegDef_t *)SPI1_BASEADDR)
#define	SPI2		((SPI_RegDef_t *)SPI2_BASEADDR)
#define	SPI3		((SPI_RegDef_t *)SPI3_BASEADDR)

#define	I2C1		((I2C_RegDef_t *)I2C1_BASEADDR)
#define	I2C2		((I2C_RegDef_t *)I2C2_BASEADDR)
#define	I2C3		((I2C_RegDef_t *)I2C3_BASEADDR)

#define	USART1		((USART_RegDef_t *)USART1_BASEADDR)
#define	USART2		((USART_RegDef_t *)USART2_BASEADDR)
#define	USART3		((USART_RegDef_t *)USART3_BASEADDR)
#define	UART4		((USART_RegDef_t *)UART4_BASEADDR)
#define	UART5		((USART_RegDef_t *)UART5_BASEADDR)
#define	USART6		((USART_RegDef_t *)USART6_BASEADDR)


/*
 * Clock Enable macros for GPIOx, I2Cx, SPIx, USARTx, UARTx, SYSCFG
 */
#define	GPIOX_CLK_EN_BIT(bit)	(RCC->AHB1ENR |= (1 << (bit)))
#define	GPIOA_CLK_EN()			(GPIOX_CLK_EN_BIT(0))
#define	GPIOB_CLK_EN()			(GPIOX_CLK_EN_BIT(1))
#define	GPIOC_CLK_EN()			(GPIOX_CLK_EN_BIT(2))
#define	GPIOD_CLK_EN()			(GPIOX_CLK_EN_BIT(3))
#define	GPIOE_CLK_EN()			(GPIOX_CLK_EN_BIT(4))
#define	GPIOF_CLK_EN()			(GPIOX_CLK_EN_BIT(5))
#define	GPIOG_CLK_EN()			(GPIOX_CLK_EN_BIT(6))
#define	GPIOH_CLK_EN()			(GPIOX_CLK_EN_BIT(7))
#define	GPIOI_CLK_EN()			(GPIOX_CLK_EN_BIT(8))

#define	I2C1_CLK_EN()			(RCC->APB1ENR |= (1 << 21))
#define	I2C2_CLK_EN()			(RCC->APB1ENR |= (1 << 22))
#define	I2C3_CLK_EN()			(RCC->APB1ENR |= (1 << 23))

#define	SPI1_CLK_EN()			(RCC->APB2ENR |= (1 << 12))
#define	SPI2_CLK_EN()			(RCC->APB1ENR |= (1 << 14))
#define	SPI3_CLK_EN()			(RCC->APB1ENR |= (1 << 15))

#define	USART1_CLK_EN()			(RCC->APB2ENR |= (1 << 4))
#define	USART2_CLK_EN()			(RCC->APB1ENR |= (1 << 17))
#define	USART3_CLK_EN()			(RCC->APB1ENR |= (1 << 18))
#define	UART4_CLK_EN()			(RCC->APB1ENR |= (1 << 19))
#define	UART5_CLK_EN()			(RCC->APB1ENR |= (1 << 20))
#define	USART6_CLK_EN()			(RCC->APB2ENR |= (1 << 5))

#define	SYSCFG_CLK_EN()			(RCC->APB2ENR |= (1 << 14))


/*
 * Clock Disable macros for GPIOx, I2Cx, SPIx, USARTx, UARTx, SYSCFG
 */
#define	GPIOX_CLK_DI_BIT(bit)	(RCC->AHB1ENR &= ~(1 << (bit)))
#define	GPIOA_CLK_DI()			(GPIOX_CLK_DI_BIT(0))
#define	GPIOB_CLK_DI()			(GPIOX_CLK_DI_BIT(1))
#define	GPIOC_CLK_DI()			(GPIOX_CLK_DI_BIT(2))
#define	GPIOD_CLK_DI()			(GPIOX_CLK_DI_BIT(3))
#define	GPIOE_CLK_DI()			(GPIOX_CLK_DI_BIT(4))
#define	GPIOF_CLK_DI()			(GPIOX_CLK_DI_BIT(5))
#define	GPIOG_CLK_DI()			(GPIOX_CLK_DI_BIT(6))
#define	GPIOH_CLK_DI()			(GPIOX_CLK_DI_BIT(7))
#define	GPIOI_CLK_DI()			(GPIOX_CLK_DI_BIT(8))

#define	I2C1_CLK_DI()			(RCC->APB1ENR &= ~(1 << 21))
#define	I2C2_CLK_DI()			(RCC->APB1ENR &= ~(1 << 22))
#define	I2C3_CLK_DI()			(RCC->APB1ENR &= ~(1 << 23))

#define	SPI1_CLK_DI()			(RCC->APB2ENR &= ~(1 << 12))
#define	SPI2_CLK_DI()			(RCC->APB1ENR &= ~(1 << 14))
#define	SPI3_CLK_DI()			(RCC->APB1ENR &= ~(1 << 15))

#define	USART1_CLK_DI()			(RCC->APB2ENR &= ~(1 << 4))
#define	USART2_CLK_DI()			(RCC->APB1ENR &= ~(1 << 17))
#define	USART3_CLK_DI()			(RCC->APB1ENR &= ~(1 << 18))
#define	UART4_CLK_DI()			(RCC->APB1ENR &= ~(1 << 19))
#define	UART5_CLK_DI()			(RCC->APB1ENR &= ~(1 << 20))
#define	USART6_CLK_DI()			(RCC->APB2ENR &= ~(1 << 5))

#define	SYSCFG_CLK_DI()			(RCC->APB2ENR &= ~(1 << 14))


/*
 * Reset macros
 */
#define	GPIOX_REG_RESET_BIT(bit)	do { (RCC->AHB1RSTR |= (1 << (bit))); (RCC->AHB1RSTR &= ~(1 << (bit))); } while(0)
#define	GPIOA_REG_RESET()			(GPIOX_REG_RESET_BIT(0))
#define	GPIOB_REG_RESET()			(GPIOX_REG_RESET_BIT(1))
#define	GPIOC_REG_RESET()			(GPIOX_REG_RESET_BIT(2))
#define	GPIOD_REG_RESET()			(GPIOX_REG_RESET_BIT(3))
#define	GPIOE_REG_RESET()			(GPIOX_REG_RESET_BIT(4))
#define	GPIOF_REG_RESET()			(GPIOX_REG_RESET_BIT(5))
#define	GPIOG_REG_RESET()			(GPIOX_REG_RESET_BIT(6))
#define	GPIOH_REG_RESET()			(GPIOX_REG_RESET_BIT(7))
#define	GPIOI_REG_RESET()			(GPIOX_REG_RESET_BIT(8))


/*
 * Bit position definitions
 */

// SPI_CR1 bits
#define	SPI_CR1_CPHA		0
#define	SPI_CR1_CPOL		1
#define	SPI_CR1_MSTR		2
#define	SPI_CR1_BR			3
#define	SPI_CR1_SPE			6
#define	SPI_CR1_LSB_FIRST	7
#define	SPI_CR1_SSI			8
#define	SPI_CR1_SSM			9
#define	SPI_CR1_RX_ONLY		10
#define	SPI_CR1_DFF			11
#define	SPI_CR1_CRC_NEXT	12
#define	SPI_CR1_CRC_EN		13
#define	SPI_CR1_BIDI_OE		14
#define	SPI_CR1_BIDI_MODE	15

// SPI_CR2 bits
#define	SPI_CR2_RXDMAEN		0
#define	SPI_CR2_TXDMAEN		1
#define	SPI_CR2_SSOE		2
#define	SPI_CR2_FRF			4
#define	SPI_CR2_ERRIE		5
#define	SPI_CR2_RXNEIE		6
#define	SPI_CR2_TXEIE		7

// SPI_SR bits
#define	SPI_SR_RXNE			0
#define	SPI_SR_TXE			1
#define	SPI_SR_CHSIDE		2
#define	SPI_SR_UDR			3
#define	SPI_SR_CRC_ERR		4
#define	SPI_SR_MODF			5
#define	SPI_SR_OVR			6
#define	SPI_SR_BSY			7
#define	SPI_SR_FRE			8

#define	SPI_SR_RXNE_FLAG	(1 << SPI_SR_RXNE)
#define	SPI_SR_TXE_FLAG		(1 << SPI_SR_TXE)
#define	SPI_SR_CHSIDE_FLAG	(1 << SPI_SR_CHSIDE)
#define	SPI_SR_UDR_FLAG		(1 << SPI_SR_UDR)
#define	SPI_SR_CRC_ERR_FLAG	(1 << SPI_SR_CRC_ERR)
#define	SPI_SR_MODF_FLAG	(1 << SPI_SR_MODF)
#define	SPI_SR_OVR_FLAG		(1 << SPI_SR_OVR)
#define	SPI_SR_BSY_FLAG		(1 << SPI_SR_BSY)
#define	SPI_SR_FRE_FLAG		(1 << SPI_SR_FRE)


// I2C_CR1 bits
#define	I2C_CR1_PE				0
#define	I2C_CR1_NOSTRETCH		7
#define	I2C_CR1_START			8
#define	I2C_CR1_STOP			9
#define	I2C_CR1_ACK				10
#define	I2C_CR1_SWRST			15

// I2C_CR2 bits
#define I2C_CR2_FREQ			0
#define I2C_CR2_ITERREN			8
#define I2C_CR2_ITEVTEN			9
#define I2C_CR2_ITBUFEN 		10

// I2C_SR1 bits
#define I2C_SR1_SB 				0
#define I2C_SR1_ADDR 			1
#define I2C_SR1_BTF 			2
#define I2C_SR1_ADD10 			3
#define I2C_SR1_STOPF 			4
#define I2C_SR1_RXNE 			6
#define I2C_SR1_TXE 			7
#define I2C_SR1_BERR 			8
#define I2C_SR1_ARLO 			9
#define I2C_SR1_AF 				10
#define I2C_SR1_OVR 			11
#define I2C_SR1_TIMEOUT 		14

#define I2C_SR1_SB_FLAG 		(1 << I2C_SR1_SB)
#define I2C_SR1_ADDR_FLAG 		(1 << I2C_SR1_ADDR)
#define I2C_SR1_BTF_FLAG 		(1 << I2C_SR1_BTF)
#define I2C_SR1_ADD10_FLAG 		(1 << I2C_SR1_ADD10)
#define I2C_SR1_STOPF_FLAG 		(1 << I2C_SR1_STOPF)
#define I2C_SR1_RXNE_FLAG 		(1 << I2C_SR1_RXNE)
#define I2C_SR1_TXE_FLAG 		(1 << I2C_SR1_TXE)
#define I2C_SR1_BERR_FLAG 		(1 << I2C_SR1_BERR)
#define I2C_SR1_ARLO_FLAG 		(1 << I2C_SR1_ARLO)
#define I2C_SR1_AF_FLAG      	(1 << I2C_SR1_AF)
#define I2C_SR1_OVR_FLAG 		(1 << I2C_SR1_OVR)
#define I2C_SR1_TIMEOUT_FLAG 	(1 << I2C_SR1_TIMEOUT)

// I2C_SR2 bits
#define I2C_SR2_MSL				0
#define I2C_SR2_BUSY 			1
#define I2C_SR2_TRA 			2
#define I2C_SR2_GENCALL 		4
#define I2C_SR2_DUALF 			7

#define I2C_SR2_MSL_FLAG		(1 << I2C_SR2_MSL)
#define I2C_SR2_BUSY_FLAG 		(1 << I2C_SR2_BUSY)
#define I2C_SR2_TRA_FLAG 		(1 << I2C_SR2_TRA)
#define I2C_SR2_GENCALL_FLAG	(1 << I2C_SR2_GENCALL)
#define I2C_SR2_DUALF_FLAG 		(1 << I2C_SR2_DUALF)

// I2C_CCR bits
#define I2C_CCR_CCR 			0
#define I2C_CCR_DUTY 			14
#define I2C_CCR_FS  			15

// USART_SR bits
#define USART_SR_PE 			0
#define USART_SR_FE 			1
#define USART_SR_NF 			2
#define USART_SR_ORE 			3
#define USART_SR_IDLE 			4
#define USART_SR_RXNE 			5
#define USART_SR_TC 			6
#define USART_SR_TXE 			7
#define USART_SR_LBD 			8
#define USART_SR_CTS 			9

#define USART_SR_PE_FLAG		(1 << USART_SR_PE)
#define USART_SR_FE_FLAG		(1 << USART_SR_FE)
#define USART_SR_NF_FLAG		(1 << USART_SR_NF)
#define USART_SR_ORE_FLAG		(1 << USART_SR_ORE)
#define USART_SR_IDLE_FLAG		(1 << USART_SR_IDLE)
#define USART_SR_RXNE_FLAG		(1 << USART_SR_RXNE)
#define USART_SR_TC_FLAG		(1 << USART_SR_TC)
#define USART_SR_TXE_FLAG		(1 << USART_SR_TXE)
#define USART_SR_LBD_FLAG		(1 << USART_SR_LBD)
#define USART_SR_CTS_FLAG		(1 << USART_SR_CTS)

// USART_CR1 bits
#define USART_CR1_SBK			0
#define USART_CR1_RWU 			1
#define USART_CR1_RE  			2
#define USART_CR1_TE 			3
#define USART_CR1_IDLEIE 		4
#define USART_CR1_RXNEIE  		5
#define USART_CR1_TCIE			6
#define USART_CR1_TXEIE			7
#define USART_CR1_PEIE 			8
#define USART_CR1_PS 			9
#define USART_CR1_PCE 			10
#define USART_CR1_WAKE  		11
#define USART_CR1_M 			12
#define USART_CR1_UE 			13
#define USART_CR1_OVER8  		15

// USART_CR2 bits
#define USART_CR2_ADD   		0
#define USART_CR2_LBDL   		5
#define USART_CR2_LBDIE  		6
#define USART_CR2_LBCL   		8
#define USART_CR2_CPHA   		9
#define USART_CR2_CPOL   		10
#define USART_CR2_STOP   		12
#define USART_CR2_LINEN   		14

// USART_CR3 bits
#define USART_CR3_EIE   		0
#define USART_CR3_IREN   		1
#define USART_CR3_IRLP  		2
#define USART_CR3_HDSEL   		3
#define USART_CR3_NACK   		4
#define USART_CR3_SCEN   		5
#define USART_CR3_DMAR  		6
#define USART_CR3_DMAT   		7
#define USART_CR3_RTSE   		8
#define USART_CR3_CTSE   		9
#define USART_CR3_CTSIE   		10
#define USART_CR3_ONEBIT   		§11


/*
 * IRQ numbers of STM32F407xx MCU
 */
#define	IRQ_NO_EXTI0		6
#define	IRQ_NO_EXTI1		7
#define	IRQ_NO_EXTI2		8
#define	IRQ_NO_EXTI3		9
#define	IRQ_NO_EXTI4		10
#define	IRQ_NO_EXTI9_5		23
#define	IRQ_NO_EXTI15_10	40

#define IRQ_NO_SPI1         35
#define IRQ_NO_SPI2         36
#define IRQ_NO_SPI3         51
#define IRQ_NO_SPI4

#define IRQ_NO_I2C1_EV    	31
#define IRQ_NO_I2C1_ER    	32

#define IRQ_NO_USART1       37
#define IRQ_NO_USART2       38
#define IRQ_NO_USART3       39
#define IRQ_NO_UART4        52
#define IRQ_NO_UART5        53
#define IRQ_NO_USART6       71


/*
 * IRQ configuration
 */
void IRQEnable(uint8_t irqNumber, uint8_t en_di);
void IRQPriority(uint8_t irqNumber, uint32_t irqPriority);

/*
 * PCLKx
 */
void RCC_SysClkSwitch(uint8_t source);
uint32_t RCC_GetSysClkValue();
uint32_t RCC_GetPCLK1Value();
uint32_t RCC_GetPCLK2Value();


/*
 * Delay
 */
void delay(uint32_t ms);
void udelay(uint32_t us);

#endif /* INC_STM32F407XX_H_ */
