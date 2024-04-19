
#ifndef INC_STM32F407XX_SPI_H_
#define INC_STM32F407XX_SPI_H_

#include "stm32f407xx.h"

typedef struct {
	uint8_t deviceMode;		/*!< possible values from @SPI_DeviceMode >*/
	uint8_t busConfig;		/*!< possible values from @SPI_BusConfig >*/
	uint8_t sclkSpeed;		/*!< possible values from @SPI_SclkSpeed >*/
	uint8_t dff;			/*!< possible values from @SPI_DFF >*/
	uint8_t cpol;			/*!< possible values from @SPI_CPOL >*/
	uint8_t cpha;			/*!< possible values from @SPI_CPHA >*/
	uint8_t ssm;			/*!< possible values from @SPI_SSM >*/
} SPI_Config_t;

typedef struct {
	SPI_RegDef_t *pSPIx;		// base address to SPIx peripheral
	SPI_Config_t config;		// configuration settings
	uint8_t		*pTxBuffer;
	uint8_t		*pRxBuffer;
	uint32_t	txLen;
	uint32_t	rxLen;
	uint8_t		txState;
	uint8_t		rxState;
} SPI_Handle_t;


/*
 * SPI application states
 */
#define	SPI_READY			0
#define	SPI_BUSY_IN_TX		1
#define	SPI_BUSY_IN_RX		2


/*
 * SPI application events
 */
#define	SPI_EVENT_TX_CMPLT	1
#define	SPI_EVENT_RX_CMPLT	2
#define	SPI_EVENT_ORV_ERR	3
#define	SPI_EVENT_CRC_ERR	4


/*
 * @SPI_DeviceMode
 */
#define	SPI_DEVICE_MODE_SLAVE	0
#define	SPI_DEVICE_MODE_MASTER	1


/*
 * @SPI_BusConfig
 */
#define	SPI_BUS_CONFIG_FD		0
#define	SPI_BUS_CONFIG_HD		1
#define	SPI_BUS_CONFIG_SIMPLEX_RX	2


/*
 * @SPI_SclkSpeed
 */
#define	SPI_SCLK_SPEED_DIV2		0
#define	SPI_SCLK_SPEED_DIV4		1
#define	SPI_SCLK_SPEED_DIV8		2
#define	SPI_SCLK_SPEED_DIV16	3
#define	SPI_SCLK_SPEED_DIV32	4
#define	SPI_SCLK_SPEED_DIV64	5
#define	SPI_SCLK_SPEED_DIV128	6
#define	SPI_SCLK_SPEED_DIV256	7

/*
 * @SPI_DFF
 */
#define	SPI_DFF_8BIT			0
#define	SPI_DFF_16BIT			1

/*
 * @SPI_CPOL
 */
#define	SPI_CPO_0_IDLE			0
#define	SPI_CPO_1_IDLE			1

/*
 * @SPI_CPHA
 */
#define	SPI_CPHA_1ST_CLK_TRANS	0
#define	SPI_CPHA_2ND_CLK_TRANS	1

/*
 * @SPI_SSM
 */
#define	SPI_SSM_DI				0
#define	SPI_SSM_EN				1

/*
 * @SPI_SSI
 */
#define	SPI_SSI_CLEAR			0
#define	SPI_SSI_SET				1



// API supported by SPI driver ////////////////////////////////////////////////////////////////////
//

/*
 * Peripheral clock control
 */
void SPI_ClkCtrl(SPI_RegDef_t *pSPIx, uint8_t en_di);

/*
 * Init, Reset
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_Reset(SPI_RegDef_t *pSPIx);

/*
 * Data send, receive
 */
void SPI_TransferData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint8_t *pRxBuffer, uint8_t len);
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint8_t len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint8_t len);
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint8_t len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint8_t len);

/*
 * ISR handling
 */
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

/*
 * Other peripheral control APIs
 */
void SPI_PeriphCtrl(SPI_RegDef_t *pSPIx, uint8_t en_di);
void SPI_SSICtrl(SPI_RegDef_t *pSPIx, uint8_t en_di);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t en_di);
uint16_t SPI_GetFlag(SPI_RegDef_t *pSPIx, uint8_t flag);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);
void SPI_ClearOVRFlag(SPI_Handle_t *pSPIHandle);

/*
 * Application event callback
 */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t event);


#endif /* INC_STM32F407XX_SPI_H_ */
