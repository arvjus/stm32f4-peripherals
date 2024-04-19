
#ifndef INC_STM32F407XX_USART_H_
#define INC_STM32F407XX_USART_H_

#include "stm32f407xx.h"

typedef struct {
	uint8_t		mode;				/*!< possible values from @USART_Mode >*/
	uint32_t	baudRate;			/*!< possible values from @USART_Baud >*/
	uint8_t		stopBits;			/*!< possible values from @USART_NoOfStopBits >*/
	uint8_t		wordLength;			/*!< possible values from @USART_WordLength >*/
	uint8_t		parityCtrl;			/*!< possible values from @USART_ParityControl >*/
	uint8_t		hwFlowCtrl;			/*!< possible values from @USART_HWFlowControl >*/
} USART_Config_t;

typedef struct {
	USART_RegDef_t	*pUSARTx;
	USART_Config_t 	config;
	uint8_t			*pTxBuffer;
	uint8_t			*pRxBuffer;
	uint32_t		txLen;
	uint32_t		rxLen;
	uint8_t			txState;
	uint8_t			rxState;
} USART_Handle_t;


/*
 * USART application states
 */
#define	USART_READY				0
#define	USART_BUSY_IN_TX		1
#define	USART_BUSY_IN_RX		2


/*
 * USART application events
 */
#define	USART_EVENT_TX_CMPLT	1
#define	USART_EVENT_RX_CMPLT	2
#define	USART_EVENT_CTS			3
#define	USART_EVENT_IDLE		4
#define	USART_EVENT_ORE			5
#define	USART_ERROR_PE			6
#define	USART_ERROR_FE			7
#define	USART_ERROR_NF			8
#define	USART_ERROR_ORE			9

/*
 * @USART_Mode
 */
#define USART_MODE_ONLY_TX		0
#define USART_MODE_ONLY_RX		1
#define USART_MODE_TXRX			2

/*
 * @USART_Baud
 */
#define USART_STD_BAUD_1200					1200
#define USART_STD_BAUD_2400					2400
#define USART_STD_BAUD_9600					9600
#define USART_STD_BAUD_19200 				19200
#define USART_STD_BAUD_38400 				38400
#define USART_STD_BAUD_57600 				57600
#define USART_STD_BAUD_115200 				115200
#define USART_STD_BAUD_230400 				230400
#define USART_STD_BAUD_460800 				460800
#define USART_STD_BAUD_921600 				921600
#define USART_STD_BAUD_2M 					2000000
#define SUART_STD_BAUD_3M 					3000000


/*
 * @USART_ParityControl
 */
#define USART_PARITY_DI			0
#define USART_PARITY_ODD		1
#define USART_PARITY_EVEN		2

/*
 * @USART_WordLength
 */
#define USART_WORDLEN_8BITS		0
#define USART_WORDLEN_9BITS		1

/*
 * @USART_NoOfStopBits
 */
#define USART_STOPBITS_1		0
#define USART_STOPBITS_0_5		1
#define USART_STOPBITS_2		2
#define USART_STOPBITS_1_5		3

/*
 * @USART_HWFlowControl
 */
#define USART_HW_FLOW_CTRL_NONE    	0
#define USART_HW_FLOW_CTRL_CTS    	1
#define USART_HW_FLOW_CTRL_RTS    	2
#define USART_HW_FLOW_CTRL_CTS_RTS	3



// API supported by USART driver ////////////////////////////////////////////////////////////////////
//

/*
 * Peripheral clock control
 */
void USART_ClkCtrl(USART_RegDef_t *pUSARTx, uint8_t en_di);

/*
 * Init, Reset
 */
void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_Reset(USART_RegDef_t *pUSARTx);

/*
 * Data send, receive
 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint8_t len);
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint8_t len);
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint8_t len);
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint8_t len);

/*
 * ISR handling
 */
void USART_IRQHandling(USART_Handle_t *pUSARTHandle);

/*
 * Other peripheral control APIs
 */
void USART_PeriphCtrl(USART_RegDef_t *pUSARTx, uint8_t en_di);
uint16_t USART_GetFlag(USART_RegDef_t *pUSARTx, uint16_t flag);
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t flag);

/*
 * Application event callback
 */
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t event);


#endif /* INC_STM32F407XX_USART_H_ */
