
#ifndef INC_STM32F407XX_I2C_H_
#define INC_STM32F407XX_I2C_H_

#include <stdbool.h>
#include "stm32f407xx.h"

typedef struct {
	uint32_t	sclSpeed;		/*!< possible values from @I2C_SclSpeed >*/
	uint8_t		deviceAddress;	/*!< possible values set by the user - 7 bits >*/
	uint8_t		ackControl;		/*!< possible values from @I2C_AckControl >*/
	uint8_t		fmDutyCycle;	/*!< possible values from @I2C_FMDutyCycle >*/

} I2C_Config_t;

typedef struct {
	I2C_RegDef_t	*pI2Cx;
	I2C_Config_t 	config;
	uint8_t			*pTxBuffer;
	uint8_t			*pRxBuffer;
	uint32_t		txLen;
	uint32_t		rxLen;
	uint32_t		rxSize;
	uint8_t			state;
	uint8_t			devAddress;
	uint8_t			sr;
} I2C_Handle_t;

/*
 * I2C application states
 */
#define	I2C_READY			0
#define	I2C_BUSY_IN_TX		1
#define	I2C_BUSY_IN_RX		2

/*
 * I2C application events
 */
#define	I2C_EVENT_TX_CMPLT	1
#define	I2C_EVENT_RX_CMPLT	2
#define	I2C_EVENT_STOP		3
#define	I2C_EVENT_DATA_REQ	4
#define	I2C_EVENT_DATA_RCV	5

#define I2C_ERROR_BERR  	6
#define I2C_ERROR_ARLO  	7
#define I2C_ERROR_AF    	8
#define I2C_ERROR_OVR   	9
#define I2C_ERROR_TIMEOUT 	10

/*
 * @I2C_SclSpeed
 */
#define	I2C_SCL_SPEED_SM	100000
#define	I2C_SCL_SPEED_FM	400000

/*
 * @I2C_AckControl
 */
#define	I2C_ACK_DI			0
#define	I2C_ACK_EN			1

/*
 * @I2C_FMDutyCycle
 */
#define	I2C_FM_DUTY_2		0
#define	I2C_FM_DUTY_16_9	1


/*
 * Repeated START
 */
#define	I2C_SR_DI		0
#define	I2C_SR_EN		1


// API supported by I2C driver ////////////////////////////////////////////////////////////////////
//

/*
 * Peripheral clock control
 */
void I2C_ClkCtrl(I2C_RegDef_t *pI2Cx, uint8_t en_di);

/*
 * Init, Reset
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_Reset(I2C_RegDef_t *pI2Cx);

/*
 * Data send, receive
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t slaveAddress, uint8_t sr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t len, uint8_t slaveAddress, uint8_t sr);
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t slaveAddress, uint8_t sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t len, uint8_t slaveAddress, uint8_t sr);

void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx);


/*
 * ISR handling
 */
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);

/*
 * Other peripheral control APIs
 */
void I2C_PeriphCtrl(I2C_RegDef_t *pI2Cx, uint8_t en_di);
uint16_t I2C_GetFlag1(I2C_RegDef_t *pI2Cx, uint8_t flag);
uint16_t I2C_GetFlag2(I2C_RegDef_t *pI2Cx, uint8_t flag);
void I2C_AckCtrl(I2C_RegDef_t *pI2Cx, uint8_t en_di);
void I2C_EventCtrl(I2C_RegDef_t *pI2Cx, uint8_t en_di);
bool I2C_MasterProbe(I2C_Handle_t *pI2CHandle, uint8_t slaveAddress);

/*
 * Application event callback
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t event);



#endif /* INC_STM32F407XX_I2C_H_ */
