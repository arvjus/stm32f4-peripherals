
#ifndef INC_STM32F407XX_GPIO_H_
#define INC_STM32F407XX_GPIO_H_

#include "stm32f407xx.h"

typedef struct {
	uint8_t number;			/*!< possible values from @GPIO_PIN_NO >*/
	uint8_t mode;			/*!< possible values from @GPIO_PIN_MODES >*/
	uint8_t speed;			/*!< possible values from @GPIO_PIN_OPTYPES >*/
	uint8_t puPdControl;	/*!< possible values from @GPIO_PIN_PUPD >*/
	uint8_t opType;			/*!< possible values from @GPIO_PIN_OPTYPES >*/
	uint8_t altFunMode;		/*!< possible values AF0 - AF15 (see datasheet)  >*/
} GPIO_PinConfig_t;

typedef struct {
	GPIO_RegDef_t *pGPIOx;			// base address to GPIO peripheral
	GPIO_PinConfig_t pinConfig;		// configuration settings
} GPIO_Handle_t;


/*
 * @GPIO_PIN_NO
 * GPIO pin numbers
 */
#define GPIO_PIN_NO_0			0
#define GPIO_PIN_NO_1			1
#define GPIO_PIN_NO_2			2
#define GPIO_PIN_NO_3			3
#define GPIO_PIN_NO_4			4
#define GPIO_PIN_NO_5			5
#define GPIO_PIN_NO_6			6
#define GPIO_PIN_NO_7			7
#define GPIO_PIN_NO_8			8
#define GPIO_PIN_NO_9			9
#define GPIO_PIN_NO_10			10
#define GPIO_PIN_NO_11			11
#define GPIO_PIN_NO_12			12
#define GPIO_PIN_NO_13			13
#define GPIO_PIN_NO_14			14
#define GPIO_PIN_NO_15			15

/*
 * @GPIO_PIN_MODES
 * GPIO pin modes
 */
#define GPIO_PIN_MODE_IN		0
#define GPIO_PIN_MODE_OUT		1
#define GPIO_PIN_MODE_ALTFN		2
#define GPIO_PIN_MODE_ANALOG	3
#define GPIO_PIN_MODE_IT_FT		4
#define GPIO_PIN_MODE_IT_RT		5
#define GPIO_PIN_MODE_IT_RFT	6

/*
 * @GPIO_PIN_SPEEDS
 * GPIO pin output speed
 */
#define	GPIO_PIN_SPEED_LOW		0
#define	GPIO_PIN_SPEED_MEDIUM	1
#define	GPIO_PIN_SPEED_HIGH		2
#define	GPIO_PIN_SPEED_VHIGH	3

/*
 * @GPIO_PIN_PUPD
 * GPIO pin pull up, pull down
 */
#define	GPIO_PIN_NONE_PU_PD		0
#define	GPIO_PIN_PU				1
#define	GPIO_PIN_PD				2

/*
 * @GPIO_PIN_OPTYPES
 * GPIO pin output types
 */
#define	GPIO_PIN_OPTYPE_PP		0
#define	GPIO_PIN_OPTYPE_OD		1


// API supported by GPIO driver ///////////////////////////////////////////////////////////////////
//

/*
 * Peripheral clock control
 */
void GPIO_ClkCtrl(GPIO_RegDef_t *pGPIOx, uint8_t en_di);

/*
 * Init, Reset
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_Reset(GPIO_RegDef_t *pGPIOx);

/*
 * Read, Write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pin);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pin, uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pin);

/*
 * IRQ handling
 */
void GPIO_IRQHandling(uint8_t pin);

#endif /* INC_STM32F407XX_GPIO_H_ */
