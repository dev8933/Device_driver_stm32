/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Jul 13, 2024
 *      Author: Dkgir
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

//this is a configuration structure for GOPI pin
typedef struct
{
	uint8_t GPIO_PinNumber;				//refer this section @GPIO_PIN_NUMBER
	uint8_t GPIO_PinMode;				//refer this section @GPIO_PIN_MODE
	uint8_t GPIO_PinSpeed;				//refer this section @GPIO_PIN_SPEED
	uint8_t GPIO_PinOPType;				//refer this section @GPIO_OP_TYPE
	uint8_t GPIO_PinPuPdControl;		//refer this section @GPIO_PIN_PUPD
	uint8_t GPIO_PinAltFnMode;			//refer this section @GPIO_PIN_MODE

}GPIO_PinConfig_t;


//this is a Handle structure for GPIO pin
typedef struct
{
	GPIO_RegDef_t *pGPIOx;	// this hold the base address of the GPIO post which the pin belongs
	GPIO_PinConfig_t GPIO_PinConfig;	// this holds the Gpio pin configuration settings

}GPIO_Handle_t;


/*@GPIO_PIN_MODE
GPIO pin possible port mode*/
#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FE		4
#define GPIO_MODE_IT_RE		5
#define GPIO_MODE_IT_RFT	6


// GPIO pin number @GPIO_PIN_NUMBER
#define GPIO_PIN_NO_0 			0
#define GPIO_PIN_NO_1 			1
#define GPIO_PIN_NO_2 			2
#define GPIO_PIN_NO_3			3
#define GPIO_PIN_NO_4			4
#define GPIO_PIN_NO_5 			5
#define GPIO_PIN_NO_6 			6
#define GPIO_PIN_NO_7			7
#define GPIO_PIN_NO_8 			8
#define GPIO_PIN_NO_9 			9
#define GPIO_PIN_NO_10 			10
#define GPIO_PIN_NO_11 			11
#define GPIO_PIN_NO_12 			12
#define GPIO_PIN_NO_13 			13
#define GPIO_PIN_NO_14 			14
#define GPIO_PIN_NO_15 			15


//GPIO pin output type register @GPIO_OP_TYPE
#define GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OD		1


//GPIO pin Possible Speed register @GPIO_PIN_SPEED
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MID		1
#define GPIO_SPEED_HIGH		2
#define GPIO_SPEED_VHIGH	3

//GPIO pin Possible PULLUP/PULL DOWN configration MACROS  @GPIO_PIN_PUPD
#define GPIO_NO_PUPD		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2

/************************************************************************************
 	 	 	 	 	 	 APi's supported by this driver
 	 	 	 for more information check the function definition.
************************************************************************************/

// GPIO peripheral clock setup
 void GPIO_PeriClkControl (GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

// GPIO init and Deinit
 void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
 void GPIO_Deinit(GPIO_RegDef_t *pGPIOx);

// Gpio pin and Port read write

 uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

 uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);

 void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value);

 void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);

 void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

// Interrupt configuration and ISR handling

 void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);

 void GPIO_IRQHandling(uint8_t PinNumber);





















#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
