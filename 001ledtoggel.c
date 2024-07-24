/*
 * 001ledtoggel.c
 *
 *  Created on: 18-Jul-2024
 *      Author: Dkgir
 */

//#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"

void delay(void)
{

	 for (uint32_t i=0; i<500000; i++);
}


int main(void)
{
	// Code for led on Port D pin 12 with no pupd
	GPIO_Handle_t GpioLed;
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType =GPIO_OP_TYPE_PP;

	// Code for activating Buzzer
	GPIO_Handle_t Gpiobuzz;
	Gpiobuzz.pGPIOx = GPIOA;
	Gpiobuzz.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_1;
	Gpiobuzz.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	Gpiobuzz.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	Gpiobuzz.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	Gpiobuzz.GPIO_PinConfig.GPIO_PinOPType =GPIO_OP_TYPE_PP;

	/**Gpio config for Rain water sensor
	 * Rain sensor gives High(1) when there is No water, the on board led will be off
	 * when there is water the output is Low (0), on board Led will be ON
	 */
#define NOWATER		1
#define WATER		0
	GPIO_Handle_t Gpio_Rain_sensor;
	Gpio_Rain_sensor.pGPIOx =GPIOA;
	Gpio_Rain_sensor.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
	Gpio_Rain_sensor.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	Gpio_Rain_sensor.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	Gpio_Rain_sensor.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD ;


	// to enable clock for peripheral, we can use single port as well
	GPIO_PeriClkControl(GPIOD, ENABLE);
	GPIO_PeriClkControl(GPIOA, ENABLE);
	GPIO_Init(&GpioLed);
	GPIO_Init(&Gpiobuzz);
	GPIO_Init(&Gpio_Rain_sensor);

	while(1)
	{
		uint8_t rainDetected = GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_9);

		if (rainDetected == NOWATER)
		{
			GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_1, GPIO_PIN_SET);
			delay();
		}
		else
		{
			GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_1, GPIO_PIN_RESET);
		}
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
		delay();
		//GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_1);
	}

	return 0;
}
