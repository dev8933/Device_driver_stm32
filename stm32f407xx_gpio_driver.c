/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Jul 13, 2024
 *      Author: Dkgir
 */

#include "stm32f407xx_gpio_driver.h"

/****************************************************************
 * @fn				-GPIO_PeriClkControl
 *
 * @brief			-this function Enable or Disable the Peripheral clock for the GPIO port
 *
 * @param[IN]		-Base address of GPIO peripheral
 * @param[IN]		- ENABLE or DISABLE Macros
 * @param[IN]		-
 *
 * @return			-None
 *
 *
 * @note			-none
 * GPIO peripheral clock setup */


 void GPIO_PeriClkControl (GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
 {
	 if (EnorDi == ENABLE)
	 {
		 if (pGPIOx == GPIOA)
		 {
			 GPIOA_PCLK_EN();
		 }else if(pGPIOx == GPIOB)
		 {
			 GPIOB_PCLK_EN();
		 }
		 else if(pGPIOx == GPIOC)
		 {
			 GPIOC_PCLK_EN();
		 }
		 else if(pGPIOx == GPIOD)
	 	 {
			 GPIOD_PCLK_EN();
	 	 }
	 	 else if(pGPIOx == GPIOE)
	 	 {
	 		 GPIOE_PCLK_EN();
	 	 }
	 	 else if(pGPIOx == GPIOF)
 		 {
 			 GPIOF_PCLK_EN();
 		 }
 		 else if(pGPIOx == GPIOG)
 		 {
 			 GPIOG_PCLK_EN();
 		 }
 		 else if(pGPIOx == GPIOH)
	 	 {
 			 GPIOH_PCLK_EN();
	 	 }
	 	 else if(pGPIOx == GPIOI)
	 	 {
	 		GPIOI_PCLK_EN();
	 	 }
	     else if(pGPIOx == GPIOJ)
	 	 {
	 		GPIOJ_PCLK_EN();
	 	 }
	 	 else if(pGPIOx == GPIOK)
	 	 {
	 	    GPIOK_PCLK_EN();
	 	 }
	 }
	 else
	 {
		 	 	 if (pGPIOx == GPIOA)
				 {
					 GPIOA_PCLK_DI();
				 }else if(pGPIOx == GPIOB)
				 {
					 GPIOB_PCLK_DI();
				 }
				 else if(pGPIOx == GPIOC)
				 {
					 GPIOC_PCLK_DI();
				 }
				 else if(pGPIOx == GPIOD)
			 	 {
					 GPIOD_PCLK_DI();
			 	 }
			 	 else if(pGPIOx == GPIOE)
			 	 {
			 		 GPIOE_PCLK_DI();
			 	 }
			 	 else if(pGPIOx == GPIOF)
		 		 {
		 			 GPIOF_PCLK_DI();
		 		 }
		 		 else if(pGPIOx == GPIOG)
		 		 {
		 			 GPIOG_PCLK_DI();
		 		 }
		 		 else if(pGPIOx == GPIOH)
			 	 {
		 			 GPIOH_PCLK_DI();
			 	 }
			 	 else if(pGPIOx == GPIOI)
			 	 {
			 		GPIOI_PCLK_DI();
			 	 }
			     else if(pGPIOx == GPIOJ)
			 	 {
			 		GPIOJ_PCLK_DI();
			 	 }
			 	 else if(pGPIOx == GPIOK)
			 	 {
			 	    GPIOK_PCLK_DI();
			 	 }
	 }

 }

 /****************************************************************
  * @fn				-GPIO_Init
  *
  * @brief			-
  *
  * @param[IN]		-Base address of GPIO
  * @param[IN]		-none
  * @param[IN]		-none
  *
  * @return			-none
  *
  *
  * @note			-none
  */
// GPIO init and Deinit
 void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
 {
	 // configure the mode of GPIO pin
	 uint32_t temp=0;

	 if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	 {
		 temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		 pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);  //clearing
		 pGPIOHandle->pGPIOx->MODER |= temp;	//setting

	 }
	 else
	 {
		 if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FE)
		 {
			 //configure FTSR register
			 EXTI->FTSR |= (1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			 EXTI->RTSR &= ~(1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
		 }
		 else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RE)
		 {
			 //configure RTST register
			 EXTI->RTSR |= (1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			 EXTI->FTSR &= ~(1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
		 }
		 else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		 {
			 // configure both FTSR and RTSR register
			 EXTI->RTSR |= (1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			 EXTI->FTSR |= (1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
		 }
		 //2. Configure the gpio port selection register SYSCFG_EXTICR
		 uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber /4;
		 uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber %4;
		 uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);

		 SYSCFG_PCLK_EN();
		 SYSCFG->EXTICR[temp1] = portcode << (temp2*4);



		 //3.enable the EXTI interrupt delivery using IMR
		 EXTI->IMR |= 1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	 }// end of else block


	 temp =0;
	 // configure the speed
	 temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	 	 	 pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);  //clearing
	 		 pGPIOHandle->pGPIOx->OSPEEDR |= temp;	//setting

	 temp =0;
	 // configure the PUPD settings
	 temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	 	 	 	 pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);  //clearing
	 	 		 pGPIOHandle->pGPIOx->PUPDR |= temp;		//setting

	 temp =0;
	 // configure the OP type
	 temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << ( pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	 	 	 	 	 pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);  //clearing111
	 	 	 		 pGPIOHandle->pGPIOx->OTYPER |= temp;		//setting

	 temp=0;
	 // configure the ALT functionality
	 if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ALTFN)
		 {
			 uint8_t temp1, temp2;
			 temp1 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber /8);
			 temp2 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber %8);
			 pGPIOHandle->pGPIOx->AFR[temp1] &=  ~( 0xF << (4* temp2));
			 pGPIOHandle->pGPIOx->AFR[temp1] |=  (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFnMode << (4* temp2));

		 }
 }

 /****************************************************************
   * @fn				-GPIO_Deinit
   *
   * @brief			- it Deinitialize the GPIO
   *
   * @param[IN]		- base address of GPIO
   * @param[IN]		-
   * @param[IN]		-
   *
   * @return			-
   *
   *
   * @note			-none
   */
  void GPIO_Deinit(GPIO_RegDef_t *pGPIOx)
  {

 			 if (pGPIOx == GPIOA)
 			 {
 				 GPIOA_REG_RESET();
 			 }else if(pGPIOx == GPIOB)
 			 {
 				 GPIOA_REG_RESET();
 			 }
 			 else if(pGPIOx == GPIOC)
 			 {
 				 GPIOA_REG_RESET();
 			 }
 			 else if(pGPIOx == GPIOD)
 		 	 {
 				 GPIOA_REG_RESET();
 		 	 }
 		 	 else if(pGPIOx == GPIOE)
 		 	 {
 		 		GPIOA_REG_RESET();
 		 	 }
 		 	 else if(pGPIOx == GPIOF)
 	 		 {
 		 		GPIOA_REG_RESET();
 	 		 }
 	 		 else if(pGPIOx == GPIOG)
 	 		 {
 	 			GPIOA_REG_RESET();
 	 		 }
 	 		 else if(pGPIOx == GPIOH)
 		 	 {
 	 			GPIOA_REG_RESET();
 		 	 }
 		 	 else if(pGPIOx == GPIOI)
 		 	 {
 		 		GPIOA_REG_RESET();
 		 	 }
 		     else if(pGPIOx == GPIOJ)
 		 	 {
 		    	 GPIOA_REG_RESET();
 		 	 }
 		 	 else if(pGPIOx == GPIOK)
 		 	 {
 		 		GPIOA_REG_RESET();
 		 	 }
  	  }

  // Gpio pin and Port read write
   /****************************************************************
    * @fn				-GPIO_ReadFromInputPin
    *
    * @brief			- from the pin number we need to read the input to the
    * 				the input data register
    *
    * @param[IN]		- pGPIOx @ base address of gpio
    * @param[IN]		- PinNumber @ which pin to read from
    * @param[IN]		-
    *
    * @return			-
    *
    *
    * @note			-none
    */
   uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
   {
  	 uint8_t value;
  	 value = ((pGPIOx->IDR >> PinNumber ) & 0x00000001); // to extract the data at pin number we shift that pin to LSB
  	 return value;													//and the extract it by & it with 0x00000001

   }


   /****************************************************************
    * @fn				-GPIO_ReadFromInputPort
    *
    * @brief			-
    *
    * @param[IN]		-
    * @param[IN]		-
    * @param[IN]		-
    *
    * @return			-
    *
    *
    * @note			-none
    */
   uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
   {
  	uint16_t value;
  	value = (uint16_t)pGPIOx->IDR; // to extract the data at input data port
  	return value;												//and the extract it by & it with 0x00000001
   }

/****************************************************************
     * @fn				-GPIO_WriteToOutputPin
     *
     * @brief			-
     *
     * @param[IN]		-
     * @param[IN]		-
     * @param[IN]		-
     *
     * @return			-
     *
     *
     * @note			-none
     */
    void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value)
    {
   	 if (value == GPIO_PIN_SET)
   	 {// write 1 to the output data register at the bit field corresponding to the pin number

   		 pGPIOx->ODR |=(1 << PinNumber);
   	 }
   	 else
   	 {
   		 // write 0
   		 pGPIOx->ODR &= ~(1 << PinNumber);
   	 }

    }

/****************************************************************
     * @fn				-GPIO_WriteToOutputPort
     *
     * @brief			-
     *
     * @param[IN]		-
     * @param[IN]		-
     * @param[IN]		-
     *
     * @return			-
     *
     *
     * @note			-none
     */
    void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value)
    {

   	 pGPIOx->ODR = value;

    }

/****************************************************************
     * @fn				-GPIO_ToggleOutputPin
     *
     * @brief			-
     *
     * @param[IN]		-
     * @param[IN]		-
     * @param[IN]		-
     *
     * @return			-
     *
     *
     * @note			-none
     */
    void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
    {

   	 pGPIOx->ODR ^= (1 <<  PinNumber);

    }
