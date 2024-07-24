/*
 * stm32f407xx.h
 *
 *  Created on: Jul 13, 2024
 *      Author: Dkgir
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_9

#include <stdint.h>
/*
Base address of flash and SRAM memories
you can look this in refrence manual of micro
*/
#define ENABLE			1
#define DISABLE 		0
#define SET				ENABLE
#define RESET 			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET 	RESET
#define FLASH_BASEADDR		0x08000000U
#define SRAM1_BASEADDR		0x20000000U
#define SRAM2_BASEADDR		0x2001C000U			// sram2 starts after sram 1(112kb - (112*1024)= 1c000 convert in hex)
#define ROM					0x1FFF0000U			// base add of ROM main memory called as system memory
#define SRAM 				SRAM1_BASEADDR

/*
Base address of AHBx and APBx bus peripheral
you can look this in rf manual it starts from 0x4000 0000
*/

#define APB1PERIPH_BASE		0x40000000U
#define APB2PERIPH_BASE		0x40010000U
#define AHB1PERIPH_BASE		0x40020000U
#define AHB2PERIPH_BASE		0x50000000U

/*
Base address of  peripheral which are hanging on AHB1 bus
*/
#define GPIOA_BASEADDR 			(AHB1PERIPH_BASE + 0x0000)			// base addr of ahb1 + offset
#define GPIOB_BASEADDR 			(AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR 			(AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR 			(AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASEADDR 			(AHB1PERIPH_BASE + 0x1000)
#define GPIOF_BASEADDR 			(AHB1PERIPH_BASE + 0x1400)
#define GPIOG_BASEADDR 			(AHB1PERIPH_BASE + 0x1800)
#define GPIOH_BASEADDR 			(AHB1PERIPH_BASE + 0x1C00)
#define GPIOI_BASEADDR 			(AHB1PERIPH_BASE + 0x2000)
#define GPIOJ_BASEADDR 			(AHB1PERIPH_BASE + 0x2400)
#define GPIOK_BASEADDR 			(AHB1PERIPH_BASE + 0x2800)
#define RCC_BASEADDR			(AHB1PERIPH_BASE+0x3800)
/*
Base address of  peripheral which are hanging on APB1 bus
*/
#define I2C1_BASEADDR			(APB1PERIPH_BASE + 0x5400)
#define I2C2_BASEADDR			(APB1PERIPH_BASE + 0x5800)
#define I2C3_BASEADDR			(APB1PERIPH_BASE + 0x5C00)
#define SPI2_BASEADDR			(APB1PERIPH_BASE + 0x3800)
#define SPI3_BASEADDR			(APB1PERIPH_BASE + 0x3C00)
#define USART2_BASEADDR			(APB1PERIPH_BASE + 0x4400)
#define USART3_BASEADDR			(APB1PERIPH_BASE + 0x4800)
#define UART4_BASEADDR			(APB1PERIPH_BASE + 0x4C00)
#define UART5_BASEADDR			(APB1PERIPH_BASE + 0x5000)

/*
Base address of  peripheral which are hanging on APB2 bus
*/
#define	SPI1_BASEADDR			(APB2PERIPH_BASE + 0x3000)
#define	USART1_BASEADDR			(APB2PERIPH_BASE + 0x1000)
#define	USART6_BASEADDR			(APB2PERIPH_BASE + 0x1400)
#define	EXTI_BASEADDR			(APB2PERIPH_BASE + 0x3C00)
#define	SYSCFG_BASEADDR			(APB2PERIPH_BASE + 0x3800)




/***************RCC PERIPHERAL REGISTER*************/

typedef struct{
	volatile uint32_t CR;
	volatile uint32_t PLLCFGR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t AHB1RSTR;
	volatile uint32_t AHB2RSTR;
	volatile uint32_t AHB3RSTR;
	uint32_t 		RESERVED0;
	volatile uint32_t APB1RSTR;
	volatile uint32_t APB2RSTR;
	uint32_t 		RESERVED1[2];
	volatile uint32_t AHB1ENR;
	volatile uint32_t AHB2ENR;
	volatile uint32_t AHB3ENR;
	uint32_t 		RESERVED2;
	volatile uint32_t APB1ENR;
	volatile uint32_t APB2ENR;
	uint32_t 		RESERVED3[2];
	volatile uint32_t AHB1LPENR;
	volatile uint32_t AHB2LPENR;
	volatile uint32_t AHB3LPENR;
	uint32_t 		RESERVED4;
	volatile uint32_t APB1LPENR;
	volatile uint32_t APB2LPENR;
	uint32_t 		RESERVED5[2];
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	uint32_t 		RESERVED6[2];
	volatile uint32_t SSCGR;
	volatile uint32_t PLLI2SCFGR;
	volatile uint32_t PLLSAICFGR;
	volatile uint32_t DCKCFGR;

}RCC_RegDef_t;

//peripheral definition(peripheral base address type casted for GPIO_RegDef_t )

#define GPIOA 	((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB 	((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC 	((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD 	((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE 	((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF 	((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG 	((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH 	((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI 	((GPIO_RegDef_t*)GPIOI_BASEADDR)
#define GPIOJ 	((GPIO_RegDef_t*)GPIOJ_BASEADDR)
#define GPIOK 	((GPIO_RegDef_t*)GPIOK_BASEADDR)
#define RCC 	((RCC_RegDef_t*)RCC_BASEADDR)


#define EXTI	((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG	((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)






//peripheral definition structure register for EXTI
typedef struct{
	volatile uint32_t IMR;				//Interrupt mask register (EXTI_IMR)					Address offset: 0x00
	volatile uint32_t EMR;				//Event mask register (EXTI_EMR) 						Address offset: 0x04
	volatile uint32_t RTSR;				//Rising trigger selection register (EXTI_RTSR)			Address offset: 0x08
	volatile uint32_t FTSR;				//Falling trigger selection register (EXTI_FTSR)		Address offset: 0x0C
	volatile uint32_t SWIER;			//Software interrupt event register (EXTI_SWIER)		Address offset: 0x10
	volatile uint32_t PR;				//Pending register (EXTI_PR)							Address offset: 0x14

}EXTI_RegDef_t;


//peripheral definition structure register for SYSCFGCR
typedef struct{
	volatile uint32_t MEMRMP;			//SYSCFG memory remap register (SYSCFG_MEMRMP)					Address offset: 0x00
	volatile uint32_t PMC;				//SYSCFG peripheral mode configuration register (SYSCFG_PMC)	Address offset: 0x04
	volatile uint32_t EXTICR[4];		//SYSCFG external interrupt configuration register 1(SYSCFG_EXTICR1))	Address offset: 0x08
		uint32_t RESERVED1[2];
	volatile uint32_t CMPCR;				//Compensation cell control register (SYSCFG_CMPCR)		Address offset: 0x0C
		uint32_t RESERVED2[2];
	volatile uint32_t CFGR;

}SYSCFG_RegDef_t;


/******************** peripheral register definition structure
 	 	 	 * Lets write a generic structure for GPIO resister variable***************/
typedef struct{
	volatile uint32_t MODER;			// GPIO port mode register
	volatile uint32_t OTYPER;		//GPIO port output type register
	volatile uint32_t OSPEEDR;		//GPIO port output speed register
	volatile uint32_t PUPDR;			//GPIO port pull-up/pull-down register
	volatile uint32_t IDR;			//GPIO port input data register
	volatile uint32_t ODR;			//GPIO port output data register
	volatile uint32_t BSRR;			//GPIO port bit set/reset register
	volatile uint32_t LCKR;			//GPIO port configuration lock register
	volatile uint32_t AFR[2];		//AFR[0]GPIO alternate function low register|
									//|AFR[1]GPIO alternate function high register
}GPIO_RegDef_t;

/*clock enable macros for GPIO peripherals */

#define GPIOA_PCLK_EN()		(RCC->AHB1ENR |= (1<<0) )
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1<<1) )
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1<<2) )
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1<<3) )
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1<<4) )
#define GPIOF_PCLK_EN()		(RCC->AHB1ENR |= (1<<5) )
#define GPIOG_PCLK_EN()		(RCC->AHB1ENR |= (1<<6) )
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1<<7) )
#define GPIOI_PCLK_EN()		(RCC->AHB1ENR |= (1<<8) )
#define GPIOJ_PCLK_EN()		(RCC->AHB1ENR |= (1<<9) )
#define GPIOK_PCLK_EN()		(RCC->AHB1ENR |= (1<<10) )

/*clock enable macros for i2C peripherals */
#define I2C1_PCLK_EN()		(RCC->APB1ENR |= (1<<21) )
#define I2C2_PCLK_EN()		(RCC->APB1ENR |= (1<<22) )
#define I2C3_PCLK_EN()		(RCC->APB1ENR |= (1<<23) )

/*clock enable macros for SPI peripherals */
#define SPI1_PCLK_EN()		(RCC->APB2ENR |= (1<<12) )
#define SPI2_PCLK_EN()		(RCC->APB1ENR |= (1<<14) )
#define SPI3_PCLK_EN()		(RCC->APB1ENR |= (1<<15) )

/*clock enable macros for UARTx peripherals */
#define USART1_PCLK_EN()		(RCC->APB2ENR |= (1<<4) )
#define USART2_PCLK_EN()		(RCC->APB1ENR |= (1<<4) )
#define USART3_PCLK_EN()		(RCC->APB1ENR |= (1<<4) )
#define USART6_PCLK_EN()		(RCC->APB2ENR |= (1<<5) )
#define UART4_PCLK_EN()			(RCC->APB1ENR |= (1<<19) )
#define UART5_PCLK_EN()			(RCC->APB1ENR |= (1<<20) )

/*clock enable macros for SYSCFG peripherals */
#define SYSCFG_PCLK_EN()		(RCC->APB2ENR |= (1<<14) )

/**************************************************************/
					//DISABLE//
/**************************************************************/

/*clock DISABLE disable macros for GPIO peripherals */

#define GPIOA_PCLK_DI()		(RCC->AHB1ENR &= ~ (1<<0) )
#define GPIOB_PCLK_DI()		(RCC->AHB1ENR &= ~ (1<<1) )
#define GPIOC_PCLK_DI()		(RCC->AHB1ENR &= ~ (1<<2) )
#define GPIOD_PCLK_DI()		(RCC->AHB1ENR &= ~ (1<<3) )
#define GPIOE_PCLK_DI()		(RCC->AHB1ENR &= ~ (1<<4) )
#define GPIOF_PCLK_DI()		(RCC->AHB1ENR &= ~ (1<<5) )
#define GPIOG_PCLK_DI()		(RCC->AHB1ENR &= ~ (1<<6) )
#define GPIOH_PCLK_DI()		(RCC->AHB1ENR &= ~ (1<<7) )
#define GPIOI_PCLK_DI()		(RCC->AHB1ENR &= ~ (1<<8) )
#define GPIOJ_PCLK_DI()		(RCC->AHB1ENR &= ~ (1<<9) )
#define GPIOK_PCLK_DI()		(RCC->AHB1ENR &= ~ (1<<10) )

/*clock DISABLE macros for i2C peripherals */
#define I2C1_PCLK_DI()		(RCC->APB1ENR &= ~ (1<<21) )
#define I2C2_PCLK_DI()		(RCC->APB1ENR &= ~ (1<<22) )
#define I2C3_PCLK_DI()		(RCC->APB1ENR &= ~ (1<<23) )

/*clock DISABLE macros for SPI peripherals */
#define SPI1_PCLK_DI()		(RCC->APB2ENR &= ~ (1<<12) )
#define SPI2_PCLK_DI()		(RCC->APB1ENR &= ~ (1<<14) )
#define SPI3_PCLK_DI()		(RCC->APB1ENR &= ~ (1<<15) )

/*clock DISABLE macros for UARTx peripherals */
#define USART1_PCLK_DI()		(RCC->APB2ENR &= ~ (1<<4) )
#define USART2_PCLK_DI()		(RCC->APB1ENR &= ~ (1<<4) )
#define USART3_PCLK_DI()		(RCC->APB1ENR &= ~ (1<<4) )
#define USART6_PCLK_DI()		(RCC->APB2ENR &= ~ (1<<5) )
#define UART4_PCLK_DI()			(RCC->APB1ENR &= ~ (1<<19) )
#define UART5_PCLK_DI()			(RCC->APB1ENR &= ~ (1<<20) )

/*clock DISABLE macros for SYSCFG peripherals */
#define SYSCFG_PCLK_DI()		(RCC->APB2ENR &= ~ (1<<14) )

//MACRO TO RESET GPIOx PERIPHERALS
#define GPIOA_REG_RESET()		do{ (RCC->AHB1RSTR |=  (1<<0));  (RCC->AHB1RSTR &=  ~(1<<0)); }while(0)
#define GPIOB_REG_RESET()		do{ (RCC->AHB1RSTR |=  (1<<1));  (RCC->AHB1RSTR &=  ~(1<<1)); }while(0)
#define GPIOC_REG_RESET()		do{ (RCC->AHB1RSTR |=  (1<<2));  (RCC->AHB1RSTR &=  ~(1<<2)); }while(0)
#define GPIOD_REG_RESET()		do{ (RCC->AHB1RSTR |=  (1<<3));  (RCC->AHB1RSTR &=  ~(1<<3)); }while(0)
#define GPIOE_REG_RESET()		do{ (RCC->AHB1RSTR |=  (1<<4));  (RCC->AHB1RSTR &=  ~(1<<4)); }while(0)
#define GPIOF_REG_RESET()		do{ (RCC->AHB1RSTR |=  (1<<5));  (RCC->AHB1RSTR &=  ~(1<<5)); }while(0)
#define GPIOG_REG_RESET()		do{ (RCC->AHB1RSTR |=  (1<<6));  (RCC->AHB1RSTR &=  ~(1<<6)); }while(0)
#define GPIOH_REG_RESET()		do{ (RCC->AHB1RSTR |=  (1<<7));  (RCC->AHB1RSTR &=  ~(1<<7)); }while(0)
#define GPIOI_REG_RESET()		do{ (RCC->AHB1RSTR |=  (1<<8));  (RCC->AHB1RSTR &=  ~(1<<8)); }while(0)
#define GPIOJ_REG_RESET()		do{ (RCC->AHB1RSTR |=  (1<<9));  (RCC->AHB1RSTR &=  ~(1<<9)); }while(0)
#define GPIOK_REG_RESET()		do{ (RCC->AHB1RSTR |=  (1<<10)); (RCC->AHB1RSTR &= ~(1<<10)); }while(0)

// return port code for given GPIOx base address
#define GPIO_BASEADDR_TO_CODE(x)		((x == GPIOA)?0:\
										(x == GPIOB)?1:\
										(x == GPIOC)?2:\
										(x == GPIOD)?3:\
										(x == GPIOE)?4:\
										(x == GPIOF)?5:\
										(x == GPIOG)?6:\
										(x == GPIOH)?7:0 )



#include "stm32f407xx_gpio_driver.h"









#endif /* INC_STM32F407XX_H_ */
