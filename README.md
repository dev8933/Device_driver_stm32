# Device_driver_stm32
Working with Stm32f407 disc board, developing device driver for this board peripherals 
For every peripheral, you have to enable disable the clock as well.

In this tutorial, we will walk through the process of creating a controller-specific header file for the STM32F407xx microcontroller. This file defines the base addresses of various peripherals and includes macros and structures needed for GPIO configuration.
Step 1: Define Base Addresses of Memories
First, we need to define the base addresses of the flash memory, SRAM, and ROM. These addresses can be found in the reference manual of the microcontroller.

c
#define FLASH_BASEADDR    0x08000000U
#define SRAM1_BASEADDR    0x20000000U
#define SRAM2_BASEADDR    0x2001C000U  // SRAM2 starts after SRAM1 (112KB)
#define ROM_BASEADDR      0x1FFF0000U  // System memory
#define SRAM              SRAM1_BASEADDR
Step 2: Define Base Addresses of AHBx and APBx Bus Peripherals
Next, we define the base addresses for the Advanced High-Performance Bus (AHB) and Advanced Peripheral Bus (APB) peripherals.

c
#define APB1PERIPH_BASE   0x40000000U
#define APB2PERIPH_BASE   0x40010000U
#define AHB1PERIPH_BASE   0x40020000U
#define AHB2PERIPH_BASE   0x50000000U
Step 3: Define Base Addresses of GPIO Ports
We then define the base addresses for the GPIO ports that are connected to the AHB1 bus.

c
#define GPIOA_BASEADDR    (AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASEADDR    (AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR    (AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR    (AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASEADDR    (AHB1PERIPH_BASE + 0x1000)
#define GPIOF_BASEADDR    (AHB1PERIPH_BASE + 0x1400)
#define GPIOG_BASEADDR    (AHB1PERIPH_BASE + 0x1800)
#define GPIOH_BASEADDR    (AHB1PERIPH_BASE + 0x1C00)
#define GPIOI_BASEADDR    (AHB1PERIPH_BASE + 0x2000)
#define GPIOJ_BASEADDR    (AHB1PERIPH_BASE + 0x2400)
#define GPIOK_BASEADDR    (AHB1PERIPH_BASE + 0x2800)
#define RCC_BASEADDR      (AHB1PERIPH_BASE + 0x3800)
Step 4: Define Peripheral Register Structures
We define structures that represent the registers of the RCC, GPIO, EXTI, and SYSCFG peripherals. These structures map the memory layout of the registers.
RCC Register Definition

c
typedef struct {
    volatile uint32_t CR;
    volatile uint32_t PLLCFGR;
    volatile uint32_t CFGR;
    volatile uint32_t CIR;
    volatile uint32_t AHB1RSTR;
    volatile uint32_t AHB2RSTR;
    volatile uint32_t AHB3RSTR;
    uint32_t RESERVED0;
    volatile uint32_t APB1RSTR;
    volatile uint32_t APB2RSTR;
    uint32_t RESERVED1[2];
    volatile uint32_t AHB1ENR;
    volatile uint32_t AHB2ENR;
    volatile uint32_t AHB3ENR;
    uint32_t RESERVED2;
    volatile uint32_t APB1ENR;
    volatile uint32_t APB2ENR;
    uint32_t RESERVED3[2];
    volatile uint32_t AHB1LPENR;
    volatile uint32_t AHB2LPENR;
    volatile uint32_t AHB3LPENR;
    uint32_t RESERVED4;
    volatile uint32_t APB1LPENR;
    volatile uint32_t APB2LPENR;
    uint32_t RESERVED5[2];
    volatile uint32_t BDCR;
    volatile uint32_t CSR;
    uint32_t RESERVED6[2];
    volatile uint32_t SSCGR;
    volatile uint32_t PLLI2SCFGR;
    volatile uint32_t PLLSAICFGR;
    volatile uint32_t DCKCFGR;
} RCC_RegDef_t;
GPIO Register Definition

c
typedef struct {
    volatile uint32_t MODER;
    volatile uint32_t OTYPER;
    volatile uint32_t OSPEEDR;
    volatile uint32_t PUPDR;
    volatile uint32_t IDR;
    volatile uint32_t ODR;
    volatile uint32_t BSRR;
    volatile uint32_t LCKR;
    volatile uint32_t AFR[2];
} GPIO_RegDef_t;
Step 5: Define Peripheral Definitions
We define the peripheral definitions by typecasting the base addresses to the corresponding structures.

c
#define GPIOA    ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB    ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC    ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD    ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE    ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF    ((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG    ((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH    ((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI    ((GPIO_RegDef_t*)GPIOI_BASEADDR)
#define GPIOJ    ((GPIO_RegDef_t*)GPIOJ_BASEADDR)
#define GPIOK    ((GPIO_RegDef_t*)GPIOK_BASEADDR)
#define RCC      ((RCC_RegDef_t*)RCC_BASEADDR)
Step 6: Define Clock Enable Macros
We create macros to enable and disable the clocks for the GPIO peripherals.

c
#define GPIOA_PCLK_EN()   (RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()   (RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()   (RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()   (RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()   (RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()   (RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()   (RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()   (RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()   (RCC->AHB1ENR |= (1 << 8))
#define GPIOJ_PCLK_EN()   (RCC->AHB1ENR |= (1 << 9))
#define GPIOK_PCLK_EN()   (RCC->AHB1ENR |= (1 << 10))
Step 7: Define Clock Disable Macros
Similarly, we create macros to disable the clocks for the GPIO peripherals.

c
#define GPIOA_PCLK_DI()   (RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()   (RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()   (RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()   (RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()   (RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()   (RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()   (RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()   (RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI()   (RCC->AHB1ENR &= ~(1 << 8))
#define GPIOJ_PCLK_DI()   (RCC->AHB1ENR &= ~(1 << 9))
#define GPIOK_PCLK_DI()   (RCC->AHB1ENR &= ~(1 << 10))
Step 8: Include the GPIO Driver Header
Finally, include the GPIO driver header file to complete the setup.

c
#include "stm32f407xx_gpio_driver.h"
Conclusion
By following these steps, you have successfully created a controller-specific header file for the STM32F407xx microcontroller. This file will serve as a foundation for developing and configuring various peripherals in your embedded projects.
![image](https://github.com/user-attachments/assets/ae7dc1b7-0d60-4acf-8a99-3fb1bf1fd52b)
