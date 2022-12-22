#include <stdint.h>

#define __IO volatile
#define GPIO_ODR_ODR_2 		  ((uint32_t)0x00000004)

/*!< Peripheral base address */
#define PERIPH_BASE           ((uint32_t)0x40000000)

/*!< Peripheral memory map */
#define APB1PERIPH_BASE        PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x00010000)
#define AHB1PERIPH_BASE       (PERIPH_BASE + 0x00020000)
#define AHB2PERIPH_BASE       (PERIPH_BASE + 0x08000000)

/*!< AHB2 peripherals */
#define GPIOA_BASE            (AHB2PERIPH_BASE + 0x0000)
#define GPIOB_BASE            (AHB2PERIPH_BASE + 0x0400)
#define GPIOC_BASE            (AHB2PERIPH_BASE + 0x0800)
#define GPIOD_BASE            (AHB2PERIPH_BASE + 0x0C00)
#define GPIOE_BASE            (AHB2PERIPH_BASE + 0x1000)
#define GPIOF_BASE            (AHB2PERIPH_BASE + 0x1400)
#define GPIOG_BASE            (AHB2PERIPH_BASE + 0x1800)
#define GPIOH_BASE            (AHB2PERIPH_BASE + 0x1C00)

/*!< APB1 peripherals */
#define TIM6_BASE             (APB1PERIPH_BASE + 0x1000)
#define TIM7_BASE			  (APB1PERIPH_BASE + 0x1400)

typedef struct
{
	__IO uint32_t MODER;       /*!< GPIO port mode register,               Address offset: 0x00      */
	__IO uint32_t OTYPER;      /*!< GPIO port output type register,        Address offset: 0x04      */
	__IO uint32_t OSPEEDR;     /*!< GPIO port output speed register,       Address offset: 0x08      */
	__IO uint32_t PUPDR;       /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
	__IO uint32_t IDR;         /*!< GPIO port input data register,         Address offset: 0x10      */
	__IO uint32_t ODR;         /*!< GPIO port output data register,        Address offset: 0x14      */
	__IO uint32_t BSRR;        /*!< GPIO port bit set/reset  register,     Address offset: 0x18      */
	__IO uint32_t LCKR;        /*!< GPIO port configuration lock register, Address offset: 0x1C      */
	__IO uint32_t AFR[2];      /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
	__IO uint32_t BRR;         /*!< GPIO Bit Reset register,               Address offset: 0x28      */
	__IO uint32_t ASCR;        /*!< GPIO analog switch control register,   Address offset: 0x2C      */

}GPIO_TypeDef;

typedef struct
{
	__IO uint32_t CR;          /*!< RCC clock control register,                                              Address offset: 0x00 */
	__IO uint32_t ICSCR;       /*!< RCC Internal Clock Sources Calibration Register,                         Address offset: 0x04 */
	__IO uint32_t CFGR;        /*!< RCC clock configuration register,                                        Address offset: 0x08 */
	__IO uint32_t PLLCFGR;     /*!< RCC System PLL configuration register,                                   Address offset: 0x0C */
	__IO uint32_t PLLSAI1CFGR; /*!< RCC PLL SAI1 Configuration Register,                                     Address offset: 0x10 */
	__IO uint32_t PLLSAI2CFGR; /*!< RCC PLL SAI2 Configuration Register,                                     Address offset: 0x14 */
	__IO uint32_t CIER;        /*!< RCC Clock Interrupt Enable Register,                                     Address offset: 0x18 */
	__IO uint32_t CIFR;        /*!< RCC Clock Interrupt Flag Register,                                       Address offset: 0x1C */
	__IO uint32_t CICR;        /*!< RCC Clock Interrupt Clear Register,                                      Address offset: 0x20 */
		 uint32_t RESERVED0;   /*!< Reserved,                                                                Address offset: 0x24 */
	__IO uint32_t AHB1RSTR;    /*!< RCC AHB1 peripheral reset register,                                      Address offset: 0x28 */
	__IO uint32_t AHB2RSTR;    /*!< RCC AHB2 peripheral reset register,                                      Address offset: 0x2C */
	__IO uint32_t AHB3RSTR;    /*!< RCC AHB3 peripheral reset register,                                      Address offset: 0x30 */
		 uint32_t RESERVED1;   /*!< Reserved,                                                                Address offset: 0x34 */
	__IO uint32_t APB1RSTR1;   /*!< RCC APB1 macrocells resets Low Word,                                     Address offset: 0x38 */
	__IO uint32_t APB1RSTR2;   /*!< RCC APB1 macrocells resets High Word,                                    Address offset: 0x3C */
	__IO uint32_t APB2RSTR;    /*!< RCC APB2 macrocells resets,                                              Address offset: 0x40 */
		 uint32_t RESERVED2;   /*!< Reserved,                                                                Address offset: 0x44 */
	__IO uint32_t AHB1ENR;     /*!< RCC AHB1 peripheral clock enable register,                               Address offset: 0x48 */
	__IO uint32_t AHB2ENR;     /*!< RCC AHB2 peripheral clock enable register,                               Address offset: 0x4C */
	__IO uint32_t AHB3ENR;     /*!< RCC AHB3 peripheral clock enable register,                               Address offset: 0x50 */
		 uint32_t RESERVED3;   /*!< Reserved,                                                                Address offset: 0x54 */
	__IO uint32_t APB1ENR1;    /*!< RCC APB1 macrocells clock enables Low Word,                              Address offset: 0x58 */
	__IO uint32_t APB1ENR2;    /*!< RCC APB1 macrocells clock enables High Word,                             Address offset: 0x5C */
	__IO uint32_t APB2ENR;     /*!< RCC APB2 macrocells clock enabled,                                       Address offset: 0x60 */
		 uint32_t RESERVED4;   /*!< Reserved,                                                                Address offset: 0x64 */
	__IO uint32_t AHB1SMENR;   /*!< RCC AHB1 macrocells clocks enables in sleep mode,                        Address offset: 0x60 */
	__IO uint32_t AHB2SMENR;   /*!< RCC AHB2 macrocells clock enables in sleep mode,                         Address offset: 0x64 */
	__IO uint32_t AHB3SMENR;   /*!< RCC AHB3 macrocells clock enables in sleep mode,                         Address offset: 0x70 */
		 uint32_t RESERVED5;   /*!< Reserved,                                                                Address offset: 0x74 */
	__IO uint32_t APB1SMENR1;  /*!< RCC APB1 macrocells clock enables in sleep mode Low Word,                Address offset: 0x78 */
	__IO uint32_t APB1SMENR2;  /*!< RCC APB1 macrocells clock enables in sleep mode High Word,               Address offset: 0x7C */
	__IO uint32_t APB2SMENR;   /*!< RCC APB2 macrocells clock enabled in sleep mode,                         Address offset: 0x80 */
		 uint32_t RESERVED6;   /*!< Reserved,                                                                Address offset: 0x84 */
	__IO uint32_t CCIPR;       /*!< RCC IPs Clocks Configuration Register,                                   Address offset: 0x88 */
	__IO uint32_t RESERVED7;   /*!< Reserved,                                                                Address offset: 0x8C */
	__IO uint32_t BDCR;        /*!< RCC Vswitch Backup Domain Control Register,                              Address offset: 0x90 */
	__IO uint32_t CSR;         /*!< RCC clock control & status register,                                     Address offset: 0x94 */
}RCC_TypeDef;


typedef struct
{
	__IO uint32_t CR1; 		   /*!< TIMx Control register 1,												 Address offset: 0x00 */
	__IO uint32_t CR2; 	 	   /*!< TIMx Control register 2,												 Address offset: 0x04 */
	__IO uint32_t SMCR;        /*!< TIM slave mode control register,          								 Address offset: 0x08 */
	__IO uint32_t DIER; 	   /*!< TIMx DMA/Interrupt enable register,									   	 Address offset: 0x0C */
	__IO uint32_t SR; 		   /*!< TIMx Status register,													 Address offset: 0x10 */
	__IO uint32_t EGR; 		   /*!< TIMx Event Generation register,										   	 Address offset: 0x14 */
	__IO uint32_t CCMR1;       /*!< TIM capture/compare mode register 1,      								 Address offset: 0x18 */
	__IO uint32_t CCMR2;       /*!< TIM capture/compare mode register 2,      								 Address offset: 0x1C */
	__IO uint32_t CCER;        /*!< TIM capture/compare enable register,      								 Address offset: 0x20 */
	__IO uint32_t CNT; 		   /*!< TIMx Counter,															 Address offset: 0x24 */
	__IO uint32_t PSC; 		   /*!< TIMx Prescaler,										   	 				 Address offset: 0x28 */
	__IO uint32_t ARR; 		   /*!< TIMx Auto Reload Register,										   	 	 Address offset: 0x2C */

}TIM_TypeDef;

#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD               ((GPIO_TypeDef *) GPIOD_BASE)
#define GPIOE               ((GPIO_TypeDef *) GPIOE_BASE)
#define GPIOF               ((GPIO_TypeDef *) GPIOF_BASE)
#define GPIOG               ((GPIO_TypeDef *) GPIOG_BASE)
#define GPIOH               ((GPIO_TypeDef *) GPIOH_BASE)

#define RCC ((RCC_TypeDef *) 0x40021000)

/********************  Bit definition for RCC_AHB2ENR register  ***************/
#define  RCC_AHB2ENR_GPIOAEN                 ((uint32_t)0x00000001)
#define  RCC_AHB2ENR_GPIOBEN                 ((uint32_t)0x00000002)
#define  RCC_AHB2ENR_GPIOCEN                 ((uint32_t)0x00000004)
#define  RCC_AHB2ENR_GPIODEN                 ((uint32_t)0x00000008)
#define  RCC_AHB2ENR_GPIOEEN                 ((uint32_t)0x00000010)
#define  RCC_AHB2ENR_GPIOFEN                 ((uint32_t)0x00000020)
#define  RCC_AHB2ENR_GPIOGEN                 ((uint32_t)0x00000040)
#define  RCC_AHB2ENR_GPIOHEN                 ((uint32_t)0x00000080)
#define  RCC_AHB2ENR_OTGFSEN                 ((uint32_t)0x00001000)
#define  RCC_AHB2ENR_ADCEN                   ((uint32_t)0x00002000)
#define  RCC_AHB2ENR_RNGEN                   ((uint32_t)0x00040000)

/********************  Bit definition for RCC_APB1ENR1 register  ***************/
#define  RCC_APB1ENR1_TIM6EN                 ((uint32_t)0x00000010)
#define  RCC_APB1ENR1_TIM7EN					((uint32_t)0x00000020)

#define  TIM6                ((TIM_TypeDef *) TIM6_BASE)
#define  TIM7 				((TIM_TypeDef *) TIM7_BASE)
#define  TIM_CR1_CEN ((uint32_t)0x00000001)	/*!<Counter enable */
#define  TIM_SR_UIF ((uint32_t)0x00000001) /*!<Update interrupt Flag */

void delay(uint16_t ms)
{
	if (ms == 0)
		return;

	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM6EN;
	TIM6->CR1 &= ~TIM_CR1_CEN;
	TIM6->SR = 0;
	TIM6->CNT = 0;
	TIM6->PSC = 3999;
	TIM6->ARR = ms -1;
	TIM6->CR1 |= TIM_CR1_CEN;

	while((TIM7->SR & TIM_SR_UIF) == 0);
}

void delay_us(uint32_t us)
{
	if (us == 0)
		return;

	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM7EN;
	TIM7->CR1 &= ~TIM_CR1_CEN;
	TIM7->SR = 0;
	TIM7->CNT = 0;
	TIM7->PSC = 79;
	TIM7->ARR = us -1;
	TIM7->CR1 |= TIM_CR1_CEN;

	while((TIM7->SR & TIM_SR_UIF) == 0);
}

void GPIO_Clock_Enable()
{
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOCEN | RCC_AHB2ENR_GPIOEEN;
}

void GPIO_Pin_Init()
{
	GPIOB->MODER &= ~(3UL<<4);
	GPIOB->MODER |= 1UL<<4;
	GPIOB->OTYPER &= ~(1<<2);
	GPIOB->OSPEEDR &= ~(3UL<<4);
	GPIOB->PUPDR &= ~(3UL<<4);

	GPIOA->MODER &= ~3UL;
	GPIOA->PUPDR &= ~3UL;

	GPIOE->MODER &= ~(3UL<<20); //Clear bit 20 and 21
	GPIOE->MODER |= (1UL<<20); 	//Set PE.10 to Output
	GPIOE->OTYPER &= ~(0x01<<10);
	GPIOE->OSPEEDR |= (0x03<<20);
	GPIOE->PUPDR |= (0x00<<20);

	GPIOE->MODER &= ~(3UL<<28); //Clear bit 28 and 29
	GPIOE->MODER |= (0x00<<28); 	//Set PC.14 to Input
	GPIOE->OTYPER &= ~(0x01<<14);
	GPIOE->OSPEEDR |= (0x01<<28);
 	GPIOE->PUPDR |= (0x00<<28);
}



long duration;
int distance;
int test_distance = 0;
long count;
long pulseIn()
{
	count = 0;
	while ((GPIOE->IDR & 0x4000) == 0x4000)
	{
		delay_us(2);
		count++;
	}
	return count;
}

int sensorDistance()
{
	GPIOE->ODR &= ~(3UL<<10);
	delay_us(2);
	GPIOE->ODR |= 1UL<<10;
 	delay_us(10);
	GPIOE->ODR &= ~(3UL<<10);

	duration = pulseIn();
	if(duration>50)
	{
		distance = duration * 0.034/2;
		return distance;
	}
	distance = duration * 0.034/2;

	return distance;
}

int main()
{
	GPIO_Clock_Enable();
	GPIO_Pin_Init();
	GPIOB->ODR |= 1UL<<2;

 	while(1)
 	{
 		//GPIOB->ODR ^= GPIO_ODR_ODR_2;
 		test_distance = sensorDistance();
 		//delay(1000);
 		/*if ((GPIOA->IDR & 0x1) == 0x1)
 		{
 			GPIOB->ODR ^= GPIO_ODR_ODR_2;
 			while((GPIOA->IDR & 0x1) == 0x00);
 		}*/
 	}
}
