#ifndef __IOE_H__
#define __IOE_H__
#include <rtthread.h>
#include <stdbool.h>
#include <stdint.h>
#include "main.h"
//mcu stm32F103C8T6
// sx1278 Çý¶¯
// DIO0->PB8
// DIO1->PB5
// DIO2->PB6
// DIO3->PB7
// SPI1_MISO->PA6
// SPI1_MOSI->PA7
// SPI1_SCLK->PA5
// SPI1_NSS-> PA4
// LED_GREEN ->PB14
// LED_RED	 ->PB15
/** 
  * @brief  IO Expander Interrupt line on EXTI  
  */ 
#if defined( STM32F4XX ) || defined( STM32F2XX )

#define IOE_IT_PIN                                  GPIO_Pin_4
#define IOE_IT_GPIO_PORT                            GPIOA
#define IOE_IT_GPIO_CLK                             RCC_AHB1Periph_GPIOA
#define IOE_IT_EXTI_PORT_SOURCE                     EXTI_PortSourceGPIOA
#define IOE_IT_EXTI_PIN_SOURCE                      EXTI_PinSource4
#define IOE_IT_EXTI_LINE                            EXTI_Line4
#define IOE_IT_EXTI_IRQn                            EXTI4_IRQn   

//#else
#elif defined( STM32F10X )
#define IOE_IT_PIN                                  GPIO_Pin_4
#define IOE_IT_GPIO_PORT                            GPIOC
#define IOE_IT_GPIO_CLK                             RCC_APB2Periph_GPIOC
#define IOE_IT_EXTI_PORT_SOURCE                     GPIO_PortSourceGPIOC
#define IOE_IT_EXTI_PIN_SOURCE                      GPIO_PinSource4
#define IOE_IT_EXTI_LINE                            EXTI_Line4
#define IOE_IT_EXTI_IRQn                            EXTI4_IRQn   

#endif
#if defined( STM32F4XX ) || defined( STM32F2XX )

#elif defined( STM32F10X )
	#define SPI1_MOSI_PIN      	GPIO_Pin_7
	#define SPI1_MISO_PIN      	GPIO_Pin_6
	#define SPI1_SCLK_PIN     	GPIO_Pin_5
	#define SPI1_GPIO       						GPIOA
	#define SPI1_GPIO_PeriphClock       RCC_APB2Periph_GPIOA
	#define SPI1_PeriphClock						RCC_APB2Periph_SPI1

	#define SX12XX_SPI_DR           SPI1->DR
	#define SX12XX_SPI_BUSY        (SPI1->SR & SPI_I2S_FLAG_RXNE)
	#define SX12XX_SPI_EMPTY       (SPI1->SR & SPI_I2S_FLAG_TXE)
#else
 #error "Please edit pin config micro in ioe.h"
#endif

/*!
 * SX1276 SPI NSS I/O definitions
 */
#if defined( STM32F4XX ) || defined( STM32F2XX )
		#define NSS_IOPORT                                  GPIOA
		#define NSS_PIN                                     GPIO_Pin_15
#elif defined( STM32F429_439xx )
		#define NSS_IOPORT                                  GPIOA
		#define NSS_PIN                                     GPIO_Pin_4
//#else
#elif defined( STM32F10X )
		#define SX12XX_NSS_IOPORT          GPIOA
		#define SX12XX_NSS_PIN             GPIO_Pin_4
		#define SX12XX_NSS_Sel()						PAout(4)=0
		#define SX12XX_NSS_Desel()					PAout(4)=1
#endif

/*!
 * SX1276 RESET I/O definitions
 */
#if defined( STM32F4XX ) || defined( STM32F2XX ) 
#define RESET_IOPORT                                GPIOB
#define RESET_PIN                                   GPIO_Pin_9
#elif defined( STM32F429_439xx )
#define RESET_IOPORT                                GPIOB
#define RESET_PIN                                   GPIO_Pin_9
//#else
#elif defined( STM32F10X )
#define RESET_IOPORT                                GPIOB
#define RESET_PIN                                   GPIO_Pin_9
#define RESET_ON()                                  	PBout(9)	=	0
#define RESET_OFF()                                  	PBout(9)	=	1
#else
#error "Reset Pin definitions err in ioe.h"
#endif
/*!
 * SX1276 DIO pins  I/O definitions
 */
#if defined( STM32F4XX ) || defined( STM32F2XX ) 
	#define DIO0_IOPORT                                 GPIOB
	#define DIO1_IOPORT                                 GPIOB
	#define DIO2_IOPORT                                 GPIOB
	#define DIO3_IOPORT                                 GPIOB
	//#define DIO4_IOPORT                                 GPIOB
	#define DIO0_PIN                                    GPIO_Pin_0
	#define DIO1_PIN                                    GPIO_Pin_1
	#define DIO2_PIN                                    GPIO_Pin_2
	#define DIO3_PIN                                    GPIO_Pin_3
//	#define DIO4_PIN                                    GPIO_Pin_4
#elif defined( STM32F429_439xx )
	#define DIO0_IOPORT                                 GPIOB
	#define DIO1_IOPORT                                 GPIOB
	#define DIO2_IOPORT                                 GPIOB
	#define DIO3_IOPORT                                 GPIOB
//	#define DIO4_IOPORT                                 GPIOB
	#define DIO0_PIN                                    GPIO_Pin_0
	#define DIO1_PIN                                    GPIO_Pin_1
	#define DIO2_PIN                                    GPIO_Pin_2
	#define DIO3_PIN                                    GPIO_Pin_3
//	#define DIO4_PIN                                    GPIO_Pin_4
//#else
#elif defined( STM32F10X )
#define DIO0_IOPORT                                 GPIOB
#define DIO1_IOPORT                                 GPIOB
#define DIO2_IOPORT                                 GPIOB
#define DIO3_IOPORT                                 GPIOB
//#define DIO4_IOPORT                                 GPIOB


#define DIO0_PIN                                    GPIO_Pin_8
#define DIO1_PIN                                    GPIO_Pin_5
#define DIO2_PIN                                    GPIO_Pin_6
#define DIO3_PIN                                    GPIO_Pin_7
//#define DIO4_PIN                                    GPIO_Pin_10


#define DIO0_PIN_IN()				PBin(8)
#define DIO1_PIN_IN()				PBin(5)
#define DIO2_PIN_IN()				PBin(6)
#define DIO3_PIN_IN()				PBin(7)
//#define DIO4_PIN_IN()				PBin(10)

#else
#error "DIO Pin definitions err in ioe.h"
#endif

//led  pins
// LED_GREEN -->PB14
// LED_RED	 -->PB15
#define LED_GREEN_PIN 	GPIO_Pin_14
#define LED_RED_PIN		 	GPIO_Pin_15
#define LED_GREEN 	14
#define LED_RED		 	15
#define LedToggle(pin) 		PBout(pin) 			=	!PBin(pin)
#define LedOn(pin_on)			PBout(pin_on) 	=  0
#define LedOff(pin_off)		PBout(pin_off) 	=  1
#if defined( STM32F4XX ) || defined( STM32F2XX ) || defined( STM32F429_439xx )
#define RXTX_IOPORT                                 
#define RXTX_PIN                                    FEM_CTX_PIN
//#else
#elif defined( STM32F10X )
#define RXTX_IOPORT    		GPIOB                             
#define RXTX_PIN          GPIO_Pin_6
#define RXTX_PIN_OUT(Set_Bit_value)			PBout(6) =Set_Bit_value

#else
#error "RXTX Pin definitions err in ioe.h"
#endif
int gpio_io_init(void );
uint16_t Spi_xfer_data( uint16_t outData );
#endif // __IOE_H__
