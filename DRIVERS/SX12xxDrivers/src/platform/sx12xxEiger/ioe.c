
#include "ioe.h"
int gpio_io_init(void )
{
	GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);
  GPIO_InitStructure.GPIO_Pin = DIO0_PIN|DIO1_PIN|DIO2_PIN|DIO3_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin 	= RESET_PIN|LED_GREEN_PIN|LED_RED_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
  //config spi interface
	SPI_InitTypeDef  SPI_InitStructure;
  // Enable SPI1 and GPIO clocks 
  RCC_APB2PeriphClockCmd(SPI1_PeriphClock|RCC_APB2Periph_AFIO|SPI1_GPIO_PeriphClock, ENABLE);//SPI_FLASH_SPI Periph clock enable 
	SPI_Cmd(SPI1, DISABLE);

  // Configure SPI pins: MISO
  GPIO_InitStructure.GPIO_Pin 	= 	SPI1_MOSI_PIN|SPI1_SCLK_PIN;
	GPIO_InitStructure.GPIO_Speed = 	GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode 	= 	GPIO_Mode_AF_PP;//∏¥”√Õ∆√‚ ‰≥ˆ
  GPIO_Init(SPI1_GPIO, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin 	= 	SPI1_MISO_PIN;
	GPIO_InitStructure.GPIO_Speed = 	GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_Mode 	= 	GPIO_Mode_IN_FLOATING;
	GPIO_Init(SPI1_GPIO, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin 	= 	SX12XX_NSS_PIN;
	GPIO_InitStructure.GPIO_Speed = 	GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode 	= 	GPIO_Mode_Out_PP;
	GPIO_Init(SX12XX_NSS_IOPORT, &GPIO_InitStructure);
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode 			= SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize 	= SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL 			= SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA 			= SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS 			= SPI_NSS_Soft;//SPI_NSS_Hard;SPI_NSS_Soft
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI1, &SPI_InitStructure);
	SPI_SSOutputCmd(SPI1, ENABLE);
	SPI_Cmd(SPI1, ENABLE);			//Enable SPI1
	SX12XX_NSS_Desel();
	LedOn(LED_GREEN);
	LedOn(LED_RED);
return 1;
}
INIT_COMPONENT_EXPORT(gpio_io_init);
uint16_t Spi_xfer_data( uint16_t outData )
{
	  while (SX12XX_SPI_EMPTY == 0); 
			SX12XX_SPI_DR = outData;
    while (SX12XX_SPI_BUSY == 0);
		return SX12XX_SPI_DR;
}
