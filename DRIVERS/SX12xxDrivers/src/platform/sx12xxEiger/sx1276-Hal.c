/*
 * THE FOLLOWING FIRMWARE IS PROVIDED: (1) "AS IS" WITH NO WARRANTY; AND 
 * (2)TO ENABLE ACCESS TO CODING INFORMATION TO GUIDE AND FACILITATE CUSTOMER.
 * CONSEQUENTLY, SEMTECH SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR
 * CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT
 * OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION
 * CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 * 
 * Copyright (C) SEMTECH S.A.
 */
/*! 
 * \file       sx1276-Hal.c
 * \brief      SX1276 Hardware Abstraction Layer
 *
 * \version    2.0.B2 
 * \date       Nov 21 2012
 * \author     Miguel Luis
 *
 * Last modified by Miguel Luis on Jun 19 2013
 */


#include "platform.h"

#if defined( USE_SX1276_RADIO )


#include "spi.h"
#include "sx1276-Hal.h"
void SX1276SetReset( uint8_t state )
{
    GPIO_InitTypeDef GPIO_InitStructure;

//    if( state == RADIO_RESET_ON )
//    {
//        // Configure RESET as output
//#if defined( STM32F4XX ) || defined( STM32F2XX ) || defined( STM32F429_439xx )
//        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//#elif defined (STM32F10X)
//        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
//#endif        
//        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//        GPIO_InitStructure.GPIO_Pin = RESET_PIN;
//        GPIO_Init( RESET_IOPORT, &GPIO_InitStructure );
//			 // Set RESET pin to 0
//        GPIO_WriteBit( RESET_IOPORT, RESET_PIN, Bit_RESET );
//    }
//    else
//    {
//#if FPGA == 0    
//        // Configure RESET as input
//#if defined( STM32F4XX ) || defined( STM32F2XX ) || defined( STM32F429_439xx )
//        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
//#else
//        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
//#endif        
//        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//        GPIO_InitStructure.GPIO_Pin =  RESET_PIN;
//        GPIO_Init( RESET_IOPORT, &GPIO_InitStructure );
//#else
//        GPIO_WriteBit( RESET_IOPORT, RESET_PIN, Bit_SET );
//#endif
//    }
    if( state == RADIO_RESET_ON )
    {
			RESET_ON();
		}else
		{
			RESET_OFF();
		}
}

void SX1276Write( uint8_t addr, uint8_t data )
{
    SX1276WriteBuffer( addr, &data, 1 );
}

void SX1276Read( uint8_t addr, uint8_t *data )
{
    SX1276ReadBuffer( addr, data, 1 );
}

void SX1276WriteBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )
{
    uint8_t i;

    //NSS = 0;
   // GPIO_WriteBit( NSS_IOPORT, NSS_PIN, Bit_RESET );
		SX12XX_NSS_Sel();
    Spi_xfer_data( addr | 0x80 );
    for( i = 0; i < size; i++ )
    {
        Spi_xfer_data( buffer[i] );
    }

    //NSS = 1;
   // GPIO_WriteBit( NSS_IOPORT, NSS_PIN, Bit_SET );
		SX12XX_NSS_Desel();
}

void SX1276ReadBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )
{
    uint8_t i;

    //NSS = 0;
   // GPIO_WriteBit( NSS_IOPORT, NSS_PIN, Bit_RESET );
		SX12XX_NSS_Sel()	;
    Spi_xfer_data( addr & 0x7F );
    for( i = 0; i < size; i++ )
    {
        buffer[i] = Spi_xfer_data( 0 );
    }

    //NSS = 1;
    //GPIO_WriteBit( NSS_IOPORT, NSS_PIN, Bit_SET );
		SX12XX_NSS_Desel();
}



#endif // USE_SX1276_RADIO
