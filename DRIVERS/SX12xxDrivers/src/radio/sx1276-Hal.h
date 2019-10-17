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
 * \file       sx1276-Hal.h
 * \brief      SX1276 Hardware Abstraction Layer
 *
 * \version    2.0.B2 
 * \date       May 6 2013
 * \author     Gregory Cristian
 *
 * Last modified by Miguel Luis on Jun 19 2013
 */
#ifndef __SX1276_HAL_H__
#define __SX1276_HAL_H__

#include "ioe.h"
#include "main.h"
#include <stdbool.h>
#include <stdint.h>
/*!
 * DIO state read functions mapping
 */
#define DIO0      DIO0_PIN_IN()	// SX1276ReadDio0( )
#define DIO1      DIO1_PIN_IN() // SX1276ReadDio1( ) //FifoLevel
#define DIO2      DIO2_PIN_IN()	//SX1276ReadDio2( )
#define DIO3    	DIO3_PIN_IN()	//  SX1276ReadDio3( )
#define DIO4      1		//DIO4_PIN_IN()	// SX1276ReadDio4( )
#define DIO5      1		//DIO5_PIN_IN()	//  SX1276ReadDio5( )

//// RXTX pin control see errata note
//#define RXTX( txEnable )                            SX1276WriteRxTx( txEnable );
#define RXTX( txEnable )  {if( txEnable != 0 ) 		RXTX_PIN_OUT(1); else RXTX_PIN_OUT(0);}

#define GET_TICK_COUNT( )                           ( TickCounter )
#define TICK_RATE_MS( ms )                          ( ms )

typedef enum
{
    RADIO_RESET_OFF,
    RADIO_RESET_ON,
}tRadioResetState;

//exchange spi data
/*!
 * \brief Initializes the radio interface I/Os
 */
//void SX1276InitIo( void );

/*!
 * \brief Set the radio reset pin state
 * 
 * \param state New reset pin state
 */
void SX1276SetReset( uint8_t state );

/*!
 * \brief Writes the radio register at the specified address
 *
 * \param [IN]: addr Register address
 * \param [IN]: data New register value
 */
void SX1276Write( uint8_t addr, uint8_t data );
//#define SX1276Write(  addr,  data )  SX1276WriteBuffer( addr,data, 1 )
/*!
 * \brief Reads the radio register at the specified address
 *
 * \param [IN]: addr Register address
 * \param [OUT]: data Register value
 */
void SX1276Read( uint8_t addr, uint8_t *data );
//#define SX1276Read(  addr, data )  SX1276ReadBuffer( addr, data, 1 )
/*!
 * \brief Writes multiple radio registers starting at address
 *
 * \param [IN] addr   First Radio register address
 * \param [IN] buffer Buffer containing the new register's values
 * \param [IN] size   Number of registers to be written
 */
void SX1276WriteBuffer( uint8_t addr, uint8_t *buffer, uint8_t size );

/*!
 * \brief Reads multiple radio registers starting at address
 *
 * \param [IN] addr First Radio register address
 * \param [OUT] buffer Buffer where to copy the registers data
 * \param [IN] size Number of registers to be read
 */
void SX1276ReadBuffer( uint8_t addr, uint8_t *buffer, uint8_t size );

/*!
 * \brief Writes the buffer contents to the radio FIFO
 *
 * \param [IN] buffer Buffer containing data to be put on the FIFO.
 * \param [IN] size Number of bytes to be written to the FIFO
 */
//void SX1276WriteFifo( uint8_t *buffer, uint8_t size );
#define SX1276WriteFifo( buffer,  size )   SX1276WriteBuffer( 0, buffer, size )
/*!
 * \brief Reads the contents of the radio FIFO
 *
 * \param [OUT] buffer Buffer where to copy the FIFO read data.
 * \param [IN] size Number of bytes to be read from the FIFO
 */
//void SX1276ReadFifo( uint8_t *buffer, uint8_t size );
#define SX1276ReadFifo( buffer,  size )  SX1276ReadBuffer( 0, buffer, size )


#endif //__SX1276_HAL_H__

