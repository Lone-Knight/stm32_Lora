#include "main.h"
#include <rtthread.h>
#include "rtdevice.h"
#include "usart.h"
#include <string.h>
#include "iwdg.h"
#include "platform.h"
#include "radio.h"
#include "ioe.h"
#include "sx1276-LoRa.h"
#include "sx1276-Hal.h"

#define BUFFER_SIZE        9 									// Define the payload size here
static uint16_t BufferSize = BUFFER_SIZE;			// RF buffer size
static uint8_t Buffer[BUFFER_SIZE];						// RF buffer
static uint8_t EnableMaster = true; 					// Master/Slave selection
tRadioDriver *Radio = NULL;
const uint8_t PingMsg[] = "PING";
const uint8_t PongMsg[] = "PONG";
#if (LoRA_master==1)
	void OnMaster( void );
#else
	void OnSlave( void );
#endif
int main(void)
{
	uint8_t RegVersion = 0;
	IWDG_Init(4, 0x271);	//时间估算: T = ((4 * 2^prer) * rlr)/40, 现在约1s
	Radio = RadioDriverInit( );
	Radio->Init( );
	Radio->StartRx( );
	
  SX1276Read( REG_LR_VERSION, &RegVersion );
  
  if(RegVersion != 0x12)
  {
    rt_kprintf("LoRa read Error!\r\n");
    rt_kprintf("LoRa RegVersion = %d!\r\n",RegVersion);
  }
  else
  {
    rt_kprintf("LoRa read Ok!\r\n");
    rt_kprintf("LoRa RegVersion = %d!\r\n",RegVersion);
	}	
	while(1)
	{
		IWDG_FeedDog();
			#if (LoRA_master==1)
			{
					OnMaster( );
			}
			#else
			{
					OnSlave( );
			} 
			#endif				
		rt_thread_delay(100);
	}
	return 1;
}
#if (LoRA_master==1)
/*
 * Manages the master operation
 */
void OnMaster( void )
{
   uint8_t i;
    u8 state = Radio->Process( );
    switch( state )
    {
    case RF_RX_TIMEOUT:
			rt_kprintf("Tx PING!\r\n");
        // Send the next PING frame
        Buffer[0] = 'P';
        Buffer[1] = 'I';
        Buffer[2] = 'N';
        Buffer[3] = 'G';
//        for( i = 4; i < BufferSize; i++ )
//        {
//            Buffer[i] = i - 4;
//        }
        Radio->SetTxPacket( Buffer, BufferSize );
        break;
    case RF_RX_DONE:
        Radio->GetRxPacket( Buffer, ( uint16_t* )&BufferSize );
				rt_kprintf("RX DONE!\r\n");
				rt_thread_delay(300);
        if( BufferSize > 0 )
        {
            if( strncmp( ( const char* )Buffer, ( const char* )PongMsg, 4 ) == 0 )
            {
                // Indicates on a LED that the received frame is a PONG
                LedToggle( LED_GREEN );
								rt_kprintf("RX PONG!\r\n");
                // Send the next PING frame            
                Buffer[0] = 'P';
                Buffer[1] = 'I';
                Buffer[2] = 'N';
                Buffer[3] = 'G';
                // We fill the buffer with numbers for the payload 
//                for( i = 4; i < BufferSize; i++ )
//                {
//                    Buffer[i] = i - 4;
//                }
                Radio->SetTxPacket( Buffer, BufferSize );
            }
            else if( strncmp( ( const char* )Buffer, ( const char* )PingMsg, 4 ) == 0 )
            { // A master already exists then become a slave
                EnableMaster = false;
								rt_kprintf("RX NOT PONG!\r\n");
                LedOff( LED_RED );
            }
        }            
        break;
    case RF_TX_DONE:
        // Indicates on a LED that we have sent a PING
        LedToggle( LED_RED );
				rt_kprintf("TX DONE!\r\n");
        Radio->StartRx( );
        break;
    default:
        break;
    }  
}
#else
/*
 * Manages the slave operation
 */
void OnSlave( void )
{
    uint8_t i;

    switch( Radio->Process( ) )
    {
    case RF_RX_DONE:
        Radio->GetRxPacket( Buffer, ( uint16_t* )&BufferSize );
        if( BufferSize > 0 )
        {
            if( strncmp( ( const char* )Buffer, ( const char* )PingMsg, 4 ) == 0 )
            {
                // Indicates on a LED that the received frame is a PING
                LedToggle( LED_GREEN );

               // Send the reply to the PONG string
                Buffer[0] = 'P';
                Buffer[1] = 'O';
                Buffer[2] = 'N';
                Buffer[3] = 'G';
                // We fill the buffer with numbers for the payload 
//                for( i = 4; i < BufferSize; i++ )
//                {
//                    Buffer[i] = i - 4;
//                }
								rt_thread_delay(200);
								rt_kprintf("switch to tx\r\n");
                Radio->SetTxPacket( Buffer, BufferSize );
            }
        }
        break;
    case RF_TX_DONE:
        // Indicates on a LED that we have sent a PONG
        LedToggle( LED_RED );
				rt_kprintf("tx done!\r\n");
        Radio->StartRx( );
        break;
    default:
			//	rt_kprintf("switch to default");
        break;
    }
}
#endif
