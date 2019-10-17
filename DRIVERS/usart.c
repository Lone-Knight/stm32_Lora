/*
 * File      : usart.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006-2013, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 * 2010-03-29     Bernard      remove interrupt Tx and DMA Rx mode
 * 2013-05-13     aozima       update for kehong-lingtai.
 * 2015-01-31     armink       make sure the serial transmit complete in putc()
 * 2016-05-13     armink       add DMA Rx mode
 * 2017-01-19     aubr.cool    add interrupt Tx mode
 * 2017-04-13     aubr.cool    correct Rx parity err
 */
//lxs   config uart3 for dma transmit
#include "stm32f10x.h"
#include "usart.h"
#include "rtdevice.h"
#include <rtthread.h>
#ifdef RT_USING_SERIAL
#include "serial.h"
#endif /* RT_USING_SERIAL */
/* USART1 */
#define UART1_GPIO_TX        GPIO_Pin_9
#define UART1_GPIO_RX        GPIO_Pin_10
#define UART1_GPIO           GPIOA

/* USART2 */
#define UART2_GPIO_TX        GPIO_Pin_2
#define UART2_GPIO_RX        GPIO_Pin_3
#define UART2_GPIO           GPIOA

/* USART3_REMAP[1:0] = 00 */
#define UART3_GPIO_TX        GPIO_Pin_10
#define UART3_GPIO_RX        GPIO_Pin_11
#define UART3_GPIO           GPIOB

/* USART4 */
#define UART4_GPIO_TX        GPIO_Pin_10
#define UART4_GPIO_RX        GPIO_Pin_11
#define UART4_GPIO           GPIOC


/* STM32 uart driver */
struct stm32_uart
{
    USART_TypeDef *uart_device;
    IRQn_Type irq;
    struct stm32_uart_dma
    {
        /* dma channel */
        DMA_Channel_TypeDef *rx_ch;
        /* dma global flag */
        uint32_t rx_gl_flag;
				uint32_t	ch_tc_flag;//add by lxs
        /* dma irq channel */
        uint8_t rx_irq_ch;
        /* setting receive len */
        rt_size_t setting_recv_len;
        /* last receive index */
        rt_size_t last_recv_index;
    } rx_dma;
		struct stm32_uart_txdma//add by lxs
    {
				DMA_Channel_TypeDef *tx_ch;//DMA channel
				uint32_t ch_gl_flag; //dma channel glob interrupt flag
				uint32_t ch_tc_flag; //dma channel transmitt complete
				uint8_t  irq_channel;
				rt_size_t setting_data_len;
		}tx_dma;
};

static void DMA_Configuration(struct rt_serial_device *serial,int bRxorTx);

static rt_err_t stm32_configure(struct rt_serial_device *serial, struct serial_configure *cfg)
{
    struct stm32_uart* uart;
    USART_InitTypeDef USART_InitStructure;

    RT_ASSERT(serial != RT_NULL);
    RT_ASSERT(cfg != RT_NULL);

    uart = (struct stm32_uart *)serial->parent.user_data;

    USART_InitStructure.USART_BaudRate = cfg->baud_rate;

    if (cfg->data_bits == DATA_BITS_8){
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    } else if (cfg->data_bits == DATA_BITS_9) {
        USART_InitStructure.USART_WordLength = USART_WordLength_9b;
    }

    if (cfg->stop_bits == STOP_BITS_1){
        USART_InitStructure.USART_StopBits = USART_StopBits_1;
    } else if (cfg->stop_bits == STOP_BITS_2){
        USART_InitStructure.USART_StopBits = USART_StopBits_2;
    }

    if (cfg->parity == PARITY_NONE){
        USART_InitStructure.USART_Parity = USART_Parity_No;
    } else if (cfg->parity == PARITY_ODD) {
        USART_InitStructure.USART_Parity = USART_Parity_Odd;
    } else if (cfg->parity == PARITY_EVEN) {
        USART_InitStructure.USART_Parity = USART_Parity_Even;
    }

    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(uart->uart_device, &USART_InitStructure);

    /* Enable USART */
    USART_Cmd(uart->uart_device, ENABLE);

    return RT_EOK;
}

static rt_err_t stm32_control(struct rt_serial_device *serial, int cmd, void *arg)
{
    struct stm32_uart* uart;
    rt_uint32_t ctrl_arg = (rt_uint32_t)(arg);

    RT_ASSERT(serial != RT_NULL);
    uart = (struct stm32_uart *)serial->parent.user_data;

    switch (cmd)
    {
        /* disable interrupt */
    case RT_DEVICE_CTRL_CLR_INT:
        /* disable rx irq */
        UART_DISABLE_IRQ(uart->irq);
        /* disable interrupt */
        USART_ITConfig(uart->uart_device, USART_IT_RXNE, DISABLE);
        break;
        /* enable interrupt */
    case RT_DEVICE_CTRL_SET_INT:
        /* enable rx irq */
        UART_ENABLE_IRQ(uart->irq);
        /* enable interrupt */
        USART_ITConfig(uart->uart_device, USART_IT_RXNE, ENABLE);
        break;
        /* USART config */
    case RT_DEVICE_CTRL_CONFIG :
//        if (ctrl_arg == RT_DEVICE_FLAG_DMA_RX) {
//            DMA_Configuration(serial);
//        }
						//modify by lxs 2019/1/18,add DMA receive data  for stm32F10x
		        if (RT_DEVICE_FLAG_DMA_RX ==ctrl_arg) //config rx dma
					{
            DMA_Configuration(serial,1);
					}else if(RT_DEVICE_FLAG_DMA_TX==ctrl_arg)
					{
						DMA_Configuration(serial,0);//config tx dma
					}
        break;
    }
    return RT_EOK;
}

static int stm32_putc(struct rt_serial_device *serial, char c)
{
    struct stm32_uart* uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct stm32_uart *)serial->parent.user_data;

    if (serial->parent.open_flag & RT_DEVICE_FLAG_INT_TX)
    {
        if (!(uart->uart_device->SR & USART_FLAG_TXE))
        {
            USART_ITConfig(uart->uart_device, USART_IT_TC, ENABLE);
            return -1;
        }
        uart->uart_device->DR = c;
        USART_ITConfig(uart->uart_device, USART_IT_TC, ENABLE);
    }
    else
    {
        uart->uart_device->DR = c;
        while (!(uart->uart_device->SR & USART_FLAG_TC));
    }

    return 1;
}

static int stm32_getc(struct rt_serial_device *serial)
{
    int ch;
    struct stm32_uart* uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct stm32_uart *)serial->parent.user_data;

    ch = -1;
    if (uart->uart_device->SR & USART_FLAG_RXNE)
    {
        ch = uart->uart_device->DR & 0xff;
    }

    return ch;
}

/**
 * Serial port receive idle process. This need add to uart idle ISR.
 *
 * @param serial serial device
 */
static void dma_uart_rx_idle_isr(struct rt_serial_device *serial) {
    struct stm32_uart *uart = (struct stm32_uart *) serial->parent.user_data;
    rt_size_t recv_total_index, recv_len;
    rt_base_t level;

    /* disable interrupt */
    level = rt_hw_interrupt_disable();

    recv_total_index = uart->rx_dma.setting_recv_len - DMA_GetCurrDataCounter(uart->rx_dma.rx_ch);
    recv_len = recv_total_index - uart->rx_dma.last_recv_index;
    uart->rx_dma.last_recv_index = recv_total_index;
    /* enable interrupt */
    rt_hw_interrupt_enable(level);

    if (recv_len) 
			rt_hw_serial_isr(serial, RT_SERIAL_EVENT_RX_DMADONE | (recv_len << 8));

    /* read a data for clear receive idle interrupt flag */
    USART_ReceiveData(uart->uart_device);
    DMA_ClearFlag(uart->rx_dma.rx_gl_flag);
}

/**
 * DMA receive done process. This need add to DMA receive done ISR.
 *
 * @param serial serial device
 */
//static void dma_rx_done_isr(struct rt_serial_device *serial) {
//    struct stm32_uart *uart = (struct stm32_uart *) serial->parent.user_data;
//    rt_size_t recv_len;
//    rt_base_t level;

//    /* disable interrupt */
//    level = rt_hw_interrupt_disable();

//    recv_len = uart->rx_dma.setting_recv_len - uart->rx_dma.last_recv_index;
//    /* reset last recv index */
//    uart->rx_dma.last_recv_index = 0;
//    /* enable interrupt */
//    rt_hw_interrupt_enable(level);

//    if (recv_len) rt_hw_serial_isr(serial, RT_SERIAL_EVENT_RX_DMADONE | (recv_len << 8));

//    DMA_ClearFlag(uart->rx_dma.rx_gl_flag);
//}
static void dma_tx_done_isr(struct rt_serial_device *serial) 
{
    struct stm32_uart *uart = (struct stm32_uart *) serial->parent.user_data;
	//	DMA_Cmd(uart->tx_dma.,DISABLE);
    rt_base_t level;
	if(DMA_GetFlagStatus(uart->tx_dma.ch_tc_flag) != RESET)//check is transmit interr
	{
    /* disable interrupt */
    level = rt_hw_interrupt_disable();
    rt_hw_serial_isr(serial,RT_SERIAL_EVENT_TX_DMADONE);//invoke function
    /* enable interrupt */
    rt_hw_interrupt_enable(level);
	}
  //  rt_hw_serial_isr(serial, RT_SERIAL_EVENT_TX_DMADONE | (recv_len << 8));
    DMA_ClearFlag(uart->tx_dma.ch_gl_flag);//clear global interrupt 
}
/**
 * Uart common interrupt process. This need add to uart ISR.
 *
 * @param serial serial device
 */
static void uart_isr(struct rt_serial_device *serial) {
    struct stm32_uart *uart = (struct stm32_uart *) serial->parent.user_data;

    RT_ASSERT(uart != RT_NULL);

    if(USART_GetITStatus(uart->uart_device, USART_IT_RXNE) != RESET)
    {
        if(USART_GetFlagStatus(uart->uart_device, USART_FLAG_PE) == RESET)
        {
            rt_hw_serial_isr(serial, RT_SERIAL_EVENT_RX_IND);
        }
        /* clear interrupt */
        USART_ClearITPendingBit(uart->uart_device, USART_IT_RXNE);
    }
    if(USART_GetITStatus(uart->uart_device, USART_IT_IDLE) != RESET)
    {
        dma_uart_rx_idle_isr(serial);
    }
    if (USART_GetITStatus(uart->uart_device, USART_IT_TC) != RESET)
    {
        /* clear interrupt */
        if(serial->parent.open_flag & RT_DEVICE_FLAG_INT_TX)
        {
            rt_hw_serial_isr(serial, RT_SERIAL_EVENT_TX_DONE);
        }
        USART_ITConfig(uart->uart_device, USART_IT_TC, DISABLE);
        USART_ClearITPendingBit(uart->uart_device, USART_IT_TC);
    }
    if (USART_GetFlagStatus(uart->uart_device, USART_FLAG_ORE) == SET)
    {
        stm32_getc(serial);
    }
}

static const struct rt_uart_ops stm32_uart_ops =
{
    stm32_configure,
    stm32_control,
    stm32_putc,
    stm32_getc,
		dma_transmit//added by lxs  2019/1/18
};

#if defined(RT_USING_UART1)
/* UART1 device driver structure */
struct stm32_uart uart1 =
{
    USART1,
    USART1_IRQn,
    {
        DMA1_Channel5,
        DMA1_FLAG_GL5,
				DMA1_FLAG_TC5,
        DMA1_Channel5_IRQn,
        0,
    },
};
struct rt_serial_device serial1;

void USART1_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    uart_isr(&serial1);

    /* leave interrupt */
    rt_interrupt_leave();
}

//void DMA1_Channel5_IRQHandler(void) {
//    /* enter interrupt */
//    rt_interrupt_enter();

//    dma_rx_done_isr(&serial1);

//    /* leave interrupt */
//    rt_interrupt_leave();
//}


#endif /* RT_USING_UART1 */

#if defined(RT_USING_UART2)
/* UART2 device driver structure */
struct stm32_uart uart2 =
{
    USART2,
    USART2_IRQn,
    {
        DMA1_Channel6,
        DMA1_FLAG_GL6,
				DMA1_FLAG_TC6,
        DMA1_Channel6_IRQn,
        0,
    },
		{
			DMA1_Channel7,
			DMA1_FLAG_GL7,
			DMA1_FLAG_TC7,
			DMA1_Channel7_IRQn,
			0
		}
};
struct rt_serial_device serial2;

void USART2_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();
    uart_isr(&serial2);
    /* leave interrupt */
    rt_interrupt_leave();
}
//void DMA1_Channel6_IRQHandler(void) {
//    /* enter interrupt */
//    rt_interrupt_enter();
//    dma_rx_done_isr(&serial2);
//    /* leave interrupt */
//    rt_interrupt_leave();
//}
void DMA1_Channel7_IRQHandler(void) {
    /* enter interrupt */
    rt_interrupt_enter();
    dma_tx_done_isr(&serial2);
    /* leave interrupt */
    rt_interrupt_leave();
}

#endif /* RT_USING_UART2 */




#if defined(RT_USING_UART3)
/* UART3 device driver structure */
struct stm32_uart uart3 =
{
    USART3,
    USART3_IRQn,
    {
        DMA1_Channel3,
        DMA1_FLAG_GL3,
				DMA1_FLAG_TC3,
        DMA1_Channel3_IRQn,
        0,
    },
		{
			DMA1_Channel2,
			DMA1_FLAG_GL2,
			DMA1_FLAG_TC2,
			DMA1_Channel2_IRQn,
			0
		}
};
struct rt_serial_device serial3;

void USART3_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    uart_isr(&serial3);

    /* leave interrupt */
    rt_interrupt_leave();
}

//void DMA1_Channel3_IRQHandler(void) {
//    /* enter interrupt */
//    rt_interrupt_enter();

//    dma_rx_done_isr(&serial3);

//    /* leave interrupt */
//    rt_interrupt_leave();
//}
//usart3 DMA tx interrput
//void DMA1_Channel2_IRQHandler (void)
//{
//	 /* enter interrupt */
//	rt_interrupt_enter();
//	dma_tx_done_isr(&serial3);
//	/* leave interrupt */
//	rt_interrupt_leave();
//}
#endif /* RT_USING_UART3 */

#if defined(RT_USING_UART4)
/* UART4 device driver structure */
struct stm32_uart uart4 =
{
    UART4,
    UART4_IRQn,
    {
        DMA2_Channel3,
        DMA2_FLAG_GL3,
        DMA2_Channel3_IRQn,
        0,
    },
};
struct rt_serial_device serial4;

void UART4_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    uart_isr(&serial4);

    /* leave interrupt */
    rt_interrupt_leave();
}

void DMA2_Channel3_IRQHandler(void) {
    /* enter interrupt */
    rt_interrupt_enter();

    dma_rx_done_isr(&serial4);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* RT_USING_UART4 */

static void RCC_Configuration(void)
{
#if defined(RT_USING_UART1)
    /* Enable UART GPIO clocks */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    /* Enable UART clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
#endif /* RT_USING_UART1 */

#if defined(RT_USING_UART2)
    /* Enable UART GPIO clocks */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    /* Enable UART clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
#endif /* RT_USING_UART2 */

#if defined(RT_USING_UART3)
    /* Enable UART GPIO clocks */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    /* Enable UART clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
#endif /* RT_USING_UART3 */

#if defined(RT_USING_UART4)
    /* Enable UART GPIO clocks */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
    /* Enable UART clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
#endif /* RT_USING_UART4 */
}

static void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

#if defined(RT_USING_UART1)
    /* Configure USART Rx/tx PIN */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = UART1_GPIO_RX;
    GPIO_Init(UART1_GPIO, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = UART1_GPIO_TX;
    GPIO_Init(UART1_GPIO, &GPIO_InitStructure);
#endif /* RT_USING_UART1 */

#if defined(RT_USING_UART2)
    /* Configure USART Rx/tx PIN */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = UART2_GPIO_RX;
    GPIO_Init(UART2_GPIO, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = UART2_GPIO_TX;
    GPIO_Init(UART2_GPIO, &GPIO_InitStructure);
#endif /* RT_USING_UART2 */

#if defined(RT_USING_UART3)
    /* Configure USART Rx/tx PIN */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = UART3_GPIO_RX;
    GPIO_Init(UART3_GPIO, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = UART3_GPIO_TX;
    GPIO_Init(UART3_GPIO, &GPIO_InitStructure);
#endif /* RT_USING_UART3 */

#if defined(RT_USING_UART4)
    /* Configure USART Rx/tx PIN */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = UART4_GPIO_RX;
    GPIO_Init(UART4_GPIO, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = UART4_GPIO_TX;
    GPIO_Init(UART4_GPIO, &GPIO_InitStructure);
#endif /* RT_USING_UART4 */
}

static void NVIC_Configuration(struct stm32_uart* uart)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable the USART1 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = uart->irq;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

//dma transmit function
rt_size_t dma_transmit(struct rt_serial_device *serial, rt_uint8_t *buf, rt_size_t size, int direction)
{
	  struct stm32_uart* uart;
    RT_ASSERT(serial != RT_NULL);
		RT_ASSERT(buf != RT_NULL);
		uart = (struct stm32_uart *)serial->parent.user_data;
	if(RT_SERIAL_DMA_TX==direction)
	{
		DMA_Cmd(uart->tx_dma.tx_ch,DISABLE);
//		DMA_DeInit(uart->tx_dma.tx_ch);
		//while(DISABLE!=DMA_GetCmdStatus (uart->tx_dma.tx_ch));
		uart->tx_dma.tx_ch->CNDTR=(uint16_t)size;//Write the number of data units to be transferred
		uart->tx_dma.tx_ch->CMAR=(u32)buf;
		
	//	DMA_ITConfig(uart->tx_dma.tx_ch, DMA_IT_TC, ENABLE);//DMA  transmit complete
  // USART_DMACmd(uart->uart_device, USART_DMAReq_Tx, ENABLE);
		DMA_Cmd(uart->tx_dma.tx_ch,ENABLE);//channel enable
		USART_DMACmd(uart->uart_device, USART_DMAReq_Tx, ENABLE);
	}
	else
	{

	}
	return RT_EOK;
}
static void DMA_Configuration(struct rt_serial_device *serial,int bRxorTx)//���Ӳ���int bRxorTx
	{
    struct stm32_uart *uart = (struct stm32_uart *) serial->parent.user_data;
    struct rt_serial_rx_fifo *rx_fifo = (struct rt_serial_rx_fifo *)serial->serial_rx;
		//unsigned int a;
    DMA_InitTypeDef DMArx_Init,DMAtx_Init;
    NVIC_InitTypeDef NVICtx_Init;

    uart->rx_dma.setting_recv_len = serial->config.bufsz;
    /* DMA clock enable */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);
		if(bRxorTx)//config rx dma
	{
     /* rx dma config */
    DMA_DeInit(uart->rx_dma.rx_ch);
	//	while (DMA_GetCmdStatus(uart->rx_dma.rx_ch) != DISABLE);
    DMArx_Init.DMA_PeripheralBaseAddr = (uint32_t)&(uart->uart_device->DR);
    DMArx_Init.DMA_MemoryBaseAddr = (uint32_t) rx_fifo->buffer;
    DMArx_Init.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMArx_Init.DMA_BufferSize = serial->config.bufsz;
    DMArx_Init.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMArx_Init.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMArx_Init.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMArx_Init.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMArx_Init.DMA_Mode = DMA_Mode_Circular;//DMA_Mode_Circular;
    DMArx_Init.DMA_Priority = DMA_Priority_High;
    DMArx_Init.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(uart->rx_dma.rx_ch, &DMArx_Init);
		//    DMA_ClearFlag(uart->rx_dma.rx_gl_flag);
		//		DMA_ClearFlag(uart->rx_dma.ch_tc_flag);
    /* rx dma interrupt config */
		//    NVICrx_Init.NVIC_IRQChannel = uart->rx_dma.rx_irq_ch;
		//    NVICrx_Init.NVIC_IRQChannelPreemptionPriority = 1;
		//    NVICrx_Init.NVIC_IRQChannelSubPriority = 1;
		//    NVICrx_Init.NVIC_IRQChannelCmd = ENABLE;
		//    NVIC_Init(&NVICrx_Init);
				//DMA_ITConfig(uart->rx_dma.rx_ch, DMA_IT_TC, ENABLE);
				
		//    NVICrx_Init.NVIC_IRQChannel = uart->irq;
		//    NVICrx_Init.NVIC_IRQChannelPreemptionPriority = 0;
		//    NVICrx_Init.NVIC_IRQChannelSubPriority = 1;
		//    NVICrx_Init.NVIC_IRQChannelCmd = ENABLE;
		// NVIC_Init(&NVICrx_Init);
		USART_ITConfig(uart->uart_device, USART_IT_RXNE, DISABLE);
		USART_ITConfig(uart->uart_device, USART_IT_TC,   DISABLE);
		USART_ITConfig(uart->uart_device, USART_IT_TXE,  DISABLE);  
		USART_ITConfig(uart->uart_device, USART_IT_IDLE , ENABLE); // enable transmit idle interrupt 

		USART_DMACmd(uart->uart_device, USART_DMAReq_Rx, ENABLE);
	//	uart->uart_device->CR1|=USART_FLAG_IDLE;
		//USART_Cmd(USART1, ENABLE);
		DMA_Cmd(uart->rx_dma.rx_ch, ENABLE);
	}else{//tx dma config
    /* tx dma config */
		// dmatx_uart_config((struct stm32_uart *) serial->parent.user_data,0,0);//buf and size get  from invoke transmit function
		uart->tx_dma.setting_data_len = 0;//����ʱ����
    DMA_DeInit(uart->tx_dma.tx_ch);
    //while (DMA_GetCmdStatus(uart->tx_dma.stream) != DISABLE );
    DMAtx_Init.DMA_PeripheralBaseAddr = (uint32_t) &(uart->uart_device->DR);
    DMAtx_Init.DMA_MemoryBaseAddr = (uint32_t)NULL;
    DMAtx_Init.DMA_DIR = DMA_DIR_PeripheralDST;
    DMAtx_Init.DMA_BufferSize = 0;
    DMAtx_Init.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMAtx_Init.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMAtx_Init.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMAtx_Init.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMAtx_Init.DMA_Mode = DMA_Mode_Normal;//DMA_Mode_Normal;DMA_Mode_Circular
    DMAtx_Init.DMA_Priority = DMA_Priority_High;
		DMAtx_Init.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(uart->tx_dma.tx_ch, &DMAtx_Init);
		//DMA_Cmd(uart->tx_dma.DMArx_Init,DISABLE);
		//while(DISABLE!=DMA_GetCmdStatus (uart->tx_dma.stream));
		DMA_ClearFlag(uart->tx_dma.ch_gl_flag);
		DMA_ClearFlag(uart->tx_dma.ch_tc_flag);//clear flag

	  NVICtx_Init.NVIC_IRQChannel = uart->tx_dma.irq_channel;
    NVICtx_Init.NVIC_IRQChannelPreemptionPriority = 2;
    NVICtx_Init.NVIC_IRQChannelSubPriority = 2;
    NVICtx_Init.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVICtx_Init);
		DMA_ITConfig(uart->tx_dma.tx_ch, DMA_IT_TC, ENABLE);			//dma complete interrupt
    USART_DMACmd(uart->uart_device, USART_DMAReq_Tx, ENABLE);			//specifies the DMA request
		USART_Cmd(USART1, ENABLE);
    DMA_Cmd(uart->tx_dma.tx_ch, DISABLE);		
	}		
}
//ʹ���жϺ�DMA����ʱ��ע�����ý��ջ����С��Ĭ����200
void rt_hw_usart_init(void)
{
    struct stm32_uart* uart;
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;

    RCC_Configuration();
    GPIO_Configuration();

#if defined(RT_USING_UART1)
    uart = &uart1;
    config.baud_rate = BAUD_RATE_115200;

    serial1.ops    = &stm32_uart_ops;
    serial1.config = config;

    NVIC_Configuration(uart);

    /* register UART1 device */
    rt_hw_serial_register(&serial1, "uart1",
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX |
                          RT_DEVICE_FLAG_INT_TX |   RT_DEVICE_FLAG_DMA_RX,
                          uart);
#endif /* RT_USING_UART1 */

#if defined(RT_USING_UART2)
    uart = &uart2;

    config.baud_rate = BAUD_RATE_115200;
    serial2.ops    = &stm32_uart_ops;
    serial2.config = config;

    NVIC_Configuration(uart);

    /* register UART2 device */
    rt_hw_serial_register(&serial2, "uart2",
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX |
                          RT_DEVICE_FLAG_INT_TX |   RT_DEVICE_FLAG_DMA_RX|RT_DEVICE_FLAG_DMA_TX,
                          uart);
#endif /* RT_USING_UART2 */

#if defined(RT_USING_UART3)
    uart = &uart3;

    config.baud_rate = BAUD_RATE_115200;

    serial3.ops    = &stm32_uart_ops;
    serial3.config = config;
		//serial3.config.bufsz=10;
    NVIC_Configuration(uart);

    // register UART3 device 
    rt_hw_serial_register(&serial3, "uart3",
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX |
                          RT_DEVICE_FLAG_INT_TX | RT_DEVICE_FLAG_DMA_RX|RT_DEVICE_FLAG_DMA_TX,
                          uart);
#endif /* RT_USING_UART3 */

#if defined(RT_USING_UART4)
    uart = &uart4;

    config.baud_rate = BAUD_RATE_115200;

    serial4.ops    = &stm32_uart_ops;
    serial4.config = config;

    NVIC_Configuration(uart);

    /* register UART4 device */
    rt_hw_serial_register(&serial4, "uart4",
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX |
                          RT_DEVICE_FLAG_INT_TX |   RT_DEVICE_FLAG_DMA_RX,
                          uart);
#endif /* RT_USING_UART4 */
}
INIT_BOARD_EXPORT(rt_hw_usart_init);
