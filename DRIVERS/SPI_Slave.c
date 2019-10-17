#include "SPI_Slave.h"
#include "CRC16.h"
#include "main.h"
#include <string.h>
#define SPI1_MOSI_PIN      	GPIO_Pin_7
#define SPI1_MISO_PIN      	GPIO_Pin_6
#define SPI1_SCLK_PIN     	GPIO_Pin_5
#define SPI1_NSS_PIN      	GPIO_Pin_4

#define SPI1_GPIO       						GPIOA
#define SPI1_GPIO_PeriphClock       RCC_APB2Periph_GPIOA
#define SPI1_PeriphClock						RCC_APB2Periph_SPI1

void  DMA1_Spi1_Init(void);
void	Timer_Config(void);
void SPI_Slave_Init(void)
{
  SPI_InitTypeDef  SPI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  // Enable SPI1 and GPIO clocks 
  RCC_APB2PeriphClockCmd(SPI1_PeriphClock|RCC_APB2Periph_AFIO|SPI1_GPIO_PeriphClock, ENABLE);//SPI_FLASH_SPI Periph clock enable 
	SPI_Cmd(SPI1, DISABLE);
  // Configure SPI pins: MISO
  GPIO_InitStructure.GPIO_Pin 	= 	SPI1_MISO_PIN;
	GPIO_InitStructure.GPIO_Speed = 	GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode 	= 	GPIO_Mode_AF_PP;//复用推免输出
  GPIO_Init(SPI1_GPIO, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin 	= 	SPI1_MOSI_PIN|SPI1_SCLK_PIN;
	GPIO_InitStructure.GPIO_Speed = 	GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode 	= 	GPIO_Mode_IN_FLOATING;
	GPIO_Init(SPI1_GPIO, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin 	= 	SPI1_NSS_PIN;
	GPIO_InitStructure.GPIO_Speed = 	GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode 	= 	GPIO_Mode_IN_FLOATING;
	GPIO_Init(SPI1_GPIO, &GPIO_InitStructure);
	
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode 			= SPI_Mode_Slave;
  SPI_InitStructure.SPI_DataSize 	= SPI_DataSize_16b;
  SPI_InitStructure.SPI_CPOL 			= SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA 			= SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_NSS 			= SPI_NSS_Hard;//SPI_NSS_Hard;SPI_NSS_Soft
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI1, &SPI_InitStructure);
	//Timer_Config();
	//SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, ENABLE); 

	DMA1_Spi1_Init();
	DMA_Cmd(DMA1_Channel2,ENABLE);
	DMA_Cmd(DMA1_Channel3,ENABLE);
	SPI_Cmd(SPI1, ENABLE);			//Enable SPI1
	
}

char flag=1;
volatile u16 data;
void TIM2_IRQHandler(void)
{
	//TIM2->CR1 &= ~TIM_CR1_CEN;
	TIM2->SR = (uint16_t)~TIM_FLAG_Update;
	if(flag)
	{
		flag=0;
		GPIO_ResetBits(GPIOB, GPIO_Pin_12);
	}else{
		flag=1;
		GPIO_SetBits(GPIOB, GPIO_Pin_12);
	}
  rt_kprintf("\r\n IRQ 0x%x\r\n",data);
}


//DMA传输尺寸
#define Pkagesiz 6
#define SPI1DMA_TRRXSIZ  20  
#define SPI1DMA_TRTXSIZ  10  
u16 SPI1_RCVDAT[SPI1DMA_TRRXSIZ]={0};
u16 SPI1_TXDDAT[SPI1DMA_TRTXSIZ]={0};
u16 parFramedata[SPI1DMA_TRRXSIZ]={0};
#define SPI_SLAVE_DMA		DMA1_Channel2
static u16  curr_recv_index=0;
static u16  last_recv_index=0;
static u16 recv_len=0;
volatile u8 bHandleSPIFrame=0;
static u16 * pMem ;
void  DMA1_Spi1_Init(void)
{
		DMA_InitTypeDef DMA1_Init;
		NVIC_InitTypeDef NVIC_InitSPI1TX,NVIC_InitTyp;
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
		EXTI_InitTypeDef EXTI_InitTyp;
		//NVIC_InitTyp.NVIC_IRQChannel 									= DMA1_Channel3_IRQn;
		//NVIC_InitTyp.NVIC_IRQChannelPreemptionPriority = 1;
		//NVIC_InitTyp.NVIC_IRQChannelSubPriority 				= 2;
		//NVIC_InitTyp.NVIC_IRQChannelCmd 								= ENABLE;
		//NVIC_Init(&NVIC_InitTyp);
		NVIC_InitTyp.NVIC_IRQChannel 										= DMA1_Channel2_IRQn;
		NVIC_InitTyp.NVIC_IRQChannelPreemptionPriority 	= 1;
		NVIC_InitTyp.NVIC_IRQChannelSubPriority 				= 2;
		NVIC_InitTyp.NVIC_IRQChannelCmd 								= ENABLE;
		NVIC_Init(&NVIC_InitTyp);
		//DMA_SPI_RX  SPI->RAM的数据传输
		DMA_DeInit(DMA1_Channel2);	
		DMA1_Init.DMA_PeripheralBaseAddr	=	(u32)&SPI1->DR;//启动传输前装入实际RAM地址
		DMA1_Init.DMA_MemoryBaseAddr			=	(u32)SPI1_RCVDAT;
		DMA1_Init.DMA_DIR									=	DMA_DIR_PeripheralSRC;
		DMA1_Init.DMA_BufferSize					=	SPI1DMA_TRRXSIZ;//DMA_MemoryDataSize个数
		DMA1_Init.DMA_MemoryInc						=	DMA_MemoryInc_Enable;
		DMA1_Init.DMA_PeripheralInc				=	DMA_PeripheralInc_Disable;
		DMA1_Init.DMA_MemoryDataSize			=	DMA_MemoryDataSize_HalfWord;
		DMA1_Init.DMA_PeripheralDataSize	=	DMA_PeripheralDataSize_HalfWord;
		DMA1_Init.DMA_Mode								=	DMA_Mode_Normal;
		DMA1_Init.DMA_Priority						=	DMA_Priority_High; 
		DMA1_Init.DMA_M2M									=	DMA_M2M_Disable;

		DMA_Init(DMA1_Channel2,&DMA1_Init);	 //对DMA通道3进行初始化
		//DMA_SPI_TX  RAM->SPI的数据传输
		DMA_DeInit(DMA1_Channel3);
		DMA1_Init.DMA_PeripheralBaseAddr	=	(u32)&SPI1->DR;//启动传输前装入实际RAM地址
		DMA1_Init.DMA_MemoryBaseAddr			=	(u32)&SPI1_TXDDAT;
		DMA1_Init.DMA_DIR									=	DMA_DIR_PeripheralDST; 
		DMA1_Init.DMA_BufferSize					=	SPI1DMA_TRTXSIZ;
		DMA1_Init.DMA_PeripheralInc				=	DMA_PeripheralInc_Disable;
		DMA1_Init.DMA_MemoryInc						=	DMA_MemoryInc_Enable;
		DMA1_Init.DMA_PeripheralDataSize	=	DMA_PeripheralDataSize_HalfWord;
		DMA1_Init.DMA_MemoryDataSize			=	DMA_MemoryDataSize_HalfWord;
		DMA1_Init.DMA_Mode								=	DMA_Mode_Normal;
		DMA1_Init.DMA_Priority						=	DMA_Priority_High; 
		DMA1_Init.DMA_M2M									=	DMA_M2M_Disable;
		DMA_Init(DMA1_Channel3,&DMA1_Init); //对DMA通道3进行初始化
		SPI_I2S_DMACmd(SPI1,SPI_I2S_DMAReq_Rx,ENABLE); //使能SPI的DMA接收请求
		SPI_I2S_DMACmd(SPI1,SPI_I2S_DMAReq_Tx,ENABLE); //使能SPI的DMA发送请求
		DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, ENABLE);//使能DMA传输完成中断
		//DMA_ITConfig(DMA1_Channel3, DMA_IT_TC, ENABLE);//使能DMA传输完成中断
		
		curr_recv_index=0;
		pMem	=	SPI1_RCVDAT;
		
		NVIC_InitTyp.NVIC_IRQChannel = EXTI4_IRQn;						//EXTI中断通道
		NVIC_InitTyp.NVIC_IRQChannelPreemptionPriority=2;		//抢占优先级
		NVIC_InitTyp.NVIC_IRQChannelSubPriority =3;					//子优先级
		NVIC_InitTyp.NVIC_IRQChannelCmd = ENABLE;						//IRQ通道使能
		NVIC_Init(&NVIC_InitTyp);	//根据指定的参数初始化VIC寄存器
		
		EXTI_InitTyp.EXTI_Line=EXTI_Line4; 
		EXTI_InitTyp.EXTI_Mode=EXTI_Mode_Interrupt;
		EXTI_InitTyp.EXTI_Trigger=EXTI_Trigger_Rising;
		EXTI_InitTyp.EXTI_LineCmd=ENABLE;
		EXTI_Init(&EXTI_InitTyp);
		GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource4);
		SPI1_TXDDAT[0]=1;
		SPI1_TXDDAT[1]=2;
		SPI1_TXDDAT[2]=3;
		SPI1_TXDDAT[3]=4;
		SPI1_TXDDAT[4]=5;
		SPI1_TXDDAT[5]=6;
		SPI1_TXDDAT[6]=7;
		SPI1_TXDDAT[7]=8;
		SPI1_TXDDAT[8]=9;
		SPI1_TXDDAT[9]=10;
}
volatile static u8  bDMAFlag=0;
extern struct	rt_messagequeue  alarm_mq;
void EXTI4_IRQHandler(void)
{	
	volatile u16 * src_ptr=0;
	volatile u16  CNT= (u16)(DMA1_Channel2->CNDTR);
	alarm_data msg_alarm_data={0};
	u32 tmp32=0;
	u8 j=0;
	u16 i =0;
	if(EXTI_GetITStatus(EXTI_Line4)==1)
	{
			//rt_kprintf("\r\nCNDTR%d\r\n",CNT);
		//拷贝接收数据
		recv_len	=	SPI1DMA_TRRXSIZ	-	SPI_SLAVE_DMA->CNDTR;
		memcpy(	parFramedata	,	SPI1_RCVDAT,	recv_len<<1);
		SPI_SLAVE_DMA->CCR 		&= 	(uint16_t)(~DMA_CCR1_EN);
		SPI_SLAVE_DMA->CMAR 	 =	(u32)	SPI1_RCVDAT;
		SPI_SLAVE_DMA->CNDTR	 =	SPI1DMA_TRRXSIZ;	
		SPI_SLAVE_DMA->CCR 		|=  DMA_CCR1_EN;
		//准备下次返回的数据
		DMA1_Channel3->CCR 		 &= (uint16_t)(~DMA_CCR1_EN);
		DMA1_Channel3->CNDTR		=	SPI1DMA_TRTXSIZ-1;
		SPI1->DR								=	SPI1_TXDDAT[0];
		DMA1_Channel3->CMAR			=	(u32)&SPI1_TXDDAT[1];
		DMA1_Channel3->CCR		 |= DMA_CCR1_EN;
		#if(using_debug)
		rt_kprintf("recv:\n");
		for(i =0 ;i<recv_len;i++)
		{
				rt_kprintf("%4x;",SPI1_RCVDAT[i]);
		}
		rt_kprintf("\n");
		#endif

			if((0x55AA==parFramedata[0]))
			{
				if(Pkagesiz==recv_len)
				{
					if(funCRC16((u8 *)parFramedata,(Pkagesiz-1)<<1)==parFramedata[Pkagesiz-1])
					{	
						tmp32=parFramedata[3];//位置值高半字
						tmp32=tmp32<<16;
						tmp32=tmp32+parFramedata[2];//位置值低半字
						msg_alarm_data.type		=	(parFramedata[1]>>8);//高字节报警类型
						msg_alarm_data.chanel	=	parFramedata[1]&0x00FF;//低字节通道号
						msg_alarm_data.pos		=	tmp32;
						msg_alarm_data.temp=parFramedata[4]%0x0100;//温度值
						rt_mq_send(&alarm_mq,&msg_alarm_data,sizeof(msg_alarm_data));
					}
				}
			}
		EXTI_ClearITPendingBit(EXTI_Line4);
	}
}
//SPI1 DMA channel2 中断
void DMA1_Channel2_IRQHandler(void)//接收中断
{
 if(DMA_GetFlagStatus(DMA1_IT_TC2) == SET)
 {
	bDMAFlag=1;
  DMA_ClearITPendingBit(DMA1_IT_TC2);
 }
}
//#define StartTimer()  { TIM2->EGR = TIM_PSCReloadMode_Immediate;TIM2->CR1 |= TIM_CR1_CEN;}
//void SPI1_IRQHandler(void) 
//{
//	static u16 i=0;
//	if(SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_RXNE) == SET)
//	{ 
//			
//		//	StartTimer();
//			SPI_I2S_ClearITPendingBit(SPI1, SPI_I2S_IT_RXNE);
//	} 
//} 
//void	Timer_Config(void)
//{ 
//		TIM_TimeBaseInitTypeDef TIM_TimeBase;
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE); 
//    TIM_DeInit(TIM2);
//    TIM_TimeBase.TIM_Period=4000-1;  //自动重装载寄存器的值 4ms
//    TIM_TimeBase.TIM_Prescaler=(72-1);         //时钟预分频数
//    TIM_TimeBase.TIM_ClockDivision=TIM_CKD_DIV1;  //采样分频
//    TIM_TimeBase.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
//    TIM_TimeBaseInit(TIM2,&TIM_TimeBase);
//    TIM_ClearFlag(TIM2,TIM_FLAG_Update);               //清除溢出中断标志
//    TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
//    // TIM_Cmd(TIM2,ENABLE);                              //开启时钟
//} 
//SPI 数据包定义 以 U16为最小单位
// 		header		Lenth   	data		tail
//	  1个长度	 1个长度	2个长度	 1个长度
//		0X55AA			2				xx xx		CRC16