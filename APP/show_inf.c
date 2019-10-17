#include <rtthread.h>
#include "rtdevice.h"
#include "stm32f10x.h"
#include "usart.h"
#include "show_inf.h"
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
const unsigned char alarm_str[3][7]={{"����"},{"����"},{"����"}};
static rt_device_t _Serial1_device = RT_NULL;
rt_err_t ShowUart_tx_complete(rt_device_t dev, void *buffer);
rt_err_t ShowUart_rx_indicate(rt_device_t dev, rt_size_t size);	
struct  rt_messagequeue  vgus_rx_mq;
struct	rt_messagequeue  alarm_mq;
record_data gRecord;
const unsigned char GET_RTC_CMD[6]={0xA5,0x5A,0x03,0x81,0x20,0x07};//��ȡ��ǰRTC���ڼ�ʱ��
#define getcurtime() {RT_ASSERT(_Serial1_device != RT_NULL);_Serial1_device->write(_Serial1_device,0,GET_RTC_CMD,6);}			//��ȡ��ǰ���ڼ�ʱ��
#define Text_BUF_SIZE 120
static  char log_buf[Text_BUF_SIZE];
//####################################���������ݷ��ʹ�����############
static bool flag_ShowUarttx=1;//Ϊ1ʱ��ʹ������Դ����ʹ�ã�Ϊ0ʱ��ʹ���ڷ��Ͳ���ʹ��
rt_err_t ShowUart_tx_complete(rt_device_t dev, void *buffer)
{
	flag_ShowUarttx=1;//�ͷ�Ӳ����Դ
	return 0;
}
//####################################���������ݽ��մ�����############
#define show_rxbuf_len 100
static uint8 show_rxbuf[show_rxbuf_len];
void parse_inf(uint8 * data ,unsigned char len);
rt_err_t ShowUart_rx_indicate(rt_device_t dev, rt_size_t size)
{
		static unsigned long cnt1=0;
		dev->read(dev,0,show_rxbuf,size);
		parse_inf(show_rxbuf,size);
		return 0;
}
void parse_inf(uint8* data ,unsigned char len);
#define vgus_rx_msg_pool_siz 10
#define alarm_msg_pool_siz 20
message vgus_rx_msg_pool[vgus_rx_msg_pool_siz]={0};
alarm_data alarm_msg_pool[alarm_msg_pool_siz]={0};

//bool check_flash=false;
void SPI_FLASH_BulkErase(void);
extern	struct   rt_messagequeue  store_mq;//������Ϣ
//bool bGetDateTime=0;
static bool bDateTimeIsget=0;
static bool brecordsave=0;
void test_alarm(void )
{
	static  u32 i=1;
	static u8 ch=1,temp=10,z=0;
	alarm_data msg_alarm_data={0};
	i=i+5;
	msg_alarm_data.type=z++%3;//FIRETYPE_CHAWEN
	msg_alarm_data.temp=temp++;
	msg_alarm_data.pos=i;
	msg_alarm_data.chanel=ch++%4;
	rt_mq_send(&alarm_mq,&msg_alarm_data,sizeof(msg_alarm_data));//�����̷߳�����ϢRT_EOK !=
	//ʹ����Ϣ���д���������
}
//ʹ����Ϣ���д���
#define firerecordshow_num 			2
#define faultrecordshow_num 		2
#define firerecordshow_his_num 	2
#define faultrecordshow_his_num 2
static record_data fire_record[firerecordshow_num]={0};
static record_data fault_record[faultrecordshow_num]={0};
static record_data fire_his_record[firerecordshow_his_num]={0};
static record_data fault_his_record[faultrecordshow_his_num]={0};
static u32 fire_his_recordnum=0,fault_his_recordnum=0;
u32 fire_recordnum=0,fault_recordnum=0;//��¼�ϵ��ı�����
static u32 fire_his_recordIndex=0,fault_his_recordIndex=0;
static u32 fire_recordIndex=0,fault_recordIndex=0;
void showchanged_thread_entry(void *parameter)
{
		rt_device_t new, old;
		rt_err_t result;
		uint16 tempdata;
		uint16 untemp;
		u32 temp=0;
		message msg_vgus={0};
		message msg={0};
		alarm_data msg_alarm_data={0};
		record_data temp_record={0};
		result = rt_mq_init(&vgus_rx_mq,"vgus_rx_mq",vgus_rx_msg_pool,sizeof(message),sizeof(vgus_rx_msg_pool),RT_IPC_FLAG_FIFO ); //����ж���̵߳ȴ������������ȵõ��ķ���������Ϣ
		result = rt_mq_init(&alarm_mq,"alarm_mq",alarm_msg_pool,sizeof(alarm_data),sizeof(alarm_msg_pool),RT_IPC_FLAG_FIFO ); 			//����ж���̵߳ȴ������������ȵõ��ķ���������Ϣ
		RT_ASSERT(RT_EOK==result)
		//#################################################################
	  old = _Serial1_device;
		old=old;
    new = rt_device_find("uart2");    // find new console device 
    if (new != RT_NULL)
    {
        if (_Serial1_device != RT_NULL)
        {
						//close old console device
            rt_device_close(_Serial1_device);
        }
				//set new console device 
        rt_device_open(new, RT_DEVICE_OFLAG_RDWR|RT_DEVICE_FLAG_STREAM|RT_DEVICE_FLAG_DMA_TX|RT_DEVICE_FLAG_DMA_RX);//RT_DEVICE_FLAG_DMA_TX
        _Serial1_device = new;
				_Serial1_device->tx_complete=ShowUart_tx_complete;
				_Serial1_device->rx_indicate=ShowUart_rx_indicate;
			}
			#if(using_debug)
			rt_kprintf("\nshowchanged_thread!!");  
			#endif
//				fire_his_recordnum=get_Record_num(IS_FIRE);//��ȡ�𾯱�����¼
//				fault_his_recordnum=get_Record_num(IS_FAULT);//��ȡ�𾯱�����¼
//				if(fire_his_recordnum)
//				{
//					#if(using_debug)
//					rt_kprintf("\r\nmutex1\r\n");
//					#endif
//					fire_record	[0]	=	fire_his_record	[0]	=	read_record(fire_his_recordnum,IS_FIRE);
//					#if(using_debug)
//					rt_kprintf("\r\nmutex2\r\n");
//					#endif
//					Show_Record(SHOW_HIS_FIRUP,fire_his_record[0],fire_his_recordIndex+1);
//					if(1<fire_his_recordnum)
//					{
//						fire_record	[1]	=	fire_his_record	[1]	=	read_record(fire_his_recordnum-1,IS_FIRE);
//						Show_Record(SHOW_HIS_FIRDN,fire_his_record[0],fire_his_recordIndex+2);
//					}else
//					{
//						Show_Text(SHOW_HIS_FIRDN,FMT_NULL_REC,fire_his_recordIndex+2);
//					}
//				}		
//				Show_Text(SHOW_HIS_FIRENUM,FMT_RECHIS_NUM,fire_his_recordnum);//��ʾ��ʷ��¼����
//				if(fault_his_recordnum)
//				{
//					fault_record	[0]	=	fault_his_record	[0]	=	read_record(fault_his_recordnum,IS_FAULT);
//					Show_Record(SHOW_HIS_FAULTUP,fault_his_record[0],fault_his_recordIndex+1);
//					if(1<fault_his_recordnum)
//					{
//						fault_record	[1]	=	fault_his_record	[1]	=	read_record(fault_his_recordnum-1,IS_FAULT);
//						Show_Record(SHOW_HIS_FAULTDN,fault_his_record[1],fault_his_recordIndex+2);
//					}
//					else
//					{
//						Show_Text(SHOW_HIS_FAULTDN,FMT_NULL_REC,fault_his_recordIndex+2);
//					}
//				}	
//				Show_Text(SHOW_HIS_FAULTNUM,FMT_RECHIS_NUM,fault_his_recordnum);//��ʾ��ʷ��¼����	
					
					msg.datalen=0;
					msg.data_ptr=NULL;//��ȡ�ڴ�ռ�
					
					msg.msgid=buttn_fireup_addr;
					rt_mq_send(&vgus_rx_mq,&msg,sizeof(msg));//�����̷߳�����Ϣ
					msg.msgid=buttn_faultup_addr;
					rt_mq_send(&vgus_rx_mq,&msg,sizeof(msg));//�����̷߳�����Ϣ
					msg.msgid=buttn_fireup_his_addr;
					rt_mq_send(&vgus_rx_mq,&msg,sizeof(msg));//�����̷߳�����Ϣ
					msg.msgid=buttn_faultup_his_addr;
					rt_mq_send(&vgus_rx_mq,&msg,sizeof(msg));//�����̷߳�����Ϣ
					
					
		while(1)
		{
			if(RT_EOK==rt_mq_recv (&alarm_mq,&msg_alarm_data,sizeof(msg_alarm_data),RT_WAITING_NO))
			{
				gRecord.pos		=msg_alarm_data.pos;
				gRecord.temp	=msg_alarm_data.temp;
				gRecord.type	=msg_alarm_data.type;
				gRecord.chanel=msg_alarm_data.chanel;
				untemp=10;
				do
				{			
						if(flag_ShowUarttx)
					{
							flag_ShowUarttx=0;
							//Show_Text(SHOW_FIRUP,"%03d %02d/%02d/%02d-%02d:%02d:%02d %01dͨ��%05d.%01dm %s",1,19,1,2,3,4,5,1,34,5,"����");
							bDateTimeIsget=0;	//��bDateTimeIsget=1ʱ��ʾ��ǰʱ���Ǹ��¹���
							//bGetDateTime=0;
							brecordsave=1;		//��ȡ�¼�����Ҫ�洢
							getcurtime();			//���ͻ�ȡʱ��ָ��	
							break;
					} else
					{//�ȴ�������Դ�ͷ�
						rt_thread_delay(50);
					}
					untemp--;
				}while(untemp);
			}
			else
			if(RT_EOK==rt_mq_recv (&vgus_rx_mq,&msg_vgus,sizeof(msg_vgus),RT_WAITING_NO))
			{
						tempdata=0;
						if(msg_vgus.data_ptr)
						{
							tempdata=*((unsigned char*) msg_vgus.data_ptr);
							tempdata=tempdata<<8;
							tempdata+=*((unsigned char*) msg_vgus.data_ptr+1);
						}
				switch(msg_vgus.msgid)//���н�����Ϣ��������Ӧ�Ĵ���
				{
					case vgus_rtc_reg://���ص���RTC����,ʹ�ô�����Ļ��ʱ��Դ
							bDateTimeIsget=1;
						if(brecordsave)
						{
								gRecord.year=	BCD2byte(*((u8*) msg_vgus.data_ptr));
								gRecord.moth=	BCD2byte(*((u8*) msg_vgus.data_ptr+1));
								gRecord.day	=	BCD2byte(*((u8*) msg_vgus.data_ptr+2));
								gRecord.hour=	BCD2byte(*((u8*) msg_vgus.data_ptr+4));
								gRecord.min	=	BCD2byte(*((u8*) msg_vgus.data_ptr+5));
								gRecord.sec	=	BCD2byte(*((u8*) msg_vgus.data_ptr+6));	
								brecordsave=0;
								rt_mq_send(&store_mq,&gRecord,sizeof(gRecord));//�����̷߳�����Ϣ	���д洢	
						}			
						break;
					case buttn_fireup_addr:			//���Ϸ���	
							fire_his_recordnum=get_Record_num(IS_FIRE);//��ȡ�𾯱�����¼
							if(fire_recordIndex) fire_recordIndex--;
							if(1<fire_recordnum)//����ʾ����
							{
								fire_record	[0]	=	read_record(fire_his_recordnum-fire_recordIndex,IS_FIRE);
								Show_Record(SHOW_FIRUP,fire_record[0],fire_recordIndex+1);
								fire_record	[1]	=	read_record(fire_his_recordnum-fire_recordIndex-1,IS_FIRE);
								Show_Record(SHOW_FIRDN,fire_record[1],fire_recordIndex+2);
			
							}else if(0<fire_recordnum)//����ʾһ��
							{
								fire_recordIndex=0;
								fire_record	[0]	=	read_record(fire_his_recordnum-fire_recordIndex,IS_FIRE);
								Show_Record(SHOW_FIRUP,fire_record[0],fire_recordIndex+1);
								Show_Text(SHOW_FIRDN,FMT_NULL_REC,fire_recordIndex+2);
							}else
							{
								fire_recordIndex=0;
								Show_Text(SHOW_FIRUP,FMT_NULL_REC,fire_recordIndex+1);
								Show_Text(SHOW_FIRDN,FMT_NULL_REC,fire_recordIndex+2);
							}
							Show_Text(SHOW_FIRENUM,FMT_REC_NUM,fire_recordnum);//��ʾ��ʷ��¼����
						break;
					case buttn_firedn_addr:			//���·���
							fire_his_recordnum=get_Record_num(IS_FIRE);//��ȡ�𾯱�����¼
							fire_recordIndex++;
							if((fire_recordIndex+2)<=fire_recordnum)//����ʾ����
							{
							fire_record	[0]	=	read_record(fire_his_recordnum-fire_recordIndex,IS_FIRE);
							Show_Record(SHOW_FIRUP,fire_record[0],fire_recordIndex+1);
							fire_record	[1]	=	read_record(fire_his_recordnum-fire_recordIndex-1,IS_FIRE);
							Show_Record(SHOW_FIRDN,fire_record[1],fire_recordIndex+2);

							}else if((fire_recordIndex+1)<=fire_recordnum)//����ʾһ��
							{
							fire_record	[0]	=	read_record(fire_his_recordnum-fire_recordIndex,IS_FIRE);
							Show_Record(SHOW_FIRUP,fire_record[0],fire_recordIndex+1);
							Show_Text(SHOW_FIRDN,FMT_NULL_REC,fire_recordIndex+2);
							}else
							{
							fire_recordIndex=fire_recordnum-1;
							}
							Show_Text(SHOW_FIRENUM,FMT_RECHIS_NUM,fire_recordnum);//��ʾ��ʷ��¼����	
						break;				
					case buttn_faultup_addr:		//�����Ϸ���
						fault_his_recordnum=get_Record_num(IS_FAULT);//��ȡ�𾯱�����¼
						if(fault_recordIndex) fault_recordIndex--;
						if(1<fault_recordnum)//����ʾ����
						{
							fault_record	[0]	=	read_record(fault_his_recordnum-fault_recordIndex,IS_FAULT);
							Show_Record(SHOW_FAULTUP,fault_record[0],fault_recordIndex+1);
							fault_record	[1]	=	read_record(fault_his_recordnum-fault_recordIndex-1,IS_FAULT);
							Show_Record(SHOW_FAULTDN,fault_record[1],fault_recordIndex+2);

						}else if(0<fault_recordnum)//����ʾһ��
						{
							fault_recordIndex=0;
							fault_record	[0]	=	read_record(fault_his_recordnum-fault_recordIndex,IS_FAULT);
							Show_Record(SHOW_FAULTUP,fault_record[0],fault_recordIndex+1);
							Show_Text(SHOW_FAULTDN,FMT_NULL_REC,fault_recordIndex+2);
						}else
						{
							fault_recordIndex=0;
						}
						Show_Text(SHOW_FAULTNUM,FMT_RECHIS_NUM,fault_recordnum);//��ʾ��ʷ��¼����
						break;
					case buttn_faultdn_addr:		//�����·���
						fault_his_recordnum=get_Record_num(IS_FAULT);//��ȡ�𾯱�����¼
						fault_recordIndex++;
						if((fault_recordIndex+2)<=fault_recordnum)//����ʾ����
						{
							fault_record	[0]	=	read_record(fault_his_recordnum-fault_recordIndex,IS_FAULT);
							Show_Record(SHOW_FAULTUP,fault_record[0],fault_recordIndex+1);
							fault_record	[1]	=	read_record(fault_his_recordnum-fault_recordIndex-1,IS_FAULT);
							Show_Record(SHOW_FAULTDN,fault_record[1],fault_recordIndex+2);

						}else if((fault_recordIndex+1)<=fault_recordnum)//����ʾһ��
						{
							fault_record	[0]	=	read_record(fault_his_recordnum-fault_recordIndex,IS_FAULT);
							Show_Record(SHOW_FAULTUP,fault_record[0],fault_recordIndex+1);
							Show_Text(SHOW_FAULTDN,FMT_NULL_REC,fault_recordIndex+2);
						}else
						{
							fault_recordIndex=fault_recordnum-1;
						}
						Show_Text(SHOW_FAULTNUM,FMT_RECHIS_NUM,fault_recordnum);//��ʾ��ʷ��¼����
						break;
					case buttn_fireup_his_addr:	//����ʷ�Ϸ�
							fire_his_recordnum=get_Record_num(IS_FIRE);//��ȡ�𾯱�����¼
							if(fire_his_recordIndex) fire_his_recordIndex--;
							if(1<fire_his_recordnum)//����ʾ����
							{
								fire_his_record	[0]	=	read_record(fire_his_recordnum-fire_his_recordIndex,IS_FIRE);
								Show_Record(SHOW_HIS_FIRUP,fire_his_record[0],fire_his_recordIndex+1);
								fire_his_record	[1]	=	read_record(fire_his_recordnum-fire_his_recordIndex-1,IS_FIRE);
								Show_Record(SHOW_HIS_FIRDN,fire_his_record[1],fire_his_recordIndex+2);
			
							}else if(0<fire_his_recordnum)//����ʾһ��
							{
								fire_his_recordIndex=0;
								fire_his_record	[0]	=	read_record(fire_his_recordnum-fire_his_recordIndex,IS_FIRE);
								Show_Record(SHOW_HIS_FIRUP,fire_his_record[0],fire_his_recordIndex+1);
								Show_Text(SHOW_HIS_FIRDN,FMT_NULL_REC,fire_his_recordIndex+2);
							}else
							{
								fire_his_recordIndex=0;
								Show_Text(SHOW_HIS_FIRUP,FMT_NULL_REC,fire_his_recordIndex+1);
								Show_Text(SHOW_HIS_FIRDN,FMT_NULL_REC,fire_his_recordIndex+2);
							}
							Show_Text(SHOW_HIS_FIRENUM,FMT_RECHIS_NUM,fire_his_recordnum);//��ʾ��ʷ��¼����
						break;
					case buttn_firedn_his_addr:	//����ʷ�·�
							fire_his_recordnum=get_Record_num(IS_FIRE);//��ȡ�𾯱�����¼
							fire_his_recordIndex++;
							if((fire_his_recordIndex+2)<=fire_his_recordnum)//����ʾ����
							{
								
								fire_his_record	[0]	=	read_record(fire_his_recordnum-fire_his_recordIndex,IS_FIRE);
								Show_Record(SHOW_HIS_FIRUP,fire_his_record[0],fire_his_recordIndex+1);
								fire_his_record	[1]	=	read_record(fire_his_recordnum-fire_his_recordIndex-1,IS_FIRE);
								Show_Record(SHOW_HIS_FIRDN,fire_his_record[1],fire_his_recordIndex+2);
			
							}else if((fire_his_recordIndex+1)<=fire_his_recordnum)//����ʾһ��
							{
								fire_his_record	[0]	=	read_record(fire_his_recordnum-fire_his_recordIndex,IS_FIRE);
								Show_Record(SHOW_HIS_FIRUP,fire_his_record[0],fire_his_recordIndex+1);
								Show_Text(SHOW_HIS_FIRDN,FMT_NULL_REC,fire_his_recordIndex+2);
							}else
							{
								fire_his_recordIndex=fire_his_recordnum-1;
							}
							Show_Text(SHOW_HIS_FIRENUM,FMT_RECHIS_NUM,fire_his_recordnum);//��ʾ��ʷ��¼����	
						break;
					case buttn_faultup_his_addr://������ʷ�Ϸ�
							fault_his_recordnum=get_Record_num(IS_FAULT);//��ȡ�𾯱�����¼
							if(fault_his_recordIndex) fault_his_recordIndex--;
							if(1<fault_his_recordnum)//����ʾ����
							{
								fault_his_record	[0]	=	read_record(fault_his_recordnum-fault_his_recordIndex,IS_FAULT);
								Show_Record(SHOW_HIS_FAULTUP,fault_his_record[0],fault_his_recordIndex+1);
								fault_his_record	[1]	=	read_record(fault_his_recordnum-fault_his_recordIndex-1,IS_FAULT);
								Show_Record(SHOW_HIS_FAULTDN,fault_his_record[1],fault_his_recordIndex+2);

							}else if(0<fault_his_recordnum)//����ʾһ��
							{
								fault_his_recordIndex=0;
								fault_his_record	[0]	=	read_record(fault_his_recordnum-fault_his_recordIndex,IS_FAULT);
								Show_Record(SHOW_HIS_FAULTUP,fault_his_record[0],fault_his_recordIndex+1);
								Show_Text(SHOW_HIS_FAULTDN,FMT_NULL_REC,fault_his_recordIndex+2);
							}else
							{
								fault_his_recordIndex=0;
							}
							Show_Text(SHOW_HIS_FAULTNUM,FMT_RECHIS_NUM,fault_his_recordnum);//��ʾ��ʷ��¼����
						break;
					case buttn_faultdn_his_addr://������ʷ�·�
							fault_his_recordnum=get_Record_num(IS_FAULT);//��ȡ�𾯱�����¼
							fault_his_recordIndex++;
							if((fault_his_recordIndex+2)<=fault_his_recordnum)//����ʾ����
							{
								fault_his_record	[0]	=	read_record(fault_his_recordnum-fault_his_recordIndex,IS_FAULT);
								Show_Record(SHOW_HIS_FAULTUP,fault_his_record[0],fault_his_recordIndex+1);
								fault_his_record	[1]	=	read_record(fault_his_recordnum-fault_his_recordIndex-1,IS_FAULT);
								Show_Record(SHOW_HIS_FAULTDN,fault_his_record[1],fault_his_recordIndex+2);

							}else if((fault_his_recordIndex+1)<=fault_his_recordnum)//����ʾһ��
							{
								fault_his_record	[0]	=	read_record(fault_his_recordnum-fault_his_recordIndex,IS_FAULT);
								Show_Record(SHOW_HIS_FAULTUP,fault_his_record[0],fault_his_recordIndex+1);
								Show_Text(SHOW_HIS_FAULTDN,FMT_NULL_REC,fault_his_recordIndex+2);
							}else
							{
								fault_his_recordIndex=fault_his_recordnum-1;
							}
							Show_Text(SHOW_HIS_FAULTNUM,FMT_RECHIS_NUM,fault_his_recordnum);//��ʾ��ʷ��¼����
		
						break;
					case 0x0400:		//�����ã�ģ��洢���ϼ�¼
					//	bGetDateTime=1;
					test_alarm();
						break;
					case 0x0402://��������洢 оƬ
						//�������
						#if(using_debug)
						rt_kprintf("\r\nbulkerase systick 0x%X \r\n",rt_tick_get());
						#endif
						SPI_FLASH_BulkErase();
						#if(using_debug)
						rt_kprintf("\r\nbulkerased systick 0x%X \r\n",rt_tick_get());
						#endif
					break;
					case 0x0406://������
						#if(using_debug)
						rt_kprintf("\r\ncheckflash%X \r\n");
						#endif
					break;
				case 0x0100://����
						#if(using_debug)
						rt_kprintf("\r\nmute%X \r\n");
						#endif
						tempdata=0;
						if(msg_vgus.data_ptr)
						{
							tempdata=*((u16*) msg_vgus.data_ptr);
							if(0x0100==tempdata) 				 //��ֵΪ0x0001ʱ�Ż�ȷ�ϼ����1����
							{
								BTN_MUTE_BIT	=		1;
							} else
							{
								BTN_MUTE_BIT	=		0;		
							}
						}
				
					break;
				}
				if(msg_vgus.data_ptr) free(msg_vgus.data_ptr);
			}else	
			rt_thread_delay(RT_TICK_PER_SECOND/5+10);
		}
}
void Show_Text(enum vgus_addr vgus_text,const char *fmt, ...)//ʹ�ô��� 2
{
	va_list args;
	unsigned int length;
	char * ptr=log_buf;
	*ptr++=ProFix0;
	*ptr++=ProFix1;
	*ptr++=0x00;//ָ��������� CMD_OFFET
	*ptr++=0x82;   
	*ptr++=vgus_text>>8;
	*ptr++=(char)vgus_text;
	//��ʽ���ı�����
	va_start(args,fmt);
	length = _vsnprintf(ptr, sizeof(log_buf)- 1-6, fmt, args);
	log_buf[LEN_OFFET]=length+3;			//���ݳ���+1byte cmd+2byte address
	while(!flag_ShowUarttx){rt_thread_delay(10);};
		 if ((_Serial1_device != RT_NULL))
	 {
			flag_ShowUarttx=0;
			_Serial1_device->write(_Serial1_device,0,log_buf,log_buf[LEN_OFFET]+3);
	 }
	va_end(args);
	 rt_thread_delay(20);
}
void parse_inf(uint8* data ,unsigned char len)
{
	message msg={0};
	CLI();
	if(4<len)
	{
		if((ProFix0==data[0])&&(ProFix1==data[1]))//ǰ׺���
		{
			if((data[2]+3)==len)//���ݰ����ȼ��
			{
				if((0x83==data[3]))//���ص���0x83��0x81ָ����н���
				{
					msg.msgid=data[4];
					msg.msgid=msg.msgid<<8;
					msg.msgid+=data[5];//�üĴ���
					msg.datalen=data[6]<<1;//���ݳ��ȣ�Э����߸��ֽ��ǳ��ȵ�λ����
					msg.data_ptr=malloc(msg.datalen);//��ȡ�ڴ�ռ�
					if(NULL!=msg.data_ptr)	
					{						
						memcpy(msg.data_ptr,(const void*)(data+7),(uint16)msg.datalen);
					//	rt_mq_send(&vgus_rx_mq,&msg,sizeof(msg));//�����̷߳�����Ϣ
						if(RT_EOK !=rt_mq_send(&vgus_rx_mq,&msg,sizeof(msg)))//�����̷߳�����Ϣ
						{
							free(msg.data_ptr);
						}	
					}
				}else if((0x81==data[3])){
					msg.msgid=data[4];
					msg.msgid=msg.msgid<<8;
					msg.msgid+=data[5];		//�üĴ���
					msg.datalen=data[2]-3;//���ݳ��ȣ���ȡRTC Э���6���ֽ�֮����Ƕ�ȡ�����ݣ�û�����ݳ��ȵ��ֽ���e
					msg.data_ptr=malloc(msg.datalen);//��ȡ�ڴ�ռ�
					if(NULL!=msg.data_ptr)	
					{						
						memcpy(msg.data_ptr,(const void*)(data+6),(uint16)msg.datalen);
						if(RT_EOK !=rt_mq_send(&vgus_rx_mq,&msg,sizeof(msg)))//�����̷߳�����Ϣ
						{
							free(msg.data_ptr);
						}	
					}
				}
			}
		}
	}
	SEI();
	return;
}