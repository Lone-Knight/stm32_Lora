
#ifndef __show_inf_H__
#define __show_inf_H__
#include "main.h"
struct s_msg
{
	uint16	msgid;
	uint8	datalen;
	void * data_ptr;
};
typedef struct s_msg message;
//VGUS text address
//#############################page 1.jpg#################################
#define buttn_fireup_addr  		0x0500	//���Ϸ���
#define buttn_firedn_addr   	0x0502	//���·���
#define buttn_faultup_addr   	0x0504	//�����Ϸ���
#define buttn_faultdn_addr  	0x0506	//�����·���
#define Text_add_firenum 			0x0600	//�𾯼���
#define Text_add_faultnum  		0x0680	//���ϼ���
#define Text_add_firerecdup 	0x0700	//�𾯼�¼��
#define Text_add_firerecddn 	0x0780	//�𾯼�¼��
#define Text_add_faultrecdup	0x0800	//���ϼ�¼��
#define Text_add_faultrecddn	0x0880	//���ϼ�¼��
//#############################page 3.jpg#################################
#define buttn_fireup_his_addr  		0x0510	//����ʷ�Ϸ�
#define buttn_firedn_his_addr   	0x0512	//����ʷ�·�
#define buttn_faultup_his_addr   	0x0514	//������ʷ�Ϸ�
#define buttn_faultdn_his_addr  	0x0516	//����ʷ�·�

#define Text_add_his_firenum 			0x0900	//����ʷ����
#define Text_add_his_faultnum			0x0980	//������ʷ����
#define Text_add_his_firerecdup		0x1000	//����ʷ��¼��
#define Text_add_his_firerecddn		0x1080	//����ʷ��¼��
#define Text_add_his_faultrecdup	0x1100	//������ʷ��¼��
#define Text_add_his_faultrecddn 	0x1180	//������ʷ��¼��
//enum commond{SHOW_FIRUP=0,SHOW_FIRDN,SHOW_FSULTUP,SHOW_FSULTDN,SHOW_FIRENUM,SHOW_FSULTNUM};
#define vgus_rtc_reg   				0x2007
#define ProFix0   					0xA5
#define ProFix1   					0x5A
//[ָ��֡ͷ][ָ���][ָ��][��ʼ��ַ][���ݳ���][��������][CRC У����]
#define LEN_OFFET   				0x0002
enum vgus_addr
	{
    SHOW_FIRUP				=	Text_add_firerecdup,
    SHOW_FIRDN				=	Text_add_firerecddn,
    SHOW_FAULTUP			=	Text_add_faultrecdup,
    SHOW_FAULTDN			=	Text_add_faultrecddn,
    SHOW_FIRENUM			=	Text_add_firenum,
    SHOW_FAULTNUM			=	Text_add_faultnum,
		
		SHOW_HIS_FIRUP		=	Text_add_his_firerecdup,
    SHOW_HIS_FIRDN		=	Text_add_his_firerecddn,
    SHOW_HIS_FAULTUP	=	Text_add_his_faultrecdup,
    SHOW_HIS_FAULTDN	=	Text_add_his_faultrecddn,
    SHOW_HIS_FIRENUM	=	Text_add_his_firenum,
    SHOW_HIS_FAULTNUM	=	Text_add_his_faultnum
  };
	#define IS_FIRE 1
	#define IS_FAULT 0
	#define FMT_RECHIS_NUM  "%05d"   //��ʾ��ʷ��¼����
	#define FMT_REC_NUM  		"%05d"   //��ʾ��¼����
	#define FMT_NULL_REC 		"%04d                    ��                                  "
void showchanged_thread_entry(void *parameter);
void Show_Text(enum vgus_addr vgus_text,const char *fmt, ...);//ʹ�ô��� 2
#define Show_Record(vgus_text,rec,index)   			Show_Text(vgus_text,\
"%04d %02d/%02d/%02d-%02d:%02d:%02d %01dͨ��%05d.%01dm %s",\
index,\
(rec.year),\
(rec.moth),\
(rec.day),\
(rec.hour),\
(rec.min),\
(rec.sec),\
rec.chanel,\
rec.pos/10,\
rec.pos%10,\
alarm_str[rec.type%3]);
#endif