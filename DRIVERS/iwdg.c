#include "iwdg.h"

void IWDG_Init(u8 pre,u16 rlr)
{
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); //ȡ���Ĵ���д����
	IWDG_SetPrescaler(pre);//����Ԥ��Ƶϵ�� 0-6
	IWDG_SetReload(rlr);//������װ��ֵ
	IWDG_ReloadCounter();  //��װ�س�ֵ
	IWDG_Enable(); //�򿪶������Ź�
}