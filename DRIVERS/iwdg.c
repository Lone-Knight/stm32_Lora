#include "iwdg.h"

void IWDG_Init(u8 pre,u16 rlr)
{
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); //取消寄存器写保护
	IWDG_SetPrescaler(pre);//设置预分频系数 0-6
	IWDG_SetReload(rlr);//设置重装载值
	IWDG_ReloadCounter();  //重装载初值
	IWDG_Enable(); //打开独立看门狗
}