#ifndef _iwdg_H
#define _iwdg_H

#include "stm32f10x.h"
void IWDG_Init(u8 pre,u16 rlr);
void IWDG_FeedDog(void);  //Î¹¹·
#define  IWDG_FeedDog()  IWDG_ReloadCounter()
#endif

