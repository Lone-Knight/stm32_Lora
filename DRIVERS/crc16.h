#ifndef _CRC16_H
#define _CRC16_H
#include "stm32f10x.h"
#define USHORT  u16
#define UCHAR  u8
USHORT          funCRC16( UCHAR * pucFrame, USHORT usLen );
#endif
