#ifndef __MAIN_H
#define __MAIN_H

	// Includes ------------------------------------------------------------------
	#include "stm32f10x.h"
	#define using_debug false //true

	#ifdef __cplusplus
	extern "C" {
	#endif 
	//----------------------------------------------------------------------------
	#ifndef NULL
		#define NULL        ((void *) 0)
	#endif
	//---------------------------------------------------------------------------
//	typedef enum { false=0, true } bool;
	#ifndef _SIZE_T
	#define _SIZE_T
	typedef unsigned int size_t;
	#endif

	typedef char int8;//The 8-bit signed data type.
	typedef volatile char vint8;//The volatile 8-bit signed data type.
	typedef unsigned char uint8;//The 8-bit unsigned data type.
	typedef volatile unsigned char vuint8;//The volatile 8-bit unsigned data type.
	typedef short int16;//The 16-bit signed data type.
	typedef unsigned short uint16;//The 16-bit unsigned data type.
	typedef long int32;//The 32-bit signed data type.
	typedef unsigned long uint32;  	//The 32-bit unsigned data type.
	typedef volatile short vint16;		// The volatile 16-bit signed data type.
	typedef volatile long vint32;		//The volatile 32-bit signed data type.
	typedef volatile unsigned short vuint16;		//The volatile 16-bit unsigned data type.
	typedef volatile unsigned long vuint32;		  //The volatile 32-bit unsigned data type.
	// bsd 
	typedef uint8           u_char;     /**< 8-bit value */
	typedef uint16          u_short;    /**< 16-bit value */
	typedef uint16          u_int;      /**< 16-bit value */
	typedef uint32          u_long;     /**< 32-bit value */
	
	//#########################位带操作################################
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
//IO口地址映射
#define GPIOA_ODR_Addr    (GPIOA_BASE+12) //0x4001080C 
#define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C 
#define GPIOC_ODR_Addr    (GPIOC_BASE+12) //0x4001100C 
#define GPIOD_ODR_Addr    (GPIOD_BASE+12) //0x4001140C 
#define GPIOE_ODR_Addr    (GPIOE_BASE+12) //0x4001180C 
#define GPIOF_ODR_Addr    (GPIOF_BASE+12) //0x40011A0C    
#define GPIOG_ODR_Addr    (GPIOG_BASE+12) //0x40011E0C 
#define GPIOA_IDR_Addr    (GPIOA_BASE+8) //0x40010808 
#define GPIOB_IDR_Addr    (GPIOB_BASE+8) //0x40010C08 
#define GPIOC_IDR_Addr    (GPIOC_BASE+8) //0x40011008 
#define GPIOD_IDR_Addr    (GPIOD_BASE+8) //0x40011408 
#define GPIOE_IDR_Addr    (GPIOE_BASE+8) //0x40011808 
#define GPIOF_IDR_Addr    (GPIOF_BASE+8) //0x40011A08 
#define GPIOG_IDR_Addr    (GPIOG_BASE+8) //0x40011E08 
//IO口操作,只对单一的IO口!
//确保n的值小于16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //输出 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //输入 
#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //输出 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //输入 
#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //输出 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //输入 
#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //输出 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //输入 
#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //输出 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //输入
#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //输出 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //输入
#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //输出 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //输入
//####################################################################

	#define CLI()      __set_PRIMASK(1)  
	#define SEI()      __set_PRIMASK(0) 
	
	#define BTN_MUTE_BIT		PAout(0)
	#define M_BAK_IN_BIT		PAin(1)
#ifdef __cplusplus
}
#endif
#endif //__MAIN_H 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
