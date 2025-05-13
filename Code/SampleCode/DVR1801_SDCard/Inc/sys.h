#ifndef __SYS_H
#define __SYS_H	
//#include "GPCM1Fx.h"
//#include "GPIO_GPCM1Fx.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32开发板		   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/8/18
//版本：V1.7
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved
////////////////////////////////////////////////////////////////////////////////// 	 

//0,不支持ucos
//1,支持ucos
//#define SYSTEM_SUPPORT_OS		0		//定义系统文件夹是否支持UCOS
//																	    
//	 
////位带操作,实现51类似的GPIO控制功能
////具体实现思想,参考<<CM3权威指南>>第五章(87页~92页).
////IO口操作宏定义
//#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
//#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
//#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
////IO口地址映射
//#define GPIOA_ODR_Addr    (GPIOA_BASE+0x14) //0x4001080C 
//#define GPIOB_ODR_Addr    (GPIOB_BASE+0x14) //0x40010C0C 
////#define GPIOC_ODR_Addr    (GPIOC_BASE+12) //0x4001100C 
////#define GPIOD_ODR_Addr    (GPIOD_BASE+12) //0x4001140C 
////#define GPIOE_ODR_Addr    (GPIOE_BASE+12) //0x4001180C 
////#define GPIOF_ODR_Addr    (GPIOF_BASE+12) //0x40011A0C    
////#define GPIOG_ODR_Addr    (GPIOG_BASE+12) //0x40011E0C    

//#define GPIOA_IDR_Addr    (GPIOA_BASE+0x14) //0x40010808 
//#define GPIOB_IDR_Addr    (GPIOB_BASE+0x14) //0x40010C08 
////#define GPIOC_IDR_Addr    (GPIOC_BASE+8) //0x40011008 
////#define GPIOD_IDR_Addr    (GPIOD_BASE+8) //0x40011408 
////#define GPIOE_IDR_Addr    (GPIOE_BASE+8) //0x40011808 
////#define GPIOF_IDR_Addr    (GPIOF_BASE+8) //0x40011A08 
////#define GPIOG_IDR_Addr    (GPIOG_BASE+8) //0x40011E08 
// 
////IO口操作,只对单一的IO口!
////确保n的值小于16!
//#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //输出 
//#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //输入 

//#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //输出 
//#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //输入 

////#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //输出 
////#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //输入 

////#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //输出 
////#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //输入 

////#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //输出 
////#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //输入

////#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //输出 
////#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //输入

////#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //输出 
////#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //输入

#define u8   unsigned char
#define u16  unsigned int
#define u32  unsigned long 


 typedef struct _32_Bits_Struct
{
	u32	bit0	:	1;
	u32	bit1	:	1;
	u32	bit2	:	1;
	u32	bit3	:	1;
	u32	bit4	:	1;
	u32	bit5	:	1;
	u32	bit6	:	1;
	u32	bit7	:	1;
	u32	bit8	:	1;
	u32	bit9	:	1;
	u32	bit10	:	1;
	u32	bit11	:	1;
	u32	bit12	:	1;
	u32	bit13	:	1;
	u32	bit14	:	1;
	u32	bit15	:	1;
	u32	bit16	:	1;
	u32	bit17	:	1;
	u32	bit18	:	1;
	u32	bit19	:	1;
	u32	bit20	:	1;
	u32	bit21	:	1;
	u32	bit22	:	1;
	u32	bit23	:	1;
	u32	bit24	:	1;
	u32	bit25	:	1;
	u32	bit26	:	1;
	u32	bit27	:	1;
	u32	bit28	:	1;
	u32	bit29	:	1;
	u32	bit30	:	1;
	u32	bit31	:	1;
}Bits_32_TYPEDef;			//对结构体声明一个位域
//以下为汇编函数
//void WFI_SET(void);		//执行WFI指令
//void INTX_DISABLE(void);//关闭所有中断
//void INTX_ENABLE(void);	//开启所有中断
//void MSR_MSP(u32 addr);	//设置堆栈地址

#endif
