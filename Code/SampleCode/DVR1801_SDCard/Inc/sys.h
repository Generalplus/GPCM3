#ifndef __SYS_H
#define __SYS_H	
//#include "GPCM1Fx.h"
//#include "GPIO_GPCM1Fx.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32������		   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/8/18
//�汾��V1.7
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved
////////////////////////////////////////////////////////////////////////////////// 	 

//0,��֧��ucos
//1,֧��ucos
//#define SYSTEM_SUPPORT_OS		0		//����ϵͳ�ļ����Ƿ�֧��UCOS
//																	    
//	 
////λ������,ʵ��51���Ƶ�GPIO���ƹ���
////����ʵ��˼��,�ο�<<CM3Ȩ��ָ��>>������(87ҳ~92ҳ).
////IO�ڲ����궨��
//#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
//#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
//#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
////IO�ڵ�ַӳ��
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
////IO�ڲ���,ֻ�Ե�һ��IO��!
////ȷ��n��ֵС��16!
//#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //��� 
//#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //���� 

//#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //��� 
//#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //���� 

////#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //��� 
////#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //���� 

////#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //��� 
////#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //���� 

////#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //��� 
////#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //����

////#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //��� 
////#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //����

////#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //��� 
////#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //����

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
}Bits_32_TYPEDef;			//�Խṹ������һ��λ��
//����Ϊ��ຯ��
//void WFI_SET(void);		//ִ��WFIָ��
//void INTX_DISABLE(void);//�ر������ж�
//void INTX_ENABLE(void);	//���������ж�
//void MSR_MSP(u32 addr);	//���ö�ջ��ַ

#endif
