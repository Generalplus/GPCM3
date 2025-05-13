#ifndef __SYS_H
#define __SYS_H
#define u8   unsigned char
#define u16  unsigned int
#define u32  unsigned long

//#include "GPCM1Fx.h"
#include "GPCM3_FM1.h"
/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C"{
#endif

void delay_us(uint32_t time);
void delay_ms(uint32_t time);


#ifdef __cplusplus
}
#endif
#endif
