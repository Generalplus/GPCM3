/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:     
 *
 * @Version:  
 *   V0.9.0
 * @Date:     
 *   30th, April 2019
 * @Abstract: 
 *
 **************************************************************************************************/
#ifndef _PWMIO_GPCM1F_H_
#define _PWMIO_GPCM1F_H_


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "GPCM1Fx.h"


/*---------------------------------------------------------------------------------------
 * Definition Area
 *---------------------------------------------------------------------------------------*/
#define PWMIO_GPIOA5                  0
#define PWMIO_GPIOA6                  1
#define PWMIO_GPIOA7                  2
#define PWMIO_GPIOA8                  3
#define PWMIO_GPIOA9                  4
#define PWMIO_GPIOA10                 5
#define PWMIO_GPIOA11                 6
#define PWMIO_GPIOA12                 7

/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C"{
#endif
	
void PWMIO_Open(void);
void PWMIO_Close(void);
void PWMIO_SetClkIn(uint32_t ClkIn);
void PWMIO_SetDuty(uint32_t NumPin, uint8_t Duty);
	
#ifdef __cplusplus
}
#endif	
	
#endif







