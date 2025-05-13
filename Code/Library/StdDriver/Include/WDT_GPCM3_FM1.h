/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   WDT_GPCM3_FM1.h
 * @Version:
 *   V0.9.1
 * @Date:
 *   October 7, 2020
 * @Abstract:
 *
 **************************************************************************************************/
#ifndef _WDT_GPCM3_FM1_H_
#define _WDT_GPCM3_FM1_H_


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include <stdint.h>


/*---------------------------------------------------------------------------------------
 * Definition Area
 *---------------------------------------------------------------------------------------*/

/*
 * Definition for the 1st parameter of WDT_SetClk()
 */
#define WDG_CLK_32768_DIV_1               WDG_CTRL_CLK_SEL_32768_DIV_1
#define WDG_CLK_32768_DIV_4               WDG_CTRL_CLK_SEL_32768_DIV_4
#define WDG_CLK_32768_DIV_8               WDG_CTRL_CLK_SEL_32768_DIV_8
#define WDG_CLK_32768_DIV_16              WDG_CTRL_CLK_SEL_32768_DIV_16
#define WDG_CLK_32768_DIV_32              WDG_CTRL_CLK_SEL_32768_DIV_32
#define WDG_CLK_32768_DIV_64              WDG_CTRL_CLK_SEL_32768_DIV_64
#define WDG_CLK_32768_DIV_128             WDG_CTRL_CLK_SEL_32768_DIV_128
#define WDG_CLK_32768_DIV_256             WDG_CTRL_CLK_SEL_32768_DIV_256


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C"{
#endif

void WDT_Open(void);
void WDT_Close(void);
void WDT_Clear(void);
void WDT_SetClk(uint32_t ClkSel);
void WDT_SetCount(uint32_t WdtCnt);


#ifdef __cplusplus
}
#endif

#endif
