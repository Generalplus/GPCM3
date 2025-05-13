/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   Timer_GPCM3_FM1.h
 * @Version:
 *   V1.0.0
 * @Date:
 *   May 20, 2021
 * @Abstract:
 *
 **************************************************************************************************/
#ifndef _TIMER_GPCM3_FM1_H_
#define _TIMER_GPCM3_FM1_H_


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "GPCM3_FM1.h"


/*---------------------------------------------------------------------------------------
 * Definition Area
 *---------------------------------------------------------------------------------------*/
#define CALC_TIMER_PLOAD(TimerClk, TimerClkDiv, Freq)      (((TimerClk) / (TimerClkDiv) / (Freq)) - 1)

/*
 * Definition for the 2nd parameter of TIMER_SetClkInDiv()
 */
#define TM_CLK_SEL_HCLK                                    TIMER_CTRL_CLK_SEL_HCLK
#define TM_CLK_SEL_HCLK_DIV_2                              TIMER_CTRL_CLK_SEL_HCLK_DIV_2
#define TM_CLK_SEL_HCLK_DIV_4                              TIMER_CTRL_CLK_SEL_HCLK_DIV_4
#define TM_CLK_SEL_HCLK_DIV_8                              TIMER_CTRL_CLK_SEL_HCLK_DIV_8
#define TM_CLK_SEL_HCLK_DIV_16                             TIMER_CTRL_CLK_SEL_HCLK_DIV_16
#define TM_CLK_SEL_HCLK_DIV_32                             TIMER_CTRL_CLK_SEL_HCLK_DIV_32
#define TM_CLK_SEL_EXTCLK                                  TIMER_CTRL_CLK_SEL_EXTCLK
#define TM_CLK_SEL_32K                                     TIMER_CTRL_CLK_SEL_32K

/*
 * Definition for the 2nd parameter of TIMER_IntDiv()
 */
#define TM_INT_DIV_1                                       TIMER_CTRL_TMINT_DIV_1
#define TM_INT_DIV_2                                       TIMER_CTRL_TMINT_DIV_2
#define TM_INT_DIV_3                                       TIMER_CTRL_TMINT_DIV_3
#define TM_INT_DIV_4                                       TIMER_CTRL_TMINT_DIV_4
#define TM_INT_DIV_5                                       TIMER_CTRL_TMINT_DIV_5
#define TM_INT_DIV_6                                       TIMER_CTRL_TMINT_DIV_6
#define TM_INT_DIV_7                                       TIMER_CTRL_TMINT_DIV_7
#define TM_INT_DIV_8                                       TIMER_CTRL_TMINT_DIV_8
#define TM_INT_DIV_9                                       TIMER_CTRL_TMINT_DIV_9
#define TM_INT_DIV_10                                      TIMER_CTRL_TMINT_DIV_10
#define TM_INT_DIV_11                                      TIMER_CTRL_TMINT_DIV_11
#define TM_INT_DIV_12                                      TIMER_CTRL_TMINT_DIV_12
#define TM_INT_DIV_13                                      TIMER_CTRL_TMINT_DIV_13
#define TM_INT_DIV_14                                      TIMER_CTRL_TMINT_DIV_14
#define TM_INT_DIV_15                                      TIMER_CTRL_TMINT_DIV_15
#define TM_INT_DIV_16                                      TIMER_CTRL_TMINT_DIV_16


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C"{
#endif

void TIMER_Open(TIMER_TYPE_DEF *TimerHandle);
void TIMER_Close(TIMER_TYPE_DEF *TimerHandle);
void TIMER_SetClkIn(TIMER_TYPE_DEF *TimerHandle, uint32_t ClkIn);
int32_t TIMER_SetFreq(TIMER_TYPE_DEF *TimerHandle, uint32_t Frequence);
void TIMER_EnableInt(TIMER_TYPE_DEF *TimerHandle);
void TIMER_DisableInt(TIMER_TYPE_DEF *TimerHandle);
void TIMER_IntDiv(TIMER_TYPE_DEF *TimerHandle, uint32_t TimerIntDiv);
void TIMER_Reload(TIMER_TYPE_DEF *TimerHandle);


#ifdef __cplusplus
}
#endif

#endif
