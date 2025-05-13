/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   TimeBase_GPCM3_FM1.h
 * @Version:
 *   V0.9.1
 * @Date:
 *   July 13, 2022
 * @Abstract:
 *
 *
 **************************************************************************************************/
#ifndef _TIMEBASE_GPCM3_FM1_H_
#define _TIMEBASE_GPCM3_FM1_H_


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "GPCM3_FM1.h"


/*---------------------------------------------------------------------------------------
 * Definition Area
 *---------------------------------------------------------------------------------------*/

/*
 * Definition for the 1st parameter of TIMEBASE_Open().
 */
#define TB_CLK_SEL_IOSC32K             CLOCK_AHBCKSEL_SYS32K_SEL_IOSC32K
#define TB_CLK_SEL_XTAL32K             CLOCK_AHBCKSEL_SYS32K_SEL_XTAL32K

/*
 * Definition for the 1st parameter of TIMEBASE_EnableInt(), TIMEBASE_DisableInt().
 */
#define TB_INT_0D125HZ                 TIMEBASE_CTRL_0D125HZ_ENABLE
#define TB_INT_0D25HZ                  TIMEBASE_CTRL_0D25HZ_ENABLE
#define TB_INT_0D5HZ                   TIMEBASE_CTRL_0D5HZ_ENABLE
#define TB_INT_1HZ                     TIMEBASE_CTRL_1HZ_ENABLE
#define TB_INT_2HZ                     TIMEBASE_CTRL_2HZ_ENABLE
#define TB_INT_16HZ                    TIMEBASE_CTRL_16HZ_ENABLE
#define TB_INT_64HZ                    TIMEBASE_CTRL_64HZ_ENABLE
#define TB_INT_512HZ                   TIMEBASE_CTRL_512HZ_ENABLE
#define TB_INT_2KHZ                    TIMEBASE_CTRL_2KHZ_ENABLE
#define TB_INT_4KHZ                    TIMEBASE_CTRL_4KHZ_ENABLE


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C"{
#endif

void TIMEBASE_Open(uint32_t SelSys32K);
void TIMEBASE_Close(void);
void TIMEBASE_Reset(void);
void TIMEBASE_EnableInt(uint32_t TimeBaseIntSel);
void TIMEBASE_DisableInt(uint32_t TimeBaseIntSel);


#ifdef __cplusplus
}
#endif

#endif
