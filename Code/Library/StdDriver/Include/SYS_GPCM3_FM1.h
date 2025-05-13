/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *
 * @Version:
 *   V0.9.4
 * @Date:
 *   November 30, 2022
 * @Abstract:
 *
 **************************************************************************************************/
#ifndef _SYS_GPCM3_FM1_H_
#define _SYS_GPCM3_FM1_H_


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include <stdint.h>


/*---------------------------------------------------------------------------------------
 * Definition Area
 *---------------------------------------------------------------------------------------*/

/*
 * Definition for the 1st parameter of System_WaitState()
 */
#define ONE_WAITSTATE                                      FLASH_OPTION0_STS_1WAITSTATE
#define TWO_WAITSTATE                                      FLASH_OPTION0_STS_2WAITSTATE
#define THREE_WAITSTATE                                    FLASH_OPTION0_STS_3WAITSTATE
#define FOUR_WAITSTATE                                     FLASH_OPTION0_STS_4WAITSTATE
#define FIVE_WAITSTATE                                     FLASH_OPTION0_STS_5WAITSTATE
#define SIX_WAITSTATE                                      FLASH_OPTION0_STS_6WAITSTATE
#define SEVEN_WAITSTATE                                    FLASH_OPTION0_STS_7WAITSTATE

/*
 * Definition for the 1st parameter of PLL_ChangeClk()
 */
#define PLL_122D88M                                        ACU_PLL_CTRL_SYS_DIVC_122D88M
#define PLL_98D304M                                        ACU_PLL_CTRL_SYS_DIVC_98D304M
#define PLL_81D920M                                        ACU_PLL_CTRL_SYS_DIVC_81D920M
#define PLL_70D217M                                        ACU_PLL_CTRL_SYS_DIVC_70D217M
#define PLL_61D440M                                        ACU_PLL_CTRL_SYS_DIVC_61D440M
#define PLL_54D613M                                        ACU_PLL_CTRL_SYS_DIVC_54D613M
#define PLL_49D152M                                        ACU_PLL_CTRL_SYS_DIVC_49D152M
#define PLL_44D684M                                        ACU_PLL_CTRL_SYS_DIVC_44D684M
#define PLL_40D960M                                        ACU_PLL_CTRL_SYS_DIVC_40D960M
#define PLL_30D720M                                        ACU_PLL_CTRL_SYS_DIVC_30D720M
#define PLL_24D576M                                        ACU_PLL_CTRL_SYS_DIVC_24D576M
#define PLL_20D480M                                        ACU_PLL_CTRL_SYS_DIVC_20D480M
#define PLL_15D360M                                        ACU_PLL_CTRL_SYS_DIVC_15D360M

/*
 * Definition for the 1st parameter of HCLK_SrcSel()
 */
#define HCLK_SEL_SYS32K                                    CLOCK_AHBCKSEL_HCLK_SEL_SYS32K
#define HCLK_SEL_PLL                                       CLOCK_AHBCKSEL_HCLK_SEL_PLL
#define HCLK_SEL_IOSC12M                                   CLOCK_AHBCKSEL_HCLK_SEL_IOSC12M


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C"{
#endif

void Cache_Init(void);
void Cache_Enable(void);
void Cache_Disable(void);
void Cache_Enable(void);
void Cache_Disable(void);
uint32_t System_GetCoreClock(void);
void System_WaitState(uint32_t WaitState);
void PLL_Enable(void);
void PLL_Disable(void);
void PLL_ChangeClk(uint32_t PllClkFreq);
void HCLK_SrcSel(uint32_t HclkSrc);


#ifdef __cplusplus
}
#endif

#endif
