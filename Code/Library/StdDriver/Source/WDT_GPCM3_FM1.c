/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   WDT_GPCM3_FM1.c
 * @Version:
 *   V0.9.1
 * @Date:
 *   October 7, 2020
 * @Abstract:
 *
 **************************************************************************************************/


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "GPCM3_FM1.h"


/*---------------------------------------------------------------------------------------
 * Subroutine Area
 *---------------------------------------------------------------------------------------*/
/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void WDT_Open()
{
  WDG->KEYCODE = WDG_KEYCODE_ACCESS_ENABLE;
  WDG->KEYCODE = WDG_KEYCODE_RESET_ENABLE;
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void WDT_Close()
{
  WDG->KEYCODE = WDG_KEYCODE_ACCESS_ENABLE;
  WDG->KEYCODE = WDG_KEYCODE_WDG_DISABLE;
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void WDT_Clear()
{
  WDG->KEYCODE = WDG_KEYCODE_WDG_CLEAR;
}

/**
 * @brief
 *
 * @param
 *   ClkSel [in]: Watchdog clock selection
 *   - WDG_CLK_32768_DIV_1, WDG_CLK_32768_DIV_4, WDG_CLK_32768_DIV_8, WDG_CLK_32768_DIV_16
 *     WDG_CLK_32768_DIV_32, WDG_CLK_32768_DIV_64, WDG_CLK_32768_DIV_128, WDG_CLK_32768_DIV_256
 * @return
 *   None.
 */
void WDT_SetClk(uint32_t ClkSel)
{
  WDG->KEYCODE = WDG_KEYCODE_ACCESS_ENABLE;
	MODIFY_REG(WDG->CTRL, WDG_CTRL_CLK_SEL_MSK, ClkSel);
	WDG->KEYCODE = WDG_KEYCODE_ACCESS_DISABLE;
}

/**
 * @brief
 *
 * @param
 *   WdtCnt [in]: Watchdog counter reload value (0x000 ~ 0xFFF)
 * @return
 *   None.
 */
void WDT_SetCount(uint32_t WdtCnt)
{
  WDG->KEYCODE = WDG_KEYCODE_ACCESS_ENABLE;
	MODIFY_REG(WDG->CTRL, WDG_CTRL_CNT_MSK, ((WdtCnt << WDG_CTRL_CNT_POS) & WDG_CTRL_CNT_MSK));
	WDG->KEYCODE = WDG_KEYCODE_ACCESS_DISABLE;
}
