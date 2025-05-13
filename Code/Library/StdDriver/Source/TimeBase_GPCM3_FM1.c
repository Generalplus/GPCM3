/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   TimeBase_GPCM3_FM1.c
 * @Version:
 *   V0.9.2
 * @Date:
 *   June 16, 2022
 * @Abstract:
 *   2022.06.16 Add the definitions of the Timebase 0.125 ~ 1Hz
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
 *   TimeBaseClkSrc [in]: TimeBase clock source select
 *    - TB_CLK_SEL_IOSC32K, TB_CLK_SEL_XTAL32K
 * @return
 *   None.
 */
void TIMEBASE_Open(uint32_t SelSys32K)
{
	uint32_t iCount;

	SET_BIT(CLOCK->APBCKEN, CLOCK_APBCKEN_TB_CLK_ENABLE);                                  // TimeBase clock enable
	SET_BIT(TIMEBASE->CTRL, TIMEBASE_CTRL_TB_ENABLE);                                      // Timebase enable
	MODIFY_REG(CLOCK->AHBCKSEL, CLOCK_AHBCKSEL_SYS32K_SEL_MSK, SelSys32K);
  MODIFY_REG(SMU->SYSLOCK, SMU_SYSLOCK_UNLOCK_KEY_MSK, SMU_SYSLOCK_UNLOCK_KEY1);
  MODIFY_REG(SMU->SYSLOCK, SMU_SYSLOCK_UNLOCK_KEY_MSK, SMU_SYSLOCK_UNLOCK_KEY2);	       // SMU nulock

	if(SelSys32K == TB_CLK_SEL_IOSC32K)
	{
    MODIFY_REG(ACU->I32K_CTRL, ACU_I32K_CTRL_I32K_EN_MSK, ACU_I32K_CTRL_I32K_ENABLE);	   // Iosc32K	enabled
    while((CLOCK->CLKSTS & CLOCK_CLKSTS_IOSC32K_RDY) == 0);                              // Wait Iosc32K stable
	  // MODIFY_REG(ACU->APAD_CTRL, ACU_APAD_X32K_PAD_EN_MSK, ACU_APAD_X32K_PAD_DISABLE);    // IOA[8:7] as GPIO
	  // MODIFY_REG(ACU->X32K_CTRL, ACU_X32K_CTRL_X32K_EN_MSK, ACU_X32K_CTRL_X32K_DISABLE);	 // XTAL32K	disabled
	}
	else
	{
    MODIFY_REG(ACU->APAD_CTRL, ACU_APAD_X32K_PAD_EN_MSK, ACU_APAD_X32K_PAD_ENABLE);		   // IOA[8:7] as XTAL32K pins.
    MODIFY_REG(ACU->X32K_CTRL, ACU_X32K_CTRL_X32K_EN_MSK, ACU_X32K_CTRL_X32K_ENABLE);	   // XTAL32K enabled
    MODIFY_REG(ACU->X32K_CTRL, ACU_X32K_CTRL_MODE_SEL_MSK, ACU_X32K_CTRL_MODE_STRONG);	 // XTAL32K strong mode
    while((CLOCK->CLKSTS & CLOCK_CLKSTS_XTAL32K_RDY) == 0);                              // Wait XTAL32K stable
    // MODIFY_REG(ACU->I32K_CTRL, ACU_I32K_CTRL_I32K_EN_MSK, ACU_I32K_CTRL_I32K_DISABLE);	 // Iosc32K	disabled
	}
	MODIFY_REG(SMU->SYSLOCK, SMU_SYSLOCK_UNLOCK_KEY_MSK, 0);	                             // SMU locked
	TIMEBASE_Reset();
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void TIMEBASE_Close(void)
{
  MODIFY_REG(CLOCK->AHBCKSEL, CLOCK_AHBCKSEL_SYS32K_SEL_MSK, CLOCK_AHBCKSEL_SYS32K_SEL_IOSC32K);

	MODIFY_REG(SMU->SYSLOCK, SMU_SYSLOCK_UNLOCK_KEY_MSK, SMU_SYSLOCK_UNLOCK_KEY1);
	MODIFY_REG(SMU->SYSLOCK, SMU_SYSLOCK_UNLOCK_KEY_MSK, SMU_SYSLOCK_UNLOCK_KEY2);	       // SMU nulock
	MODIFY_REG(ACU->X32K_CTRL, ACU_X32K_CTRL_X32K_EN_MSK, ACU_X32K_CTRL_X32K_DISABLE);	   // XTAL32K	disable
	MODIFY_REG(ACU->APAD_CTRL, ACU_APAD_X32K_PAD_EN_MSK, ACU_APAD_X32K_PAD_DISABLE);       // IOA[8:7] as GPIO
  MODIFY_REG(SMU->SYSLOCK, SMU_SYSLOCK_UNLOCK_KEY_MSK, 0);	                             // SMU lock

	TIMEBASE->CTRL = 0;
	CLEAR_BIT(CLOCK->APBCKEN, CLOCK_APBCKEN_TB_CLK_ENABLE);                                // TimeBase clock disable
}

/**
 * @brief
 *   Clear TimeBase count
 * @param
 *   None.
 * @return
 *   None.
 */
void TIMEBASE_Reset(void)
{
	SET_BIT(TIMEBASE->CTRL, TIMEBASE_CTRL_CLRCNT_ENABLE);                                  // Clear TimeBase count
}

/**
 * @brief
 *
 * @param
 *  TimeBaseIntSel (Multiple selection)
 *  - TB_INT_0D125HZ,TB_INT_0D25HZ, TB_INT_0D25HZ, TB_INT_1HZ
 *  - TB_INT_2HZ,    TB_INT_16HZ,   TB_INT_64HZ,
 *    TB_INT_512HZ,  TB_INT_2KHZ,   TB_INT_4KHZ
 * @return
 *   None.
 */
void TIMEBASE_EnableInt(uint32_t TimeBaseIntSel)
{
	TIMEBASE->STS = TimeBaseIntSel;
	TIMEBASE->CTRL |= TimeBaseIntSel;
}

/**
 * @brief
 *
 * @param
 *  TimeBaseIntSel (Multiple selection)
 *  - TB_INT_0D125HZ,TB_INT_0D25HZ, TB_INT_0D25HZ, TB_INT_1HZ
 *  - TB_INT_2HZ,    TB_INT_16HZ,   TB_INT_64HZ,
 *    TB_INT_512HZ,  TB_INT_2KHZ,   TB_INT_4KHZ
 * @return
 *   None.
 */
void TIMEBASE_DisableInt(uint32_t TimeBaseIntSel)
{
  TIMEBASE->CTRL &= ~TimeBaseIntSel;
}
