/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   SYS_GPCM3_FM1.c
 * @Version:
 *   V0.9.4
 * @Date:
 *   July 29, 2022
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
 *  Clear cache
 * @param
 *   None.
 * @return
 *   None.
 */
void Cache_Init()
{
  SMU->CACHE_CTRL &= ~SMU_CACHE_CTRL_CACHE_ENABLE;
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	SMU->CACHE_CTRL |= SMU_CACHE_CTRL_CACHE_ENABLE;
}

 /**
 * @brief
 *  Enable cache
 * @param
 *   None.
 * @return
 *   None.
 */
void Cache_Enable()
{
	SMU->CACHE_CTRL |= SMU_CACHE_CTRL_CACHE_ENABLE;
}

/**
 * @brief
 *   Disable cache
 * @param
 *   None.
 * @return
 *   None.
 */
void Cache_Disable()
{
  SMU->CACHE_CTRL &= ~SMU_CACHE_CTRL_CACHE_ENABLE;
	__NOP();
	__NOP();
	__NOP();
	__NOP();
}

 /**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   System core clock (Hz)
 */
uint32_t System_GetCoreClock(void)
{
    uint32_t l_SysDivCnt;

    if((ACU->PLL_CTRL & ACU_PLL_CTRL_PLL_ENABLE) != 0)
    {
        l_SysDivCnt = (ACU->PLL_CTRL & ACU_PLL_CTRL_SYS_DIVC_MSK) >> ACU_PLL_CTRL_SYS_DIVC_POS;
        return (491520000 / (l_SysDivCnt + 1));
    }
    else
    {
        return 12288000;
    }
}


/**
 * @brief
 *   Enable PLL
 * @param
 *   None.
 * @return
 *   None.
 */
void PLL_Enable()
{
  uint32_t iCount;

	MODIFY_REG(SMU->SYSLOCK, SMU_SYSLOCK_UNLOCK_KEY_MSK, SMU_SYSLOCK_UNLOCK_KEY1);
	MODIFY_REG(SMU->SYSLOCK, SMU_SYSLOCK_UNLOCK_KEY_MSK, SMU_SYSLOCK_UNLOCK_KEY2);	       // SMU nulock
	while((CLOCK->CLKSTS & CLOCK_CLKSTS_IOSC12M_RDY) == 0);                                // Wait IOSC12M stable
	SET_BIT(ACU->PLL_CTRL, ACU_PLL_CTRL_PLL_ENABLE);                                       // PLL enable
    for(iCount = 0; iCount < 20; iCount++);                                                // Wait H/W update PLL Ready Flag
	while((CLOCK->CLKSTS & CLOCK_CLKSTS_PLL_RDY) == 0);                                    // Wait PLL stable
	MODIFY_REG(SMU->SYSLOCK, SMU_SYSLOCK_UNLOCK_KEY_MSK, 0);	                             // SMU lock
}

/**
 * @brief
 *   Disable PLL
 * @param
 *   None.
 * @return
 *   None.
 */
void PLL_Disable()
{
  uint32_t iCount;

	MODIFY_REG(SMU->SYSLOCK, SMU_SYSLOCK_UNLOCK_KEY_MSK, SMU_SYSLOCK_UNLOCK_KEY1);
	MODIFY_REG(SMU->SYSLOCK, SMU_SYSLOCK_UNLOCK_KEY_MSK, SMU_SYSLOCK_UNLOCK_KEY2);	       // SMU nulock
	while((CLOCK->CLKSTS & CLOCK_CLKSTS_IOSC12M_RDY) == 0);                                // Wait IOSC12M stable
	CLEAR_BIT(ACU->PLL_CTRL, ACU_PLL_CTRL_PLL_ENABLE);                                     // PLL disable
    for(iCount = 0; iCount < 20; iCount++);
	MODIFY_REG(SMU->SYSLOCK, SMU_SYSLOCK_UNLOCK_KEY_MSK, 0);	                             // SMU lock
}

/**
 * @brief
 *
 * @param
 *   PllClkFreq [in]:
 *    - PLL_24D576M, PLL_32D768M, PLL_40D960M, PLL_49D192M,
 *      PLL_57D344M, PLL_65D536M, PLL_73D728M, PLL_81D920M
 * @return
 *   None.
 */
void ChangeSysClk(uint32_t SysClkFreq)
{
    MODIFY_REG(SMU->SYSLOCK, SMU_SYSLOCK_UNLOCK_KEY_MSK, 0);	                             // SMU lock
	MODIFY_REG(SMU->SYSLOCK, SMU_SYSLOCK_UNLOCK_KEY_MSK, SMU_SYSLOCK_UNLOCK_KEY1);
	MODIFY_REG(SMU->SYSLOCK, SMU_SYSLOCK_UNLOCK_KEY_MSK, SMU_SYSLOCK_UNLOCK_KEY2);	       // SMU nulock
	MODIFY_REG(ACU->PLL_CTRL, ACU_PLL_CTRL_SYS_DIVC_MSK, SysClkFreq);
	MODIFY_REG(SMU->SYSLOCK, SMU_SYSLOCK_UNLOCK_KEY_MSK, 0);	                             // SMU lock
}


/**
 * @brief
 *
 * @param
 *   HclkSrc [in]:
 *    - HCLK_SEL_SYS32K, HCLK_SEL_PLL, HCLK_SEL_OSC12M
 * @return
 *   None.
 */
void HCLK_SrcSel(uint32_t HclkSrc)
{
	MODIFY_REG(CLOCK->AHBCKSEL, CLOCK_AHBCKSEL_HCLK_SEL_MSK, (HclkSrc & CLOCK_AHBCKSEL_HCLK_SEL_MSK));
}

