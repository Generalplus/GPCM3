/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   Timer_GPCM3_FM1.c
 * @Version:
 *   V1.0.0
 * @Date:
 *   May 20, 2021
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
 *   *TimerHandle [in]: The base address of Timer module
 *    - TM0, TM1, TM2
 * @return
 *   None.
 */
void TIMER_Open(TIMER_TYPE_DEF *TimerHandle)
{
	SET_BIT(CLOCK->APBCKEN, CLOCK_APBCKEN_TIMER_CLK_ENABLE);                     // Timer clock enable
  SET_BIT(TimerHandle->CTRL, TIMER_CTRL_TM_ENABLE);                            // Enable timer
	MODIFY_REG(TimerHandle->CTRL, TIMER_CTRL_CLK_SEL_MSK, TIMER_CTRL_CLK_SEL_HCLK);
}

/**
 * @brief
 *
 * @param
 *   *TimerHandle [in]: The base address of Timer module
 *    - TM0, TM1, TM2
 * @return
 *   None.
 */
void TIMER_Close(TIMER_TYPE_DEF *TimerHandle)
{
  TimerHandle->CTRL = 0;

	if((TM0->CTRL == 0) && (TM1->CTRL == 0) && (TM2->CTRL == 0))
	{
		CLEAR_BIT(CLOCK->APBCKEN, CLOCK_APBCKEN_TIMER_CLK_ENABLE);                 // Timer clock disable
	}
}

/**
 * @brief
 *
 * @param
 *   *TimerHandle [in]: The base address of Timer module
 *    - TM0, TM1, TM2
 *   ClkInDiv [in]:
 *    - TM_CLK_SEL_HCLK, TM_CLK_SEL_HCLK_DIV_2, TM_CLK_SEL_HCLK_DIV_4, TM_CLK_SEL_HCLK_DIV_8
 *      TM_CLK_SEL_HCLK_DIV_16, TM_CLK_SEL_HCLK_DIV_32, TM_CLK_SEL_EXTCLK, TM_CLK_SEL_32K
 * @return
 *   None.
 */
void TIMER_SetClkIn(TIMER_TYPE_DEF *TimerHandle, uint32_t ClkIn)
{
  MODIFY_REG(TimerHandle->CTRL, TIMER_CTRL_CLK_SEL_MSK, ClkIn);
}

/**
 * @brief
 *
 * @param
 *   *TimerHandle [in]: The base address of Timer module
 *    - TM0, TM1, TM2
 *   Frequence [in]: Working frequency
 * @return
 *   - 0  => Set timer frequency pass
 *   - -1 => Set timer frequency fail
 */
int32_t TIMER_SetFreq(TIMER_TYPE_DEF *TimerHandle, uint32_t Frequence)
{
  switch (TimerHandle->CTRL & TIMER_CTRL_CLK_SEL_MSK)
	{
		case TIMER_CTRL_CLK_SEL_HCLK:
		case TIMER_CTRL_CLK_SEL_HCLK_DIV_2:
		case TIMER_CTRL_CLK_SEL_HCLK_DIV_4:
		case TIMER_CTRL_CLK_SEL_HCLK_DIV_8:
		case TIMER_CTRL_CLK_SEL_HCLK_DIV_16:
		case TIMER_CTRL_CLK_SEL_HCLK_DIV_32:
			WRITE_REG(TimerHandle->PLOAD, CALC_TIMER_PLOAD(System_GetCoreClock(), (0x0001 << ((TimerHandle->CTRL & TIMER_CTRL_CLK_SEL_MSK) >> TIMER_CTRL_CLK_SEL_POS)), Frequence));
			break;

		case TIMER_CTRL_CLK_SEL_32K:
			WRITE_REG(TimerHandle->PLOAD, CALC_TIMER_PLOAD(32768, 1, Frequence));
      break;

    case TIMER_CTRL_CLK_SEL_EXTCLK:
    default:
		  return -1;
	}

	WRITE_REG(TimerHandle->COUNT, 0);                                            // Timer reload

	return 0;
}

/**
 * @brief
 *    Enable TMx interrupt
 * @param
 *   *TimerHandle [in]: The base address of Timer module
 *    - TM0, TM1, TM2
 * @return
 *   None.
 */
void TIMER_EnableInt(TIMER_TYPE_DEF *TimerHandle)
{
  if(TimerHandle == TM0)
	{
		TM_INT->INTEN |= TIMER_INTEN_TM0_INT_ENABLE;
	}
	else if(TimerHandle == TM1)
	{
		TM_INT->INTEN |= TIMER_INTEN_TM1_INT_ENABLE;
	}
	else if(TimerHandle == TM2)
	{
		TM_INT->INTEN |= TIMER_INTEN_TM2_INT_ENABLE;
	}
}

/**
 * @brief
 *    Disable TMx interrupt
 * @param
 *   *TimerHandle [in]: The base address of Timer module
 *    - TM0, TM1, TM2
 * @return
 *   None.
 */
void TIMER_DisableInt(TIMER_TYPE_DEF *TimerHandle)
{
  if(TimerHandle == TM0)
	{
		TM_INT->INTEN &= ~TIMER_INTEN_TM0_INT_ENABLE;
	}
	else if(TimerHandle == TM1)
	{
		TM_INT->INTEN &= ~TIMER_INTEN_TM1_INT_ENABLE;
	}
	else if(TimerHandle == TM2)
	{
		TM_INT->INTEN &= ~TIMER_INTEN_TM2_INT_ENABLE;
	}
}

/**
 * @brief
 *
 * @param
 *   *TimerHandle [in]: The base address of Timer module
 *    - TM0, TM1, TM2
 *   TimerIntDiv:
 *    - TM_INT_DIV_1, TM_INT_DIV_2, TM_INT_DIV_3, TM_INT_DIV_4, TM_INT_DIV_5, TM_INT_DIV_6, TM_INT_DIV_7, TM_INT_DIV_8,
 *      TM_INT_DIV_9, TM_INT_DIV_10, TM_INT_DIV_11, TM_INT_DIV_12, TM_INT_DIV_13, TM_INT_DIV_14, TM_INT_DIV_15, TM_INT_DIV_16
 * @return
 *   None.
 */
void TIMER_IntDiv(TIMER_TYPE_DEF *TimerHandle, uint32_t TimerIntDiv)
{
  MODIFY_REG(TimerHandle->CTRL, TIMER_CTRL_TMINT_DIV_MSK, TimerIntDiv);
}

/**
 * @brief
 *
 * @param
 *   *TimerHandle [in]: The base address of Timer module
 *    - TM0, TM1, TM2
 * @return
 *   None.
 */
void TIMER_Reload(TIMER_TYPE_DEF *TimerHandle)
{
	WRITE_REG(TimerHandle->COUNT, 0);                                            // Timer reload
}
