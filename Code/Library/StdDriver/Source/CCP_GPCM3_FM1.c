/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   CCP_GPCM3_FM1.c
 * @Version:
 *   V0.9.0
 * @Date:
 *   May. 04, 2022
 * @Abstract:
 *
 **************************************************************************************************/


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "GPCM3_FM1.h"
#include "CCP_GPCM3_FM1.h"


/*---------------------------------------------------------------------------------------
 * Subroutine Area
 *---------------------------------------------------------------------------------------*/

/**
 * @brief
 *  CCP0 Timer Reload
 * @param
 *    -
 * @return
 *          None.
 */
void CCP0_Timer_Reload(uint16_t TMR_RELOAD)
{
  WRITE_REG(CCP0->TMR_PLOAD, TMR_RELOAD);
}

/**
 * @brief
 *  CCP1 Timer Reload
 * @param
 *    -
 * @return
 *          None.
 */
void CCP1_Timer_Reload(uint16_t TMR_RELOAD)
{
  WRITE_REG(CCP1->TMR_PLOAD, TMR_RELOAD);
}

/**
 * @brief
 *  CCP0 disabled.
 * @param
 *   None.
 * @return
 *   None.
 */
void CCP0_Close(void)
{
	CLEAR_BIT(CLOCK->AHBCKEN, CLOCK_AHBCKEN_CCP0_CLK_ENABLE);	        //	Disable CCP0 Clock
}

/**
 * @brief
 *  CCP1 disabled.
 * @param
 *   None.
 * @return
 *   None.
 */
void CCP1_Close(void)
{
	CLEAR_BIT(CLOCK->AHBCKEN, CLOCK_AHBCKEN_CCP1_CLK_ENABLE);	        //	Disable CCP1 Clock
}


/**
 * @brief
 *  Initialize & open CCP0 PWMIO
 * @param
 *   IOSEL [in]:
 *    - IoSel_IOA3_6, IoSel_IOA13_16
 * @return
 *   None.
 */
void CCP0_PWMIO_Open(uint8_t IOSEL)
{
	SET_BIT(CLOCK->AHBCKEN, CLOCK_AHBCKEN_CCP0_CLK_ENABLE);	        //	Enable CCP0 Clock
  if(IOSEL == IoSel_IOA3_6)
  {
    MODIFY_REG(IOFUNC->CTRL0,GPIOFUNC_CTRL0_CCP0_IOSEL_MSK,GPIOFUNC_CTRL0_CCP0_IOA3_6);
  }
  else
  {
    MODIFY_REG(IOFUNC->CTRL0,GPIOFUNC_CTRL0_CCP0_IOSEL_MSK,GPIOFUNC_CTRL0_CCP0_IOA13_16);
  }
	WRITE_REG(CCP0->TMR_PLOAD, CCP0_TMR_PLOAD);
	WRITE_REG(CCP0->CCPR0, CCP0_PWM0_DUTY);
	WRITE_REG(CCP0->CCPR1, CCP0_PWM1_DUTY);
	WRITE_REG(CCP0->CCPR2, CCP0_PWM2_DUTY);
	WRITE_REG(CCP0->CCPR3, CCP0_PWM3_DUTY);
	WRITE_REG(CCP0->PWM_DTIME, CCP0_PWM_DELAY_TIME);
	WRITE_REG(CCP0->PWM_CTRL, (CCP_PWMCTRL_PWM0_ENABLE | CCP_PWMCTRL_PWM1_ENABLE | CCP_PWMCTRL_PWM2_ENABLE | CCP_PWMCTRL_PWM3_ENABLE));
	WRITE_REG(CCP0->TMCMP_CTRL, (CCP_TMCMPCTRL_TMR_ENABLE | CCP_TMCMPCTRL_TMR_MODE_PWM));	                     //	Enable CCP0 Timer
}

/**
 * @brief
 *  Initialize & open CCP1 PWMIO
 * @param
 *   IOSEL [in]:
 *    - IoSel_IOA9_12, IoSel_IOA26_29
 * @return
 *   None.
 */
void CCP1_PWMIO_Open(uint8_t IOSEL)
{
	SET_BIT(CLOCK->AHBCKEN, CLOCK_AHBCKEN_CCP1_CLK_ENABLE);	        //	Enable CCP1 Clock
  if(IOSEL == IoSel_IOA9_12)
  {
    MODIFY_REG(IOFUNC->CTRL0,GPIOFUNC_CTRL0_CCP1_IOSEL_MSK,GPIOFUNC_CTRL0_CCP1_IOA9_12);
  }
  else
  {
    MODIFY_REG(IOFUNC->CTRL0,GPIOFUNC_CTRL0_CCP1_IOSEL_MSK,GPIOFUNC_CTRL0_CCP1_IOA26_29);
  }
	WRITE_REG(CCP1->TMR_PLOAD, CCP1_TMR_PLOAD);
	WRITE_REG(CCP1->CCPR0, CCP1_PWM0_DUTY);
	WRITE_REG(CCP1->CCPR1, CCP1_PWM1_DUTY);
	WRITE_REG(CCP1->CCPR2, CCP1_PWM2_DUTY);
	WRITE_REG(CCP1->CCPR3, CCP1_PWM3_DUTY);
	WRITE_REG(CCP1->PWM_DTIME, CCP1_PWM_DELAY_TIME);
	WRITE_REG(CCP1->PWM_CTRL, (CCP_PWMCTRL_PWM0_ENABLE | CCP_PWMCTRL_PWM1_ENABLE | CCP_PWMCTRL_PWM2_ENABLE | CCP_PWMCTRL_PWM3_ENABLE));
	WRITE_REG(CCP1->TMCMP_CTRL, (CCP_TMCMPCTRL_TMR_ENABLE | CCP_TMCMPCTRL_TMR_MODE_PWM));	                     //	Enable CCP1 Timer
}

/**
 * @brief
 *   CCP0 PWMIO select
 * @param
  *   PWMx_Enable [in]:
 *    - CCP_PWMCTRL_PWM0_ENABLE, CCP_PWMCTRL_PWM1_ENABLE, CCP_PWMCTRL_PWM2_ENABLE, CCP_PWMCTRL_PWM3_ENABLE
 * @return
 *   None.
 */
void CCP0_PWMIOx_Ctrl(uint32_t PWMx_Enable)
{
	WRITE_REG(CCP0->PWM_CTRL,PWMx_Enable);
}

/**
 * @brief
 *   CCP1 PWMIO select
 * @param
  *   PWMx_Enable [in]:
 *    - CCP_PWMCTRL_PWM0_ENABLE, CCP_PWMCTRL_PWM1_ENABLE, CCP_PWMCTRL_PWM2_ENABLE, CCP_PWMCTRL_PWM3_ENABLE
 * @return
 *   None.
 */
void CCP1_PWMIOx_Ctrl(uint32_t PWMx_Enable)
{
	WRITE_REG(CCP1->PWM_CTRL,PWMx_Enable);
}

/**
 * @brief
 *   Set CCP0 PWMIOx duty
 * @param
 *   PWMx [in]:
 *    - PWM0, PWM1, PWM2, PWM3
 *   Duty [in]:
 *    - Range: 0 ~ 0xFFFF
 * @return
 *   None.
 */
void CCP0_PWMIOx_SetDuty(uint8_t PWMx,uint32_t Duty)
{
  switch(PWMx)
  {
    case PWM0:
      WRITE_REG(CCP0->CCPR0, Duty);
      break;
    case PWM1:
      WRITE_REG(CCP0->CCPR1, Duty);
      break;
    case PWM2:
      WRITE_REG(CCP0->CCPR2, Duty);
      break;
    default:
      WRITE_REG(CCP0->CCPR3, Duty);
      break;
  }
}

/**
 * @brief
 *   Set CCP1 PWMIOx duty
 * @param
 *   PWMx [in]:
 *    - PWM0, PWM1, PWM2, PWM3
 *   Duty [in]:
 *    - Range: 0 ~ 0xFFFF
 * @return
 *   None.
 */
void CCP1_PWMIOx_SetDuty(uint8_t PWMx,uint32_t Duty)
{
  switch(PWMx)
  {
    case PWM0:
      WRITE_REG(CCP1->CCPR0, Duty);
      break;
    case PWM1:
      WRITE_REG(CCP1->CCPR1, Duty);
      break;
    case PWM2:
      WRITE_REG(CCP1->CCPR2, Duty);
      break;
    default:
      WRITE_REG(CCP1->CCPR3, Duty);
      break;
  }
}

/**
 * @brief
 *   Set CCP0 PWMxx Mode(Complement or Independent)
 * @param
 *   PWMxx [in]:
 *    - PWM01, PWM23
 *   Mode [in]:
 *    - PWM01: CCP_PWMCTRL_PWM01_COMPLEMENT, CCP_PWMCTRL_PWM01_INDEPENDENT
 *    - PWM23: CCP_PWMCTRL_PWM23_COMPLEMENT, CCP_PWMCTRL_PWM23_INDEPENDENT
 * @return
 *   None.
 */
void CCP0_PWMxx_ModeSelect(uint8_t PWMxx, uint32_t Mode)
{
  if(PWMxx == PWM01)
  {
    MODIFY_REG(CCP0->PWM_CTRL, CCP_PWMCTRL_PWM01_CHM_MSK, Mode);
  }
  else
  {
    MODIFY_REG(CCP0->PWM_CTRL, CCP_PWMCTRL_PWM23_CHM_MSK, Mode);
  }
}

/**
 * @brief
 *   Set CCP1 PWMxx Mode(Complement or Independent)
 * @param
 *   PWMxx [in]:
 *    - PWM01, PWM23
 *   Mode [in]:
 *    - PWM01: CCP_PWMCTRL_PWM01_COMPLEMENT, CCP_PWMCTRL_PWM01_INDEPENDENT
 *    - PWM23: CCP_PWMCTRL_PWM23_COMPLEMENT, CCP_PWMCTRL_PWM23_INDEPENDENT
 * @return
 *   None.
 */
void CCP1_PWMxx_ModeSelect(uint8_t PWMxx, uint32_t Mode)
{
  if(PWMxx == PWM01)
  {
    MODIFY_REG(CCP1->PWM_CTRL, CCP_PWMCTRL_PWM01_CHM_MSK, Mode);
  }
  else
  {
    MODIFY_REG(CCP1->PWM_CTRL, CCP_PWMCTRL_PWM23_CHM_MSK, Mode);
  }
}

/**
 * @brief
 *   CCP0 PWMx Polarity Inverse
 * @param
 *   PWMx [in]:
 *    - PWM0, PWM1, PWM2, PWM3
 * @return
 *   None.
 */
void CCP0_PWMIOx_Inverse(uint8_t PWMx)
{
  switch(PWMx)
  {
    case PWM0:
      if((READ_REG(CCP0->PWM_CTRL) & CCP_PWMCTRL_PWM0_POL_MSK)== CCP_PWMCTRL_PWM0_POL0)
      {
        MODIFY_REG(CCP0->PWM_CTRL,CCP_PWMCTRL_PWM0_POL_MSK,CCP_PWMCTRL_PWM0_POL1);
      }
      else
      {
        MODIFY_REG(CCP0->PWM_CTRL,CCP_PWMCTRL_PWM0_POL_MSK,CCP_PWMCTRL_PWM0_POL0);
      }
      break;

    case PWM1:
      if((READ_REG(CCP0->PWM_CTRL) & CCP_PWMCTRL_PWM1_POL_MSK)== CCP_PWMCTRL_PWM1_POL0)
      {
        MODIFY_REG(CCP0->PWM_CTRL,CCP_PWMCTRL_PWM1_POL_MSK,CCP_PWMCTRL_PWM1_POL1);
      }
      else
      {
        MODIFY_REG(CCP0->PWM_CTRL,CCP_PWMCTRL_PWM1_POL_MSK,CCP_PWMCTRL_PWM1_POL0);
      }
      break;

    case PWM2:
      if((READ_REG(CCP0->PWM_CTRL) & CCP_PWMCTRL_PWM2_POL_MSK)== CCP_PWMCTRL_PWM2_POL0)
      {
        MODIFY_REG(CCP0->PWM_CTRL,CCP_PWMCTRL_PWM2_POL_MSK,CCP_PWMCTRL_PWM2_POL1);
      }
      else
      {
        MODIFY_REG(CCP0->PWM_CTRL,CCP_PWMCTRL_PWM2_POL_MSK,CCP_PWMCTRL_PWM2_POL0);
      }
      break;

    default:
      if((READ_REG(CCP0->PWM_CTRL) & CCP_PWMCTRL_PWM3_POL_MSK)== CCP_PWMCTRL_PWM3_POL0)
      {
        MODIFY_REG(CCP0->PWM_CTRL,CCP_PWMCTRL_PWM3_POL_MSK,CCP_PWMCTRL_PWM3_POL1);
      }
      else
      {
        MODIFY_REG(CCP0->PWM_CTRL,CCP_PWMCTRL_PWM3_POL_MSK,CCP_PWMCTRL_PWM3_POL0);
      }
      break;
  }
}

/**
 * @brief
 *   CCP1 PWMx Polarity Inverse
 * @param
 *   PWMx [in]:
 *    - PWM0, PWM1, PWM2, PWM3
 * @return
 *   None.
 */
void CCP1_PWMIOx_Inverse(uint8_t PWMx)
{
  switch(PWMx)
  {
    case PWM0:
      if((READ_REG(CCP1->PWM_CTRL) & CCP_PWMCTRL_PWM0_POL_MSK)== CCP_PWMCTRL_PWM0_POL0)
      {
        MODIFY_REG(CCP1->PWM_CTRL,CCP_PWMCTRL_PWM0_POL_MSK,CCP_PWMCTRL_PWM0_POL1);
      }
      else
      {
        MODIFY_REG(CCP1->PWM_CTRL,CCP_PWMCTRL_PWM0_POL_MSK,CCP_PWMCTRL_PWM0_POL0);
      }
      break;

    case PWM1:
      if((READ_REG(CCP1->PWM_CTRL) & CCP_PWMCTRL_PWM1_POL_MSK)== CCP_PWMCTRL_PWM1_POL0)
      {
        MODIFY_REG(CCP1->PWM_CTRL,CCP_PWMCTRL_PWM1_POL_MSK,CCP_PWMCTRL_PWM1_POL1);
      }
      else
      {
        MODIFY_REG(CCP1->PWM_CTRL,CCP_PWMCTRL_PWM1_POL_MSK,CCP_PWMCTRL_PWM1_POL0);
      }
      break;

    case PWM2:
      if((READ_REG(CCP1->PWM_CTRL) & CCP_PWMCTRL_PWM2_POL_MSK)== CCP_PWMCTRL_PWM2_POL0)
      {
        MODIFY_REG(CCP1->PWM_CTRL,CCP_PWMCTRL_PWM2_POL_MSK,CCP_PWMCTRL_PWM2_POL1);
      }
      else
      {
        MODIFY_REG(CCP1->PWM_CTRL,CCP_PWMCTRL_PWM2_POL_MSK,CCP_PWMCTRL_PWM2_POL0);
      }
      break;

    default:
      if((READ_REG(CCP1->PWM_CTRL) & CCP_PWMCTRL_PWM3_POL_MSK)== CCP_PWMCTRL_PWM3_POL0)
      {
        MODIFY_REG(CCP1->PWM_CTRL,CCP_PWMCTRL_PWM3_POL_MSK,CCP_PWMCTRL_PWM3_POL1);
      }
      else
      {
        MODIFY_REG(CCP1->PWM_CTRL,CCP_PWMCTRL_PWM3_POL_MSK,CCP_PWMCTRL_PWM3_POL0);
      }
      break;
  }
}


/**
 * @brief
 *  CCP0 capture mode initial
 * @param
 *   IOSEL [in]:
 *    - IoSel_IOA3_6, IoSel_IOA13_16
 *   TRG_MODE [in]:
 *    - CAP_TRG_RISING, CAP_TRG_FALLING, CAP_TRG_DUAL
 * @return
 *          None.
 */
void CCP0_Capture_Initial(uint8_t IOSEL, uint8_t TRG_MODE)
{
	SET_BIT(CLOCK->AHBCKEN, CLOCK_AHBCKEN_CCP0_CLK_ENABLE);	        //	Enable CCP0 Clock
  if(IOSEL == IoSel_IOA3_6)
  {
    MODIFY_REG(IOFUNC->CTRL0,GPIOFUNC_CTRL0_CCP0_IOSEL_MSK,GPIOFUNC_CTRL0_CCP0_IOA3_6);
    GPIO_SetMode(GPIOA, (BIT0|BIT1|BIT2|BIT3), GPIO_MODE_INPUT);
    MODIFY_REG(GPIOB->OBUF, (BIT0|BIT1|BIT2|BIT3), 0x00000000);
    MODIFY_REG(GPIOB->SMT, (BIT0|BIT1|BIT2|BIT3), 0x0000000F);
  }
  else
  {
    MODIFY_REG(IOFUNC->CTRL0,GPIOFUNC_CTRL0_CCP0_IOSEL_MSK,GPIOFUNC_CTRL0_CCP0_IOA13_16);
    GPIO_SetMode(GPIOA, (BIT13|BIT14|BIT15|BIT16), GPIO_MODE_INPUT);
    GPIO_SetMode(GPIOA, (BIT13|BIT14|BIT15|BIT16), 0x00000000);
    MODIFY_REG(GPIOB->SMT, (BIT13|BIT14|BIT15|BIT16), 0x0001E000);
  }

	WRITE_REG(CCP0->CAP_CTRL, (CCP_CAPCTRL_CAP0_ENABLE | CCP_CAPCTRL_CAP1_ENABLE | CCP_CAPCTRL_CAP2_ENABLE | CCP_CAPCTRL_CAP3_ENABLE | CCP_CAPCTRL_CAP_LPF_SEL_16HCLK));
  switch(TRG_MODE)
  {
    case CAP_TRG_RISING:
      MODIFY_REG(CCP0->CAP_CTRL, 0x0FF0, (CCP_CAPCTRL_CAP0_EDGE_RISE | CCP_CAPCTRL_CAP1_EDGE_RISE | CCP_CAPCTRL_CAP2_EDGE_RISE | CCP_CAPCTRL_CAP3_EDGE_RISE));
      break;

    case CAP_TRG_FALLING:
      MODIFY_REG(CCP0->CAP_CTRL, 0x0FF0, (CCP_CAPCTRL_CAP0_EDGE_FALL | CCP_CAPCTRL_CAP1_EDGE_FALL | CCP_CAPCTRL_CAP2_EDGE_FALL | CCP_CAPCTRL_CAP3_EDGE_FALL));
      break;

    default:
      MODIFY_REG(CCP0->CAP_CTRL, 0x0FF0, (CCP_CAPCTRL_CAP0_EDGE_DUAL | CCP_CAPCTRL_CAP1_EDGE_DUAL | CCP_CAPCTRL_CAP2_EDGE_DUAL | CCP_CAPCTRL_CAP3_EDGE_DUAL));
      break;
  }
  NVIC_EnableIRQ(CCP0_IRQn);
	WRITE_REG(CCP0->INTEN, (CCP_INTEN_CAP0_INT_ENABLE | CCP_INTEN_CAP1_INT_ENABLE | CCP_INTEN_CAP2_INT_ENABLE | CCP_INTEN_CAP3_INT_ENABLE));
	WRITE_REG(CCP0->TMR_PLOAD, CCP0_CAPTMR_PLOAD);
	WRITE_REG(CCP0->TMCMP_CTRL, (CCP_TMCMPCTRL_TMR_ENABLE  | CCP_TMCMPCTRL_DIRECT_WR_ENABLE | CCP_TMCMPCTRL_TMR_MODE_CAPTURE | CCP_TMCMPCTRL_TMR_CLK_HCLK_DIV_4096 | CCP_TMCMPCTRL_TMR_UDC_UP_CNT));
}

/**
 * @brief
 *  CCP1 capture mode initial
 * @param
 *   IOSEL [in]:
 *    - IoSel_IOA9_12, IoSel_IOA26_29
 *   TRG_MODE [in]:
 *    - CAP_TRG_RISING, CAP_TRG_FALLING, CAP_TRG_DUAL
 * @return
 *          None.
 */
void CCP1_Capture_Initial(uint8_t IOSEL, uint8_t TRG_MODE)
{
	SET_BIT(CLOCK->AHBCKEN, CLOCK_AHBCKEN_CCP1_CLK_ENABLE);	        //	Enable CCP1 Clock

  if(IOSEL == IoSel_IOA9_12)
  {
    MODIFY_REG(IOFUNC->CTRL0,GPIOFUNC_CTRL0_CCP1_IOSEL_MSK,GPIOFUNC_CTRL0_CCP1_IOA9_12);
    GPIO_SetMode(GPIOA, (BIT9|BIT10|BIT11|BIT12), GPIO_MODE_INPUT);
    MODIFY_REG(GPIOB->OBUF,(BIT9|BIT10|BIT11|BIT12), 0x00000000);
    MODIFY_REG(GPIOB->SMT, (BIT9|BIT10|BIT11|BIT12), 0x00001E00);
  }
  else
  {
    MODIFY_REG(IOFUNC->CTRL0,GPIOFUNC_CTRL0_CCP1_IOSEL_MSK,GPIOFUNC_CTRL0_CCP1_IOA26_29);
    GPIO_SetMode(GPIOA, (BIT26|BIT27|BIT28|BIT29), GPIO_MODE_INPUT);
    MODIFY_REG(GPIOB->OBUF,(BIT26|BIT27|BIT28|BIT29), 0x00000000);
    MODIFY_REG(GPIOB->SMT, (BIT26|BIT27|BIT28|BIT29), 0x3C000000);
  }

	WRITE_REG(CCP1->CAP_CTRL, (CCP_CAPCTRL_CAP0_ENABLE | CCP_CAPCTRL_CAP1_ENABLE | CCP_CAPCTRL_CAP2_ENABLE | CCP_CAPCTRL_CAP3_ENABLE | CCP_CAPCTRL_CAP_LPF_SEL_16HCLK));
  switch(TRG_MODE)
  {
    case CAP_TRG_RISING:
      MODIFY_REG(CCP1->CAP_CTRL, 0x0FF0, (CCP_CAPCTRL_CAP0_EDGE_RISE | CCP_CAPCTRL_CAP1_EDGE_RISE | CCP_CAPCTRL_CAP2_EDGE_RISE | CCP_CAPCTRL_CAP3_EDGE_RISE));
      break;

    case CAP_TRG_FALLING:
      MODIFY_REG(CCP1->CAP_CTRL, 0x0FF0, (CCP_CAPCTRL_CAP0_EDGE_FALL | CCP_CAPCTRL_CAP1_EDGE_FALL | CCP_CAPCTRL_CAP2_EDGE_FALL | CCP_CAPCTRL_CAP3_EDGE_FALL));
      break;

    default:
      MODIFY_REG(CCP1->CAP_CTRL, 0x0FF0, (CCP_CAPCTRL_CAP0_EDGE_DUAL | CCP_CAPCTRL_CAP1_EDGE_DUAL | CCP_CAPCTRL_CAP2_EDGE_DUAL | CCP_CAPCTRL_CAP3_EDGE_DUAL));
      break;
  }
  NVIC_EnableIRQ(CCP1_IRQn);
	WRITE_REG(CCP1->INTEN, (CCP_INTEN_CAP0_INT_ENABLE | CCP_INTEN_CAP1_INT_ENABLE | CCP_INTEN_CAP2_INT_ENABLE | CCP_INTEN_CAP3_INT_ENABLE));
	WRITE_REG(CCP1->TMR_PLOAD, CCP1_CAPTMR_PLOAD);
	WRITE_REG(CCP1->TMCMP_CTRL, (CCP_TMCMPCTRL_TMR_ENABLE  | CCP_TMCMPCTRL_DIRECT_WR_ENABLE | CCP_TMCMPCTRL_TMR_MODE_CAPTURE | CCP_TMCMPCTRL_TMR_CLK_HCLK_DIV_4096 | CCP_TMCMPCTRL_TMR_UDC_UP_CNT));
}

/**
 * @brief
 *   CCP0 Capture pins enable/ disable
 * @param
  *   PWMx_Enable [in]:
 *    - CCP_CAPCTRL_CAP0_ENABLE, CCP_CAPCTRL_CAP1_ENABLE, CCP_CAPCTRL_CAP2_ENABLE, CCP_CAPCTRL_CAP3_ENABLE
 * @return
 *   None.
 */
void CCP0_CAPx_Ctrl(uint32_t CAP_IOx_Enable)
{
	MODIFY_REG(CCP0->CAP_CTRL,0x0000000F,CAP_IOx_Enable);
}

/**
 * @brief
 *   CCP1 Capture pins enable/ disable
 * @param
  *   PWMx_Enable [in]:
 *    - CCP_CAPCTRL_CAP0_ENABLE, CCP_CAPCTRL_CAP1_ENABLE, CCP_CAPCTRL_CAP2_ENABLE, CCP_CAPCTRL_CAP3_ENABLE
 * @return
 *   None.
 */
void CCP1_CAPx_Ctrl(uint32_t CAP_IOx_Enable)
{
	MODIFY_REG(CCP1->CAP_CTRL,0x0000000F, CAP_IOx_Enable);
}

/**
 * @brief
 *  CCP0 compare mode initial
 * @param
 *          None.
 *
 * @return
 *          None.
 */
void CCP0_Compare_Initial()
{
  NVIC_EnableIRQ(CCP0_IRQn);
	SET_BIT(CLOCK->AHBCKEN, CLOCK_AHBCKEN_CCP0_CLK_ENABLE);	        //	Enable CCP0 Clock
	WRITE_REG(CCP0->TMR_PLOAD, CCP0_CMPTMR_PLOAD);
	WRITE_REG(CCP0->CCPR0, CCP0_CMP0_Data);
	WRITE_REG(CCP0->CCPR1, CCP0_CMP1_Data);
	WRITE_REG(CCP0->CCPR2, CCP0_CMP2_Data);
	WRITE_REG(CCP0->CCPR3, CCP0_CMP3_Data);
	WRITE_REG(CCP0->INTEN, (CCP_INTEN_CMP0_INT_ENABLE | CCP_INTEN_CMP1_INT_ENABLE | CCP_INTEN_CMP2_INT_ENABLE | CCP_INTEN_CMP3_INT_ENABLE));
	WRITE_REG(CCP0->TMCMP_CTRL, (CCP_TMCMPCTRL_TMR_ENABLE  | CCP_TMCMPCTRL_TMR_MODE_COMPARE | CCP_TMCMPCTRL_CMP0_ENABLE |
	                             CCP_TMCMPCTRL_CMP1_ENABLE | CCP_TMCMPCTRL_CMP2_ENABLE | CCP_TMCMPCTRL_CMP3_ENABLE));
}

/**
 * @brief
 *  CCP1 compare mode initial
 * @param
 *          None.
 *
 * @return
 *          None.
 */
void CCP1_Compare_Initial()
{
  NVIC_EnableIRQ(CCP1_IRQn);
	SET_BIT(CLOCK->AHBCKEN, CLOCK_AHBCKEN_CCP1_CLK_ENABLE);	        //	Enable CCP1 Clock
	WRITE_REG(CCP1->TMR_PLOAD, CCP1_CMPTMR_PLOAD);
	WRITE_REG(CCP1->CCPR0, CCP1_CMP0_Data);
	WRITE_REG(CCP1->CCPR1, CCP1_CMP1_Data);
	WRITE_REG(CCP1->CCPR2, CCP1_CMP2_Data);
	WRITE_REG(CCP1->CCPR3, CCP1_CMP3_Data);
	WRITE_REG(CCP1->INTEN, (CCP_INTEN_CMP0_INT_ENABLE | CCP_INTEN_CMP1_INT_ENABLE | CCP_INTEN_CMP2_INT_ENABLE | CCP_INTEN_CMP3_INT_ENABLE));
	WRITE_REG(CCP1->TMCMP_CTRL, (CCP_TMCMPCTRL_TMR_ENABLE  | CCP_TMCMPCTRL_TMR_MODE_COMPARE | CCP_TMCMPCTRL_CMP0_ENABLE |
	                             CCP_TMCMPCTRL_CMP1_ENABLE | CCP_TMCMPCTRL_CMP2_ENABLE | CCP_TMCMPCTRL_CMP3_ENABLE));
}


/**
 * @brief
 *   CCP0 Compare x enable/ disable
 * @param
  *   PWMx_Enable [in]:
 *    - CCP_TMCMPCTRL_CMP0_ENABLE, CCP_TMCMPCTRL_CMP1_ENABLE, CCP_TMCMPCTRL_CMP2_ENABLE, CCP_TMCMPCTRL_CMP3_ENABLE
 * @return
 *   None.
 */
void CCP0_CMPx_Ctrl(uint32_t CMPx_Enable)
{
	MODIFY_REG(CCP0->TMCMP_CTRL,0x0000F000, CMPx_Enable);
}

/**
 * @brief
 *   CCP1 Compare x enable/ disable
 * @param
  *   PWMx_Enable [in]:
 *    - CCP_TMCMPCTRL_CMP0_ENABLE, CCP_TMCMPCTRL_CMP1_ENABLE, CCP_TMCMPCTRL_CMP2_ENABLE, CCP_TMCMPCTRL_CMP3_ENABLE
 * @return
 *   None.
 */
void CCP1_CMPx_Ctrl(uint32_t CMPx_Enable)
{
	MODIFY_REG(CCP1->TMCMP_CTRL,0x0000F000, CMPx_Enable);
}


