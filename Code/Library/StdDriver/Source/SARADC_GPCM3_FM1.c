/**************************************************************************************************
 * Copyright(c) 2022 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   SARADC_GPCM3_FM1.c
 * @Version:
 *   V1.0.0
 * @Date:
 *   April 17, 2024
 * @Abstract:
 *   2024.04.17 Update the SAR_ADC_SetAdcMultiChSampleTime() function.
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
 *   Open SAR-ADC
 * @param
 *   None.
 * @return
 *   None.
 */
void SAR_ADC_Open(void)
{
  SET_BIT(CLOCK->APBCKEN, CLOCK_APBCKEN_SARADC_CLK_ENABLE);	         //	SAR-ADC clock enable
	SAR_ADC->CTRL = 0;
	SAR_ADC->INJ_SEQ = 0;
	SAR_ADC->REG_SEQ = 0;
	SAR_ADC->SMP0 = 0;
	SAR_ADC->SMP1 = 0;
	ACU->APAD_CTRL &= ~ACU_APAD_LINE_PAD_EN_MSK;
	// SAR_ADC->GCTRL = SAR_ADC_GCTRL_ADC_ENABLE | SAR_ADC_GCTRL_ADC_FMT_SEL_UNSIGN | SAR_ADC_GCTRL_DAT_ALIGN_LEFT | SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV32;
		SAR_ADC->GCTRL = SAR_ADC_GCTRL_ADC_ENABLE | SAR_ADC_GCTRL_ADC_FMT_SEL_SIGN | SAR_ADC_GCTRL_DAT_ALIGN_LEFT | SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV32;
}

/**
 * @brief
 *   Close SAR-ADC
 * @param
 *   None.
 * @return
 *   None.
 */
void SAR_ADC_Close(void)
{
	SAR_ADC->GCTRL = 0;
	SAR_ADC->CTRL = 0;
	SAR_ADC->INJ_SEQ = 0;
	SAR_ADC->REG_SEQ = 0;
	SAR_ADC->SMP0 = 0;
	SAR_ADC->SMP1 = 0;
	ACU->APAD_CTRL &= ~ACU_APAD_LINE_PAD_EN_MSK;
  CLEAR_BIT(CLOCK->APBCKEN, CLOCK_APBCKEN_SARADC_CLK_ENABLE);	       //	SAR-ADC clock disable
}

/**
 * @brief
 *   Select data format (sign data/unsign data)
 * @param
 *   DataFmt [in]:
 *    - SAR_ADC_SIGN, SAR_ADC_UNSIGN
 * @return
 *   None.
 */
void SAR_ADC_DataFormatSel(uint32_t DataFmt)
{
  MODIFY_REG(SAR_ADC->GCTRL, SAR_ADC_GCTRL_ADC_FMT_SEL_MSK, DataFmt);
}

/**
 * @brief
 *   Select data alignment
 * @param
 *   DataFmt [in]:
 *    - SAR_ADC_DAT_ALIGN_RIGHT, SAR_ADC_DAT_ALIGN_LEFT
 * @return
 *   None.
 */
void SAR_ADC_DataAlignSel(uint32_t DataAlign)
{
  MODIFY_REG(SAR_ADC->GCTRL, SAR_ADC_GCTRL_DAT_ALIGN_MSK, DataAlign);
}

/**
 * @brief
 *   SAR-ADC clock select.
 * @param
 *   SarAdcClkSel [in]:
 *    - SAR_ADC_CLK_SEL_PCLK_DIV2,  SAR_ADC_CLK_SEL_PCLK_DIV4,  SAR_ADC_CLK_SEL_PCLK_DIV6,  SAR_ADC_CLK_SEL_PCLK_DIV8,  SAR_ADC_CLK_SEL_PCLK_DIV10,
 *      SAR_ADC_CLK_SEL_PCLK_DIV12, SAR_ADC_CLK_SEL_PCLK_DIV14, SAR_ADC_CLK_SEL_PCLK_DIV16, SAR_ADC_CLK_SEL_PCLK_DIV18, SAR_ADC_CLK_SEL_PCLK_DIV20,
 *      SAR_ADC_CLK_SEL_PCLK_DIV22, SAR_ADC_CLK_SEL_PCLK_DIV24, SAR_ADC_CLK_SEL_PCLK_DIV26, SAR_ADC_CLK_SEL_PCLK_DIV28, SAR_ADC_CLK_SEL_PCLK_DIV30,
 *      SAR_ADC_CLK_SEL_PCLK_DIV32, SAR_ADC_CLK_SEL_PCLK_DIV34, SAR_ADC_CLK_SEL_PCLK_DIV36, SAR_ADC_CLK_SEL_PCLK_DIV38, SAR_ADC_CLK_SEL_PCLK_DIV40,
 *      SAR_ADC_CLK_SEL_PCLK_DIV42, SAR_ADC_CLK_SEL_PCLK_DIV44, SAR_ADC_CLK_SEL_PCLK_DIV46, SAR_ADC_CLK_SEL_PCLK_DIV48, SAR_ADC_CLK_SEL_PCLK_DIV50,
 *      SAR_ADC_CLK_SEL_PCLK_DIV52, SAR_ADC_CLK_SEL_PCLK_DIV54, SAR_ADC_CLK_SEL_PCLK_DIV56, SAR_ADC_CLK_SEL_PCLK_DIV58, SAR_ADC_CLK_SEL_PCLK_DIV60,
 * @return
 *   None.
 */
void SAR_ADC_ClkSel(uint32_t SarAdcClkSel)
{
	MODIFY_REG(SAR_ADC->GCTRL, SAR_ADC_GCTRL_ADC_CLK_SEL_MSK, SarAdcClkSel);
}

/**
 * @brief
 *   Set ADC channel sample time
 * @param
 *   AdcChannel [in]:
 *    - 0 ~ 9
 *   AdcChSampleTime [in]:
 *    - SAR_ADC_SMPTIME_1ADCCLK,  SAR_ADC_SMPTIME_2ADCCLK,  SAR_ADC_SMPTIME_4ADCCLK,  SAR_ADC_SMPTIME_8ADCCLK,
 *      SAR_ADC_SMPTIME_16ADCCLK, SAR_ADC_SMPTIME_32ADCCLK, SAR_ADC_SMPTIME_48ADCCLK, SAR_ADC_SMPTIME_64ADCCLK
 * @return
 *   None.
 */
void SAR_ADC_SetAdcChSampleTime(uint32_t AdcChannel, uint32_t AdcChSampleTime)
{
	if(AdcChannel < 8)
	{
		SAR_ADC->SMP0 |= (AdcChSampleTime << (4 * AdcChannel));
	}
	else if((AdcChannel >= 8) && (AdcChannel < 10))
	{
		SAR_ADC->SMP1 |= (AdcChSampleTime << (4 * (AdcChannel - 8)));
	}
}

/**
 * @brief
 *
 * @param
 *  *SetAdcChSampleTime [in]:
 *    - SAR_ADC_SMPTIME_1ADCCLK,  SAR_ADC_SMPTIME_2ADCCLK,  SAR_ADC_SMPTIME_4ADCCLK,  SAR_ADC_SMPTIME_8ADCCLK,
 *      SAR_ADC_SMPTIME_16ADCCLK, SAR_ADC_SMPTIME_32ADCCLK, SAR_ADC_SMPTIME_48ADCCLK, SAR_ADC_SMPTIME_64ADCCLK
 *  *AdcChannel [in]:
 *    - 0 ~ 9
 *   AdcChannelNum [in]:
 *    - 1 ~ 10
 * @return
 *   None.
 */
void SAR_ADC_SetAdcMultiChSampleTime(uint8_t *AdcChSampleTime, uint8_t *AdcChannel,  uint8_t AdcChannelNum)
{
	uint8_t iCount;

	SAR_ADC->SMP0 = 0;
	SAR_ADC->SMP1 = 0;
	for(iCount = 0; iCount < AdcChannelNum; iCount++)
	{
		if(iCount < 8)
		{
			SAR_ADC->SMP0 |= (AdcChSampleTime[iCount] << (4 * AdcChannel[iCount]));
		}
		else if((iCount >= 8) && (iCount < 10))
		{
			SAR_ADC->SMP1 |= (AdcChSampleTime[iCount] << (4 * (AdcChannel[iCount] - 8)));
		}
	}
}

/**
 * @brief
 *   Line-in pad enable
 * @param
 *   PinMask [in]:
 *    - LINE_PAD_IOA24, LINE_PAD_IOA25, LINE_PAD_IOA26, LINE_PAD_IOA27,
 *    - LINE_PAD_IOA28, LINE_PAD_IOA29, LINE_PAD_IOA30, LINE_PAD_IOA31
 * @return
 *   None.
 */
void SAR_ADC_LineInPadEnable(uint32_t PinMask)
{
	MODIFY_REG(ACU->APAD_CTRL, ACU_APAD_LINE_PAD_EN_MSK, PinMask);
}

/**
 * @brief
 *   Line-in pad disable
 * @param
 *   PinMask [in]:
 *    - LINE_PAD_IOA24, LINE_PAD_IOA25, LINE_PAD_IOA26, LINE_PAD_IOA27,
 *    - LINE_PAD_IOA28, LINE_PAD_IOA29, LINE_PAD_IOA30, LINE_PAD_IOA31
 * @return
 *   None.
 */
void SAR_ADC_LineInPadDisable(uint32_t PinMask)
{
	ACU->APAD_CTRL &= ~PinMask;
}

/**
 * @brief
 *   SAR-ADC regular mode initial
 * @param
 *  *SetSequence [in]:
 *   SetChannelNum [in]:
 *    - SAR_ADC_REG_CH_NUM_1, SAR_ADC_REG_CH_NUM_2 ,SAR_ADC_REG_CH_NUM_3, SAR_ADC_REG_CH_NUM_4,
 *    - SAR_ADC_REG_CH_NUM_5, SAR_ADC_REG_CH_NUM_6, SAR_ADC_REG_CH_NUM_7, SAR_ADC_REG_CH_NUM_8
 *   SelGap [in]:
 *    - SAR_ADC_GAP_SEL_TM0, SAR_ADC_GAP_SEL_TM1, SAR_ADC_GAP_SEL_TM2,
 *    - SAR_ADC_GAP_SEL_CCP0_TM, SAR_ADC_GAP_SEL_CCP1_TM,
 *    - SAR_ADC_GAP_SEL_CTS_TM0, SAR_ADC_GAP_SEL_CTS_TM1,
 *    - SAR_ADC_GAP_SEL_0ADCCLK, SAR_ADC_GAP_SEL_1ADCCLK, SAR_ADC_GAP_SEL_2ADCCLK, SAR_ADC_GAP_SEL_3ADCCLK,
 *    - SAR_ADC_GAP_SEL_4ADCCLK, SAR_ADC_GAP_SEL_5ADCCLK, SAR_ADC_GAP_SEL_6ADCCLK, SAR_ADC_GAP_SEL_7ADCCLK,
 *    - SAR_ADC_GAP_SEL_MANUAL,
 *   LoopModeEn [in]:
 *    - SAR_ADC_LOOP_ENABLE, SAR_ADC_LOOP_DISABLE
 * @return
 *   None.
 */
void SAR_ADC_RegMode_Init(uint8_t *RegModeSequence, uint32_t SeqNum, uint32_t SelGap, uint32_t LoopEn)
{
	uint32_t iCount;
	uint32_t ChannelNum = (SeqNum >> SAR_ADC_CTRL_REG_CH_NUM_POS);

	SAR_ADC->CTRL = LoopEn | SAR_ADC_CTRL_REG_ENABLE | SelGap | SeqNum;
	SAR_ADC->REG_SEQ = 0;
	for(iCount = 0; iCount <= ChannelNum; iCount++)
	{
		SAR_ADC->REG_SEQ |= (RegModeSequence[iCount] << (4 * iCount));
	}
}

/**
 * @brief
 *   SAR-ADC regular mode start
 * @param
 *   None.
 * @return
 *   None.
 */
void SAR_ADC_RegMode_Start()
{
	uint32_t SarAdcCtrlTemp;

	SarAdcCtrlTemp = SAR_ADC->CTRL;
	SAR_ADC->GCTRL &= ~SAR_ADC_GCTRL_ADC_ENABLE;
	SAR_ADC->CTRL = (SarAdcCtrlTemp & ~(SAR_ADC_CTRL_SFT_START | SAR_ADC_CTRL_LOOP_ENABLE));
	SAR_ADC->CTRL |= (SarAdcCtrlTemp & ~(SAR_ADC_CTRL_REG_SEQ_GAP_SEL_MSK));
	SAR_ADC->GCTRL |= SAR_ADC_GCTRL_ADC_ENABLE;
	SAR_ADC->CTRL |= (SarAdcCtrlTemp & SAR_ADC_CTRL_LOOP_ENABLE);
	SAR_ADC->CTRL |= (SarAdcCtrlTemp & SAR_ADC_CTRL_REG_SEQ_GAP_SEL_MSK);
	SAR_ADC->CTRL |= SAR_ADC_CTRL_SFT_START;
}

/**
 * @brief
 *   SAR-ADC regular mode stop
 * @param
 *   None.
 * @return
 *   None.
 */
void SAR_ADC_RegMode_Stop()
{
	SAR_ADC->CTRL &= ~SAR_ADC_CTRL_SFT_START;
}

/**
 * @brief
 *   Enable SAR-ADC regulator mode interrupt
 * @param
 *   None.
 * @return
 *   None.
 */
void SAR_ADC_RegMode_EnableInt()
{
  SAR_ADC->GCTRL |= SAR_ADC_GCTRL_REG_MODE_INT_ENABLE;
}

/**
 * @brief
 *   Disable SAR-ADC regulator mode interrupt
 * @param
 *   None.
 * @return
 *   None.
 */
void SAR_ADC_RegMode_DisableInt()
{
  SAR_ADC->GCTRL &= ~SAR_ADC_GCTRL_REG_MODE_INT_ENABLE;
}

/**
 * @brief
 *   SAR-ADC injection mode initial
 * @param
 *   InjChannel [in]:
 *    - 0 ~ 3
 *   AdcChannelSel [in]:
 *    -
 *   TriggerSrc [in]:
 *    -
 * @return
 *   None.
 */
void SAR_ADC_InjMode_Init(uint32_t InjChannel, uint32_t AdcChannelSel, uint32_t TriggerSrc)
{
	MODIFY_REG(SAR_ADC->CTRL, (SAR_ADC_CTRL_INJ0_EN_MSK << (InjChannel << 2)), (SAR_ADC_CTRL_INJ0_ENABLE << (InjChannel << 2)));
	MODIFY_REG(SAR_ADC->CTRL, (SAR_ADC_CTRL_INJ0_TRG_SEL_MSK << (InjChannel << 2)), (TriggerSrc << (InjChannel << 2)));
	MODIFY_REG(SAR_ADC->INJ_SEQ, (SAR_ADC_INJ_SEQ_SEQ0TH_SEL_MSK << (InjChannel << 2)), (AdcChannelSel << (InjChannel << 2)));
}

/**
 * @brief
 *   Enable SAR-ADC injection conversion interrupt
 * @param
 *   InjChannel [in]:
 *    - 0 ~ 3
 * @return
 *   None.
 */
void SAR_ADC_InjMode_EnableInt(uint32_t InjChannel)
{
  SAR_ADC->GCTRL |= (SAR_ADC_GCTRL_INJ0_INT_EN_MSK << InjChannel);
}

/**
 * @brief
 *   Disable SAR-ADC injection conversion interrupt
 * @param
 *   InjChannel [in]:
 *    - 0 ~ 3
 * @return
 *   None.
 */
void SAR_ADC_InjMode_DisableInt(uint32_t InjChannel)
{
  SAR_ADC->GCTRL &= ~(SAR_ADC_GCTRL_INJ0_INT_EN_MSK << InjChannel);
}
