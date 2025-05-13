/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   DSADC_GPCM3_FM1.c
 * @Version:
 *   V1.0.2
 * @Date:
 *   October 24, 2022
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
 *  Initialize Delta-sigma ADC
 * @param
 *   DsAdcSmpRate [in]: Delta-sigma ADC sample rate
 * @return
 *   Real sample rate (Hz)
 */
uint32_t DSADC_Init(uint32_t DsAdcSmpRate)
{
	SET_BIT(CLOCK->APBCKEN, CLOCK_APBCKEN_DSADC_CLK_ENABLE);	         // DS-ADC clock enable

  DSADC_SetSampleRate(DsAdcSmpRate);
	DS_ADC->CTRL = DS_ADC_CTRL_MIC_ENABLE | DS_ADC_CTRL_HPF_ENABLE | DS_ADC_CTRL_DSADC_NORMAL | DS_ADC_CTRL_DSADC_ENABLE | DS_ADC_CTRL_BOOST_GAIN_LV3 | DS_ADC_CTRL_FIFO_ENABLE | DS_ADC_CTRL_FIFO_LVL_0 | DS_ADC_CTRL_INT_DISBLE | DS_ADC_CTRL_PGA_GAIN_LV24 | DS_ADC_CTRL_FMT_SEL_SIGN;

	return DSADC_GetSampleRate();
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void DSADC_Close()
{
  DS_ADC->CTRL  = 0;
	CLEAR_BIT(CLOCK->APBCKEN, CLOCK_APBCKEN_DSADC_CLK_ENABLE);	       // DS-ADC clock disable
}

/**
 * @brief
 *
 * @param
 *   DsAdcSmpRate [in]: Delta-sigma ADC sample rate
 * @return
 *   Real DSADC sample rate (Hz)
 */
uint32_t DSADC_SetSampleRate(uint32_t DsAdcSmpRate)
{
	uint32_t DsAdcClkInDiv;

	if((DsAdcClkInDiv = CALC_DSADC_CLKIN_D1_DIV(System_GetCoreClock(), DsAdcSmpRate)) < 16)
	{
		MODIFY_REG(CLOCK->CLKDIV3, CLOCK_CLKDIV3_DSADC_CLK_DIVDER_MSK, CLOCK_CLKDIV3_DSADC_CLK_DIV_1);
	}
	else if((DsAdcClkInDiv = CALC_DSADC_CLKIN_D2_DIV(System_GetCoreClock(), DsAdcSmpRate)) < 16)
	{
		MODIFY_REG(CLOCK->CLKDIV3, CLOCK_CLKDIV3_DSADC_CLK_DIVDER_MSK, CLOCK_CLKDIV3_DSADC_CLK_DIV_2);
	}
  else if((DsAdcClkInDiv = CALC_DSADC_CLKIN_D4_DIV(System_GetCoreClock(), DsAdcSmpRate)) < 16)
	{
		MODIFY_REG(CLOCK->CLKDIV3, CLOCK_CLKDIV3_DSADC_CLK_DIVDER_MSK, CLOCK_CLKDIV3_DSADC_CLK_DIV_4);
	}

	MODIFY_REG(CLOCK->CLKDIV1, CLOCK_CLKDIV1_DSADC_CLK_DIVDER_MSK, (DsAdcClkInDiv << CLOCK_CLKDIV1_DSADC_CLK_DIVDER_POS));
	MODIFY_REG(CLOCK->CLKDIV1, CLOCK_CLKDIV1_DSADC_CLK_SRC_MSK, CLOCK_CLKDIV1_DSADC_CLK_SRC_SYS);    // GPCM3 only

	return DSADC_GetSampleRate();
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
uint32_t DSADC_GetSampleRate()
{
	return CALC_DSADC_SMP_RATE(System_GetCoreClock(), GET_DSADC_CLKIN_DIV1, GET_DSADC_CLKIN_DIV2);
}

/**
 * @brief
 *
 * @param
 *   BoostGainLvl [in]: Delta-Sigma ADC Boost Amplifier Gain Level
 *    - 0x0(Min.) ~ 0x3(Max.)
 * @return
 *   None.
 */
void DSADC_SetBoostGain(uint8_t BoostGainLvl)
{
  MODIFY_REG(DS_ADC->CTRL, DS_ADC_CTRL_BOOST_GAIN_MSK, (BoostGainLvl << DS_ADC_CTRL_BOOST_GAIN_POS) & DS_ADC_CTRL_BOOST_GAIN_MSK);
}

/**
 * @brief
 *
 * @param
 *   PgaGainLvl [in]: Delta-Sigma ADC PGA Gain Level
 *    - 0x1F(Min.) ~ 0x00(Max.)
 * @return
 *   None.
 */
void DSADC_SetPgaGain(uint8_t PgaGainLvl)
{
  MODIFY_REG(DS_ADC->CTRL, DS_ADC_CTRL_PGA_GAIN_MSK, (PgaGainLvl << DS_ADC_CTRL_PGA_GAIN_POS ) & DS_ADC_CTRL_PGA_GAIN_MSK);
}

/**
 * @brief
 *
 * @param
 *   SilenceTH [in]: Silence threshold
 *    - 0x0000 ~ 0xFFFF
 * @return
 *   None.
 */
void DSADC_AutoMuteInit(uint16_t SilenceTH)
{
	DS_ADC->MUTE_CTRL0 = DS_ADC_MUTE_CTRL0_MIC_AUTOMUTE_ENABLE | DS_ADC_MUTE_CTRL0_FMT_SEL_SIGN | (SilenceTH << DS_ADC_MUTE_CTRL0_SILENCE_TH_POS);
	DS_ADC->MUTE_CTRL1 = 0x00010001;                                   // default value
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void DSADC_AutoMuteDisable()
{
  CLEAR_BIT(DS_ADC->MUTE_CTRL0, DS_ADC_MUTE_CTRL0_MIC_AUTOMUTE_ENABLE);
}

/**
 * @brief
 *
 * @param
 *   SilenceDebounce [in]: Debounce count to enter silence mode
 *   NormalDebounce [in]: Debounce count to enter normal mode
 *   RampCntStep [in]: Ramp counter step
 * @return
 *   None.
 */
void DSADC_AutoMuteDebounce(uint16_t SilenceDebounce, uint16_t NormalDebounce, uint8_t RampCntStep)
{
	DS_ADC->MUTE_CTRL1 = SilenceDebounce | (NormalDebounce << DS_ADC_MUTE_CTRL1_NORMAL_DEBOUNCE_POS);
	MODIFY_REG(DS_ADC->MUTE_CTRL0, DS_ADC_MUTE_CTRL0_RAMPCNT_STEP_MSK, RampCntStep);
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void DAGC_Enable()
{
	SET_BIT(DS_ADC->AGC_CTRL0, DS_ADC_AGC_CTRL0_DAGC_ENABLE);
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void DAGC_Disable()
{
	CLEAR_BIT(DS_ADC->AGC_CTRL0, DS_ADC_AGC_CTRL0_DAGC_ENABLE);
}

/**
 * @brief
 *
 * @param
 *   AgcCenter [in]:
 *   AgcRange [in]:
 * @return
 *   None.
 */
void DAGC_SetCenterBoundary(uint8_t AgcCenter, uint16_t AgcRange)
{
	MODIFY_REG(DS_ADC->AGC_CTRL0, DS_ADC_AGC_CTRL0_THRESHOLD_MSK, (AgcCenter << DS_ADC_AGC_CTRL0_THRESHOLD_POS) & DS_ADC_AGC_CTRL0_THRESHOLD_MSK);
	MODIFY_REG(DS_ADC->AGC_CTRL0, DS_ADC_AGC_CTRL0_TOGGLE_THRESHOLD_MSK, (AgcRange << DS_ADC_AGC_CTRL0_TOGGLE_THRESHOLD_POS) & DS_ADC_AGC_CTRL0_TOGGLE_THRESHOLD_MSK);
}

/**
 * @brief
 *
 * @param
 *   UpdateFreq [in]: PGA gain update frequency
 *    - 0x000 ~ 0xFFF
 *   Attack [in]:
 *    - 0x00 ~ 0xFF
 *   Release [in]:
 *    - 0x00 ~ 0xFF
 * @return
 *   None.
 */
void DAGC_SetSensitivity(uint16_t UpdateFreq, uint16_t Attack, uint16_t Release)
{
	MODIFY_REG(DS_ADC->AGC_CTRL1, DS_ADC_AGC_CTRL1_UPDATE_FREQ_MSK, (UpdateFreq << DS_ADC_AGC_CTRL1_UPDATE_FREQ_POS) & DS_ADC_AGC_CTRL1_UPDATE_FREQ_MSK);
	MODIFY_REG(DS_ADC->AGC_CTRL1, DS_ADC_AGC_CTRL1_ATTACK_TIME_MSK, (Attack << DS_ADC_AGC_CTRL1_ATTACK_TIME_POS) & DS_ADC_AGC_CTRL1_ATTACK_TIME_MSK);
	MODIFY_REG(DS_ADC->AGC_CTRL1, DS_ADC_AGC_CTRL1_RELEASE_TIME_MSK, (Release << DS_ADC_AGC_CTRL1_RELEASE_TIME_POS) & DS_ADC_AGC_CTRL1_RELEASE_TIME_MSK);
}

/**
 * @brief
 *
 * @param
 *   AttackScale [in]: Attack scale control
 *    - ATTACK_X1, ATTACK_X4, ATTACK_X16, ATTACK_X64
 * @return
 *   None.
 */
void DAGC_SetAttackScale(uint32_t AttackScale)
{
  MODIFY_REG(DS_ADC->AGC_CTRL1, DS_ADC_AGC_CTRL1_ATTACK_SCALE_MSK, AttackScale);
}

/**
 * @brief
 *
 * @param
 *   ReleaseScale [in]: Release scale control
 *    - RELEASE_X1, RELEASE_X4, RELEASE_X16, RELEASE_X64
 * @return
 *   None.
 */
void DAGC_SetReleaseScale(uint32_t ReleaseScale)
{
  MODIFY_REG(DS_ADC->AGC_CTRL1, DS_ADC_AGC_CTRL1_RELEASE_SCALE_MSK, ReleaseScale);
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void DAGC_ZeroCross_Enable()
{
  MODIFY_REG(DS_ADC->AGC_CTRL0, DS_ADC_AGC_CTRL0_ZERO_CROSS_MSK, DS_ADC_AGC_CTRL0_ZERO_CROSS_ENABLE);
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void DAGC_ZeroCross_Disable()
{
  MODIFY_REG(DS_ADC->AGC_CTRL0, DS_ADC_AGC_CTRL0_ZERO_CROSS_MSK, DS_ADC_AGC_CTRL0_ZERO_CROSS_DISABLE);
}
