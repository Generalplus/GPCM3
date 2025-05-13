/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   DSADC_GPCM3_FM1.h
 * @Version:
 *   V1.0.1
 * @Date:
 *   May 20, 2021
 * @Abstract:
 *
 **************************************************************************************************/
#ifndef _DSADC_GPCM3_FM1_H_
#define _DSADC_GPCM3_FM1_H_


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include <stdint.h>


/*---------------------------------------------------------------------------------------
 * Definition Area
 *---------------------------------------------------------------------------------------*/
#define CALC_DSADC_CLKIN_D1_DIV(SysCoreClk, SmpRate)                       ((((SysCoreClk) >> 8) % (SmpRate)) >= ((SmpRate) >> 1) ? (((SysCoreClk) >> 8) / (SmpRate)) : ((((SysCoreClk) >> 8) / (SmpRate)) - 1))
#define CALC_DSADC_CLKIN_D2_DIV(SysCoreClk, SmpRate)                       ((((SysCoreClk) >> 9) % (SmpRate)) >= ((SmpRate) >> 1) ? (((SysCoreClk) >> 9) / (SmpRate)) : ((((SysCoreClk) >> 9) / (SmpRate)) - 1))
#define CALC_DSADC_CLKIN_D4_DIV(SysCoreClk, SmpRate)                       ((((SysCoreClk) >> 10) % (SmpRate)) >= ((SmpRate) >> 1) ? (((SysCoreClk) >> 10) / (SmpRate)) : ((((SysCoreClk) >> 10) / (SmpRate)) - 1))
#define CALC_DSADC_SMP_RATE(SysCoreClk, DsAdcClkInDiv1, DsAdcClkInDiv2)    ((((SysCoreClk) >> 8) % ((DsAdcClkInDiv1) * (DsAdcClkInDiv2))) >= (((DsAdcClkInDiv1) * (DsAdcClkInDiv2)) >> 1) ? ((((SysCoreClk) >> 8) / (DsAdcClkInDiv1) / (DsAdcClkInDiv2)) + 1) : (((SysCoreClk) >> 8) / (DsAdcClkInDiv1) / (DsAdcClkInDiv2)))
#define GET_DSADC_CLKIN_DIV1                                               (((CLOCK->CLKDIV1 & CLOCK_CLKDIV1_DSADC_CLK_DIVDER_MSK) >> CLOCK_CLKDIV1_DSADC_CLK_DIVDER_POS) + 1)
#define GET_DSADC_CLKIN_DIV2                                               (((CLOCK->CLKDIV3 & CLOCK_CLKDIV3_DSADC_CLK_DIVDER_MSK) >> CLOCK_CLKDIV3_DSADC_CLK_DIVDER_POS) + 1)

/*
 * Definition for the 1st parameter of DAGC_SetAttackScale()
 */
#define ATTACK_X1                                                          DS_ADC_AGC_CTRL1_ATTACK_TIME_X1
#define ATTACK_X4                                                          DS_ADC_AGC_CTRL1_ATTACK_TIME_X4
#define ATTACK_X16                                                         DS_ADC_AGC_CTRL1_ATTACK_TIME_X16
#define ATTACK_X64                                                         DS_ADC_AGC_CTRL1_ATTACK_TIME_X64

/*
 * Definition for the 1st parameter of DAGC_SetReleaseScale()
 */
#define RELEASE_X1                                                         DS_ADC_AGC_CTRL1_RELEASE_TIME_X1
#define RELEASE_X4                                                         DS_ADC_AGC_CTRL1_RELEASE_TIME_X4
#define RELEASE_X16                                                        DS_ADC_AGC_CTRL1_RELEASE_TIME_X16
#define RELEASE_X64                                                        DS_ADC_AGC_CTRL1_RELEASE_TIME_X64


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C"{
#endif

uint32_t DSADC_Init(uint32_t AdcSmpRate);
void DSADC_Close(void);
uint32_t DSADC_SetSampleRate(uint32_t AdcSmpRate);
uint32_t DSADC_GetSampleRate(void);
void DSADC_SetBoostGain(uint8_t BoostGainLvl);
void DSADC_SetPgaGain(uint8_t PgaGainLvl);

void DSADC_AutoMuteInit(uint16_t SilenceTH);
void DSADC_AutoMuteDisable(void);
void DSADC_AutoMuteDebounce(uint16_t SilenceDebounce, uint16_t NormalDebounce, uint8_t RampCntStep);

void DAGC_Enable(void);
void DAGC_Disable(void);
void DAGC_SetCenterBoundary(uint8_t AgcCenter, uint16_t AgcRange);
void DAGC_SetSensitivity(uint16_t UpdateFreq, uint16_t Attack, uint16_t Release);
void DAGC_SetAttackScale(uint32_t AttackScale);
void DAGC_SetReleaseScale(uint32_t ReleaseScale);
void DAGC_ZeroCross_Enable(void);
void DAGC_ZeroCross_Disable(void);

#ifdef __cplusplus
}
#endif

#endif
