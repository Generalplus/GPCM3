/**************************************************************************************************
 * Copyright(c) 2022 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   SARADC_GPCM3_FM1.h
 * @Version:
 *   V1.0.0
 * @Date:
 *   April 17, 2024
 * @Abstract:
 *   2024.04.17 Update the SAR_ADC_SetAdcMultiChSampleTime() function.
 *
 **************************************************************************************************/
#ifndef _SARADR_GPCM3_FM1_H_
#define _SARADR_GPCM3_FM1_H_


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include <stdint.h>


/*---------------------------------------------------------------------------------------
 * Definition Area
 *---------------------------------------------------------------------------------------*/

/*
 * Definition for the 1st parameter of SAR_ADC_DataAlignSel()
 */
#define SAR_ADC_SIGN                        SAR_ADC_GCTRL_ADC_FMT_SEL_SIGN
#define SAR_ADC_UNSIGN                      SAR_ADC_GCTRL_ADC_FMT_SEL_UNSIGN

/*
 * Definition for the 1st parameter of SAR_ADC_DataFormatSel()
 */
#define SAR_ADC_DAT_ALIGN_RIGHT             SAR_ADC_GCTRL_DAT_ALIGN_RIGHT
#define SAR_ADC_DAT_ALIGN_LEFT              SAR_ADC_GCTRL_DAT_ALIGN_LEFT

/*
 * Definition for the 1st parameter of SAR_ADC_ClkSel()
 */
#define SAR_ADC_CLK_SEL_PCLK_DIV2           SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV2
#define SAR_ADC_CLK_SEL_PCLK_DIV4           SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV4
#define SAR_ADC_CLK_SEL_PCLK_DIV6           SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV6
#define SAR_ADC_CLK_SEL_PCLK_DIV8           SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV8
#define SAR_ADC_CLK_SEL_PCLK_DIV10          SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV10
#define SAR_ADC_CLK_SEL_PCLK_DIV12          SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV12
#define SAR_ADC_CLK_SEL_PCLK_DIV14          SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV14
#define SAR_ADC_CLK_SEL_PCLK_DIV16          SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV16
#define SAR_ADC_CLK_SEL_PCLK_DIV18          SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV18
#define SAR_ADC_CLK_SEL_PCLK_DIV20          SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV20
#define SAR_ADC_CLK_SEL_PCLK_DIV22          SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV22
#define SAR_ADC_CLK_SEL_PCLK_DIV24          SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV24
#define SAR_ADC_CLK_SEL_PCLK_DIV26          SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV26
#define SAR_ADC_CLK_SEL_PCLK_DIV28          SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV28
#define SAR_ADC_CLK_SEL_PCLK_DIV30          SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV30
#define SAR_ADC_CLK_SEL_PCLK_DIV32          SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV32
#define SAR_ADC_CLK_SEL_PCLK_DIV34          SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV34
#define SAR_ADC_CLK_SEL_PCLK_DIV36          SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV36
#define SAR_ADC_CLK_SEL_PCLK_DIV38          SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV38
#define SAR_ADC_CLK_SEL_PCLK_DIV40          SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV40
#define SAR_ADC_CLK_SEL_PCLK_DIV42          SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV42
#define SAR_ADC_CLK_SEL_PCLK_DIV44          SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV44
#define SAR_ADC_CLK_SEL_PCLK_DIV46          SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV46
#define SAR_ADC_CLK_SEL_PCLK_DIV48          SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV48
#define SAR_ADC_CLK_SEL_PCLK_DIV50          SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV50
#define SAR_ADC_CLK_SEL_PCLK_DIV52          SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV52
#define SAR_ADC_CLK_SEL_PCLK_DIV54          SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV54
#define SAR_ADC_CLK_SEL_PCLK_DIV56          SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV56
#define SAR_ADC_CLK_SEL_PCLK_DIV58          SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV58
#define SAR_ADC_CLK_SEL_PCLK_DIV60          SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV60
#define SAR_ADC_CLK_SEL_PCLK_DIV62          SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV62
#define SAR_ADC_CLK_SEL_PCLK_DIV64          SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV64

/*
 * Definition for the 1st parameter of SAR_ADC_SetAdcChSampleTime()
 */
#define SAR_ADC_SMPTIME_1ADCCLK             SAR_ADC_SMP0_CH0_SMP_SEL_1ADCCLK
#define SAR_ADC_SMPTIME_2ADCCLK             SAR_ADC_SMP0_CH0_SMP_SEL_2ADCCLK
#define SAR_ADC_SMPTIME_4ADCCLK             SAR_ADC_SMP0_CH0_SMP_SEL_4ADCCLK
#define SAR_ADC_SMPTIME_8ADCCLK             SAR_ADC_SMP0_CH0_SMP_SEL_8ADCCLK
#define SAR_ADC_SMPTIME_16ADCCLK            SAR_ADC_SMP0_CH0_SMP_SEL_16ADCCLK
#define SAR_ADC_SMPTIME_32ADCCLK            SAR_ADC_SMP0_CH0_SMP_SEL_32ADCCLK
#define SAR_ADC_SMPTIME_48ADCCLK            SAR_ADC_SMP0_CH0_SMP_SEL_48ADCCLK
#define SAR_ADC_SMPTIME_64ADCCLK            SAR_ADC_SMP0_CH0_SMP_SEL_64ADCCLK

/*
 * Definition for the 1st parameter of SAR_ADC_LineInPadEn()
 */
#define LINE_PAD_IOA24                      ACU_APAD_LINE_PAD_IOA24
#define LINE_PAD_IOA25                      ACU_APAD_LINE_PAD_IOA25
#define LINE_PAD_IOA26                      ACU_APAD_LINE_PAD_IOA26
#define LINE_PAD_IOA27                      ACU_APAD_LINE_PAD_IOA27
#define LINE_PAD_IOA28                      ACU_APAD_LINE_PAD_IOA28
#define LINE_PAD_IOA29                      ACU_APAD_LINE_PAD_IOA29
#define LINE_PAD_IOA30                      ACU_APAD_LINE_PAD_IOA30
#define LINE_PAD_IOA31                      ACU_APAD_LINE_PAD_IOA31

/*
 * Definition for the 2nd parameter of SAR_ADC_RegMode_Init()
 */
#define SAR_ADC_REG_CH_NUM_1                SAR_ADC_CTRL_REG_CH0
#define SAR_ADC_REG_CH_NUM_2                SAR_ADC_CTRL_REG_CH1
#define SAR_ADC_REG_CH_NUM_3                SAR_ADC_CTRL_REG_CH2
#define SAR_ADC_REG_CH_NUM_4                SAR_ADC_CTRL_REG_CH3
#define SAR_ADC_REG_CH_NUM_5                SAR_ADC_CTRL_REG_CH4
#define SAR_ADC_REG_CH_NUM_6                SAR_ADC_CTRL_REG_CH5
#define SAR_ADC_REG_CH_NUM_7                SAR_ADC_CTRL_REG_CH6
#define SAR_ADC_REG_CH_NUM_8                SAR_ADC_CTRL_REG_CH7
#define SAR_ADC_REG_CH_NUM_9                SAR_ADC_CTRL_REG_VDD12
#define SAR_ADC_REG_CH_NUM_10               SAR_ADC_CTRL_REG_VDDIOD4

/*
 * Definition for the 3rd parameter of SAR_ADC_RegMode_Init()
 */
#define SAR_ADC_GAP_SEL_TM0                 SAR_ADC_CTRL_REG_SEQ_GAP_SEL_TM0
#define SAR_ADC_GAP_SEL_TM1                 SAR_ADC_CTRL_REG_SEQ_GAP_SEL_TM1
#define SAR_ADC_GAP_SEL_TM2                 SAR_ADC_CTRL_REG_SEQ_GAP_SEL_TM2
#define SAR_ADC_GAP_SEL_CCP0_TM             SAR_ADC_CTRL_REG_SEQ_GAP_SEL_CCP0_TM
#define SAR_ADC_GAP_SEL_CCP1_TM             SAR_ADC_CTRL_REG_SEQ_GAP_SEL_CCP1_TM
#define SAR_ADC_GAP_SEL_CTS_TM0             SAR_ADC_CTRL_REG_SEQ_GAP_SEL_CTS_TM0
#define SAR_ADC_GAP_SEL_CTS_TM1             SAR_ADC_CTRL_REG_SEQ_GAP_SEL_CTS_TM1
#define SAR_ADC_GAP_SEL_MANUAL              SAR_ADC_CTRL_REG_SEQ_GAP_SEL_MANUAL
#define SAR_ADC_GAP_SEL_0ADCCLK             SAR_ADC_CTRL_REG_SEQ_GAP_SEL_0ADCCLK
#define SAR_ADC_GAP_SEL_1ADCCLK             SAR_ADC_CTRL_REG_SEQ_GAP_SEL_1ADCCLK
#define SAR_ADC_GAP_SEL_2ADCCLK             SAR_ADC_CTRL_REG_SEQ_GAP_SEL_2ADCCLK
#define SAR_ADC_GAP_SEL_3ADCCLK             SAR_ADC_CTRL_REG_SEQ_GAP_SEL_3ADCCLK
#define SAR_ADC_GAP_SEL_4ADCCLK             SAR_ADC_CTRL_REG_SEQ_GAP_SEL_4ADCCLK
#define SAR_ADC_GAP_SEL_5ADCCLK             SAR_ADC_CTRL_REG_SEQ_GAP_SEL_5ADCCLK
#define SAR_ADC_GAP_SEL_6ADCCLK             SAR_ADC_CTRL_REG_SEQ_GAP_SEL_6ADCCLK
#define SAR_ADC_GAP_SEL_7ADCCLK             SAR_ADC_CTRL_REG_SEQ_GAP_SEL_7ADCCLK

/*
 * Definition for the 4th parameter of SAR_ADC_RegMode_Init()
 */
#define SAR_ADC_LOOP_ENABLE                 SAR_ADC_CTRL_LOOP_ENABLE
#define SAR_ADC_LOOP_DISABLE                SAR_ADC_CTRL_LOOP_DISABLE

/*
 * Definition for the 3rd parameter of SAR_ADC_InjMode_Init()
 */
#define SAR_ADC_INJ_TRG_SEL_TM0             SAR_ADC_CTRL_INJ0_TRG_SEL_TM0
#define SAR_ADC_INJ_TRG_SEL_TM1             SAR_ADC_CTRL_INJ0_TRG_SEL_TM1
#define SAR_ADC_INJ_TRG_SEL_TM2             SAR_ADC_CTRL_INJ0_TRG_SEL_TM2
#define SAR_ADC_INJ_TRG_SEL_CCP0_TM         SAR_ADC_CTRL_INJ0_TRG_SEL_CCP0_TM
#define SAR_ADC_INJ_TRG_SEL_CCP1_TM         SAR_ADC_CTRL_INJ0_TRG_SEL_CCP1_TM
#define SAR_ADC_INJ_TRG_SEL_CTS_TM0         SAR_ADC_CTRL_INJ0_TRG_SEL_CTS_TM0
#define SAR_ADC_INJ_TRG_SEL_CTS_TM1         SAR_ADC_CTRL_INJ0_TRG_SEL_CTS_TM1


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C"{
#endif

void SAR_ADC_Open(void);
void SAR_ADC_Close(void);
void SAR_ADC_DataFormatSel(uint32_t DataFmt);
void SAR_ADC_DataAlignSel(uint32_t DataAlign);
void SAR_ADC_ClkSel(uint32_t SarAdcClkSel);
void SAR_ADC_SetAdcChSampleTime(uint32_t AdcChannel, uint32_t AdcChSampleTime);
//void SAR_ADC_SetAdcMultiChSampleTime(uint8_t *AdcChSampleTime, uint32_t AdcChannelNum);
void SAR_ADC_SetAdcMultiChSampleTime(uint8_t *AdcChSampleTime, uint8_t *AdcChannel,  uint8_t AdcChannelNum);
void SAR_ADC_LineInPadEnable(uint32_t PinMask);
void SAR_ADC_LineInPadDisable(uint32_t PinMask);

void SAR_ADC_RegMode_Init(uint8_t *RegModeSequence, uint32_t SeqNum, uint32_t SelGap, uint32_t LoopEn);
void SAR_ADC_RegMode_Start(void);
void SAR_ADC_RegMode_Stop(void);
void SAR_ADC_RegMode_EnableInt(void);
void SAR_ADC_RegMode_DisableInt(void);

void SAR_ADC_InjMode_Init(uint32_t InjChannel, uint32_t AdcChannelSel, uint32_t TriggerSrc);
void SAR_ADC_InjMode_EnableInt(uint32_t InjChannel);
void SAR_ADC_InjMode_DisableInt(uint32_t InjChannel);


#ifdef __cplusplus
}
#endif

#endif
