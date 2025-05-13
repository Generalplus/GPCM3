/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   SACM_MIXER_Ch0_User.h
 * @Version:
 *   V0.9.1
 * @Date:
 *   Nov. 9rd, 2021
 * @Abstract:
 *
 **************************************************************************************************/
#ifndef _SACM_MIXER_CH0_USER_H_
#define _SACM_MIXER_CH0_USER_H_


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "SACM_MIXER_Ch0_API.h"


/*---------------------------------------------------------------------------------------
 * Definition Area
 *---------------------------------------------------------------------------------------*/
#define MIXER_CH0_FRAME_SIZE                  		(160)
#define MIXER_CH0_SKIP_FRAME											(30)
#define MIXER_CH0_BITS														(MIXER_CH0_MIC_OFF)

/*
 * Mode status definition
 */
#define MIC_EN                                    (0x0001)    // Mic ADC ON

/*
 * Definition for Audio output
 */
#define AUD_OUT_DAC				                  (0)
#define AUD_OUT_PWM				                  (1)


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C"{
#endif

/*
 * MIXER Callback Function Declaration
 */
void MIXER_Ch0_CB_Init(const SACM_MIXER_CH0_WORKING_RAM *MixerCh2WorkingRam);
void MIXER_Ch0_CB_StartPlay(const SACM_MIXER_CH0_WORKING_RAM *MixerCh2WorkingRam);
void MIXER_Ch0_CB_StopPlay(const SACM_MIXER_CH0_WORKING_RAM *MixerCh2WorkingRam);
void MIXER_Ch0_CB_GetAdc_DmaIsr(const SACM_MIXER_CH0_WORKING_RAM *MixerCh2WorkingRam, int8_t *DstBufAddr, uint16_t DataLen);
void MIXER_Ch0_CB_SendDac_DmaIsr(const SACM_MIXER_CH0_WORKING_RAM *MixerCh2WorkingRam, int16_t *SrcDataAddr, uint16_t DataLen);
void MIXER_Ch0_CB_VoiceProcess(const SACM_MIXER_CH0_WORKING_RAM *MixerCh2WorkingRam, int16_t *DstBufAddr, int8_t *SrcBufAddr);
void MIXER_Ch0_CB_Pause(const SACM_MIXER_CH0_WORKING_RAM *MixerCh2WorkingRam);
void MIXER_Ch0_CB_Resume(const SACM_MIXER_CH0_WORKING_RAM *MixerCh2WorkingRam);

#ifdef __cplusplus
}
#endif

#endif
