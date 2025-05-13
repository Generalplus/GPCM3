/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   SACM_MIXER_Ch1_User.h
 * @Version:
 *   V0.9.0
 * @Date:
 *   Sep. 3rd, 2021
 * @Abstract:
 *
 **************************************************************************************************/
#ifndef _SACM_MIXER_CH1_USER_H_
#define _SACM_MIXER_CH1_USER_H_


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "SACM_MIXER_Ch1_API.h"


/*---------------------------------------------------------------------------------------
 * Definition Area
 *---------------------------------------------------------------------------------------*/
#define MIXER_CH1_FRAME_SIZE                  		(320)
#define MIXER_CH1_SKIP_FRAME											(30)
#define MIXER_CH1_BITS														(MIXER_CH1_MIC_OFF)

/*
 * Mode status definition
 */
#define MIC_EN                                    (0x0001)    // Mic ADC ON


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C"{
#endif

/*
 * MIXER Callback Function Declaration
 */
void MIXER_Ch1_CB_Init(const SACM_MIXER_CH1_WORKING_RAM *MixerCh2WorkingRam);
void MIXER_Ch1_CB_StartPlay(const SACM_MIXER_CH1_WORKING_RAM *MixerCh2WorkingRam);
void MIXER_Ch1_CB_StopPlay(const SACM_MIXER_CH1_WORKING_RAM *MixerCh2WorkingRam);
void MIXER_Ch1_CB_GetAdc_DmaIsr(const SACM_MIXER_CH1_WORKING_RAM *MixerCh2WorkingRam, int8_t *DstBufAddr, uint16_t DataLen);
void MIXER_Ch1_CB_SendDac_DmaIsr(const SACM_MIXER_CH1_WORKING_RAM *MixerCh2WorkingRam, int16_t *SrcDataAddr, uint16_t DataLen);
void MIXER_Ch1_CB_VoiceProcess(const SACM_MIXER_CH1_WORKING_RAM *MixerCh2WorkingRam, int16_t *DstBufAddr, int8_t *SrcBufAddr);
void MIXER_Ch1_CB_Pause(const SACM_MIXER_CH1_WORKING_RAM *MixerCh2WorkingRam);
void MIXER_Ch1_CB_Resume(const SACM_MIXER_CH1_WORKING_RAM *MixerCh2WorkingRam);

#ifdef __cplusplus
}
#endif

#endif
