/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   SACM_A3400Pro_USER.h
 * @Version:
 *   V1.0.0
 * @Date:
 *   December 2, 2019
 * @Abstract:
 *
 **************************************************************************************************/
#ifndef _SACM_A3400Pro_USER_H_
#define _SACM_A3400Pro_USER_H_


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "SACM_A3400Pro_API.h"


/*---------------------------------------------------------------------------------------
 * Definition Area
 *---------------------------------------------------------------------------------------*/
#define A3400PRO_FRAME_SIZE									(160)
#define AUDIO_PWM														1
#define CURRENT_DAC													0


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C"{
#endif

uint16_t GetA3400ProNum(void);
int16_t* GetA3400ProStartAddr(uint16_t SpeechIdx);

/*
 * A3400Pro Callback Function Declaration
 */
void A3400Pro_CB_Init(const SACM_A3400PRO_WORKING_RAM *A3400ProWorkingRam);
void A3400Pro_CB_StartPlay(const SACM_A3400PRO_WORKING_RAM *A3400ProWorkingRam);
void A3400Pro_CB_StartPlay_Con(const SACM_A3400PRO_WORKING_RAM *A3400ProWorkingRam);
void A3400Pro_CB_StopPlay(const SACM_A3400PRO_WORKING_RAM *A3400ProWorkingRam);
void A3400Pro_CB_Pause(const SACM_A3400PRO_WORKING_RAM *A3400ProWorkingRam);
void A3400Pro_CB_Resume(const SACM_A3400PRO_WORKING_RAM *A3400ProWorkingRam);
void A3400Pro_CB_GetData(const SACM_A3400PRO_WORKING_RAM *A3400ProWorkingRam, uint8_t *DstBufAddr, uint8_t *SrcDataAddr, uint16_t DataLen);
void A3400Pro_CB_SendDac_DmaIsr(const SACM_A3400PRO_WORKING_RAM *A3400ProWorkingRam, int16_t *SrcDataAddr, uint16_t DataLen);
void A3400Pro_CB_DecodeProcess(const SACM_A3400PRO_WORKING_RAM *A3400ProWorkingRam, int16_t *DstBufAddr, int16_t *SrcBufAddr);

#ifdef __cplusplus
}
#endif

#endif
