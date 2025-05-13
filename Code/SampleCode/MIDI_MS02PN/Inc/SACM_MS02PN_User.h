/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   SACM_MS02_User.h
 * @Version:
 *   V0.9.7
 * @Date:
 *  Aug 14, 2020
 * @Abstract:
 *
 **************************************************************************************************/
#ifndef _SACM_MS02PN_USER_H_
#define _SACM_MS02PN_USER_H_


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "SACM_MS02PN_API.h"


/*---------------------------------------------------------------------------------------
 * Definition Area
 *---------------------------------------------------------------------------------------*/
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

int16_t* GetMS02PNLibStartAddr(uint16_t SpeechIdx);

/*
 * MS02PN Callback Function Declaration
 */
void MS02PN_CB_Init(const SACM_MS02PN_WORKING_RAM *MS02PNWorkingRam, uint8_t AudOutType, uint8_t DacChannelNo, uint32_t Timer_SR);
void MS02PN_CB_StartPlay(const SACM_MS02PN_WORKING_RAM *MS02PNWorkingRam);
uint8_t MS02PN_CB_StopPlay(const SACM_MS02PN_WORKING_RAM *MS02PNWorkingRam);
void MS02PN_CB_Pause(const SACM_MS02PN_WORKING_RAM *MS02PNWorkingRam);
void MS02PN_CB_Resume(const SACM_MS02PN_WORKING_RAM *MS02PNWorkingRam);
void MS02PN_CB_GetData(const SACM_MS02PN_WORKING_RAM *MS02PNWorkingRam, int16_t *DstBufAddr, int16_t *SrcDataAddr, uint16_t DataLen);
void MS02PN_CB_SendDac_DmaIsr(const SACM_MS02PN_WORKING_RAM *MS02PNWorkingRam, int16_t *SrcDataAddr, uint16_t DataLen);
void MS02PN_CB_DecodeProcess(int16_t* DstBufAddr, int16_t OutBufSize);

#ifdef __cplusplus
}
#endif

#endif
