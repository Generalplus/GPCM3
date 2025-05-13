/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   SACM_A2000_User.h
 * @Version:
 *   V1.0.0
 * @Date:
 *   June 12, 2024
 * @Abstract:
 *
 **************************************************************************************************/
#ifndef _SACM_A2000_USER_H_
#define _SACM_A2000_USER_H_


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "SACM_A2000_API.h"


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

uint16_t A2000_User_GetA2000Num(void);
int16_t* A2000_User_GetA2000StartAddr(uint16_t SpeechIdx);

/*
 * A2000 Callback Function Declaration
 */
void A2000_CB_Init(const SACM_A2000_WORKING_RAM *A2000WorkingRam);
void A2000_CB_StartPlay(const SACM_A2000_WORKING_RAM *A2000WorkingRam);
void A2000_CB_Play_Con(const SACM_A2000_WORKING_RAM *A2000WorkingRam);
void A2000_CB_StopPlay(const SACM_A2000_WORKING_RAM *A2000WorkingRam);
void A2000_CB_Pause(const SACM_A2000_WORKING_RAM *A2000WorkingRam);
void A2000_CB_Resume(const SACM_A2000_WORKING_RAM *A2000WorkingRam);
void A2000_CB_GetData(const SACM_A2000_WORKING_RAM *A2000WorkingRam, int16_t *DstBufAddr, int16_t *SrcDataAddr, uint16_t DataLen);
void A2000_CB_SendDac_DmaIsr(const SACM_A2000_WORKING_RAM *A2000WorkingRam, int16_t *SrcDataAddr, uint16_t DataLen);
void A2000_CB_DecodeProcess(const SACM_A2000_WORKING_RAM *A2000WorkingRam, int16_t *DstBufAddr, int16_t *SrcBufAddr);


#ifdef __cplusplus
}
#endif

#endif
