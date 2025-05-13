/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:     
 *   SACM_A3400Pro_CH2_USER.h       
 * @Version: 
 *   V1.0.2
 * @Date: 
 *   September 10, 2021
 * @Abstract: 
 *
 **************************************************************************************************/
#ifndef _SACM_A3400PRO_CH2_USER_H_
#define _SACM_A3400PRO_CH2_USER_H_


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "SACM_A3400Pro_CH2_API.h"


/*---------------------------------------------------------------------------------------
 * Definition Area
 *---------------------------------------------------------------------------------------*/
#define A3400PRO_CH2_FRAME_SIZE									(160)

/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------   ------------------------------------*/
#ifdef __cplusplus
extern "C"{
#endif

uint16_t GetA3400ProNum(void);
int16_t* GetA3400ProStartAddr(uint16_t SpeechIdx);

/*
 * A3400Pro Callback Function Declaration
 */
void A3400Pro_CH2_CB_Init(const SACM_A3400PRO_WORKING_RAM *A3400ProWorkingRam);
void A3400Pro_CH2_CB_StartPlay(const SACM_A3400PRO_WORKING_RAM *A3400ProWorkingRam);
void A3400Pro_CH2_CB_StartPlay_Con(const SACM_A3400PRO_WORKING_RAM *A3400ProCh2WorkingRam);
void A3400Pro_CH2_CB_StopPlay(const SACM_A3400PRO_WORKING_RAM *A3400ProWorkingRam);
void A3400Pro_CH2_CB_Pause(const SACM_A3400PRO_WORKING_RAM *A3400ProWorkingRam);
void A3400Pro_CH2_CB_Resume(const SACM_A3400PRO_WORKING_RAM *A3400ProWorkingRam);
void A3400Pro_CH2_CB_GetData(const SACM_A3400PRO_WORKING_RAM *A3400ProCh2WorkingRam, uint8_t *DstBufAddr, uint8_t *SrcDataAddr, uint16_t DataLen);
void A3400Pro_CH2_CB_SendDac_DmaIsr(const SACM_A3400PRO_WORKING_RAM *A3400ProWorkingRam, int16_t *SrcDataAddr, uint16_t DataLen);
void A3400Pro_CH2_CB_DecodeProcess(const SACM_A3400PRO_WORKING_RAM *A3400ProWorkingRam, int16_t *DstBufAddr, int16_t *SrcBufAddr);

void F_Event_CH2_USER_Init(void);
void F_Event_CH2_USER_IoEvtStart(void);
void F_Event_CH2_USER_IoEvtEnd(void);
void F_Event_CH2_USER_EvtProcess(uint16_t EventData);


uint16_t GetA3400ProCh2Num(void);
int16_t* GetA3400ProCh2StartAddr(uint16_t SpeechIdx);

#ifdef __cplusplus
}
#endif

#endif
