/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:     
 *   SACM_A3400Pro_CH3_USER.h       
 * @Version: 
 *   V1.0.4
 * @Date: 
 *   January 13, 2025
 * @Abstract: 
 *
 **************************************************************************************************/
#ifndef _SACM_A3400PRO_CH3_USER_H_
#define _SACM_A3400PRO_CH3_USER_H_


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "SACM_A3400Pro_CH3_API.h"


/*---------------------------------------------------------------------------------------
 * Definition Area
 *---------------------------------------------------------------------------------------*/
#define A3400PRO_CH3_FRAME_SIZE									(160)

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
void A3400Pro_CH3_CB_Init(const SACM_A3400PRO_WORKING_RAM *A3400ProWorkingRam);
void A3400Pro_CH3_CB_StartPlay(const SACM_A3400PRO_WORKING_RAM *A3400ProWorkingRam);
void A3400Pro_CH3_CB_StartPlay_Con(const SACM_A3400PRO_WORKING_RAM *A3400ProCh3WorkingRam);
void A3400Pro_CH3_CB_StopPlay(const SACM_A3400PRO_WORKING_RAM *A3400ProWorkingRam);
void A3400Pro_CH3_CB_Pause(const SACM_A3400PRO_WORKING_RAM *A3400ProWorkingRam);
void A3400Pro_CH3_CB_Resume(const SACM_A3400PRO_WORKING_RAM *A3400ProWorkingRam);
void A3400Pro_CH3_CB_GetData(const SACM_A3400PRO_WORKING_RAM *A3400ProCh3WorkingRam, uint8_t *DstBufAddr, uint8_t *SrcDataAddr, uint16_t DataLen);
void A3400Pro_CH3_CB_SendDac_DmaIsr(const SACM_A3400PRO_WORKING_RAM *A3400ProWorkingRam, int16_t *SrcDataAddr, uint16_t DataLen);
void A3400Pro_CH3_CB_DecodeProcess(const SACM_A3400PRO_WORKING_RAM *A3400ProWorkingRam, int16_t *DstBufAddr, int16_t *SrcBufAddr);

void F_Event_CH3_USER_Init(void);
void F_Event_CH3_USER_IoEvtStart(void);
void F_Event_CH3_USER_IoEvtEnd(void);
void F_Event_CH3_USER_EvtProcess(uint16_t EventData);


uint16_t GetA3400ProCh3Num(void);
int16_t* GetA3400ProCh3StartAddr(uint16_t SpeechIdx);

#ifdef __cplusplus
}
#endif

#endif
