/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   SACM_A1801_User.h
 * @Version:
 *   V1.0.3
 * @Date:
 *   December 10, 2021
 * @Abstract:
 *
 **************************************************************************************************/
#ifndef _SACM_A1801_USER_H_
#define _SACM_A1801_USER_H_


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "SACM_A1801_API.h"


/*---------------------------------------------------------------------------------------
 * Definition Area
 *---------------------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C"{
#endif

uint16_t A1801_User_GetA1801Num(void);
int16_t* A1801_User_GetA1801StartAddr(uint16_t SpeechIdx);

/*
 * A1801 Callback Function Declaration
 */
void A1801_CB_Init(const SACM_A1801_WORKING_RAM *A1801WorkingRam);
void A1801_CB_StartPlay(const SACM_A1801_WORKING_RAM *A1801WorkingRam);
void A1801_CB_Play_Con(const SACM_A1801_WORKING_RAM *A1801WorkingRam);
void A1801_CB_StopPlay(const SACM_A1801_WORKING_RAM *A1801WorkingRam);
void A1801_CB_Pause(const SACM_A1801_WORKING_RAM *A1801WorkingRam);
void A1801_CB_Resume(const SACM_A1801_WORKING_RAM *A1801WorkingRam);
void A1801_CB_GetData(const SACM_A1801_WORKING_RAM *A1801WorkingRam, int16_t *DstBufAddr, int16_t *SrcDataAddr, uint16_t DataLen);
void A1801_CB_SendDac_DmaIsr(const SACM_A1801_WORKING_RAM *A1801WorkingRam, int16_t *SrcDataAddr, uint16_t DataLen);
void A1801_CB_DecodeProcess(const SACM_A1801_WORKING_RAM *A1801WorkingRam, int16_t *DstBufAddr, int16_t *SrcBufAddr);


#ifdef __cplusplus
}
#endif

#endif
