/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:     
 *   SACM_A2000_CH2_User.h      
 * @Version: 
 *   V1.0.0
 * @Date: 
 *   August 05, 2024
 * @Abstract: 
 *
 **************************************************************************************************/
#ifndef _SACM_A2000_CH2_USER_H_
#define _SACM_A2000_CH2_USER_H_


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "SACM_A2000_CH2_API.h"


/*---------------------------------------------------------------------------------------
 * Definition Area
 *---------------------------------------------------------------------------------------*/
 

/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C"{
#endif

void A2000_CH2_User_A2000Ch2WithGpEvent_ServerLoop(void);

/*
 * A2000_CH2 Callback Function Declaration
 */
void A2000_CH2_CB_Init(const SACM_A2000_WORKING_RAM *A2000Ch2WorkingRam);
void A2000_CH2_CB_StartPlay(const SACM_A2000_WORKING_RAM *A2000Ch2WorkingRam);
void A2000_CH2_CB_Play_Con(const SACM_A2000_WORKING_RAM *A2000Ch2WorkingRam);	
void A2000_CH2_CB_StopPlay(const SACM_A2000_WORKING_RAM *A2000Ch2WorkingRam);
void A2000_CH2_CB_Pause(const SACM_A2000_WORKING_RAM *A2000Ch2WorkingRam);
void A2000_CH2_CB_Resume(const SACM_A2000_WORKING_RAM *A2000Ch2WorkingRam);
void A2000_CH2_CB_GetData(const SACM_A2000_WORKING_RAM *A2000Ch2WorkingRam, int16_t *DstBufAddr, int16_t *SrcDataAddr, uint16_t DataLen);
void A2000_CH2_CB_SendDac_DmaIsr(const SACM_A2000_WORKING_RAM *A2000Ch2WorkingRam, int16_t *SrcDataAddr, uint16_t DataLen);
void A2000_CH2_CB_DecodeProcess(const SACM_A2000_WORKING_RAM *A2000Ch2WorkingRam, int16_t *DstBufAddr, int16_t *SrcBufAddr);

	
#ifdef __cplusplus
}
#endif

#endif
