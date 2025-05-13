/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:     
 *   SACM_A1801_CH3_User.h      
 * @Version: 
 *   V1.0.4
 * @Date: 
 *   March 10, 2025
 * @Abstract: 
 *
 **************************************************************************************************/
#ifndef _SACM_A1801_CH3_USER_H_
#define _SACM_A1801_CH3_USER_H_


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "SACM_A1801_CH3_API.h"


/*---------------------------------------------------------------------------------------
 * Definition Area
 *---------------------------------------------------------------------------------------*/
 

/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C"{
#endif

void A1801_CH3_User_A1801Ch3WithGpEvent_ServerLoop(void);

/*
 * A1801_CH3 Callback Function Declaration
 */
void A1801_CH3_CB_Init(const SACM_A1801_WORKING_RAM *A1801Ch3WorkingRam);
void A1801_CH3_CB_StartPlay(const SACM_A1801_WORKING_RAM *A1801Ch3WorkingRam);
void A1801_CH3_CB_Play_Con(const SACM_A1801_WORKING_RAM *A1801Ch3WorkingRam);	
void A1801_CH3_CB_StopPlay(const SACM_A1801_WORKING_RAM *A1801Ch3WorkingRam);
void A1801_CH3_CB_Pause(const SACM_A1801_WORKING_RAM *A1801Ch3WorkingRam);
void A1801_CH3_CB_Resume(const SACM_A1801_WORKING_RAM *A1801Ch3WorkingRam);
void A1801_CH3_CB_GetData(const SACM_A1801_WORKING_RAM *A1801Ch3WorkingRam, int16_t *DstBufAddr, int16_t *SrcDataAddr, uint16_t DataLen);
void A1801_CH3_CB_SendDac_DmaIsr(const SACM_A1801_WORKING_RAM *A1801Ch3WorkingRam, int16_t *SrcDataAddr, uint16_t DataLen);
void A1801_CH3_CB_DecodeProcess(const SACM_A1801_WORKING_RAM *A1801Ch3WorkingRam, int16_t *DstBufAddr, int16_t *SrcBufAddr);

	
#ifdef __cplusplus
}
#endif

#endif
