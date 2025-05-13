/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   SACM_DVRIMA_User.h
 * @Version:
 *   V0.9
 * @Date:
 *   April 16, 2025
 * @Abstract:
 *
 **************************************************************************************************/
#ifndef _SACM_DVRIMA_USER_H_
#define _SACM_DVRIMA_USER_H_


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "SACM_DVRIMA_API.h"


/*---------------------------------------------------------------------------------------
 * Definition Area
 *---------------------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C"{
#endif

uint16_t DVRIMA_User_GetDVRIMANum(void);
int16_t* DVRIMA_User_GetDVRIMAStartAddr(uint16_t SpeechIdx);

/*
 * DVRIMA Callback Function Declaration
 */
void DVRIMA_CB_Init(const SACM_DVRIMA_WORKING_RAM *DVRIMAWorkingRam);
void DVRIMA_CB_StartPlay(const SACM_DVRIMA_WORKING_RAM *DVRIMAWorkingRam);
void DVRIMA_CB_Play_Con(const SACM_DVRIMA_WORKING_RAM *DVRIMAWorkingRam);
void DVRIMA_CB_StopPlay(const SACM_DVRIMA_WORKING_RAM *DVRIMAWorkingRam);
void DVRIMA_CB_Pause(const SACM_DVRIMA_WORKING_RAM *DVRIMAWorkingRam);
void DVRIMA_CB_Resume(const SACM_DVRIMA_WORKING_RAM *DVRIMAWorkingRam);
void DVRIMA_CB_GetData(const SACM_DVRIMA_WORKING_RAM *DVRIMAWorkingRam, int16_t *DstBufAddr, int16_t *SrcDataAddr, uint16_t DataLen);
void DVRIMA_CB_SendDac_DmaIsr(const SACM_DVRIMA_WORKING_RAM *DVRIMAWorkingRam, int16_t *SrcDataAddr, uint16_t DataLen);
void DVRIMA_CB_DecodeProcess(const SACM_DVRIMA_WORKING_RAM *DVRIMAWorkingRam, int16_t *DstBufAddr, int16_t *SrcBufAddr);


#ifdef __cplusplus
}
#endif

#endif
