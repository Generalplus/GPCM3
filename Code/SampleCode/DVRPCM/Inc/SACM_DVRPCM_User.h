/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   SACM_DVRPCM_USER.h
 * @Version:
 *   V1.0.0
 * @Date:
 *   December 2, 2019
 * @Abstract:
 *
 **************************************************************************************************/
#ifndef _SACM_DVRPCM_USER_H_
#define _SACM_DVRPCM_USER_H_


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "SACM_DVRPCM_API.h"

/*
 * DvrPcm Library Constant definition
 */
#define DVRPCM_FRAME_SIZE                       (160)

/*---------------------------------------------------------------------------------------
 * Definition Area
 *---------------------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C"{
#endif

uint16_t GetPcmNum(void);
int16_t* GetPcmStartAddr(uint16_t SpeechIdx);

/*
 * DVRPCM Callback Function Declaration
 */
void DVRPCM_CB_Init(const SACM_DVRPCM_WORKING_RAM *DvrPcmWorkingRam);
void DVRPCM_CB_StartPlay(const SACM_DVRPCM_WORKING_RAM *DvrPcmWorkingRam);
void DVRPCM_CB_StopPlay(const SACM_DVRPCM_WORKING_RAM *DvrPcmWorkingRam);
void DVRPCM_CB_Pause(const SACM_DVRPCM_WORKING_RAM *DvrPcmWorkingRam);
void DVRPCM_CB_Resume(const SACM_DVRPCM_WORKING_RAM *DvrPcmWorkingRam);
void DVRPCM_CB_StartRecord(const SACM_DVRPCM_WORKING_RAM *DvrPcmWorkingRam);
void DVRPCM_CB_StopRecord(const SACM_DVRPCM_WORKING_RAM *DvrPcmWorkingRam,  uint32_t RecDataLen);
void DVRPCM_CB_GetData(const SACM_DVRPCM_WORKING_RAM *DvrPcmWorkingRam, int16_t *DstBufAddr, int16_t *SrcDataAddr, uint16_t DataLen);
uint8_t DVRPCM_CB_WriteData(const SACM_DVRPCM_WORKING_RAM *DvrPcmWorkingRam, int16_t *DstDataAddr, int16_t *SrcBufAddr, uint16_t DataLen);
void DVRPCM_CB_SendDac_DmaIsr(const SACM_DVRPCM_WORKING_RAM *DvrPcmWorkingRam, int16_t *SrcDataAddr, uint16_t DataLen);
void DVRPCM_CB_GetAdc_DmaIsr(const SACM_DVRPCM_WORKING_RAM *DvrPcmWorkingRam, int16_t *DstBufAddr, uint16_t DataLen);
void DVRPCM_CB_DecodeProcess(const SACM_DVRPCM_WORKING_RAM *DvrPcmWorkingRam, int16_t *DstBufAddr, int16_t *SrcBufAddr);
void DVRPCM_CB_EncodeProcess(const SACM_DVRPCM_WORKING_RAM *DvrPcmWorkingRam, int16_t *DstBufAddr, int16_t *SrcBufAddr);

#ifdef __cplusplus
}
#endif

#endif
