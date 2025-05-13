/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   SACM_DVR1801_User.h
 * @Version:
 *   V1.0.4
 * @Date:
 *   December 14, 2021
 * @Abstract:
 *
 **************************************************************************************************/
#ifndef _SACM_DVR1801_USER_H_
#define _SACM_DVR1801_USER_H_


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "SACM_DVR1801_API.h"


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

uint16_t GetA1801Num(void);
int16_t* GetA1801StartAddr(uint16_t SpeechIdx);
void A1801_Vc3EffectEnable(void);
void A1801_Vc3EffectDisable(void);

/*
 * DVR1801 Callback Function Declaration
 */
void DVR1801_CB_Init(const SACM_DVR1801_WORKING_RAM *Dvr1801WorkingRam);
void DVR1801_CB_StartPlay(const SACM_DVR1801_WORKING_RAM *Dvr1801WorkingRam);
void DVR1801_CB_Play_Con(const SACM_DVR1801_WORKING_RAM *Dvr1801WorkingRam);
void DVR1801_CB_StopPlay(const SACM_DVR1801_WORKING_RAM *Dvr1801WorkingRam);
void DVR1801_CB_Pause(const SACM_DVR1801_WORKING_RAM *Dvr1801WorkingRam);
void DVR1801_CB_Resume(const SACM_DVR1801_WORKING_RAM *Dvr1801WorkingRam);
void DVR1801_CB_StartRecord(const SACM_DVR1801_WORKING_RAM *Dvr1801WorkingRam);
void DVR1801_CB_StopRecord(const SACM_DVR1801_WORKING_RAM *Dvr1801WorkingRam,  uint32_t RecDataLen);
void DVR1801_CB_GetData(const SACM_DVR1801_WORKING_RAM *Dvr1801WorkingRam, int16_t *DstBufAddr, int16_t *SrcDataAddr, uint16_t DataLen);
void DVR1801_CB_WriteData(const SACM_DVR1801_WORKING_RAM *Dvr1801WorkingRam, int16_t *DstDataAddr, int16_t *SrcBufAddr, uint16_t DataLen);
void DVR1801_CB_SendDac_DmaIsr(const SACM_DVR1801_WORKING_RAM *Dvr1801WorkingRam, int16_t *SrcDataAddr, uint16_t DataLen);
void DVR1801_CB_GetAdc_DmaIsr(const SACM_DVR1801_WORKING_RAM *Dvr1801WorkingRam, int16_t *DstBufAddr, uint16_t DataLen);
void DVR1801_CB_DecodeProcess(const SACM_DVR1801_WORKING_RAM *Dvr1801WorkingRam, int16_t *DstBufAddr, int16_t *SrcBufAddr);
void DVR1801_CB_EncodeProcess(const SACM_DVR1801_WORKING_RAM *Dvr1801WorkingRam, int16_t *DstBufAddr, int16_t *SrcBufAddr);


#ifdef __cplusplus
}
#endif

#endif
