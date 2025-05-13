/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:     
 *   MP3_USER.h       
 * @Version: 
 *   V1.0.2
 * @Date: 
 *   December 14, 2021
 * @Abstract: 
 *
 **************************************************************************************************/
#ifndef _MP3_USER_H_
#define _MP3_USER_H_


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "MP3_API.h"


/*---------------------------------------------------------------------------------------
 * Definition Area
 *---------------------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C"{
#endif

uint16_t GetMp3Num(void);
uint8_t* GetMp3StartAddr(uint16_t Mp3Index);
int32_t GetMp3Length(uint16_t Mp3Index);

/*
 * MP3 Library Callback Function Declaration
 */
void MP3_CB_Init(const MP3_LIB_WORKING_RAM *Mp3WorkingRam);
void MP3_CB_StartPlay(const MP3_LIB_WORKING_RAM *Mp3WorkingRam, const int32_t SampleRate);
void MP3_CB_StopPlay(const MP3_LIB_WORKING_RAM *Mp3WorkingRam);
void MP3_CB_Pause(const MP3_LIB_WORKING_RAM *Mp3WorkingRam);
void MP3_CB_Resume(const MP3_LIB_WORKING_RAM *Mp3WorkingRam);
void MP3_CB_GetData(const MP3_LIB_WORKING_RAM *Mp3WorkingRam, uint8_t *DstBufAddr, uint8_t *SrcDataAddr, uint16_t DataLen);
void MP3_CB_SendDac_DmaIsr(const MP3_LIB_WORKING_RAM *Mp3WorkingRam, int16_t *SrcDataAddr, uint16_t DataLen);

	
#ifdef __cplusplus
}
#endif

#endif
