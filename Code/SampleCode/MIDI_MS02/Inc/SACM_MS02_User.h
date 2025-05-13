/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   SACM_MS02_User.h
 * @Version:
 *   V1.0.0
 * @Date:
 *   27, Oct, 2021
 * @Abstract:
 *  2021.10.27 Add the Duration parameter of MS02_CB_NoteOnEvent(short R_CMD_Code, short R_Duration)
 *
 **************************************************************************************************/
#ifndef _SACM_MS02_USER_H_
#define _SACM_MS02_USER_H_


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "SACM_MS02_API.h"


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

uint16_t GetMidiNum(void);
int16_t* GetMS02LibStartAddr(uint16_t SpeechIdx);
int16_t* GetMidiStartAddr(uint16_t MidiIdx);

/*
 * MS02 Callback Function Declaration
 */
void MS02_CB_Init(const SACM_MS02_WORKING_RAM *MS02WorkingRam, uint8_t AudOutType, uint8_t DacChannelNo, uint32_t Timer_SR);
void MS02_CB_StartPlay(const SACM_MS02_WORKING_RAM *MS02WorkingRam);
uint8_t MS02_CB_StopPlay(const SACM_MS02_WORKING_RAM *MS02WorkingRam);
void MS02_CB_Pause(const SACM_MS02_WORKING_RAM *MS02WorkingRam);
void MS02_CB_Resume(const SACM_MS02_WORKING_RAM *MS02WorkingRam);
void MS02_CB_GetData(const SACM_MS02_WORKING_RAM *MS02WorkingRam, int16_t *DstBufAddr, int16_t *SrcDataAddr, uint16_t DataLen);
void MS02_CB_SendDac_DmaIsr(const SACM_MS02_WORKING_RAM *MS02WorkingRam, int16_t *SrcDataAddr, uint16_t DataLen);
void MS02_CB_DecodeProcess(int16_t* DstBufAddr, int16_t OutBufSize);
void MS02_CB_SongEvent(short R_CallBack_Event, short R_CMD_Code);
void MS02_CB_NoteOnEvent(short R_CMD_Code, short R_Duration);


#ifdef __cplusplus
}
#endif

#endif
