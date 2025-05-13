/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:     
 *   SACM_A2000_CH2_API.h
 * @Version:  
 *   V1.0.0
 * @Date:     
 *   August 05, 2024
 * @Abstract: 
 *   USER DO NOT MODIFY
 **************************************************************************************************/
#ifndef _SACM_A2000_CH2_API_H_
#define _SACM_A2000_CH2_API_H_


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include <stdint.h>
#include "SACM_A2000_API.h"


/*---------------------------------------------------------------------------------------
 * Constant Definition Area
 *---------------------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------------------
 * Structure Definition Area
 *---------------------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C"{
#endif
	
void SACM_A2000_CH2_Initial(SACM_A2000_WORKING_RAM *A2000Ch2WorkingRam, SACM_A2000_TEMP_RAM *pA2000Ch2TempBuffer, void *A2000Ch2PcmBuffer);
void SACM_A2000_CH2_Play(int16_t *SpeechDataStartAddr, uint8_t DacChannelSel, uint8_t AutoRampUpDownCtrl);
void SACM_A2000_CH2_Play_Con(int16_t *SpeechDataStartAddr, uint8_t DacChannelNo, uint8_t AutoRampUpDownCtrl);
uint16_t SACM_A2000_CH2_Check_Con(void);
void SACM_A2000_CH2_Stop(void);
uint16_t SACM_A2000_CH2_GetStatus(void);
void SACM_A2000_CH2_ServiceLoop(void);
void SACM_A2000_CH2_Pause(void);
void SACM_A2000_CH2_Resume(void);
uint32_t SACM_A2000_CH2_CheckCpuOverload(void);
void SACM_A2000_CH2_ClearCpuOverload(void);	
void SACM_A2000_CH2_DmaIsrService(void);
uint32_t SACM_A2000_CH2_DecodeProcess(int16_t *DstBufAddr, int16_t *SrcBufAddr);
	

#ifdef __cplusplus
}
#endif

#endif
