/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:     
 *   SACM_A1801_CH4_API.h
 * @Version:  
 *   V1.0.4
 * @Date:     
 *   March 10, 2025
 * @Abstract: 
 *   USER DO NOT MODIFY
 **************************************************************************************************/
#ifndef _SACM_A1801_CH4_API_H_
#define _SACM_A1801_CH4_API_H_


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include <stdint.h>
#include "SACM_A1801_API.h"


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
	
void SACM_A1801_CH4_Initial(SACM_A1801_WORKING_RAM *A1801Ch4WorkingRam, SACM_A1801_TEMP_RAM *pA1801Ch4TempBuffer, void *A1801Ch4PcmBuffer);
void SACM_A1801_CH4_Play(int16_t *SpeechDataStartAddr, uint8_t DacChannelSel, uint8_t AutoRampUpDownCtrl);
void SACM_A1801_CH4_Play_Con(int16_t *SpeechDataStartAddr, uint8_t DacChannelNo, uint8_t AutoRampUpDownCtrl);
uint16_t SACM_A1801_CH4_Check_Con(void);
void SACM_A1801_CH4_Stop(void);
uint16_t SACM_A1801_CH4_GetStatus(void);
void SACM_A1801_CH4_ServiceLoop(void);
void SACM_A1801_CH4_Pause(void);
void SACM_A1801_CH4_Resume(void);
uint32_t SACM_A1801_CH4_CheckCpuOverload(void);
void SACM_A1801_CH4_ClearCpuOverload(void);	
void SACM_A1801_CH4_DmaIsrService(void);
uint32_t SACM_A1801_CH4_DecodeProcess(int16_t *DstBufAddr, int16_t *SrcBufAddr);
	

#ifdef __cplusplus
}
#endif

#endif
