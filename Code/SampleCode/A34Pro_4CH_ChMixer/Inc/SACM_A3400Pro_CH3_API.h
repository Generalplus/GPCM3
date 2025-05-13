/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:     
 *   SACM_A3400Pro_CH3_API.h
 * @Version:  
 *   V1.0.4
 * @Date:     
 *   January 13, 2025
 * @Abstract: 
 *   USER DO NOT MODIFY
 **************************************************************************************************/
#ifndef _SACM_A3400PRO_CH3_API_H_
#define _SACM_A3400PRO_CH3_API_H_


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "SACM_A3400Pro_API.h"

/*---------------------------------------------------------------------------------------
 * Constant Definition Area
 *---------------------------------------------------------------------------------------*/

/*
 * A3400Pro Library Constant definition
 */
#define	A3400PRO_KERNEL_RAM_SIZE                  (28)     // Unit: byte


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C"{
#endif
	
void SACM_A3400Pro_CH3_Initial(SACM_A3400PRO_WORKING_RAM *A3400ProWorkingRam, int16_t *A3400ProPcmBuf, int16_t lFrameSize);
void SACM_A3400Pro_CH3_Play(int16_t *SpeechDataStartAddr, uint8_t DacChannelSel, uint8_t AutoRampUpDownCtrl);
void SACM_A3400Pro_CH3_Play_Con(int16_t *SpeechDataStartAddr, uint8_t DacChannelNo, uint8_t AutoRampUpDownCtrl);
uint16_t SACM_A3400Pro_CH3_Check_Con(void);
void SACM_A3400Pro_CH3_Stop(void);
uint16_t SACM_A3400Pro_CH3_GetStatus(void);
void SACM_A3400Pro_CH3_ServiceLoop(void);
void SACM_A3400Pro_CH3_Pause(void);
void SACM_A3400Pro_CH3_Resume(void);
void SACM_A3400Pro_CH3_DmaIsrService(void);
uint32_t SACM_A3400Pro_CH3_DecodeProcess(int16_t *DstBufAddr, int16_t *SrcBufAddr);
uint32_t SACM_A3400Pro_CH3_CheckCpuOverload(void);
void SACM_A3400Pro_CH3_ClearCpuOverload(void);
	
#ifdef __cplusplus
}
#endif

#endif
