/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:     
 *   SACM_A3400Pro_CH2_API.h
 * @Version:  
 *   V1.0.2
 * @Date:     
 *   September 10, 2021
 * @Abstract: 
 *   USER DO NOT MODIFY
 **************************************************************************************************/
#ifndef _SACM_A3400PRO_CH2_API_H_
#define _SACM_A3400PRO_CH2_API_H_


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
	
void SACM_A3400Pro_CH2_Initial(SACM_A3400PRO_WORKING_RAM *A3400ProWorkingRam, int16_t *A3400ProPcmBuf, int16_t lFrameSize);
void SACM_A3400Pro_CH2_Play(int16_t *SpeechDataStartAddr, uint8_t DacChannelSel, uint8_t AutoRampUpDownCtrl);
void SACM_A3400Pro_CH2_Play_Con(int16_t *SpeechDataStartAddr, uint8_t DacChannelNo, uint8_t AutoRampUpDownCtrl);
uint16_t SACM_A3400Pro_CH2_Check_Con(void);
void SACM_A3400Pro_CH2_Stop(void);
uint16_t SACM_A3400Pro_CH2_GetStatus(void);
void SACM_A3400Pro_CH2_ServiceLoop(void);
void SACM_A3400Pro_CH2_Pause(void);
void SACM_A3400Pro_CH2_Resume(void);
void SACM_A3400Pro_CH2_DmaIsrService(void);
uint32_t SACM_A3400Pro_CH2_DecodeProcess(int16_t *DstBufAddr, int16_t *SrcBufAddr);
uint32_t SACM_A3400Pro_CH2_CheckCpuOverload(void);
void SACM_A3400Pro_CH2_ClearCpuOverload(void);
	
#ifdef __cplusplus
}
#endif

#endif
