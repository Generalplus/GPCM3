/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:     
 *   FFT_User.h
 * @Version:  
 *   V0.9.0
 * @Date:     
 *   January 21, 2021
 * @Abstract: 
 *
 **************************************************************************************************/
#ifndef _FFT_USER_H_
#define _FFT_USER_H_


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "FFT_API.h"

/*---------------------------------------------------------------------------------------
 * Definition Area
 *---------------------------------------------------------------------------------------*/ 
#define FFT_FRAME_SIZE                         	(256) 

/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C"{
#endif

/*
 * CSP16 Callback Function Declaration
 */	
void FFT_CB_Init(FFT_WORKING_RAM *FftWorkingRam);
void FFT_CB_GetAdc_DmaIsr(int16_t *DstPcmBufAddr, uint16_t DataLen);
void FFT_CB_VoiceProcess(const FFT_WORKING_RAM *FftWorkingRam, int16_t *SrcBufAddr, uint16_t DataLen);
void FFT_CB_Start(int16_t *DstPcmBufAddr, uint16_t DataLen);
void FFT_CB_Stop(void);
	

#ifdef __cplusplus
}
#endif	

#endif
