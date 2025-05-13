/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   AEC_User.h
 * @Version:
 *   V1.0.0
 * @Date:
 *   26th, December 2019
 * @Abstract:
 *	 GPCM1_AEC_20191213.lib (use HW-MAC)
 **************************************************************************************************/
#ifndef _AEC_USER_H_
#define _AEC_USER_H_

/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "AEC_API.h"
/*---------------------------------------------------------------------------------------
 * Definition Area
 *---------------------------------------------------------------------------------------*/
#define CQBuf_size			8

/*---------------------------------------------------------------------------------------
 * Structure Definition Area
 *---------------------------------------------------------------------------------------*/
typedef struct _AEC_ADC_RAM
{
  int32_t *AecPcmBufPtr;
	int32_t *AecInBufPtr;
	int32_t  AecPcmBuf[2][AEC_FRAME_SIZE];						// increase 320 bytes (40*4*2)// 160 words
	uint8_t  AEC_CQBuf_WtPtr;
} AEC_ADC_RAM;

/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C"{
#endif

//User
void AEC_DmaIsrService(void);
void AEC_GetAdc_DmaTrigger(void);
void AEC_User_ServiceLoop(void);

void EXT_INT_Initial(void);
void F_AGC_Update_ADC(int32_t	*StereoADC, int32_t Length);
void F_Copy_DS_ADC_Memory(int32_t	*StereoADC, int16_t	*monoADC,	int32_t Length);

void AEC_Open(void);
void AEC_Close(void);
void AGC_Open(void);
void AGC_Close(void);

#ifdef __cplusplus
}
#endif

#endif
