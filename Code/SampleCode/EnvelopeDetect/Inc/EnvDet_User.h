/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:     		EnvDet_User.h
 *
 * @Version:  
 *
 * @Date:     		2019.06.11
 *
 * @Abstract: 		Envelope Detection API declaration
 *
 **************************************************************************************************/
#ifndef _ENVDET_USER_H_
#define _ENVDET_USER_H_

/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "EnvDet_API.h"

/*---------------------------------------------------------------------------------------
 * Definition Area
 *---------------------------------------------------------------------------------------*/
#define	ENVDET_BUF_EVEN_FLAG     (0x01)
#define ENVDET_FRAME_SIZE					40

/*---------------------------------------------------------------------------------------
 * Structure Definition Area
 *---------------------------------------------------------------------------------------*/ 
typedef struct _ENVDET_ADC_RAM
{	
  int16_t *EnvDetPcmBufPtr;
	int16_t *EnvDetInBufPtr;
	int16_t  EnvDetPcmBuf[2][ENVDET_FRAME_SIZE];						
	uint8_t  EnvDet_WtPtr;
} ENVDET_ADC_RAM;

/*
 * EnvDet Callback Function Declaration
 */
void EnvDet_CB_Init(const ENVDET_WORKING_RAM *EnvDetWorkingRam);
void EnvDet_CB_Stop(const ENVDET_WORKING_RAM *EnvDetWorkingRam);
int16_t EnvDet_CB_GetAdc(void);
void EnvDet_DmaIsrService(void);

#endif


