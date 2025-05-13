/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:     
 *   BDM_User.h
 * @Version:  
 *   V0.9.1
 * @Date:     
 *   October 12, 2020
 * @Abstract: 
 *
 **************************************************************************************************/
#ifndef _BDM_USER_H_
#define _BDM_USER_H_


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include <stdint.h>
#include "BDM_API.h"


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C"{
#endif

	
/*
 * BDM Callback Function Declaration
 */	
void BDM_CB_Init(void);
void BDM_CB_GetAdc_DmaIsr(int16_t *DstPcmBufAddr, uint16_t DataLen);
void BDM_CB_Start(int16_t *DstPcmBufAddr, uint16_t DataLen);
void BDM_CB_Stop(void);

#ifdef __cplusplus
}
#endif	

#endif
