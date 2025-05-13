/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:     		Touch_User.h
 *
 * @Version:
 *
 * @Date:     		2019.06.11
 *
 * @Abstract: 		Touch User declaration
 *
 **************************************************************************************************/
#ifndef _TOUCH_USER_H_
#define _TOUCH_USER_H_

/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "GPCM3_FM1.h"
#include "TOUCH_SENSOR.h"

/*
 * Touch Sensor Callback Function Declaration
 */

/*---------------------------------------------------------------------------------------
 * Public Function List
 *---------------------------------------------------------------------------------------*/
void F_Touch_User_Init(void);
uint16_t F_CTS_GetScanTime(void);
void TouchScanOneRound(uint16_t *SensorResultPtr);
void CTS_CH_Sel(uint16_t CTS_Ch);
void CTS_Start(void);
uint16_t Get_SchmittL(uint32_t PadIndex, uint32_t GroupIndex);
uint16_t Get_SchmittH(uint32_t PadIndex, uint32_t GroupIndex);
void CTSTMB_IRQHandler(void);
void CTSTMA_IRQHandler(void);
void F_Touch_CB_ReInit(void);
uint16_t Get_ScanNo(uint32_t PadIndex, uint32_t GroupIndex);


/*---------------------------------------------------------------------------------------
 * Touch library funciton
 *---------------------------------------------------------------------------------------*/
void ISR_Service_CTS(void);

/*---------------------------------------------------------------------------------------
 * TouchProbe library funciton
 *---------------------------------------------------------------------------------------*/
void TP_ServiceLoop(uint16_t *addr);


#endif


