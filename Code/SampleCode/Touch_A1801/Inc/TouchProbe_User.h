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
#ifndef _TOUCHPROBE_USER_H_
#define _TOUCHPROBE_USER_H_


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "GPCM3_FM1.h"


//***************************************************************************************
// External Table Declaration
//***************************************************************************************
extern const uint16_t T_TouchInfo[];
extern const uint16_t T_TPDelay[];
extern uint16_t R_TP_Buffer[];				// For Touch Probe Used Only
extern uint16_t mTouch_Sum[];
extern uint32_t mTouch_ReferenceL[];
extern uint16_t mTouch_Reference[];
//extern uint16_t mTouch_ScanNo[];
//extern uint16_t mTouch_Schmitt_H[];
//extern uint16_t mTouch_Schmitt_L[];


/*---------------------------------------------------------------------------------------
 * Definition Area
 *---------------------------------------------------------------------------------------*/

/*
 * Touch Probe Callback Function Declaration
 */

/*---------------------------------------------------------------------------------------
 * Public Function List
 *---------------------------------------------------------------------------------------*/
void F_TP_HW_Init(void);
void F_TP_SetCSHigh(uint32_t DelayT);
void F_TP_SetCSLow(uint32_t lDelayTime);
void F_TP_Delay_1us(uint32_t DelayT);
uint32_t F_TP_SendDataToTouchProbe(uint8_t *pSrcBuf, uint32_t DataLength);
void F_TP_SendAByte(uint8_t Data);
void F_TP_SendAWord(uint16_t wData);
uint8_t F_TP_ReadAByte(void);
uint16_t F_TP_ReadAWord(void);

/*---------------------------------------------------------------------------------------
 * library funciton
 *---------------------------------------------------------------------------------------*/


#endif


