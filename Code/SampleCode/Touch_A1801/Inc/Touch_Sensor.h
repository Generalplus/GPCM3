/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:     		EnvDet_API.h
 *
 * @Version:
 *
 * @Date:     		2019.06.11
 *
 * @Abstract: 		Envelope Detection API declaration
 *
 **************************************************************************************************/
#ifndef _TOUCH_SENSOR_H_
#define _TOUCH_SENSOR_H_
#include "GPCM3_FM1.h"
#include "Touch_User.h"
/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
 * Definition Area
 *---------------------------------------------------------------------------------------*/
//#define C_CalDefault		40
//#define	C_LPointBuf						8							//
//#define	C_MedianBuf						4							//
//#define	C_PadNumber						8							// Max. = 10
//***************************************************************************************
// External Table Declaration
//***************************************************************************************
extern const uint16_t T_TouchInfo[];

//***************************************************************************************
// External Variable Declaration
//***************************************************************************************
extern uint16_t mTouch_Sum[];
extern uint16_t mTouch_Reference[];
extern uint32_t mTouch_ReferenceL[];

//extern uint16_t mTouch_ReferenceDAC[];

extern uint16_t mTouch_SchmittHighIndex;
extern uint16_t mTouch_SchmittLowIndex;
extern uint16_t mTouch_ScanNoIndex;


extern 	uint16_t mTouch_LPointBuf[];
extern 	uint16_t mTouch_MedianBuf[];


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
void F_TraceUp(uint16_t PadIndex, uint16_t L_Sum);
void F_TraceDown(uint16_t PadIndex, uint16_t L_Sum, uint16_t L_Ref);

void CTS_Initial(void);
void CTS_FilterSetting(uint16_t);
void CTS_Scan(void);
void CTS_ScanStop(void);
void CTS_ScanPause(void);
void CTS_ServiceLoop(void);
void ISR_Service_CTS(void);
void CTS_ScanPause(void);
void Touch_AbnorDet(void);
void Touch_AutoReleaseDet(void);
void AutoReleaseCount(void);
uint16_t F_GetSchmittHighIndex(void);
uint16_t F_GetSchmittLowIndex(void);
uint32_t CTS_GetResult(void);


void F_Exit_CTS_ServiceLoop(void);
void F_ReleaseKey(uint16_t PadIndex);

#endif


