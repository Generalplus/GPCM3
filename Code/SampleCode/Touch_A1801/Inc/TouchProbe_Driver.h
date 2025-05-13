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
#ifndef _TOUCHPROBE_DRIVER_H_
#define _TOUCHPROBE_DRIVER_H_

/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
 * Definition Area
 *---------------------------------------------------------------------------------------*/

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
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
void TP_Initial(void);
void TP_Start(void);
void TP_Stop(void);
void F_TP_SendHeader(uint16_t command);
void F_AddTouchResult(void);
void TP_SendAllSum_Ref(void);
void TP_ServiceLoop(uint16_t *pAddr);

#endif


