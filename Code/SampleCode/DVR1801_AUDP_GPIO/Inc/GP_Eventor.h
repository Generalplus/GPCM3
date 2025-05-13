/**************************************************************************************************
 * Copyright(c) 2020 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:     
 *   GP_Eventor.h       
 * @Version: 
 *   V0.9.0
 * @Date: 
 *   December 22, 2020
 * @Abstract: 
 *   USER DO NOT MODIFY
 **************************************************************************************************/
#ifndef _GP_EVENTOR_H_
#define _GP_EVENTOR_H_


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include <stdint.h>


/*---------------------------------------------------------------------------------------
 * Definition Area
 *---------------------------------------------------------------------------------------*/ 
/*
 * EVENTOR status definition
 */ 
#define EVENTOR_EN_FLAG                     (0x8000)
#define EVENTOR_IO_EVENT_ENABLE_FLAG        (0x0001)
#define EVENTOR_USER_EVENT_ENABLE_FLAG      (0x0002)

#define EVENTOR_CH0                         (0)
#define EVENTOR_CH1                         (1)
#define EVENTOR_CH2                         (2)
#define EVENTOR_CH3                         (3)

																							
/*---------------------------------------------------------------------------------------
 * Structure Definition Area
 *---------------------------------------------------------------------------------------*/ 
typedef struct _GP_EVENTOR_WORKING_RAM
{		
	uint8_t* EvtDataStartAddr;
	uint8_t EventorIoNum;
	uint8_t ProcessedEventCnt;
	uint8_t EventCnt;	
	uint16_t SwPwmLvl;
	volatile uint16_t SwPwmDutyCounter;
	int16_t* SwPwmWorkingRamPtr;
} GP_EVENTOR_WORKING_RAM;																							
																							

/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C"{
#endif

void GP_EVENTOR_Initial(uint8_t EventorChannelSel, GP_EVENTOR_WORKING_RAM *GpEvtWorkingRam, int16_t *GpEvtSwPwmWorkingRam, uint8_t EventIoNum, uint16_t SwPwmLvl);
void GP_EVENTOR_Start(uint8_t EventorChannelSel, uint8_t* EventDataStartAddr);
void GP_EVENTOR_Stop(uint8_t EventorChannelSel);
uint16_t GP_EVENTOR_GetStatus(uint8_t EventorChannelSel);	
void GP_EVENTOR_AddEvent(uint8_t EventorChannelSel);
void GP_EVENTOR_IoEvtEnable(uint8_t EventorChannelSel);
void GP_EVENTOR_IoEvtDisable(uint8_t EventorChannelSel);
void GP_EVENTOR_UserEvtEnable(uint8_t EventorChannelSel);
void GP_EVENTOR_UserEvtDisable(uint8_t EventorChannelSel);	
void GP_EVENTOR_ServiceLoop(uint8_t EventorChannelSel);		
void GP_EVENTOR_ISR_Service(uint8_t EventorChannelSel);	

	
#ifdef __cplusplus
}
#endif

#endif
