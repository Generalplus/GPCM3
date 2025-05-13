/**************************************************************************************************
 * Copyright(c) 2020 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   MS02_EVENTOR.h
 * @Version:
 *   V0.9.1
 * @Date:
 *   March 20, 2024
 * @Abstract:
 *   1. Modify for MS02 Event of the GPCMx series
 *
 **************************************************************************************************/
#ifndef _MS02_EVENTOR_H_
#define _MS02_EVENTOR_H_


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
#define MS02_EVENT_EN_FLAG                     (0x8000)
#define MS02_EVENT_IO_EVENT_ENABLE_FLAG        (0x0001)
#define MS02_EVENT_USER_EVENT_ENABLE_FLAG      (0x0002)

#define MS02_EVENT_CH0                         (0)
#define MS02_EVENT_CH1                         (1)
#define MS02_EVENT_CH2                         (2)

/*---------------------------------------------------------------------------------------
 * Structure Definition Area
 *---------------------------------------------------------------------------------------*/
typedef struct _MS02_EVENTOR_WORKING_RAM
{
	uint8_t* EvtDataStartAddr;
	uint8_t EventorIoNum;
	uint8_t ProcessedEventCnt;
	uint8_t EventCnt;
	uint16_t SwPwmLvl;
	volatile uint16_t SwPwmDutyCounter;
	int16_t* SwPwmWorkingRamPtr;
} MS02_EVENTOR_WORKING_RAM;


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C"{
#endif

void MS02_EVENTOR_Initial(uint8_t EventorChannelSel, MS02_EVENTOR_WORKING_RAM *MS02EvtWorkingRam, int16_t *GpEvtSwPwmWorkingRam, uint8_t EventIoNum, uint16_t SwPwmLvl);
void MS02_EVENTOR_Start(uint8_t EventorChannelSel, uint8_t* EventDataStartAddr);
void MS02_EVENTOR_Stop(uint8_t EventorChannelSel);
uint16_t MS02_EVENTOR_GetStatus(uint8_t EventorChannelSel);
void MS02_EVENTOR_AddEvent(uint8_t EventorChannelSel);
void MS02_EVENTOR_IoEvtEnable(uint8_t EventorChannelSel);
void MS02_EVENTOR_IoEvtDisable(uint8_t EventorChannelSel);
void MS02_EVENTOR_UserEvtEnable(uint8_t EventorChannelSel);
void MS02_EVENTOR_UserEvtDisable(uint8_t EventorChannelSel);
// void MS02_EVENTOR_ServiceLoop(uint8_t EventorChannelSel);
void MS02_EVENTOR_ISR_Service(uint8_t EventorChannelSel);


#ifdef __cplusplus
}
#endif

#endif
