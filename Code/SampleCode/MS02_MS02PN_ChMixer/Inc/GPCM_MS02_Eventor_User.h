/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   MS02_EVENTOR_User.h
 * @Version:
 *   V0.9.1
 * @Date:
 *   March 20, 2024
 * @Abstract:
 *   1. Modify for MS02 Event of the GPCMx series
 *
 **************************************************************************************************/
#ifndef _MS02_EVENTOR_USER_H_
#define _MS02_EVENTOR_USER_H_


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "GPCM_MS02_Eventor.h"


/*---------------------------------------------------------------------------------------
 * User Definition Area
 *---------------------------------------------------------------------------------------*/
#define MS02_EVENT_IO_NUM                (3)                                       // IO number for IO_Event. The 4th parameter of MS02_EVENTOR_Initial()
#define MS02_EVENT_SW_PWM_LEVEL          (128)                                     // S/W PWM level (32/64/128/256). The 5th parameter of MS02_EVENTOR_Initial()
#define MS02_EVENT_SW_PWM_FREQ           (60)                                      // S/W PWM frequency (Unit: Hz)


/*---------------------------------------------------------------------------------------
 * Definition Area
 *---------------------------------------------------------------------------------------*/
#define MS02_EVENT_SW_PWM_TIMER_SETTING   (MS02_EVENT_SW_PWM_FREQ * MS02_EVENT_SW_PWM_LEVEL) // Timer interrupt frequency setting for MS02_EVENTOR_ISR_Service()


/*---------------------------------------------------------------------------------------
 * Structure Definition Area
 *---------------------------------------------------------------------------------------*/
typedef struct _MS02_EVENTOR_CH2_SWPWM_WORKING_RAM
{
	int16_t SwPwmWorkingRam[MS02_EVENT_IO_NUM];
} MS02_EVENTOR_CH2_SWPWM_WORKING_RAM;


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C"{
#endif

void MS02_EVENTOR_CB_Init(const uint8_t EventorChannelSel, const MS02_EVENTOR_WORKING_RAM *MS02EvtWorkingRam);
void MS02_EVENTOR_CB_IoEvtStart(const uint8_t EventorChannelSel, const MS02_EVENTOR_WORKING_RAM *MS02EvtWorkingRam);
void MS02_EVENTOR_CB_IoEvtEnd(const uint8_t EventorChannelSel, const MS02_EVENTOR_WORKING_RAM *MS02EvtWorkingRam);
void MS02_EVENTOR_CB_UserEvtProcess(const uint8_t EventorChannelSel, const MS02_EVENTOR_WORKING_RAM *MS02EvtWorkingRam, uint16_t EventData);
void MS02_EVENTOR_CB_IoEvt_ISR_Service(const uint8_t EventorChannelSel, const MS02_EVENTOR_WORKING_RAM *MS02EvtWorkingRam);


#ifdef __cplusplus
}
#endif

#endif
