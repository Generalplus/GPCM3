/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:     
 *   GP_Eventor_User.h       
 * @Version: 
 *   V0.9.0
 * @Date: 
 *   December 22, 2020
 * @Abstract: 
 *
 **************************************************************************************************/
#ifndef _GP_EVENTOR_USER_H_
#define _GP_EVENTOR_USER_H_


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "GP_Eventor.h"


/*---------------------------------------------------------------------------------------
 * User Definition Area
 *---------------------------------------------------------------------------------------*/
#define EVENTOR_CH0_IO_NUM                (3)                                       // IO number for IO_Event. The 4th parameter of GP_EVENTOR_Initial()
#define EVENTOR_CH0_SW_PWM_LEVEL          (128)                                     // S/W PWM level (32/64/128/256). The 5th parameter of GP_EVENTOR_Initial()
#define EVENTOR_CH0_SW_PWM_FREQ           (60)                                      // S/W PWM frequency (Unit: Hz)

#define EVENTOR_CH1_IO_NUM                (3)                                       // IO number for IO_Event. The 4th parameter of GP_EVENTOR_Initial()
#define EVENTOR_CH1_SW_PWM_LEVEL          (128)                                     // S/W PWM level (32/64/128/256). The 5th parameter of GP_EVENTOR_Initial()
#define EVENTOR_CH1_SW_PWM_FREQ           (60)                                      // S/W PWM frequency (Unit: Hz)


/*---------------------------------------------------------------------------------------
 * Definition Area
 *---------------------------------------------------------------------------------------*/
#define EVENTOR_CH0_SW_PWM_TIMER_SETTING   (EVENTOR_CH0_SW_PWM_FREQ * EVENTOR_CH0_SW_PWM_LEVEL) // Timer interrupt frequency setting for GP_EVENTOR_ISR_Service() 
#define EVENTOR_CH1_SW_PWM_TIMER_SETTING   (EVENTOR_CH1_SW_PWM_FREQ * EVENTOR_CH1_SW_PWM_LEVEL) // Timer interrupt frequency setting for GP_EVENTOR_ISR_Service() 


/*---------------------------------------------------------------------------------------
 * Structure Definition Area
 *---------------------------------------------------------------------------------------*/ 
typedef struct _GP_EVENTOR_CH0_SWPWM_WORKING_RAM
{		
	int16_t SwPwmWorkingRam[EVENTOR_CH0_IO_NUM];
} GP_EVENTOR_CH0_SWPWM_WORKING_RAM;	

typedef struct _GP_EVENTOR_CH1_SWPWM_WORKING_RAM
{		
	int16_t SwPwmWorkingRam[EVENTOR_CH1_IO_NUM];
} GP_EVENTOR_CH1_SWPWM_WORKING_RAM;	


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C"{
#endif

void GP_EVENTOR_CB_Init(const uint8_t EventorChannelSel, const GP_EVENTOR_WORKING_RAM *GpEvtWorkingRam);
void GP_EVENTOR_CB_IoEvtStart(const uint8_t EventorChannelSel, const GP_EVENTOR_WORKING_RAM *GpEvtWorkingRam);
void GP_EVENTOR_CB_IoEvtEnd(const uint8_t EventorChannelSel, const GP_EVENTOR_WORKING_RAM *GpEvtWorkingRam);
void GP_EVENTOR_CB_GetData(const uint8_t EventorChannelSel, const GP_EVENTOR_WORKING_RAM *GpEvtWorkingRam, uint8_t *DstBufAddr, uint8_t *SrcDataAddr, uint16_t DataLen);
void GP_EVENTOR_CB_UserEvtProcess(const uint8_t EventorChannelSel, const GP_EVENTOR_WORKING_RAM *GpEvtWorkingRam, uint16_t EventData);
void GP_EVENTOR_CB_IoEvt_ISR_Service(const uint8_t EventorChannelSel, const GP_EVENTOR_WORKING_RAM *GpEvtWorkingRam);


#ifdef __cplusplus
}
#endif

#endif
