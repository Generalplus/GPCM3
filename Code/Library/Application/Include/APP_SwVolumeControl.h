/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   APP_SwVolumeControl.h
 * @Version:
 *   V1.0.2
 * @Date:
 *   October 7, 2020
 * @Abstract:
 *
 **************************************************************************************************/
#ifndef _APP_SW_VOLUME_CONTROL_H_
#define _APP_SW_VOLUME_CONTROL_H_


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include <stdint.h>


/*---------------------------------------------------------------------------------------
 * User Definition Area
 *---------------------------------------------------------------------------------------*/
#define    VOL_CONTROL_NUM    (4)


/*---------------------------------------------------------------------------------------
 * Definition Area
 *---------------------------------------------------------------------------------------*/
#define    MAX_VOL_GAIN       (15)
#define    MIN_VOL_GAIN       (0)


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C"{
#endif

void APP_SwVolCtrl_Init(void);
void APP_SwVolCtrl_SetVolGain(int32_t VolCtrlChannel, int32_t VolGain);
void APP_SwVolCtrl_VolProcess(int32_t VolCtrlChannel, int16_t *PcmBuffer, uint16_t DataLen);


#ifdef __cplusplus
}
#endif

#endif
