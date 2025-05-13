/**************************************************************************************************
 * Copyright(c) 2021 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   APP_SwMixer.h
 * @Version:
 *   V1.0.0
 * @Date:
 *   March 27, 2021
 * @Abstract:
 *
 **************************************************************************************************/
#ifndef _APP_SW_MIXER_H_
#define _APP_SW_MIXER_H_


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include <stdint.h>


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C"{
#endif

void APP_SwMixer_2ch(int16_t *PcmInBuffer1, int16_t *PcmInBuffer2, int16_t *DstBufAddr, uint16_t DataLen);
void APP_SwMixer_3ch(int16_t *PcmInBuffer1, int16_t *PcmInBuffer2, int16_t *PcmInBuffer3, int16_t *DstBufAddr, uint16_t DataLen);	


#ifdef __cplusplus
}
#endif

#endif
