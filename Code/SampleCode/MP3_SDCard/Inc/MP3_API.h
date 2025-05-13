/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:     
 *   MP3_API.h
 * @Version:  
 *   V1.0.2
 * @Date:     
 *   April 9, 2020
 * @Abstract: 
 *   USER DO NOT MODIFY
 **************************************************************************************************/
#ifndef _MP3_API_H_
#define _MP3_API_H_


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include <stdint.h>


/*---------------------------------------------------------------------------------------
 * Constant Definition Area
 *---------------------------------------------------------------------------------------*/

/*
 * SACM status definition
 */
#define MP3_PLAY_FLAG                       (0x0001)
#define MP3_PAUSE_FLAG                      (0x0004)
#define	MP3_AUTO_RAMP_DOWN_ENABLE_FLAG      (0x0008)
#define MP3_AUTO_RAMP_UP_ENABLE_FLAG        (0x0010)
#define MP3_ENABLE_DAC_CH0_FLAG             (0x0020)
#define MP3_ENABLE_DAC_CH1_FLAG             (0x0040)
#define MP3_DECODE_END_FLAG                 (0x0200)
#define MP3_ISR_SERVICE_ENABLE_FLAG         (0x0400)
#define MP3_PLAY_END_FLAG                   (0x0800)
#define MP3_PLAY_LAST_FRAME_FLAG            (0x1000)
#define MP3_CPU_OVERLOAD_FLAG               (0x2000)

/*
 * DAC Channel definition for the 2nd parameter of API:MP3_Play
 */
#define MP3_DAC_CH0                         (0x01)
#define MP3_DAC_CH1                         (0x02)

/*
 * DAC RampUp/RampDown definition for the 3nd parameter of API:MP3_Play
 */
#define MP3_AUTO_RAMP_DISABLE               (0x00)
#define MP3_AUTO_RAMP_UP                    (0x01)
#define MP3_AUTO_RAMP_DOWN                  (0x02)

/*
 * MP3 Library Constant definition
 */ 
#define MP3_KERNEL_RAM_SIZE                 (4440)
#define MP3_TEMP_RAM_SIZE                   (2728)
#define MP3_PCM_RING_NUM		                (36)                     // 2, 4, 6, 8, ...., 36, ...
#define MP3_PCM_BUF_LEN			                (32) 


/*---------------------------------------------------------------------------------------
 * Structure Definition Area
 *---------------------------------------------------------------------------------------*/ 
typedef struct _MP3_LIB_WORKING_RAM
{			
	uint8_t Mp3KernelRam[MP3_KERNEL_RAM_SIZE];
	uint8_t Mp3TempRam[MP3_TEMP_RAM_SIZE];
	int16_t	PcmRingBuffer[MP3_PCM_RING_NUM][MP3_PCM_BUF_LEN];
	uint16_t VolumeGain;
	uint16_t NextStatus;
	uint32_t DataLen;                                                  // Unit: byte
  uint32_t NextDataLen;                                              // Unit: byte	
	uint32_t DecodeCount;                                              // Unit: byte
	uint8_t RdPcmRingIdx;
	uint8_t WrPcmRingIdx;
	uint8_t *Mp3DataStartAddr;
  uint8_t *NextMp3DataStartAddr;
	uint16_t SilenceSubFrameCount;
	uint16_t SilenceSubFrameTH;
} MP3_LIB_WORKING_RAM;


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C"{
#endif

void MP3_Initial(MP3_LIB_WORKING_RAM *Mp3WorkingRam);
void MP3_Play(uint8_t *Mp3StartAddr, uint32_t Mp3Length, uint8_t DacChannelSel, uint8_t AutoRampUpDownCtrl);
void MP3_Play_Con(uint8_t *Mp3StartAddr, uint32_t Mp3Length, uint8_t DacChannelNo, uint8_t AutoRampUpDownCtrl);
uint16_t MP3_Check_Con(void);
void MP3_Stop(void);
uint16_t MP3_GetStatus(void);
void MP3_ServiceLoop(void);
void MP3_Pause(void);
void MP3_Resume(void);
void MP3_DmaIsrService(void);
void MP3_SetSilenceSubFrameTH(uint16_t);
uint32_t MP3_CheckCpuOverload(void);
void MP3_ClearCpuOverload(void);	


#ifdef __cplusplus
}
#endif

#endif
