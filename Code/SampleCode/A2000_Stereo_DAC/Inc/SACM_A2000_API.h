/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   SACM_A2000_API.h
 * @Version:
 *   V1.0.0
 * @Date:
 *   August 05, 2024
 * @Abstract:
 *   USER DO NOT MODIFY
 **************************************************************************************************/
#ifndef _SACM_A2000_API_H_
#define _SACM_A2000_API_H_


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
#define A2000_PLAY_FLAG                        (0x0001)
#define A2000_PAUSE_FLAG                       (0x0004)
#define	A2000_AUTO_RAMP_DOWN_ENABLE_FLAG       (0x0008)
#define A2000_AUTO_RAMP_UP_ENABLE_FLAG         (0x0010)
#define A2000_ENABLE_DAC_CH0_FLAG              (0x0020)
#define A2000_ENABLE_DAC_CH1_FLAG              (0x0040)
#define	A2000_BUF_EVEN_FLAG                    (0x0080)
#define A2000_DECODE_WORK_FLAG                 (0x0100)
#define A2000_DECODE_END_FLAG                  (0x0200)
#define A2000_ISR_SERVICE_ENABLE_FLAG          (0x0400)
#define A2000_STOP_FLAG                        (0x0800)
#define A2000_PLAY_LAST_FRAME_FLAG             (0x1000)
#define A2000_CPU_OVERLOAD_FLAG                (0x2000)


/*
 * DAC Channel definition for the 2nd parameter of API:SACM_A2000_Play and SACM_A2000_CH2_Play
 */
#define A2000_DAC_CH0                          (0x01)
#define A2000_DAC_CH1                          (0x02)

/*
 * DAC RampUp/RampDown definition for the 3nd parameter of API:SACM_A2000_Play and SACM_A2000_CH2_Play
 */
#define A2000_AUTO_RAMP_DISABLE                (0x00)
#define A2000_AUTO_RAMP_UP                     (0x01)
#define A2000_AUTO_RAMP_DOWN                   (0x02)

/*
 * A2000 Library Constant definition
 */
#define A2000_FRAME_SIZE                       (640) //(320)
#define	A2000_KERNEL_RAM_SIZE                  (700) //(380)     // Unit: byte
#define	A2000_TEMP_BUFFER_SIZE                 (1280) //(640)     // Unit: byte


/*---------------------------------------------------------------------------------------
 * Structure Definition Area
 *---------------------------------------------------------------------------------------*/
typedef struct _SACM_A2000_TEMP_RAM
{
  uint8_t A2000TempBuffer[A2000_TEMP_BUFFER_SIZE];
} SACM_A2000_TEMP_RAM;

typedef struct _SACM_A2000_WORKING_RAM
{
	uint16_t SampleRate;
	uint16_t BitRate;
	uint16_t VolumeGain;
	uint16_t HalfWordsPerFrame;
	uint32_t DataLen;                                        // Unit: halfword
	uint32_t DecodeCount;                                    // Unit: halfword
	int16_t *SpeechDataAddr;
  uint16_t NextStatus;
  int16_t *NextSpeechDataAddr;
  int16_t *A2000DecodeBufPtr;
	int16_t *PcmBufPtr;
	SACM_A2000_TEMP_RAM *A2000TempBufPtr;
  int16_t *PcmBufStartAddress;
	uint8_t A2000KernelRam[A2000_KERNEL_RAM_SIZE];
} SACM_A2000_WORKING_RAM;

typedef struct _SACM_A2000_PCM_BUFFER
{
	int16_t A2000PcmBuffer[2 * A2000_FRAME_SIZE];
} SACM_A2000_PCM_BUFFER;


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C"{
#endif

void SACM_A2000_Initial(SACM_A2000_WORKING_RAM *A2000WorkingRam, SACM_A2000_TEMP_RAM *pA2000TempBuffer, void *A2000PcmBuffer);
void SACM_A2000_Play(int16_t *SpeechDataStartAddr, uint8_t DacChannelSel, uint8_t AutoRampUpDownCtrl);
void SACM_A2000_Play_Con(int16_t *SpeechDataStartAddr, uint8_t DacChannelNo, uint8_t AutoRampUpDownCtrl);
uint16_t SACM_A2000_Check_Con(void);
void SACM_A2000_Stop(void);
uint16_t SACM_A2000_GetStatus(void);
void SACM_A2000_ServiceLoop(void);
void SACM_A2000_Pause(void);
void SACM_A2000_Resume(void);
uint32_t SACM_A2000_CheckCpuOverload(void);
void SACM_A2000_ClearCpuOverload(void);
void SACM_A2000_DmaIsrService(void);
uint32_t SACM_A2000_DecodeProcess(int16_t *DstBufAddr, int16_t *SrcBufAddr);


#ifdef __cplusplus
}
#endif

#endif
