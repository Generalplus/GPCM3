/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:     
 *   SACM_DVR1801_API.h
 * @Version:  
 *   V1.0.4
 * @Date:     
 *   October 18, 2021
 * @Abstract: 
 *   USER DO NOT MODIFY
 **************************************************************************************************/
#ifndef _SACM_DVR1801_API_H_
#define _SACM_DVR1801_API_H_


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
#define DVR1801_PLAY_FLAG                        (0x0001)
#define DVR1801_CODEC_MODE                       (0x0002)  // Set bit: Enocde Mode, Clear bit: Decode Mode
#define DVR1801_PAUSE_FLAG                       (0x0004)
#define	DVR1801_AUTO_RAMP_DOWN_ENABLE_FLAG       (0x0008)
#define DVR1801_AUTO_RAMP_UP_ENABLE_FLAG         (0x0010)
#define DVR1801_ENABLE_DAC_CH0_FLAG              (0x0020)
#define DVR1801_ENABLE_DAC_CH1_FLAG              (0x0040)
#define	DVR1801_BUF_EVEN_FLAG                    (0x0080)
#define DVR1801_CODEC_WORK_FLAG                  (0x0100)
#define DVR1801_DECODE_END_FLAG                  (0x0200)
#define DVR1801_ISR_SERVICE_ENABLE_FLAG          (0x0400)
#define DVR1801_STOP_FLAG                        (0x0800)
#define DVR1801_PLAY_LAST_FRAME_FLAG             (0x1000)
#define DVR1801_CPU_OVERLOAD_FLAG                (0x2000)

/*
 * DAC Channel definition for the 2nd parameter of API:SACM_DVR1801_Play 
 */
#define DVR1801_DAC_CH0                          (0x01)
#define DVR1801_DAC_CH1                          (0x02)

/*
 * DAC RampUp/RampDown definition for the 3nd parameter of API:SACM_DVR1801_Play 
 */
#define DVR1801_AUTO_RAMP_DISABLE                (0x00)
#define DVR1801_AUTO_RAMP_UP                     (0x01)
#define DVR1801_AUTO_RAMP_DOWN                   (0x02)

/*
 * Record bit rate definition for the 2nd parameter of API:SACM_DVR1801_Rec 
 */
#define DVR1801_RECORD_BITRATE_7200              (0x00)
#define DVR1801_RECORD_BITRATE_9600              (0x01)
#define DVR1801_RECORD_BITRATE_12000             (0x02)
#define DVR1801_RECORD_BITRATE_14400             (0x03)
#define DVR1801_RECORD_BITRATE_16000             (0x04)
#define DVR1801_RECORD_BITRATE_20000             (0x05)
#define DVR1801_RECORD_BITRATE_24000             (0x06)
#define DVR1801_RECORD_BITRATE_32000             (0x07)
#define DVR1801_RECORD_BITRATE_40000             (0x08)
#define DVR1801_RECORD_BITRATE_44000             (0x09)

/*
 * Dvr1801 Library Constant definition
 */
#define DVR1801_FRAME_SIZE                       (320)
#define	DVR_KERNEL_RAM_SIZE                      (668)     // Unit: byte
#define	DVR_TEMP_BUFFER_SIZE                     (640)     // Unit: byte


/*---------------------------------------------------------------------------------------
 * Structure Definition Area
 *---------------------------------------------------------------------------------------*/ 
typedef struct _SACM_DVR1801_TEMP_RAM
{		
  uint8_t Dvr1801TempBuffer[DVR_TEMP_BUFFER_SIZE];
} SACM_DVR1801_TEMP_RAM;

typedef struct _SACM_DVR1801_WORKING_RAM
{		
	uint16_t SampleRate;	
	uint16_t BitRate;
	uint16_t DvrVolumeGain;
	uint16_t HalfWordsPerFrame;
	uint32_t DataLen;                                        // Unit: halfword
	uint32_t DecodeCount;                                    // Unit: halfword	
	int16_t *SpeechDataAddr;  
  uint16_t DvrNextStatus;
  int16_t *NextSpeechDataAddr;	
  int16_t *DvrCodecBufPtr;
	int16_t *PcmBufPtr;
	SACM_DVR1801_TEMP_RAM *DvrTempBufPtr;
	int16_t *PcmBufStartAddress;	
	uint8_t DvrKernelRam[DVR_KERNEL_RAM_SIZE];
} SACM_DVR1801_WORKING_RAM;

typedef struct _SACM_DVR1801_PCM_BUFFER
{		
	int16_t DvrPcmBuffer[2 * DVR1801_FRAME_SIZE];	  
} SACM_DVR1801_PCM_BUFFER;


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C"{
#endif
	
void SACM_DVR1801_Initial(SACM_DVR1801_WORKING_RAM *Dvr1801WorkingRam, SACM_DVR1801_TEMP_RAM *Dvr1801TempBuffer, void *DvrPcmBuffer);
void SACM_DVR1801_Play(int16_t *SpeechDataStartAddr, uint8_t DacChannelSel, uint8_t AutoRampUpDownCtrl);
void SACM_DVR1801_Play_Con(int16_t *SpeechDataStartAddr, uint8_t DacChannelNo, uint8_t AutoRampUpDownCtrl);
uint16_t SACM_DVR1801_Check_Con(void);
void SACM_DVR1801_Stop(void);
void SACM_DVR1801_Rec(int16_t *EncodeDataStartAddr, uint8_t BitRateIndex);
uint16_t SACM_DVR1801_GetStatus(void);
void SACM_DVR1801_ServiceLoop(void);
void SACM_DVR1801_Pause(void);
void SACM_DVR1801_Resume(void);
uint32_t SACM_DVR1801_CheckCpuOverload(void);
void SACM_DVR1801_ClearCpuOverload(void);	
void SACM_DVR1801_DmaIsrService(void);
uint32_t SACM_DVR1801_DecodeProcess(int16_t *DstBufAddr, int16_t *SrcBufAddr);
void SACM_DVR1801_EncodeProcess(int16_t *DstBufAddr, int16_t *SrcBufAddr);
	

#ifdef __cplusplus
}
#endif

#endif
