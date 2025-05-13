/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:     
 *   SACM_DVRIMA_API.h
 * @Version:  
 *   V1.0.3
 * @Date:     
 *   April 16, 2025
 * @Abstract: 
 *   USER DO NOT MODIFY
 **************************************************************************************************/
#ifndef _SACM_DVRIMA_API_H_
#define _SACM_DVRIMA_API_H_


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
#define DVRIMA_PLAY_FLAG                        (0x0001)
#define DVRIMA_PAUSE_FLAG                       (0x0004)
#define	DVRIMA_AUTO_RAMP_DOWN_ENABLE_FLAG       (0x0008)
#define DVRIMA_AUTO_RAMP_UP_ENABLE_FLAG         (0x0010)
#define DVRIMA_ENABLE_DAC_CH0_FLAG              (0x0020)
#define DVRIMA_ENABLE_DAC_CH1_FLAG              (0x0040)
#define	DVRIMA_BUF_EVEN_FLAG                    (0x0080)
#define DVRIMA_DECODE_WORK_FLAG                 (0x0100)
#define DVRIMA_DECODE_END_FLAG                  (0x0200)
#define DVRIMA_ISR_SERVICE_ENABLE_FLAG          (0x0400)
#define DVRIMA_STOP_FLAG                        (0x0800)
#define DVRIMA_PLAY_LAST_FRAME_FLAG             (0x1000)
#define DVRIMA_CPU_OVERLOAD_FLAG                (0x2000)


/*
 * DAC Channel definition for the 2nd parameter of API:SACM_DVRIMA_Play and SACM_DVRIMA_CH2_Play 
 */
#define DVRIMA_DAC_CH0                          (0x01)
#define DVRIMA_DAC_CH1                          (0x02)

/*
 * DAC RampUp/RampDown definition for the 3nd parameter of API:SACM_DVRIMA_Play and SACM_DVRIMA_CH2_Play
 */
#define DVRIMA_AUTO_RAMP_DISABLE                (0x00)
#define DVRIMA_AUTO_RAMP_UP                     (0x01)
#define DVRIMA_AUTO_RAMP_DOWN                   (0x02)

/*
 * DVRIMA Library Constant definition
 */
#define DVRIMA_FRAME_SIZE                       (320)
#define	DVRIMA_KERNEL_RAM_SIZE                  (380)     // Unit: byte
#define	DVRIMA_TEMP_BUFFER_SIZE                 (640)     // Unit: byte


/*---------------------------------------------------------------------------------------
 * Structure Definition Area
 *---------------------------------------------------------------------------------------*/ 
typedef struct _SACM_DVRIMA_TEMP_RAM
{		
  uint8_t DVRIMATempBuffer[DVRIMA_TEMP_BUFFER_SIZE];
} SACM_DVRIMA_TEMP_RAM;

typedef struct _SACM_DVRIMA_WORKING_RAM
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
  int16_t *DVRIMADecodeBufPtr;
	int16_t *PcmBufPtr;	  
	SACM_DVRIMA_TEMP_RAM *DVRIMATempBufPtr;
  int16_t *PcmBufStartAddress;	
	uint8_t DVRIMAKernelRam[DVRIMA_KERNEL_RAM_SIZE];
} SACM_DVRIMA_WORKING_RAM;

typedef struct _SACM_DVRIMA_PCM_BUFFER
{		
	int16_t DVRIMAPcmBuffer[2 * DVRIMA_FRAME_SIZE];	  
} SACM_DVRIMA_PCM_BUFFER;


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C"{
#endif
	
void SACM_DVRIMA_Initial(SACM_DVRIMA_WORKING_RAM *DVRIMAWorkingRam, SACM_DVRIMA_TEMP_RAM *pDVRIMATempBuffer, void *DVRIMAPcmBuffer);
void SACM_DVRIMA_Play(int16_t *SpeechDataStartAddr, uint8_t DacChannelSel, uint8_t AutoRampUpDownCtrl);
void SACM_DVRIMA_Play_Con(int16_t *SpeechDataStartAddr, uint8_t DacChannelNo, uint8_t AutoRampUpDownCtrl);
uint16_t SACM_DVRIMA_Check_Con(void);
void SACM_DVRIMA_Stop(void);
uint16_t SACM_DVRIMA_GetStatus(void);
void SACM_DVRIMA_ServiceLoop(void);
void SACM_DVRIMA_Pause(void);
void SACM_DVRIMA_Resume(void);
uint32_t SACM_DVRIMA_CheckCpuOverload(void);
void SACM_DVRIMA_ClearCpuOverload(void);
void SACM_DVRIMA_DmaIsrService(void);
uint32_t SACM_DVRIMA_DecodeProcess(int16_t *DstBufAddr, int16_t *SrcBufAddr);
	

#ifdef __cplusplus
}
#endif

#endif
