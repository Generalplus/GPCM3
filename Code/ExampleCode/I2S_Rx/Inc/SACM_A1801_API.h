/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:     
 *   SACM_A1801_API.h
 * @Version:  
 *   V1.0.3
 * @Date:     
 *   October 14, 2021
 * @Abstract: 
 *   USER DO NOT MODIFY
 **************************************************************************************************/
#ifndef _SACM_A1801_API_H_
#define _SACM_A1801_API_H_


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
#define A1801_PLAY_FLAG                        (0x0001)
#define A1801_PAUSE_FLAG                       (0x0004)
#define	A1801_AUTO_RAMP_DOWN_ENABLE_FLAG       (0x0008)
#define A1801_AUTO_RAMP_UP_ENABLE_FLAG         (0x0010)
#define A1801_ENABLE_DAC_CH0_FLAG              (0x0020)
#define A1801_ENABLE_DAC_CH1_FLAG              (0x0040)
#define	A1801_BUF_EVEN_FLAG                    (0x0080)
#define A1801_DECODE_WORK_FLAG                 (0x0100)
#define A1801_DECODE_END_FLAG                  (0x0200)
#define A1801_ISR_SERVICE_ENABLE_FLAG          (0x0400)
#define A1801_STOP_FLAG                        (0x0800)
#define A1801_PLAY_LAST_FRAME_FLAG             (0x1000)
#define A1801_CPU_OVERLOAD_FLAG                (0x2000)


/*
 * DAC Channel definition for the 2nd parameter of API:SACM_A1801_Play and SACM_A1801_CH2_Play 
 */
#define A1801_DAC_CH0                          (0x01)
#define A1801_DAC_CH1                          (0x02)

/*
 * DAC RampUp/RampDown definition for the 3nd parameter of API:SACM_A1801_Play and SACM_A1801_CH2_Play
 */
#define A1801_AUTO_RAMP_DISABLE                (0x00)
#define A1801_AUTO_RAMP_UP                     (0x01)
#define A1801_AUTO_RAMP_DOWN                   (0x02)

/*
 * A1801 Library Constant definition
 */
#define A1801_FRAME_SIZE                       (320)
#define	A1801_KERNEL_RAM_SIZE                  (380)     // Unit: byte
#define	A1801_TEMP_BUFFER_SIZE                 (640)     // Unit: byte


/*---------------------------------------------------------------------------------------
 * Structure Definition Area
 *---------------------------------------------------------------------------------------*/ 
typedef struct _SACM_A1801_TEMP_RAM
{		
  uint8_t A1801TempBuffer[A1801_TEMP_BUFFER_SIZE];
} SACM_A1801_TEMP_RAM;

typedef struct _SACM_A1801_WORKING_RAM
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
  int16_t *A1801DecodeBufPtr;
	int16_t *PcmBufPtr;	  
	SACM_A1801_TEMP_RAM *A1801TempBufPtr;
  int16_t *PcmBufStartAddress;	
	uint8_t A1801KernelRam[A1801_KERNEL_RAM_SIZE];
} SACM_A1801_WORKING_RAM;

typedef struct _SACM_A1801_PCM_BUFFER
{		
	int16_t A1801PcmBuffer[2 * A1801_FRAME_SIZE];	  
} SACM_A1801_PCM_BUFFER;


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C"{
#endif
	
void SACM_A1801_Initial(SACM_A1801_WORKING_RAM *A1801WorkingRam, SACM_A1801_TEMP_RAM *pA1801TempBuffer, void *A1801PcmBuffer);
void SACM_A1801_Play(int16_t *SpeechDataStartAddr, uint8_t DacChannelSel, uint8_t AutoRampUpDownCtrl);
void SACM_A1801_Play_Con(int16_t *SpeechDataStartAddr, uint8_t DacChannelNo, uint8_t AutoRampUpDownCtrl);
uint16_t SACM_A1801_Check_Con(void);
void SACM_A1801_Stop(void);
uint16_t SACM_A1801_GetStatus(void);
void SACM_A1801_ServiceLoop(void);
void SACM_A1801_Pause(void);
void SACM_A1801_Resume(void);
uint32_t SACM_A1801_CheckCpuOverload(void);
void SACM_A1801_ClearCpuOverload(void);
void SACM_A1801_DmaIsrService(void);
uint32_t SACM_A1801_DecodeProcess(int16_t *DstBufAddr, int16_t *SrcBufAddr);
	

#ifdef __cplusplus
}
#endif

#endif
