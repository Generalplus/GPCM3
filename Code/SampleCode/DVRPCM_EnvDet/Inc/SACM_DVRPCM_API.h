/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:     
 *   SACM_DVRPCM_API.h
 * @Version:  
 *   V1.0.0
 * @Date:     
 *   December 2, 2019
 * @Abstract: 
 *   USER DO NOT MODIFY
 **************************************************************************************************/
#ifndef _SACM_DVRPCM_API_H_
#define _SACM_DVRPCM_API_H_


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
#define DVRPCM_PLAY_FLAG                        (0x0001)
#define DVRPCM_CODEC_MODE                       (0x0002)  // Set bit: Enocde Mode, Clear bit: Decode Mode
#define DVRPCM_PAUSE_FLAG                       (0x0004)
#define	DVRPCM_AUTO_RAMP_DOWN_ENABLE_FLAG       (0x0008)
#define DVRPCM_AUTO_RAMP_UP_ENABLE_FLAG         (0x0010)
#define DVRPCM_ENABLE_DAC_CH0_FLAG              (0x0020)
#define DVRPCM_ENABLE_DAC_CH1_FLAG              (0x0040)
#define	DVRPCM_BUF_EVEN_FLAG                    (0x0080)
#define DVRPCM_DECODE_WORK_FLAG                 (0x0100)
#define DVRPCM_DECODE_END_FLAG                  (0x0200)
#define DVRPCM_ISR_SERVICE_ENABLE_FLAG          (0x0400)
#define DVRPCM_STOP_FLAG                        (0x0800)

/*
 * DAC Channel definition for the 2nd parameter of API:SACM_DVRPCM_Play 
 */
#define DVRPCM_DAC_CH0                          (0x01)
#define DVRPCM_DAC_CH1                          (0x02)

/*
 * DAC RampUp/RampDown definition for the 3nd parameter of API:SACM_DVRPCM_Play 
 */
#define DVRPCM_AUTO_RAMP_DISABLE                (0x00)
#define DVRPCM_AUTO_RAMP_UP                     (0x01)
#define DVRPCM_AUTO_RAMP_DOWN                   (0x02)



/*---------------------------------------------------------------------------------------
 * Structure Definition Area
 *---------------------------------------------------------------------------------------*/ 
typedef struct _SACM_DVRPCM_WORKING_RAM
{		
	uint16_t FrameSize;
	uint16_t DvrVolumeGain;
	uint16_t HalfWordsPerFrame;
	uint32_t MaxRecordLen;																		// Unit: halfword
	uint32_t DataLen;                                        	// Unit: halfword
	uint32_t DecodeCount;                                    	// Unit: halfword	
	int16_t *SpeechDataAddr;  
  uint16_t DvrNextStatus;
  int16_t *NextSpeechDataAddr;	
  int16_t *DvrCodecBufPtr;
	int16_t *PcmBufPtr;
  int16_t *DvrDataBufferPtr;
} SACM_DVRPCM_WORKING_RAM;


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C"{
#endif
	
void SACM_DVRPCM_Initial(SACM_DVRPCM_WORKING_RAM *DvrPcmWorkingRam, int16_t *DvrPcmDatabufRam, int16_t lFrameSize);
void SACM_DVRPCM_Rec(int16_t *EncodeDataStartAddr);
void SACM_DVRPCM_SetRecLength(uint32_t RecordLength);	
void SACM_DVRPCM_Play(int16_t *SpeechDataStartAddr, uint8_t DacChannelSel, uint8_t AutoRampUpDownCtrl);
void SACM_DVRPCM_Play_Con(int16_t *SpeechDataStartAddr, uint8_t DacChannelNo, uint8_t AutoRampUpDownCtrl);
void SACM_DVRPCM_Stop(void);
void SACM_DVRPCM_ServiceLoop(void);
void SACM_DVRPCM_Pause(void);
void SACM_DVRPCM_Resume(void);
void SACM_DVRPCM_DmaIsrService(void);
uint16_t SACM_DVRPCM_GetStatus(void);
uint16_t SACM_DVRPCM_Check_Con(void);

#ifdef __cplusplus
}
#endif

#endif
