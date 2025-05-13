/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:     
 *   SACM_A3400Pro_API.h
 * @Version:  
 *   V1.0.2
 * @Date:     
 *   September 10, 2021
 * @Abstract: 
 *   USER DO NOT MODIFY
 **************************************************************************************************/
#ifndef _SACM_A3400Pro_API_H_
#define _SACM_A3400Pro_API_H_


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------------------
 * Constant Definition Area
 *---------------------------------------------------------------------------------------*/

/*
 * SACM status definition
 */
#define A3400PRO_PLAY_FLAG                        (0x0001)
#define A3400PRO_PAUSE_FLAG                       (0x0004)
#define	A3400PRO_AUTO_RAMP_DOWN_ENABLE_FLAG       (0x0008)
#define A3400PRO_AUTO_RAMP_UP_ENABLE_FLAG         (0x0010)
#define A3400PRO_ENABLE_DAC_CH0_FLAG              (0x0020)
#define A3400PRO_ENABLE_DAC_CH1_FLAG              (0x0040)
#define	A3400PRO_BUF_EVEN_FLAG                    (0x0080)
#define A3400PRO_DECODE_WORK_FLAG                 (0x0100)
#define A3400PRO_DECODE_END_FLAG                  (0x0200)
#define A3400PRO_ISR_SERVICE_ENABLE_FLAG          (0x0400)
#define A3400PRO_STOP_FLAG                        (0x0800)
#define A3400PRO_PLAY_LAST_FRAME_FLAG             (0x1000)
#define A3400PRO_CPU_OVERLOAD_FLAG                (0x2000)

/*
 * DAC Channel definition for the 2nd parameter of API:SACM_A3400PRO_Play and SACM_A3400PRO_CH2_Play 
 */
#define A3400PRO_DAC_CH0                          (0x01)
#define A3400PRO_DAC_CH1                          (0x02)

/*
 * DAC RampUp/RampDown definition for the 3nd parameter of API:SACM_A3400PRO_Play and SACM_A3400PRO_CH2_Play
 */
#define A3400PRO_AUTO_RAMP_DISABLE                (0x00)
#define A3400PRO_AUTO_RAMP_UP                     (0x01)
#define A3400PRO_AUTO_RAMP_DOWN                   (0x02)

/*
 * A3400Pro Library Constant definition
 */
#define	A3400PRO_KERNEL_RAM_SIZE                  (28)     // Unit: byte


/*---------------------------------------------------------------------------------------
 * Structure Definition Area
 *---------------------------------------------------------------------------------------*/ 

typedef struct _SACM_A3400PRO_WORKING_RAM
{		
	uint16_t 	FrameSize;
	uint32_t 	DecodeCount;                                    // Unit: byte
	uint8_t  *SpeechDataAddr;  
	uint8_t  *EventDataAddr;  
  uint16_t 	NextStatus;
  uint8_t   *NextSpeechDataAddr;	
  int16_t  *A3400ProDataBuffer;
  int16_t  *A3400ProDecodeBufPtr;
	int16_t  *PcmBufPtr;
  uint8_t 	A3400ProKernelRam[A3400PRO_KERNEL_RAM_SIZE];
} SACM_A3400PRO_WORKING_RAM;


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C"{
#endif
	
void SACM_A3400Pro_Initial(SACM_A3400PRO_WORKING_RAM *A3400ProWorkingRam, int16_t *A3400ProPcmBuf, int16_t lFrameSize);
void SACM_A3400Pro_Play(int16_t *SpeechDataStartAddr, uint8_t DacChannelSel, uint8_t AutoRampUpDownCtrl);
void SACM_A3400Pro_Play_Con(int16_t *SpeechDataStartAddr, uint8_t DacChannelNo, uint8_t AutoRampUpDownCtrl);
uint16_t SACM_A3400Pro_Check_Con(void);
void SACM_A3400Pro_Stop(void);
uint16_t SACM_A3400Pro_GetStatus(void);
void SACM_A3400Pro_ServiceLoop(void);
void SACM_A3400Pro_Pause(void);
void SACM_A3400Pro_Resume(void);
void SACM_A3400Pro_DmaIsrService(void);
uint32_t SACM_A3400Pro_DecodeProcess(int16_t *DstBufAddr, int16_t *SrcBufAddr);	
uint32_t SACM_A3400Pro_CheckCpuOverload(void);
void SACM_A3400Pro_ClearCpuOverload(void);
	
#ifdef __cplusplus
}
#endif

#endif
