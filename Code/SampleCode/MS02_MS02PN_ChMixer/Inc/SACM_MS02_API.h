/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   SACM_MS02_API.h
 * @Version:
 *   V1.0.2
 * @Date:
 *   22th, January 2024
 * @Abstract:
 *   USER DO NOT MODIFY
 *   V0.9.7.1 Add the MS02OutBufCount & MS02_CPU_OVERLOAD_FLAG
 *   V1.0.1  Move the MS02InBuffer to outside from the SACM_MS02_WORKING_RAM
 *   V1.0.2  Add the SACM_MS02_OKON_NoteOFF() function.
 **************************************************************************************************/
#ifndef _SACM_MS02_API_H_
#define _SACM_MS02_API_H_


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include <stdint.h>
#include "MS02Dec.h"


/*---------------------------------------------------------------------------------------
 * Constant Definition Area
 *---------------------------------------------------------------------------------------*/
/*
 * SACM MS02 status definition
 */
#define MS02_PLAY_FLAG                        (0x0001)
#define MS02_PAUSE_FLAG                       (0x0004)
#define	MS02_AUTO_RAMP_DOWN_ENABLE_FLAG       (0x0008)
#define MS02_AUTO_RAMP_UP_ENABLE_FLAG         (0x0010)
#define MS02_ENABLE_DAC_CH0_FLAG              (0x0020)
#define MS02_ENABLE_DAC_CH1_FLAG              (0x0040)
#define	MS02_BUF_EVEN_FLAG                    (0x0080)
#define MS02_DECODE_WORK_FLAG                 (0x0100)
#define MS02_DECODE_END_FLAG                  (0x0200)
#define MS02_ISR_SERVICE_ENABLE_FLAG          (0x0400)
#define MS02_STOP_FLAG                        (0x0800)
#define MS02_CPU_OVERLOAD_FLAG                (0x1000)
// #define MS02_AUD_OUT_DAC_FLAG                 (0x2000)

/*
 * Timer_SR(Timer Sampling Rate for MS02) definition for the parameter of API:SACM_MS02_Initial
 */
#define MS02_Timer_8K                         (8000 * 4)  // Sample rate: 8000 x 4 = 32000(Hz), for H/W 4x upsampling
#define MS02_Timer_10K                        (10000 * 4) // Note: Timer_SR setting must be consistent with MS02_DAC S.R
#define MS02_Timer_12K                        (12000 * 4)
#define MS02_Timer_16K                        (16000 * 4)
#define MS02_Timer_20K                        (20000 * 4)
#define MS02_Timer_24K                        (24000 * 4)
#define MS02_Timer_28K                        (28000 * 4)
#define MS02_Timer_32K                        (32000 * 4)
#define MS02_Timer_36K                        (36000 * 4)
#define MS02_Timer_40K                        (40000 * 4)

/*
 * DAC Channel definition for the 2nd parameter of API:SACM_MS02_Play
 */
#define MS02_CH0                          		(0x01)
#define MS02_CH1                          		(0x02)

/*
 * DAC RampUp/RampDown definition for the 3nd parameter of API:SACM_MS02_Play
 */
#define MS02_AUTO_RAMP_DISABLE                (0x00)
#define MS02_AUTO_RAMP_UP                     (0x01)
#define MS02_AUTO_RAMP_DOWN                   (0x02)

/*
 * MS02 Output S.R definition for the parameter of  API:SACM_MS02_Play
 */
#define MS02_DAC_8K				                    0
#define MS02_DAC_10K			                    1
#define MS02_DAC_12K			                    2
#define MS02_DAC_16K			                    3
#define MS02_DAC_20K			                    4
#define MS02_DAC_24K			                    5
#define MS02_DAC_28K			                    6
#define MS02_DAC_32K			                    7
#define MS02_DAC_36K			                    8
#define MS02_DAC_40K			                    9

/*
 * MS02 Library Constant definition
 */
#define MS02_IN_BUFFER_SIZE										(160)
#define MS02_OUT_BUFFER_SIZE									(160)


/*---------------------------------------------------------------------------------------
 * Structure Definition Area
 *---------------------------------------------------------------------------------------*/
typedef struct _SACM_MS02_WORKING_RAM
{
	uint16_t SampleRate;
  uint8_t DacChannelNo;
	uint32_t DecodeCount;                                    // Unit: halfword
	int16_t *SpeechDataAddr;
	int16_t *LibDataAddr;
  uint16_t NextStatus;
  int16_t *NextSpeechDataAddr;
  int16_t *NextLibDataAddr;
	uint16_t NextSampleRate;
  int16_t *DecodeInBufPtr;
	int16_t *DecodeOutBuf1Ptr;
	int16_t *DecodeOutBuf2Ptr;
	// int16_t MS02InBuffer[MS02_IN_BUFFER_SIZE];
	// int16_t MS02OutBuffer[2 * MS02_OUT_BUFFER_SIZE];
	int16_t *MS02OutBuffer;
	uint8_t *MS02KernelRamPtr;
	uint8_t MS02OutBufCount;
	uint16_t MS02_IN_SIZE;
	uint16_t MS02_OUT_SIZE;
} SACM_MS02_WORKING_RAM;


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C"{
#endif

// void SACM_MS02_Initial(SACM_MS02_WORKING_RAM *MS02WorkingRam, int16_t *MS02PcmBufPtr, uint8_t *MS02KernelRAMPtr, uint8_t AudOutType, uint8_t DacChannelNo, uint32_t Timer_SR);
// void SACM_MS02_Initial(SACM_MS02_WORKING_RAM *MS02WorkingRam, int16_t *MS02InBufPtr, int16_t *MS02PcmBufPtr, uint8_t *MS02KernelRAMPtr, uint16_t BUF_IN_SIZE, uint16_t BUF_OUT_SIZE, uint8_t AudOutType, uint8_t DacChannelNo, uint32_t Timer_SR);
void SACM_MS02_Initial(SACM_MS02_WORKING_RAM *MS02WorkingRam, int16_t *MS02InBufPtr, int16_t *MS02PcmBufPtr, uint8_t *MS02KernelRAMPtr, uint16_t BUF_IN_SIZE, uint16_t BUF_OUT_SIZE, uint16_t DEC_MEM_SIZE, uint8_t AudOutType, uint8_t DacChannelNo, uint32_t Timer_SR);

void SACM_MS02_ReInitial(void);
void SACM_MS02_Play(int16_t *MS02_LibPtr, int16_t *SpeechDataStartAddr, uint8_t AutoRampUpDownCtrl, int32_t SampleRate);
void SACM_MS02_Play_Con(int16_t *MS02_LibPtr, int16_t *SpeechDataStartAddr, uint8_t AutoRampUpDownCtrl, int32_t SampleRate);
void SACM_MS02_DecodeProcess(int16_t *DstBufAddr, int16_t OutBufSize);
void SACM_MS02_Pause(void);
void SACM_MS02_Resume(void);
void SACM_MS02_Stop(void);
void SACM_MS02_ServiceLoop(void);
void SACM_MS02_DmaIsrService(void);
uint16_t SACM_MS02_GetStatus(void);
uint16_t SACM_MS02_Check_Con(void);
uint8_t SACM_MS02_CheckCpuOverload(void);
void SACM_MS02_ClearCpuOverload(void);
uint16_t DataRead(uint16_t DataLen);

extern void SACM_MS02_SetChannelStatus(int16_t ChannelState);
extern void SACM_MS02_SetDrumStatus(int16_t DrumStatus);
extern void SACM_MS02_SetKeyChStatus(int16_t KeyChStatus);
extern void SACM_MS02_KeyShift(int16_t keyshift);

extern void SACM_MS02_OKON_Enable(int32_t MidiChan, int32_t Type);
extern void SACM_MS02_OKON_Disable(void);
extern void SACM_MS02_OKON_NoteOFF(int MidiChan);
extern void SACM_MS02_PlayOKON(void);
extern void SACM_MS02_ChannelOn(int32_t MidiChan);
extern void SACM_MS02_ChannelOff(int32_t MidiChan);
extern void SACM_MS02_HoldChannel(int32_t MidiChan);
extern void SACM_MS02_ReleaseChannel(int32_t MidiChan);

extern void SACM_MS02_ChangeTempo(int32_t TempoFactor);
extern void SACM_MS02_ResetTempo(void);
extern void SACM_MS02_InfiniteLoop_Enable(void);
extern void SACM_MS02_InfiniteLoop_Disable(void);

extern void SACM_MS02_ChangeInstru(int32_t MidiChan, int32_t Instrument);
extern void SACM_MS02_ResetChanControl(void);
extern void SACM_MS02_PlayNote(int16_t MidiChan, int16_t pitch, int16_t velocity, int16_t duration);

#ifdef __cplusplus
}
#endif

#endif
