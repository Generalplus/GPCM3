#include <stdint.h>

#ifndef __MS02DEC_H__
#define __MS02DEC_H__

#define MS02_CH_Num 16
//#define MS02_DEC_Memory 308 + (MS02_CH_Num * 76)
#define MS02_DEC_Memory 312 + (MS02_CH_Num * 76)   //240820 modify

#define MS02_DAC_8K				0
#define MS02_DAC_10K			1
#define MS02_DAC_12K			2
#define MS02_DAC_16K			3
#define MS02_DAC_20K			4
#define MS02_DAC_24K			5
#define MS02_DAC_28K			6
#define MS02_DAC_32K			7
#define MS02_DAC_36K			8
#define MS02_DAC_40K			9


extern void SACM_MS02_DEC_Reset(void* obj, int32_t SampleRate);
extern int32_t SACM_MS02_DEC_Init(void* obj, int16_t* ToneAddr, int16_t* InbufAddr, int16_t InbufSize, int32_t SampleRate, int32_t ChNum);  //20200805
extern int32_t SACM_MS02_DEC_Proc(void* obj, int16_t* Output, int16_t OutBufSize);
extern int32_t SACM_MS02_DEC_GetMemoryBlockSize(void);
extern void SACM_MS02_DEC_OKON_Enable(void* obj, int32_t MidiChan, int32_t OKON_Type);
extern void SACM_MS02_DEC_OKON_Disable(void* obj);
extern void SACM_MS02_DEC_PlayOKON(void* obj);
extern void SACM_MS02_DEC_ChannelOn(void* obj, int32_t MidiChan);
extern void SACM_MS02_DEC_ChannelOff(void* obj, int32_t ChannelNumber);
extern void SACM_MS02_DEC_HoldChannel(void* obj, int32_t MidiChan);
extern void SACM_MS02_DEC_ReleaseChannel(void* obj, int32_t MidiChan);
extern void SACM_MS02_DEC_ChangeTempo(void* obj, int32_t TempoFactor);
extern void SACM_MS02_DEC_ResetTempo(void* obj);
extern void SACM_MS02_DEC_ChangeInstru(void* obj, int32_t MidiChan, int32_t Instrument);
extern void SACM_MS02_DEC_ResetChanControl(void* obj);
extern void F_SACM_MS02_DEC_NoteOff(void *obj,int ch);
extern void SACM_MS02_DEC_PlayNote(void* obj, int16_t ch, int16_t pitch, int16_t velocity, int16_t duration);
extern int16_t SACM_MS02_DEC_GetChannelNum(void* obj); //20240124

//must be called before DEC_Init()
extern void SACM_MS02_DEC_SetChannelStatus(void* obj, int16_t ChannelState);
extern void SACM_MS02_DEC_SetDrumStatus(void* obj, int16_t DrumStatus);
extern void SACM_MS02_DEC_SetKeyChStatus(void* obj, int16_t KeyChStatus);
extern void SACM_MS02_DEC_KeyShift(void* obj, int32_t keyshift);
extern void SACM_MS02_DEC_InfiniteLoop_Enable(void* obj);		//240820
extern void SACM_MS02_DEC_InfiniteLoop_Disable(void* obj);		//240820

#endif //__MS02DEC_H__
