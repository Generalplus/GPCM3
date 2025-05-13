#include <stdint.h>  

#ifndef __MS02PNDEC_H__
#define __MS02PNDEC_H__

#define MS02PN_CH_Num 8//16
//#define MS02PN_DEC_Memory 256 + (MS02PN_CH_Num * 64)
//#define MS02PN_DEC_Memory 260 + (MS02PN_CH_Num * 64)		//210629
//#define MS02PN_DEC_Memory 260 + (MS02PN_CH_Num * 68)		//210908 for pitch bend
//#define MS02PN_DEC_Memory 260 + (MS02PN_CH_Num * 76)		//220427 
//#define MS02PN_DEC_Memory 264 + (MS02PN_CH_Num * 72)		//220525 
//#define MS02PN_DEC_Memory 264 + (MS02PN_CH_Num * 84)		//220728 
#define MS02PN_DEC_Memory 360 + (MS02PN_CH_Num * 84)		//221108 

#define MS02PN_DAC_8K				0	
#define MS02PN_DAC_10K			1	
#define MS02PN_DAC_12K			2	
#define MS02PN_DAC_16K			3	
#define MS02PN_DAC_20K			4	
#define MS02PN_DAC_24K			5	
#define MS02PN_DAC_28K			6	
#define MS02PN_DAC_32K			7	
#define MS02PN_DAC_36K			8	
#define MS02PN_DAC_40K			9


extern int32_t SACM_MS02PN_DEC_Init(void *obj,int16_t* ToneAddr,int32_t SampleRate,int32_t ChNum); 
extern int32_t SACM_MS02PN_DEC_Proc(void *obj,int16_t* Output, int16_t OutBufSize);
extern void SACM_MS02PN_DEC_ChangeInstru(void *obj,int16_t MidiChan , int16_t Instrument);							//20210412 rename
extern void SACM_MS02PN_DEC_PlayNote(void* obj, int16_t ch, int16_t pitch, int16_t velocity, int16_t duration);		//20210412 rename
extern void SACM_MS02PN_DEC_PlayNote(void* obj, int16_t ch, int16_t drumidx, int16_t velocity, int16_t duration);	//20210412 rename
extern void SACM_MS02PN_DEC_NoteOff(void *obj,int16_t ch);															//20210412 rename			
extern int16_t SACM_MS02PN_DEC_GetChStatus(void* obj);																//20210805
extern void SACM_MS02PN_DEC_PitchBend(void* obj, int16_t MidiChan, int16_t PitchBendValue);							//210908 for pitch bend
//220427
extern void SACM_MS02PN_DEC_SetReleaseStep(void* obj, int16_t MidiChan, int16_t CustomStep);
extern void SACM_MS02PN_DEC_ResetReleaseStep(void* obj, int16_t MidiChan);
extern void SACM_MS02PN_DEC_ChangeVelocity(void* obj, int16_t MidiChan, int16_t Velocity);
extern void SACM_MS02PN_DEC_ChangeTicker(void* obj, int16_t MidiChan, int16_t index);
//220427
//221108
extern void SACM_MS02PN_DEC_VibratioRate(void* obj, int16_t Rate);  //Rate: 0~32767
extern void SACM_MS02PN_DEC_VibrationEnable(void* obj);
extern void SACM_MS02PN_DEC_VibrationDisable(void* obj);
//221108
#endif //__MS02PNDEC_H__