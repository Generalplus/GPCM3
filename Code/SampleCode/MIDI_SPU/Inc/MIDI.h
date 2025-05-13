#ifndef _MIDI_H_
#define _MIDI_H_


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include <stdint.h>

#define ChannelToSPUCH(Channel, SPU_CH)									(SPU_CH = (0x50081000 + (Channel << 6)))
#define SPUCHToChannel(Channel, SPU_CH)									(Channel = ((SPU_CH - 0x1000) & 0xFFFF) >> 6))

#define SPU_ChannelNumber  64
#define SPU_CLK            96000000    // 96MHz  //uint32_t SystemCoreClock = 120000000UL;       // Use 12MHz XTAL
//#define SPU_CLK            98304000    // 98.304MHz

typedef struct{
	int32_t ChannelNumber;
	int32_t Pitch;
	int32_t Velocity;
	int32_t Duration;
}NoteEventStruct;

typedef struct{
	int16_t BeatCountValue;
}BeatCountEventStruct;

typedef struct{
	int32_t ChannelNumber;
	int32_t PitchBendValue;
}PitchBendEventStruct;

typedef struct{
	int32_t ChannelNumber;
	int32_t ControlNumber;
	int32_t ControlValue;
}ControlEventStruct;

typedef struct{
	int32_t ChannelNumber;
	int32_t InstrumentIndex;
}ProgramChangeEventStruct;

typedef struct{
	int32_t TempoValue;
}TempoEventStruct;

typedef struct{
	int32_t ChannelNumber;
	int32_t LyricWordCount;
	int32_t Duration;
}LyricEventStruct;

typedef struct{
	int32_t TextType;
	int32_t LyricWordCount;
}TextEventStruct;

typedef union{
	NoteEventStruct NoteEvent;
	BeatCountEventStruct BeatCountEvent;
	PitchBendEventStruct PitchBendEvent;
	ControlEventStruct ControlEvent;
	ProgramChangeEventStruct ProgramChangeEvent;
	TempoEventStruct TempoEvent;
	LyricEventStruct LyricEvent;
	TextEventStruct TextEvent;
}MIDI_EventStruct;

typedef struct{
	uint16_t R_MIDI_CH_Pan;
	uint16_t R_MIDI_CH_Volume;
	uint16_t R_MIDI_CH_Expression;
	uint16_t R_MIDI_CH_PitchBend;
	uint16_t R_MIDI_CH_Inst;			// Instrument Index mapping of Logical Channel in MIDI file
	uint32_t *R_PB_TABLE_Addr;		//PitchBend Table Address
	uint16_t R_RPN_ReceiveFlag;		//RPN Receive Flag
	uint16_t R_RPN_DATA;			  	//MIDI CH RPN Value
}MIDI_ChannelInfoStruct;

typedef struct{
	uint16_t R_SPU_Velocity;
	uint16_t R_MIDI_CH_Map;
	int32_t R_SPU_Duration;
	uint16_t R_NoteOnHist;			  // Log the NoteOn mapping channel to a Circular Queue
	uint32_t R_PB_PhaseRecord;		// Original Channel Phase Value
}SPU_ChannelInfoStruct;


typedef struct
{
  int16_t R_Beat_INT_Counter;
  int16_t R_MIDI_DeltaTime;
  int16_t R_MIDI_CH_Mute;
  uint16_t R_PitchShift;
  int16_t R_Duration_INT;
  int16_t R_SPU_CH_Number;
  uint16_t R_SPUCLK;

  int16_t MIDI_Tempo;
  int16_t MIDI_TempoUpDn;
  int16_t MIDI_OriginalTempo;
  uint16_t R_SourceMIDIMask;    // 16ch, MIDI mask
  uint64_t R_SPU_Mask;          // SPU mask, available ch set 1
  uint16_t *pMIDI_DataPtr;
  uint32_t R_MIDIIndex;
  uint16_t R_OneKeyOneNote_Ctrl;
  uint8_t R_OneKeyOneNote_SPU_CH;
  uint8_t R_OneKeyOneNote_SPU_CH_Pre;
  uint16_t R_OneKeyOneNote_Duration;
  uint16_t R_OneKeyOneNote_Duration_Ori;
  uint32_t *pT_TempoDivide;
  uint32_t *pT_VarPhaseTableBottom;
  MIDI_ChannelInfoStruct MIDI_ChannelInfo[16];
  MIDI_EventStruct MIDI_Event;
} MIDI_DRIVER_WORKING_RAM;


/*
 * MIDI_Dvr_Flag definition
 */
#define C_NoteEvent				            0x0000
#define C_BeatCountEvent		          0x0001
#define C_PitchBendEvent		          0x0002
#define C_ControlEvent			          0x0003
#define C_ProgramChangeEvent	        0x0004
#define C_TempoEvent			            0x0005
#define C_MIDI_EndEvent			          0x0006
#define C_LyricEvent			            0x0007
#define C_TextEvent				            0x0008

// the following definition is for control event
#define C_DataEntryEventMSB		        0x0006
#define C_DataEntryEventLSB		        0x0026
#define C_VolumeEvent			            0x0007
#define C_PanEvent				            0x000A
#define C_ExpressionEvent		          0x000B
#define C_RPN_LSB_Event			          0x0064
#define C_RPN_MSB_Event			          0x0065
#define  D_PITCH_BEND_TABLE_TOTAL	    0x000D


#ifdef __cplusplus
extern "C"{
#endif

void MIDI_ServiceLoop(void);
void MIDI_BeatISR(void);
void MIDI_Initial(MIDI_DRIVER_WORKING_RAM *DvrMIDIWorkingRamPtr);
void MIDI_Play(MIDI_DRIVER_WORKING_RAM *DvrMIDIWorkingRamPtr, uint32_t Index);
void MIDI_Stop(void);
void MIDI_Pause(void);
void MIDI_Resume();
void MIDI_CH_Mute(uint32_t Channel);
void MIDI_CH_Unmute(uint32_t Channel);
void MIDI_PlaySingleNote(uint32_t R_MidiIndex, uint32_t R_InstrumentIndex, uint32_t R_Pitch, uint32_t R_Velocity);
void MIDI_PlaySingleDrum(uint32_t R_MidiIndex, uint32_t R_Pitch, uint32_t R_Velocity);
void MIDI_StopSingleNote(uint32_t R_Pitch);
void MIDI_StopSingleDrum(uint32_t R_Pitch);
int32_t GetChannelDuration(uint16_t Channel);
int32_t GetStatusMIDI();
void MIDI_PitchShiftUp(void);
void MIDI_PitchShiftDown(void);
void MIDI_PitchShiftNone(void);
void MIDI_CH_Mask(uint8_t R_MaskCH);
void MIDI_CH_UnMask(uint8_t R_MaskCH);
void MIDI_OneKeyOneNote_Off(uint16_t R_MasterChannel, uint16_t R_Duration);
void MIDI_OneKeyOneNote_On(uint16_t R_MasterChannel, uint16_t R_Duration);
void MIDI_OneKeyOneNote_PlayNote();
uint64_t MIDI_User_SetSpuMask();

#ifdef __cplusplus
}
#endif

#endif




