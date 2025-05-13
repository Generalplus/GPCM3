/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   AEC_API.h
 * @Version:
 *   V1.0.0
 * @Date:
 *   26th, December 2019
 * @Abstract:
 *	 GPCM1_AEC_20191213.lib (use HW-MAC)
 **************************************************************************************************/
#ifndef _AEC_API_H_
#define _AEC_API_H_

/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include <stdint.h>

/*---------------------------------------------------------------------------------------
 * Definition Area
 *---------------------------------------------------------------------------------------*/
#define filter_order			128									// Must be integer multiple of 8
#define AEC_FRAME_SIZE		40

#define VR								1

#if VR
	#define step			(1.0)*(2.0*0.045*1.4*2*2*0.015625)	// for VR  0.007875
	#define ubias			(4.0)*(0.014)						// for VR
#else
	#define step			(2.5)*(2.0*0.045*1.4*2*2*0.015625)	// for RTVC 0.007875 * 2.5
	#define ubias			(0.3)*(0.014)						// for RTVC
#endif

#define C_STEP     			(step*65536.0/8)
#define	C_BIAS	   			(ubias*32768)

/*---------------------------------------------------------------------------------------
 * Structure Definition Area
 *---------------------------------------------------------------------------------------*/
typedef struct
{
  uint16_t	Mic_Eng;
  uint16_t	Ref_Eng;
  int16_t		Update_Flag;
  int16_t		F_CtrlStatus;
  uint16_t	T_NormalizeTable[128+1];

  int32_t	 	RemDC_HPF_Mic[5];
  int32_t	 	RemDC_HPF_Ref[5];
  int16_t	 	R_HighPass12_Buf_Mic[6];			// update
  int16_t	 	R_HighPass12_Buf_Ref[6]; 			// update

  uint8_t	 	F_CtrlStatus2;								// update
  uint8_t	 	frame_Q;											// update
  uint16_t	Mic_Peak; 										// update
  uint16_t	DAC_Peak;											// update
  uint8_t	 	feedback_delay;								// update
  uint8_t	 	feedback_delay_avg;						// update
  uint16_t	ftemp32;											// update
  uint32_t  feedback_energy;							// update
  uint32_t  feedback_time_energy;					// update
  uint32_t	feedback_effect;							// update
  uint32_t	feedback_effect_avg;					// update
} AEC_VAD_Info;

typedef struct
{
	AEC_VAD_Info 	AEC_Status;
	int32_t  			AEC_CtrlFlag;
	int32_t	 			AEC_FrameSize;
	int32_t	 			AEC_Taps;

	int32_t	 			H_coef[filter_order+(filter_order/2)];					// 1.5 x filter_order(taps)
	int16_t	 			Ref_SignalX[(filter_order-1)+AEC_FRAME_SIZE];		// frame size + filter_order
	int32_t  			AEC_OutGain;						 	// update
	uint32_t			AEC_Threshold;						// update
} AEC_MEMORY_BLOCK;

/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C"{
#endif

//API
void F_AEC_InitProc(AEC_MEMORY_BLOCK *AecWorkingRam, uint16_t g_STEP, uint16_t g_Bias, uint8_t HpfEnable, uint32_t Threshold);
void F_AEC_FrameProc(AEC_MEMORY_BLOCK *AecWorkingRam, int32_t *AEC_DMA_Buf, int16_t *AEC_Out);
void F_Memory_Copy(int16_t *src, int16_t *dst, int32_t length);

//kernel
void F_AEC_VAD_Info_InitProc(AEC_VAD_Info	*Obj, uint16_t g_STEP, uint16_t g_Bias);
void F_AEC_PreProcess(AEC_VAD_Info *Obj, int16_t *AEC_Out, int16_t *RefX ,int16_t t_FrameSize);
void F_AEC_PROCESS(short *MicIn, short *RefX, int *H, AEC_VAD_Info *Obj , int order, int CNTR);
void F_AEC_PreProcess_ChangeMode(AEC_VAD_Info	*Obj, uint16_t HPF0_Mode, uint16_t HPF1_Mode);
void F_AEC_PROCESS_enable(AEC_VAD_Info	*Obj);
void F_AEC_PROCESS_disable(AEC_VAD_Info	*Obj);

#ifdef __cplusplus
}
#endif

#endif
