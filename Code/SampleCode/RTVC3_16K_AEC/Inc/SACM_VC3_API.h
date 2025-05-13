/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:     
 *   SACM_VC3_API.h
 * @Version:  
 *   V1.0.4
 * @Date:     
 *   November 16, 2021
 * @Abstract: 
 *   USER DO NOT MODIFY
 **************************************************************************************************/
#ifndef _SACM_VC3_API_H_
#define _SACM_VC3_API_H_


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
#define VC3_PLAY_FLAG                       (0x0001)
#define VC3_PAUSE_FLAG                      (0x0004)
#define	VC3_AUTO_RAMP_DOWN_ENABLE_FLAG      (0x0008)
#define VC3_AUTO_RAMP_UP_ENABLE_FLAG        (0x0010)
#define VC3_ENABLE_DAC_CH0_FLAG             (0x0020)
#define VC3_ENABLE_DAC_CH1_FLAG             (0x0040)
#define	VC3_BUF_EVEN_FLAG                   (0x0080)
#define VC3_DECODE_WORK_FLAG                (0x0100)
#define VC3_ISR_SERVICE_ENABLE_FLAG         (0x0400)
#define VC3_ADCIN_BUF_EVEN_FLAG             (0x1000)
#define VC3_CPU_OVERLOAD_FLAG               (0x2000)

/*
 * DAC Channel definition for the 2nd parameter of API:SACM_VC3_Play 
 */
#define VC3_DAC_CH0                         (0x01)
#define VC3_DAC_CH1                         (0x02)

/*
 * DAC RampUp/RampDown definition for the 3nd parameter of API:SACM_VC3_Play 
 */
#define VC3_AUTO_RAMP_DISABLE               (0x00)
#define VC3_AUTO_RAMP_UP                    (0x01)
#define VC3_AUTO_RAMP_DOWN                  (0x02)

/*
 *  VC3 mode definition for the 1st parameter of API:SACM_VC3_Mode
 */
#define VC3_CHANGE_SPEED_MODE               (0x81)         // non-real-time 
#define VC3_SHIFT_PITCH_MODE                (0x01)
#define VC3_CONST_PITCH_MODE                (0x02)
#define VC3_ECHO_MODE                       (0x03)
#define VC3_VM_ROBOT_EFFECT1_MODE           (0x04)
#define VC3_VM_ROBOT_EFFECT2_MODE           (0x05)
#define VC3_VM_STRANGE_TONE_MODE            (0x06)
#define VC3_VM_DJ_EFFECT_MODE               (0x07)
#define VC3_VM_VIBRATION_MODE               (0x08)
#define VC3_VM_JET_PLANE_MODE               (0x09)
#define VC3_VM_DOUBLING_MODE                (0x0A)
#define VC3_VM_UNDERWATER_MODE              (0x0B)
#define VC3_USER_DEFINE_MODE1               (0x0C)
#define VC3_USER_DEFINE_MODE2               (0x0D)
#define VC3_USER_DEFINE_MODE3               (0x0E)
#define VC3_USER_DEFINE_MODE4               (0x0F)

/*
 * VC3 Library Constant definition
 */
#define VC3_8K                              (1)
#define VC3_16K                             (2)
#define VC3_SMPBASE                         VC3_16K
#if (VC3_SMPBASE == VC3_8K)
#define VC3_FRAME_SIZE                      (160)
#define VC3_N_INDEX				                  (512)
#define VC3_M_INDEX				                  (32)
#elif (VC3_SMPBASE == VC3_16K)
#define VC3_FRAME_SIZE                      (320)
#define VC3_N_INDEX				                  (1024)
#define VC3_M_INDEX				                  (64)
#endif
#define VC3_M_LENGTH			                  ((VC3_N_INDEX / VC3_M_INDEX) * 2)
#define VC3_FILTER_ORDER	                  VC3_M_LENGTH
#define MAX_VC3_SHIFT_PITCH_LIMIT	          (12)
#define MIN_VC3_SHIFT_PITCH_LIMIT	          (-12)
#define	MAX_VC3_CONST_PITCH_LIMIT           (12)
#define	MIN_VC3_CONST_PITCH_LIMIT           (-12)
#define	MAX_VC3_ECHO_GAIN_LIMIT             (12)
#define	MIN_VC3_ECHO_GAIN_LIMIT             (-12)
#define	MAX_VC3_SPEED_CTRL_LIMIT            (12)
#define	MIN_VC3_SPEED_CTRL_LIMIT            (-12)


/*---------------------------------------------------------------------------------------
 * Structure Definition Area
 *---------------------------------------------------------------------------------------*/ 
typedef struct _SACM_VC3_KERNEL_RAM
{	
	uint8_t	u8FilterMethod;
	uint8_t	u8BuffPitch;
	uint16_t u16BuffLen;
	uint8_t	u8ChangeMode;
	uint8_t	au8ChangeIndex[4];
	uint8_t	u8bWeighting;
	uint8_t	u8bReading;
	uint8_t	u8StepPitch;
	uint16_t u16OutLength;
	uint16_t u16RspLength;
	uint16_t u16HmmLength;
	uint16_t u16ModuloL;
	uint16_t u16ModuloM;
	int16_t	ai16TmpData[VC3_FILTER_ORDER + VC3_FRAME_SIZE * 4];
	int16_t	*pi16OutData;
	int16_t	*pi16InData;
	int16_t	*pi16RspData;	
} SACM_VC3_KERNEL_RAM;

typedef struct _SACM_VC3_VM_KERNEL_RAM
{	
	uint8_t	VcVmKernelRam[1056];
} SACM_VC3_VM_KERNEL_RAM;

typedef struct _SACM_VC3_API_WORKING_RAM
{		
	uint8_t Vc3Mode;		
	int16_t *pVc3InBuf;	
	int16_t *pVc3OutBuf;
	int16_t *pAdcInBuf;
	int16_t *pDacOutBuf;
	int16_t Vc3Buf[2 * 2 * VC3_FRAME_SIZE];
	int16_t *AdcInBuf;
	int16_t *DacOutBuf;	
} SACM_VC3_API_WORKING_RAM; 

typedef struct _SACM_VC3_PCM_BUFFER
{		
	int16_t VC3PcmBuffer[VC3_FRAME_SIZE * 2 * 2];	  
} SACM_VC3_PCM_BUFFER;


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C"{
#endif

void SACM_VC3_Initial(SACM_VC3_API_WORKING_RAM *Vc3ApiWorkingRam, void *Vc3PcmBuffer);
void SACM_VC3_Mode(uint8_t VcMode, void *Vc3KernelWorkingRam);
void SACM_VC3_Play(uint8_t DacChannelSel, uint8_t AutoRampUpDownCtrl);
void SACM_VC3_Stop(void);
uint16_t SACM_VC3_GetStatus(void);
void SACM_VC3_ServiceLoop(void);
void SACM_VC3_GetAdc_DmaIsrService(void);
void SACM_VC3_SendDac_DmaIsrService(void);
uint32_t SACM_VC3_CheckCpuOverload(void);
void SACM_VC3_ClearCpuOverload(void);	

void SACM_VC3_ChangeSpeedMode_SetSpeed(int8_t ChangeSpeed);	
void SACM_VC3_ChangeSpeedProcess(int16_t* VcInputBuf, int16_t* VcOutputBuf, void(*CbFunGetPcm)(int16_t* DstBuf, uint16_t DataLen));
	
void SACM_VC3_ShiftPitchMode_SetPitch(int8_t ShiftPitch);
void SACM_VC3_ShiftPitchMode_SetPitchStep(int8_t PitchStep);	
void SACM_VC3_ShiftPitchProcess(int16_t* VcInputBuf, int16_t* VcOutputBuf);

void SACM_VC3_ConstPitchMode_SetPitch(int8_t ConstPitch);
void SACM_VC3_ConstPitchProcess(int16_t* VcInputBuf, int16_t* VcOutputBuf);

void SACM_VC3_EchoMode_SetGain(int8_t EchoGain);
void SACM_VC3_EchoProcess(int16_t* VcInputBuf, int16_t* VcOutputBuf);
	
void SACM_VC3_RobotEffect1(int16_t* VcInputBuf, int16_t* VcOutputBuf, int16_t FrameSize);
void SACM_VC3_RobotEffect2(int16_t* VcInputBuf, int16_t* VcOutputBuf, int16_t FrameSize);
void SACM_VC3_StrangeTone(int16_t* VcInputBuf, int16_t* VcOutputBuf, int16_t FrameSize);
void SACM_VC3_DJEffect(int16_t* VcInputBuf, int16_t* VcOutputBuf, int16_t FrameSize);
void SACM_VC3_Vibration(int16_t* VcInputBuf, int16_t* VcOutputBuf, int16_t FrameSize);
void SACM_VC3_JetPlaneEffect(int16_t* VcInputBuf, int16_t* VcOutputBuf, int16_t FrameSize);
void SACM_VC3_Doubling(int16_t* VcInputBuf, int16_t* VcOutputBuf, int16_t FrameSize);
void SACM_VC3_UnderWater(int16_t* VcInputBuf, int16_t* VcOutputBuf, int16_t FrameSize);


#ifdef __cplusplus
}
#endif	
	
#endif
