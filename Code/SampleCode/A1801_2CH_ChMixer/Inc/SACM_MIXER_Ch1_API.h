/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   SACM_MIXER_Ch1_API.h
 * @Version:
 *   V0.9.0
 * @Date:
 *   June 08, 2021
 * @Abstract:
 *   USER DO NOT MODIFY
 **************************************************************************************************/
#ifndef _SACM_MIXER_CH1_API_H_
#define _SACM_MIXER_CH1_API_H_


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
#define MIXER_CH1_PLAY_FLAG                       (0x0001)
#define MIXER_CH1_PAUSE_FLAG                      (0x0004)
#define	MIXER_CH1_AUTO_RAMP_DOWN_ENABLE_FLAG      (0x0008)
#define MIXER_CH1_AUTO_RAMP_UP_ENABLE_FLAG        (0x0010)
#define MIXER_CH1_ENABLE_DAC_CH0_FLAG             (0x0020)
#define MIXER_CH1_ENABLE_DAC_CH1_FLAG             (0x0040)
#define	MIXER_CH1_BUF_EVEN_FLAG                   (0x0080)
#define MIXER_CH1_DECODE_WORK_FLAG                (0x0100)
#define MIXER_CH1_ISR_SERVICE_ENABLE_FLAG         (0x0400)
#define MIXER_CH1_ADCIN_BUF_EVEN_FLAG             (0x1000)
#define MIXER_CH1_CPU_OVERLOAD_FLAG               (0x2000)

/*
 * MIC BITS definition for the 4nd parameter of API:SACM_MIC2DAC_Initial
 */
#define MIXER_CH1_MIC_ON    											(0x01)
#define MIXER_CH1_MIC_OFF    											(0x00)

/*
 * DAC Channel definition for the 1st parameter of API:SACM_MIC2DAC_Play
 */
#define MIXER_CH1_DAC_CH0                         (0x01)
#define MIXER_CH1_DAC_CH1                         (0x02)

/*
 * DAC RampUp/RampDown definition for the 2nd parameter of API:SACM_MIC2DAC_Play
 */
#define MIXER_CH1_AUTO_RAMP_DISABLE               (0x00)
#define MIXER_CH1_AUTO_RAMP_UP                    (0x01)
#define MIXER_CH1_AUTO_RAMP_DOWN                  (0x02)


/*---------------------------------------------------------------------------------------
 * Structure Definition Area
 *---------------------------------------------------------------------------------------*/
typedef struct _SACM_MIXER_CH1_WORKING_RAM
{
	uint16_t FrameSize;
	uint16_t TypeSize;

	int8_t *AdcInBufPtr;
	int8_t *MicInBufPtr;
	int8_t *AdcInBuf;

	int16_t *MicOutBufPtr;
	int16_t *DacOutBufPtr;
	int16_t *DacOutBuf;
} SACM_MIXER_CH1_WORKING_RAM;


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C"{
#endif

void SACM_MIXER_Ch1_Initial(SACM_MIXER_CH1_WORKING_RAM *MixerCh0WorkingRam, int8_t *MixerCh0PcmBufPtr, int16_t FrameSize, int16_t Bits);
void SACM_MIXER_Ch1_Play(uint8_t DacChannelSel, uint8_t AutoRampUpDownCtrl);
void SACM_MIXER_Ch1_Stop(void);
void SACM_MIXER_Ch1_Pause(void);
void SACM_MIXER_Ch1_Resume(void);
void SACM_MIXER_Ch1_ServiceLoop(void);
void SACM_MIXER_Ch1_GetAdc_DmaIsrService(void);
void SACM_MIXER_Ch1_SendDac_DmaIsrService(void);
uint16_t SACM_MIXER_Ch1_GetStatus(void);
uint32_t SACM_MIXER_Ch1_CheckCpuOverload(void);
void SACM_MIXER_Ch1_ClearCpuOverload(void);

#ifdef __cplusplus
}
#endif

#endif
