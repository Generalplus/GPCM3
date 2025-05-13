/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   SACM_MIXER_Ch1_API.c
 * @Version:
 *   V0.9.0
 * @Date:
 *   Sep. 3rd, 2021
 * @Abstract:
 *
 **************************************************************************************************/


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "GPCM3_FM1.h"
#include "SACM_MIXER_Ch1_User.h"


/*---------------------------------------------------------------------------------------
 * Gobal Variable Declaration Area
 *---------------------------------------------------------------------------------------*/
uint16_t volatile mMixerCh1Status = 0;
SACM_MIXER_CH1_WORKING_RAM *mMixerCh1WorkingRamPtr;


/*---------------------------------------------------------------------------------------
 * Extern Variable Declaration Area
 *---------------------------------------------------------------------------------------*/
// extern int8_t ModeCtrl;


/*---------------------------------------------------------------------------------------
 * MIXER Library Code Section
 *---------------------------------------------------------------------------------------*/

/**
 * @brief
 *
 * @param
 *   DacChannelNo [in]:
 *    - 1 => DAC Ch0
 *    - 2 => DAC Ch1
 *    - 3 => DAC Ch0 + DAC Ch1
 *   AutoRampUpDownCtrl [in]:
 *    - 1 => Auto ramp up
 *    - 2 => Auto ramp down
 *    - 3 => Auto ramp up / Auto ramp down
 *   InitState [in]: be modifed
 * @return
 *   InitState
 */
uint16_t MIXER_Ch1_InitStatus(uint8_t DacChannelNo, uint8_t AutoRampUpDownCtrl, uint16_t InitState)
{
  if((DacChannelNo & MIXER_CH1_DAC_CH0) != 0)
	{
	  InitState |= MIXER_CH1_ENABLE_DAC_CH0_FLAG;
	}

  if((DacChannelNo & MIXER_CH1_DAC_CH1) != 0)
	{
	  InitState |= MIXER_CH1_ENABLE_DAC_CH1_FLAG;
	}

	if((AutoRampUpDownCtrl & MIXER_CH1_AUTO_RAMP_UP) != 0)
	{
	  InitState |= MIXER_CH1_AUTO_RAMP_UP_ENABLE_FLAG;
	}

	if((AutoRampUpDownCtrl & MIXER_CH1_AUTO_RAMP_DOWN) != 0)
	{
	  InitState |= MIXER_CH1_AUTO_RAMP_DOWN_ENABLE_FLAG;
	}

	return InitState;
}


/*---------------------------------------------------------------------------------------
 * API Function Area
 *---------------------------------------------------------------------------------------*/

/**
 * @brief
 *
 * @param
 *  *MixerWorkingRam [in]: MIXER working RAM address
 *  *MixerPcmBufPtr [in]: MIC PCM buffer address
 *  FrameSize [in]: MIXER_FRAME_SIZE
 *  Bits [in]: The Bits of MIC data.
 * @return
 *   None.
 */
void SACM_MIXER_Ch1_Initial(SACM_MIXER_CH1_WORKING_RAM *MixerWorkingRam, int8_t *MixerPcmBufPtr, int16_t FrameSize, int16_t Bits)
{
	mMixerCh1Status = 0x0000;
	mMixerCh1WorkingRamPtr = MixerWorkingRam;
	mMixerCh1WorkingRamPtr->FrameSize = FrameSize;
	mMixerCh1WorkingRamPtr->TypeSize = FrameSize*2;											// int16_t: 2-bytes

	mMixerCh1WorkingRamPtr->DacOutBuf = (int16_t*)MixerPcmBufPtr;
	// mMixerCh1WorkingRamPtr->AdcInBuf = MixerPcmBufPtr + 2*2*FrameSize;		// after (2 x (int16_t)DacOutBuf)

  MIXER_Ch1_CB_Init(mMixerCh1WorkingRamPtr);                           		// callback function: MIXER_Ch1_CB_Init in SACM_MIXER_Ch1_User.c
}

/**
 * @brief
 *
 * @param
 *   DacChannelNo [in]:
 *    - 1 => DAC1 enable
 *    - 2 => DAC2 enable
 *    - 3 => DAC1, DAC2 enable
 *   AutoRampUpDownCtrl [in]:
 *    - 0 => Ramp Up disable/Ramp Dn disable
 *    - 1 => Ramp Up enable/Ramp Dn disable
 *    - 2 => Ramp Up disable/Ramp Dn enable
 *    - 3 => Ramp Up enable/Ramp Dn enable
 * @return
 *   None.
 */
void SACM_MIXER_Ch1_Play(uint8_t DacChannelNo, uint8_t AutoRampUpDownCtrl)
{
  int16_t iCount;

  mMixerCh1Status = MIXER_Ch1_InitStatus( DacChannelNo, AutoRampUpDownCtrl, (MIXER_CH1_PLAY_FLAG | MIXER_CH1_ISR_SERVICE_ENABLE_FLAG) );

  /*
   * Initial AdcInBuf Pointer & DacOutBuf Pointer
   */
	// mMixerCh1WorkingRamPtr->AdcInBufPtr = &(mMixerCh1WorkingRamPtr->AdcInBuf[0]);
  // mMixerCh1WorkingRamPtr->MicInBufPtr = &(mMixerCh1WorkingRamPtr->AdcInBuf[0]);

	mMixerCh1WorkingRamPtr->DacOutBufPtr = &(mMixerCh1WorkingRamPtr->DacOutBuf[0]);
	mMixerCh1WorkingRamPtr->MicOutBufPtr = &(mMixerCh1WorkingRamPtr->DacOutBuf[mMixerCh1WorkingRamPtr->FrameSize]);

  /*
   * Clear DacOut A buffer
   */
	for(iCount = 0; iCount < (2*(mMixerCh1WorkingRamPtr->FrameSize)); iCount++)
  {
		mMixerCh1WorkingRamPtr->DacOutBuf[iCount] = 0x0000;
	}

	MIXER_Ch1_CB_StartPlay(mMixerCh1WorkingRamPtr);                          // callback function: MIXER_Ch1_CB_StartPlay in SACM_MIXER_Ch1_User.asm
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void SACM_MIXER_Ch1_Stop()
{
	MIXER_Ch1_CB_StopPlay(mMixerCh1WorkingRamPtr);                           // callback function: MIXER_CB_StopPlay in SACM_MIXER_User.asm
	mMixerCh1Status = 0x0000;
}


/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   mMixerCh1Status
 */
uint16_t SACM_MIXER_Ch1_GetStatus()
{
  return mMixerCh1Status;
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void SACM_MIXER_Ch1_ServiceLoop()
{
  if(((mMixerCh1Status & MIXER_CH1_PLAY_FLAG) == 0) || ((mMixerCh1Status & MIXER_CH1_DECODE_WORK_FLAG) == 0) || ((mMixerCh1Status & MIXER_CH1_PAUSE_FLAG) != 0))
	{
    return;
	}

  MIXER_Ch1_CB_VoiceProcess(mMixerCh1WorkingRamPtr, mMixerCh1WorkingRamPtr->MicOutBufPtr, mMixerCh1WorkingRamPtr->MicInBufPtr);
  mMixerCh1Status &= ~MIXER_CH1_DECODE_WORK_FLAG;                        // clear decode work flag
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void SACM_MIXER_Ch1_Pause()
{
	if((mMixerCh1Status & MIXER_CH1_PLAY_FLAG) != 0)
	{
	  mMixerCh1Status |= MIXER_CH1_PAUSE_FLAG;
		mMixerCh1Status &= ~MIXER_CH1_ISR_SERVICE_ENABLE_FLAG;
		MIXER_Ch1_CB_Pause(mMixerCh1WorkingRamPtr);                            // callback function: MIXER_Ch1_CB_Pause in SACM_MIXER_Ch1_User.asm
	}
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void SACM_MIXER_Ch1_Resume()
{
	if((mMixerCh1Status & MIXER_CH1_PAUSE_FLAG) != 0)
	{
	  mMixerCh1Status &= ~MIXER_CH1_PAUSE_FLAG;
		mMixerCh1Status |= MIXER_CH1_ISR_SERVICE_ENABLE_FLAG;
		MIXER_Ch1_CB_Resume(mMixerCh1WorkingRamPtr);                           // callback function: MIXER_CB_Resume in SACM_MIXER_User.c
	}
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   1 => CPU overload
 *   0 => CPU non-overload
 */
uint32_t SACM_MIXER_Ch1_CheckCpuOverload()
{
  if((mMixerCh1Status & MIXER_CH1_CPU_OVERLOAD_FLAG) == 0)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void SACM_MIXER_Ch1_ClearCpuOverload()
{
  mMixerCh1Status &= ~MIXER_CH1_CPU_OVERLOAD_FLAG;
}


/*---------------------------------------------------------------------------------------
 * MIXER ISR Service Fucntion Code Section
 *---------------------------------------------------------------------------------------*/

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void SACM_MIXER_Ch1_GetAdc_DmaIsrService()
{
/*
  if((mMixerCh1Status & MIXER_CH1_ISR_SERVICE_ENABLE_FLAG)	== 0)
	{
		return;
	}

	if((mMixerCh1Status & MIXER_CH1_PLAY_FLAG)	!= 0)
  {
		mMixerCh1Status ^= MIXER_CH1_ADCIN_BUF_EVEN_FLAG;

		if((mMixerCh1Status & MIXER_CH1_PAUSE_FLAG) == 0)
		{
		  MIXER_Ch1_CB_GetAdc_DmaIsr(mMixerCh1WorkingRamPtr, mMixerCh1WorkingRamPtr->AdcInBufPtr, mMixerCh1WorkingRamPtr->FrameSize);      // callback function: MIXER_Ch1_CB_GetAdc_DmaIsr in SACM_MIXER_Ch1_User.c
		}
	}
*/
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void SACM_MIXER_Ch1_SendDac_DmaIsrService()
{
  if((mMixerCh1Status & MIXER_CH1_ISR_SERVICE_ENABLE_FLAG)	== 0)
	{
		return;
	}

	if((mMixerCh1Status & MIXER_CH1_DECODE_WORK_FLAG) != 0)
	{
		mMixerCh1Status |= MIXER_CH1_CPU_OVERLOAD_FLAG;
	}

	if((mMixerCh1Status & MIXER_CH1_PLAY_FLAG)	!= 0)
  {
	  if((mMixerCh1Status & MIXER_CH1_BUF_EVEN_FLAG) == 0)
		{
			mMixerCh1WorkingRamPtr->MicOutBufPtr = &mMixerCh1WorkingRamPtr->DacOutBuf[0];
			mMixerCh1WorkingRamPtr->DacOutBufPtr = &mMixerCh1WorkingRamPtr->DacOutBuf[mMixerCh1WorkingRamPtr->FrameSize];
	  }
		else
		{
			mMixerCh1WorkingRamPtr->MicOutBufPtr = &mMixerCh1WorkingRamPtr->DacOutBuf[mMixerCh1WorkingRamPtr->FrameSize];
			mMixerCh1WorkingRamPtr->DacOutBufPtr = &mMixerCh1WorkingRamPtr->DacOutBuf[0];
	  }

		mMixerCh1Status ^= MIXER_CH1_BUF_EVEN_FLAG;
		mMixerCh1Status |= MIXER_CH1_DECODE_WORK_FLAG;

		if((mMixerCh1Status & MIXER_CH1_PAUSE_FLAG) == 0)
		{
		  MIXER_Ch1_CB_SendDac_DmaIsr(mMixerCh1WorkingRamPtr, mMixerCh1WorkingRamPtr->DacOutBufPtr, mMixerCh1WorkingRamPtr->FrameSize);    // callback function: MIXER_Ch1_CB_SendDac_DmaIsr in SACM_MIXER_Ch1_User.c
		}
	}
}
