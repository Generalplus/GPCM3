/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   SACM_MIXER_Ch0_API.c
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
#include "SACM_MIXER_Ch0_User.h"


/*---------------------------------------------------------------------------------------
 * Gobal Variable Declaration Area
 *---------------------------------------------------------------------------------------*/
uint16_t volatile mMixerCh0Status = 0;
SACM_MIXER_CH0_WORKING_RAM *mMixerCh0WorkingRamPtr;

/*---------------------------------------------------------------------------------------
 * Extern Variable Declaration Area
 *---------------------------------------------------------------------------------------*/
extern int8_t ModeCtrl;


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
uint16_t MIXER_Ch0_InitStatus(uint8_t DacChannelNo, uint8_t AutoRampUpDownCtrl, uint16_t InitState)
{
  if((DacChannelNo & MIXER_CH0_DAC_CH0) != 0)
	{
	  InitState |= MIXER_CH0_ENABLE_DAC_CH0_FLAG;
	}

  if((DacChannelNo & MIXER_CH0_DAC_CH1) != 0)
	{
	  InitState |= MIXER_CH0_ENABLE_DAC_CH1_FLAG;
	}

	if((AutoRampUpDownCtrl & MIXER_CH0_AUTO_RAMP_UP) != 0)
	{
	  InitState |= MIXER_CH0_AUTO_RAMP_UP_ENABLE_FLAG;
	}

	if((AutoRampUpDownCtrl & MIXER_CH0_AUTO_RAMP_DOWN) != 0)
	{
	  InitState |= MIXER_CH0_AUTO_RAMP_DOWN_ENABLE_FLAG;
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
void SACM_MIXER_Ch0_Initial(SACM_MIXER_CH0_WORKING_RAM *MixerWorkingRam, int8_t *MixerPcmBufPtr, int16_t FrameSize, int16_t Bits)
{
	mMixerCh0Status = 0x0000;
	mMixerCh0WorkingRamPtr = MixerWorkingRam;
	mMixerCh0WorkingRamPtr->FrameSize = FrameSize;
	mMixerCh0WorkingRamPtr->TypeSize = FrameSize*2;											// int16_t: 2-bytes

	mMixerCh0WorkingRamPtr->DacOutBuf = (int16_t*)MixerPcmBufPtr;

	mMixerCh0WorkingRamPtr->AdcInBuf = MixerPcmBufPtr + 2*2*FrameSize;  // after (2 x (int16_t)DacOutBuf)

  MIXER_Ch0_CB_Init(mMixerCh0WorkingRamPtr);                          // callback function: MIXER_Ch0_CB_Init in SACM_MIXER_Ch0_User.c
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
void SACM_MIXER_Ch0_Play(uint8_t DacChannelNo, uint8_t AutoRampUpDownCtrl)
{
  int16_t iCount;

  mMixerCh0Status = MIXER_Ch0_InitStatus( DacChannelNo, AutoRampUpDownCtrl, (MIXER_CH0_PLAY_FLAG | MIXER_CH0_ISR_SERVICE_ENABLE_FLAG) );

  /*
   * Initial AdcInBuf Pointer & DacOutBuf Pointer
   */
	mMixerCh0WorkingRamPtr->AdcInBufPtr = &(mMixerCh0WorkingRamPtr->AdcInBuf[0]);
  if((ModeCtrl & MIC_EN)!= 0)
	{
    mMixerCh0WorkingRamPtr->MicInBufPtr = &(mMixerCh0WorkingRamPtr->AdcInBuf[mMixerCh0WorkingRamPtr->TypeSize]);
  }
  else
  {
    mMixerCh0WorkingRamPtr->MicInBufPtr = &(mMixerCh0WorkingRamPtr->AdcInBuf[0]);
  }

	mMixerCh0WorkingRamPtr->DacOutBufPtr = &(mMixerCh0WorkingRamPtr->DacOutBuf[0]);
	mMixerCh0WorkingRamPtr->MicOutBufPtr = &(mMixerCh0WorkingRamPtr->DacOutBuf[mMixerCh0WorkingRamPtr->FrameSize]);

  /*
   * Clear DacOut A buffer
   */
	for(iCount = 0; iCount < (2*(mMixerCh0WorkingRamPtr->FrameSize)); iCount++)
  {
		mMixerCh0WorkingRamPtr->DacOutBuf[iCount] = 0x0000;
	}

	MIXER_Ch0_CB_StartPlay(mMixerCh0WorkingRamPtr);                          // callback function: MIXER_Ch0_CB_StartPlay in SACM_MIXER_Ch0_User.asm
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void SACM_MIXER_Ch0_Stop()
{
	MIXER_Ch0_CB_StopPlay(mMixerCh0WorkingRamPtr);                           // callback function: MIXER_CB_StopPlay in SACM_MIXER_User.asm
	mMixerCh0Status = 0x0000;
}


/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   mMixerCh0Status
 */
uint16_t SACM_MIXER_Ch0_GetStatus()
{
  return mMixerCh0Status;
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void SACM_MIXER_Ch0_ServiceLoop()
{
  if(((mMixerCh0Status & MIXER_CH0_PLAY_FLAG) == 0) || ((mMixerCh0Status & MIXER_CH0_DECODE_WORK_FLAG) == 0) || ((mMixerCh0Status & MIXER_CH0_PAUSE_FLAG) != 0))
	{
    return;
	}

  MIXER_Ch0_CB_VoiceProcess(mMixerCh0WorkingRamPtr, mMixerCh0WorkingRamPtr->MicOutBufPtr, mMixerCh0WorkingRamPtr->MicInBufPtr);
  mMixerCh0Status &= ~MIXER_CH0_DECODE_WORK_FLAG;                        // clear decode work flag
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void SACM_MIXER_Ch0_Pause()
{
	if((mMixerCh0Status & MIXER_CH0_PLAY_FLAG) != 0)
	{
	  mMixerCh0Status |= MIXER_CH0_PAUSE_FLAG;
		mMixerCh0Status &= ~MIXER_CH0_ISR_SERVICE_ENABLE_FLAG;
		MIXER_Ch0_CB_Pause(mMixerCh0WorkingRamPtr);                            // callback function: MIXER_Ch0_CB_Pause in SACM_MIXER_Ch0_User.asm
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
void SACM_MIXER_Ch0_Resume()
{
	if((mMixerCh0Status & MIXER_CH0_PAUSE_FLAG) != 0)
	{
	  mMixerCh0Status &= ~MIXER_CH0_PAUSE_FLAG;
		mMixerCh0Status |= MIXER_CH0_ISR_SERVICE_ENABLE_FLAG;
		MIXER_Ch0_CB_Resume(mMixerCh0WorkingRamPtr);                           // callback function: MIXER_CB_Resume in SACM_MIXER_User.c
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
uint32_t SACM_MIXER_Ch0_CheckCpuOverload()
{
  if((mMixerCh0Status & MIXER_CH0_CPU_OVERLOAD_FLAG) == 0)
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
void SACM_MIXER_Ch0_ClearCpuOverload()
{
  mMixerCh0Status &= ~MIXER_CH0_CPU_OVERLOAD_FLAG;
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
void SACM_MIXER_Ch0_GetAdc_DmaIsrService()
{
  if((mMixerCh0Status & MIXER_CH0_ISR_SERVICE_ENABLE_FLAG)	== 0)
	{
		return;
	}

	if((mMixerCh0Status & MIXER_CH0_PLAY_FLAG)	!= 0)
  {
    if((ModeCtrl & MIC_EN)!= 0)
    {
      if((mMixerCh0Status & MIXER_CH0_ADCIN_BUF_EVEN_FLAG) == 0)
      {
        mMixerCh0WorkingRamPtr->AdcInBufPtr = &(mMixerCh0WorkingRamPtr->AdcInBuf[mMixerCh0WorkingRamPtr->TypeSize]);
        mMixerCh0WorkingRamPtr->MicInBufPtr = &(mMixerCh0WorkingRamPtr->AdcInBuf[0]);
      }
      else
      {
        mMixerCh0WorkingRamPtr->AdcInBufPtr = &(mMixerCh0WorkingRamPtr->AdcInBuf[0]);
        mMixerCh0WorkingRamPtr->MicInBufPtr = &(mMixerCh0WorkingRamPtr->AdcInBuf[mMixerCh0WorkingRamPtr->TypeSize]);
      }
    }
		mMixerCh0Status ^= MIXER_CH0_ADCIN_BUF_EVEN_FLAG;

		if((mMixerCh0Status & MIXER_CH0_PAUSE_FLAG) == 0)
		{
		  MIXER_Ch0_CB_GetAdc_DmaIsr(mMixerCh0WorkingRamPtr, mMixerCh0WorkingRamPtr->AdcInBufPtr, mMixerCh0WorkingRamPtr->FrameSize);      // callback function: MIXER_Ch0_CB_GetAdc_DmaIsr in SACM_MIXER_Ch0_User.c
		}
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
void SACM_MIXER_Ch0_SendDac_DmaIsrService()
{
  if((mMixerCh0Status & MIXER_CH0_ISR_SERVICE_ENABLE_FLAG)	== 0)
	{
		return;
	}

	if((mMixerCh0Status & MIXER_CH0_DECODE_WORK_FLAG) != 0)
	{
		mMixerCh0Status |= MIXER_CH0_CPU_OVERLOAD_FLAG;
	}

	if((mMixerCh0Status & MIXER_CH0_PLAY_FLAG)	!= 0)
  {
	  if((mMixerCh0Status & MIXER_CH0_BUF_EVEN_FLAG) == 0)
		{
			mMixerCh0WorkingRamPtr->MicOutBufPtr = &mMixerCh0WorkingRamPtr->DacOutBuf[0];
			mMixerCh0WorkingRamPtr->DacOutBufPtr = &mMixerCh0WorkingRamPtr->DacOutBuf[mMixerCh0WorkingRamPtr->FrameSize];
	  }
		else
		{
			mMixerCh0WorkingRamPtr->MicOutBufPtr = &mMixerCh0WorkingRamPtr->DacOutBuf[mMixerCh0WorkingRamPtr->FrameSize];
			mMixerCh0WorkingRamPtr->DacOutBufPtr = &mMixerCh0WorkingRamPtr->DacOutBuf[0];
	  }

		mMixerCh0Status ^= MIXER_CH0_BUF_EVEN_FLAG;
		mMixerCh0Status |= MIXER_CH0_DECODE_WORK_FLAG;

		if((mMixerCh0Status & MIXER_CH0_PAUSE_FLAG) == 0)
		{
		  MIXER_Ch0_CB_SendDac_DmaIsr(mMixerCh0WorkingRamPtr, mMixerCh0WorkingRamPtr->DacOutBufPtr, mMixerCh0WorkingRamPtr->FrameSize);    // callback function: MIXER_Ch0_CB_SendDac_DmaIsr in SACM_MIXER_Ch0_User.c
		}
	}
}
