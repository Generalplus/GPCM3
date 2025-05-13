/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   SACM_MIXER_Ch1_User.c
 * @Version:
 *   V0.9.1
 * @Date:
 *   02 June 2023
 * @Abstract:
 *
 **************************************************************************************************/


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "GPCM3_FM1.h"
#include "SACM_MIXER_Ch1_User.h"
#include "APP_SwVolumeControl.h"
#include "SACM_MS02_User.h"
#include "SACM_MS02PN_API.h"
#include "SACM_MS02_API.h"


/*---------------------------------------------------------------------------------------
 * Gobal Variable Declaration Area
 *---------------------------------------------------------------------------------------*/
DMA_TYPE_DEF *mMicDsAdcDmaHandle;
TIMER_TYPE_DEF *mMicDacTimerHandle;
DMA_TYPE_DEF *mMicDacCh1DmaHandle;
static int16_t mSkipFrameNum;


/*---------------------------------------------------------------------------------------
 * Extern Variable Declaration Area
 *---------------------------------------------------------------------------------------*/
extern uint16_t mMS02Status;
extern uint16_t mMS02PNStatus;

extern SACM_MS02_WORKING_RAM MS02WorkingRam;
extern SACM_MS02PN_WORKING_RAM MS02PNWorkingRam;

// extern int8_t ModeCtrl;
extern uint8_t AudioOutType;


/*---------------------------------------------------------------------------------------
 * User Subroutine Area
 *---------------------------------------------------------------------------------------*/

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void static RampUpDacCh0()
{
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void static RampUpDacCh1()
{
	int16_t DacDataTemp1;
	int16_t DacDataTemp2;
	int16_t DacDataBuf[8];
	int16_t iCount;
	int16_t DataLen;

	DataLen = sizeof(DacDataBuf) / 2;
  DacDataTemp1 = DAC->DAC_CH1_DMA_DATA0 & 0xFFC0;
	DacDataTemp2 = DacDataTemp1;
	do
	{
		DacDataTemp2 = DacDataTemp1;
	  for(iCount = 0; iCount < DataLen; iCount++)
	  {
	    if(DacDataTemp1 > 0)
	    {
			  DacDataTemp1 -= 0x40;
		  }
	    else if(DacDataTemp1 < 0)
		  {
			  DacDataTemp1 += 0x40;
		  }
			DacDataBuf[iCount] = DacDataTemp1;
	  }
		DMA_Init(mMicDacCh1DmaHandle, DMA_REQSEL_DAC_CH1, DMA_SRC_DATA_16B, DMA_SRC_ADDR_INC, DMA_DST_DATA_16B, DMA_DST_ADDR_FIX);
		SET_BIT(mMicDacCh1DmaHandle->CTRL,DMA_CTRL_DMA_CONTINUE_ENABLE);    // DMA continuous mode enabled.
		DMA_Trigger(mMicDacCh1DmaHandle, (uint32_t)&DacDataBuf[0], (uint32_t)&DAC->DAC_CH1_DMA_DATA0, DataLen);
		while(DMA_CheckBusy(mMicDacCh1DmaHandle) != 0);                			// Wait DMAx busy
	} while(DacDataTemp2 != 0);

}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void static RampDownDacCh0()
{
	RampUpDacCh0();
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void static RampDownDacCh1()
{
	RampUpDacCh1();
}

/**
 * @brief
 *
 * @param
 *   MixerCh0Status [in]: Mixer status
 * @return
 *   None.
 */
void static DacAutoRampUp(uint16_t MixerCh0Status)
{
  if((MixerCh0Status & MIXER_CH1_AUTO_RAMP_UP_ENABLE_FLAG) != 0)
	{
		if((MixerCh0Status & MIXER_CH1_ENABLE_DAC_CH0_FLAG) != 0)
		{
			RampUpDacCh0();
		}

		if((MixerCh0Status & MIXER_CH1_ENABLE_DAC_CH1_FLAG) != 0)
		{
			RampUpDacCh1();
		}
	}
}

/**
 * @brief
 *
 * @param
 *   MixerCh0Status [in]: Mixer status
 * @return
 *   None.
 */
void static DacAutoRampDown(uint16_t MixerCh0Status)
{
  if((MixerCh0Status & MIXER_CH1_AUTO_RAMP_DOWN_ENABLE_FLAG) != 0)
	{
		if((MixerCh0Status & MIXER_CH1_ENABLE_DAC_CH0_FLAG) != 0)
		{
			RampDownDacCh0();
		}

		if((MixerCh0Status & MIXER_CH1_ENABLE_DAC_CH1_FLAG) != 0)
		{
			RampDownDacCh1();
		}
	}
}


/*---------------------------------------------------------------------------------------
 * Callback Function Area
 *---------------------------------------------------------------------------------------*/
/**
 * @brief
 *   Callback function. Implemented by user.
 *   When user call API function SACM_MIXER_Ch1_Initial, this function is called by MIXER Ch0 Library.
 * @param
 *  *MixerCh1WorkingRam [in]: Mixer ch0 working RAM address.
 * @return
 *   None.
 */
void MIXER_Ch1_CB_Init(const SACM_MIXER_CH1_WORKING_RAM *MixerCh1WorkingRam)
{
	mMicDsAdcDmaHandle = DMA3;
	mMicDacCh1DmaHandle = DMA4;
	mMicDacTimerHandle = TM1;

  TIMER_Open(mMicDacTimerHandle);
	TIMER_SetFreq(mMicDacTimerHandle, (20000 * 4));	               				// Sample rate: 20000 x 4 = 80000(Hz), for H/W 4x upsampling

	// if((SACM_MIXER_Ch0_GetStatus() & MIXER_CH0_PLAY_FLAG) == 0)
	//{
    DAC_Open();
    DAC_Scale_Enable();
    SET_BIT(DAC->CTRL, DAC_CTRL_CH0_POSTWAVE_OUT_CH0 + DAC_CTRL_CH1_POSTWAVE_OUT_CH1);
    DAC_AudioPwm_Open();
    DAC_SetAudPwmGain(31);      // 1x Gain.
	//}
	DAC_CH1_Init(DAC_TRG_SEL_TM1);
}

/**
 * @brief
 *   Callback function. To be implemented by user.
 *   When user call API function SACM_MIC2DAC_Play, this function is called by MIC Library.
 * @param
 *  *MixerCh1WorkingRam [in]: MIC2DAC working RAM address.
 * @return
 *   None.
 */
void MIXER_Ch1_CB_StartPlay(const SACM_MIXER_CH1_WORKING_RAM *MixerCh1WorkingRam)
{
  uint16_t iCount;
	mSkipFrameNum = MIXER_CH1_SKIP_FRAME;
  //if((SACM_MIXER_Ch0_GetStatus() & MIXER_CH0_PLAY_FLAG) == 0)
	//{
       if(AudioOutType == AUD_OUT_PWM)                                 // For AUD_OUT_PWM
      {
        DAC_VoltageDAC_CH0_Disable();
        DAC_VoltageDAC_CH1_Disable();
        DAC_AudioPwm_IP_Enable();
      }
      else                                                          // For AUD_OUT_DAC
      {
        DAC_AudioPwm_IP_Disable();
        DAC_VoltageDAC_CH0_Enable();
        DAC_VoltageDAC_CH1_Enable();
      }
  //}
	DacAutoRampUp(SACM_MIXER_Ch1_GetStatus());                   						// DAC auto ramp up

	if((SACM_MIXER_Ch1_GetStatus() & MIXER_CH1_ENABLE_DAC_CH1_FLAG) != 0)
	{
		DMA_Init(mMicDacCh1DmaHandle, DMA_REQSEL_DAC_CH1, DMA_SRC_DATA_16B, DMA_SRC_ADDR_INC, DMA_DST_DATA_16B, DMA_DST_ADDR_FIX);
		SET_BIT(mMicDacCh1DmaHandle->CTRL,DMA_CTRL_DMA_CONTINUE_ENABLE);    // DMA continuous mode enabled.
    if((SACM_MIXER_Ch1_GetStatus() & MIXER_CH1_ENABLE_DAC_CH0_FLAG) == 0)
		{
	    DMA_InstallIsrService(mMicDacCh1DmaHandle, SACM_MIXER_Ch1_SendDac_DmaIsrService);
	    DMA_EnableInt(mMicDacCh1DmaHandle);
	    NVIC_EnableIRQ(DMA4_IRQn);
		}
		DMA_Trigger(mMicDacCh1DmaHandle, (uint32_t)MixerCh1WorkingRam->DacOutBufPtr, (uint32_t)&DAC->DAC_CH1_DMA_DATA0, MIXER_CH1_FRAME_SIZE);
	}
}

/**
 * @brief
 *   Callback function.
 * @param
 *  *MixerCh1WorkingRam [in]: MIXER_Ch1 working RAM address.
 * @return
 *   None.
 */
void MIXER_Ch1_CB_StopPlay(const SACM_MIXER_CH1_WORKING_RAM *MixerCh1WorkingRam)
{
	if((SACM_MIXER_Ch1_GetStatus() & MIXER_CH1_PLAY_FLAG) != 0)
  {
	  if((SACM_MIXER_Ch1_GetStatus() & MIXER_CH1_ENABLE_DAC_CH1_FLAG) != 0)
	  {
		  DMA_Close(mMicDacCh1DmaHandle);
		}
		DacAutoRampDown(SACM_MIXER_Ch1_GetStatus());	                				// DAC auto ramp down
		//if((SACM_MIXER_Ch0_GetStatus() & MIXER_CH0_PLAY_FLAG) == 0)
    //{
      if(AudioOutType == AUD_OUT_PWM)                                // For AUD_OUT_PWM, Kim 2020.12.02
      {
        DAC_AudioPwm_IP_Disable();
      }
    //}
  }
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When user call API function SACM_MIC2DAC_Pause, this function is called by MIC Library.
 * @param
 *  *MixerCh1WorkingRam [in]: MIC2DAC library working RAM address.
 * @return
 *   None.
 */
void MIXER_Ch1_CB_Pause(const SACM_MIXER_CH1_WORKING_RAM *MixerCh1WorkingRam)
{
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When user call API function SACM_MIC2DAC_Resume, this function is Called by MIC Library.
 * @param
 *  *MixerCh1WorkingRam [in]: MIC2DAC library working RAM address.
 * @return
 *   None.
 */
void MIXER_Ch1_CB_Resume(const SACM_MIXER_CH1_WORKING_RAM *MixerCh1WorkingRam)
{
	if((SACM_MIXER_Ch1_GetStatus() & MIXER_CH1_ENABLE_DAC_CH1_FLAG) != 0)
	{
    DMA_Trigger(mMicDacCh1DmaHandle, (uint32_t)MixerCh1WorkingRam->DacOutBufPtr, (uint32_t)&DAC->DAC_CH1_DMA_DATA0, MIXER_CH1_FRAME_SIZE);
	}
}
/**
 * @brief
 *   Callback function. Implemented by user.
 *   When playing, this function is called by MIC Library to trigger DS-ADC DMA.
 * @param
 *  *MixerCh1WorkingRam [in]: MIC2DAC working RAM address.
 *  *DstBufAddr [out]: ADC input buffer address pointer.
 *   DataLen [in]: Data length (halfword)
 * @return
 *   None.
 */
void MIXER_Ch1_CB_GetAdc_DmaIsr(const SACM_MIXER_CH1_WORKING_RAM *MixerCh1WorkingRam, int8_t *DstBufAddr, uint16_t DataLen)
{
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When playing, this function is called by MIC Library to trigger DAC DMA.
 * @param
 *  *MixerCh1WorkingRam [in]: MIC2DAC working RAM address.
 *  *SrcBufAddr [in]: DAC Output Buffer address pointer.
 *   DataLen [in]: Data length (halfword)
 * @return
 *   None.
 */
void MIXER_Ch1_CB_SendDac_DmaIsr(const SACM_MIXER_CH1_WORKING_RAM *MixerCh1WorkingRam, int16_t *SrcBufAddr, uint16_t DataLen)
{
	if((SACM_MIXER_Ch1_GetStatus() & MIXER_CH1_ENABLE_DAC_CH1_FLAG) != 0)
	{
		while(DMA_CheckBusy(mMicDacCh1DmaHandle) != 0);        							// Check DMAx busy
    DMA_Trigger(mMicDacCh1DmaHandle, (uint32_t)SrcBufAddr, (uint32_t)&DAC->DAC_CH1_DMA_DATA0, DataLen);
	}
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When playing, this function is called by MIC library to encode data.
 * @param
 *  *MixerCh1WorkingRam [in]: MIXER_Ch1 working RAM address.
 *  *DstBufAddr [out]: Destination buffer address pointer.
 *  *SrcBufAddr [in]: Source buffer address pointer.
 * @return
 *   None.
 */
void MIXER_Ch1_CB_VoiceProcess(const SACM_MIXER_CH1_WORKING_RAM *MixerCh1WorkingRam, int16_t *DstBufAddr, int8_t *SrcBufAddr)
{
	uint32_t iCount, jCount;
  int16_t *SwMixerCh1Ptr;
  int16_t *SwMixerCh2Ptr;
  int16_t MS02PcmBuffer[MS02_OUT_BUFFER_SIZE];              // For MS02 & MS02PN data merge Temp. Buffer

  SwMixerCh1Ptr = 0x0000;

  if(((SACM_MS02_GetStatus() & MS02_PLAY_FLAG) != 0) && (SACM_MS02_GetStatus() & MS02_PAUSE_FLAG) == 0)
	{
    if((SACM_MS02_GetStatus() & MS02_DECODE_END_FLAG) != 0)
    {
      SACM_MS02_Stop();
      mMS02Status = 0;
    }
    else if((SACM_MS02_GetStatus() & MS02_STOP_FLAG) != 0)
    {
      for(iCount = 0; iCount < MIXER_CH1_FRAME_SIZE; iCount++)
      {
        DstBufAddr[iCount] = 0;
      }
    }
    else
    {
      mMS02Status |= MS02_DECODE_WORK_FLAG;
      MS02WorkingRam.DecodeOutBuf1Ptr = &MS02PcmBuffer[0];
      SACM_MS02_ServiceLoop();

      for(iCount = 0; iCount < MIXER_CH1_FRAME_SIZE; iCount++)
      {
        DstBufAddr[iCount] = MS02PcmBuffer[iCount];
      }
      APP_SwVolCtrl_VolProcess(0, &DstBufAddr[0], MIXER_CH1_FRAME_SIZE);
    }
  }
	else
	{
    for(iCount = 0; iCount < MIXER_CH1_FRAME_SIZE; iCount++)
    {
      DstBufAddr[iCount] = 0;
    }
	}

	if(((SACM_MS02PN_GetStatus() & MS02_PLAY_FLAG) != 0) && (SACM_MS02PN_GetStatus() & MS02_PAUSE_FLAG) == 0)
	{
    if((SACM_MS02PN_GetStatus() & MS02_DECODE_END_FLAG) != 0)
    {
      SACM_MS02PN_Stop();
      mMS02PNStatus = 0;
      SwMixerCh2Ptr = 0x0000;
    }
    else
    {
      mMS02PNStatus |= MS02_DECODE_WORK_FLAG;
      MS02PNWorkingRam.DecodeOutBuf1Ptr = &MS02PcmBuffer[0];
      SACM_MS02PN_ServiceLoop();

      SwMixerCh2Ptr = &MS02PcmBuffer[0];
      APP_SwVolCtrl_VolProcess(1, SwMixerCh2Ptr, MIXER_CH1_FRAME_SIZE);
    }
	}
	else
	{
    SwMixerCh2Ptr = 0x0000;
	}

  APP_SwMixer_3ch(SwMixerCh1Ptr, SwMixerCh2Ptr, DstBufAddr, DstBufAddr, MIXER_CH1_FRAME_SIZE);
}
