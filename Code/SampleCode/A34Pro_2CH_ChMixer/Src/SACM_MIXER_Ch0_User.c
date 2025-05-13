/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   SACM_MIXER_Ch0_User.c
 * @Version:
 *   V0.9.0
 * @Date:
 *   June 06, 2023
 * @Abstract:
 *
 **************************************************************************************************/


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "GPCM3_FM1.h"
#include "SACM_MIXER_Ch0_User.h"
// #include "SACM_MIXER_Ch1_User.h"
#include "APP_SwVolumeControl.h"
#include "SACM_A3400Pro_User.h"
#include "SACM_A3400Pro_CH2_User.h"


/*---------------------------------------------------------------------------------------
 * Gobal Variable Declaration Area
 *---------------------------------------------------------------------------------------*/
DMA_TYPE_DEF *mMicDsAdcDmaHandle;
TIMER_TYPE_DEF *mMicDacTimerHandle;
DMA_TYPE_DEF *mMicDacCh0DmaHandle;
static int16_t mSkipFrameNum;
uint16_t mSampleRate;


/*---------------------------------------------------------------------------------------
 * Extern Variable Declaration Area
 *---------------------------------------------------------------------------------------*/
extern uint16_t mA3400ProStatus;
extern uint16_t mA3400ProCh2Status;
extern SACM_A3400PRO_WORKING_RAM A3400ProWorkingRam;
extern SACM_A3400PRO_WORKING_RAM A3400ProCh2WorkingRam;

// extern int16_t A3400ProBuffer[];
extern int8_t ModeCtrl;
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
void RampUpDacCh0()
{
	int16_t DacDataTemp1;
	int16_t DacDataTemp2;
	int16_t DacDataBuf[8];
	int16_t iCount;
	int16_t DataLen;

	DataLen = sizeof(DacDataBuf) / 2;
  DacDataTemp1 = DAC->DAC_CH0_DMA_DATA0 & 0xFFC0;
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
	  DMA_Init(mMicDacCh0DmaHandle, DMA_REQSEL_DAC_CH0, DMA_SRC_DATA_16B, DMA_SRC_ADDR_INC, DMA_DST_DATA_16B, DMA_DST_ADDR_FIX);
	  SET_BIT(mMicDacCh0DmaHandle->CTRL,DMA_CTRL_DMA_CONTINUE_ENABLE);    // DMA continuous mode enabled.
	  DMA_Trigger(mMicDacCh0DmaHandle, (uint32_t)&DacDataBuf[0], (uint32_t)&DAC->DAC_CH0_DMA_DATA0, DataLen);
		while(DMA_CheckBusy(mMicDacCh0DmaHandle) != 0);                			// Wait DMAx busy
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
void RampUpDacCh1()
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
void RampDownDacCh0()
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
void RampDownDacCh1()
{
}

/**
 * @brief
 *
 * @param
 *   MixerCh0Status [in]: Mixer status
 * @return
 *   None.
 */
void DacAutoRampUp(uint16_t MixerCh0Status)
{
  if((MixerCh0Status & MIXER_CH0_AUTO_RAMP_UP_ENABLE_FLAG) != 0)
	{
		if((MixerCh0Status & MIXER_CH0_ENABLE_DAC_CH0_FLAG) != 0)
		{
			RampUpDacCh0();
		}

		if((MixerCh0Status & MIXER_CH0_ENABLE_DAC_CH1_FLAG) != 0)
		{
			// RampUpDacCh1();
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
void DacAutoRampDown(uint16_t MixerCh0Status)
{
  if((MixerCh0Status & MIXER_CH0_AUTO_RAMP_DOWN_ENABLE_FLAG) != 0)
	{
		if((MixerCh0Status & MIXER_CH0_ENABLE_DAC_CH0_FLAG) != 0)
		{
			RampDownDacCh0();
		}

		if((MixerCh0Status & MIXER_CH0_ENABLE_DAC_CH1_FLAG) != 0)
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
 *   When user call API function SACM_MIXER_Ch0_Initial, this function is called by MIXER Ch0 Library.
 * @param
 *  *MixerCh0WorkingRam [in]: Mixer ch0 working RAM address.
 * @return
 *   None.
 */
void MIXER_Ch0_CB_Init(const SACM_MIXER_CH0_WORKING_RAM *MixerCh0WorkingRam)
{
	mMicDsAdcDmaHandle = DMA3;
	mMicDacCh0DmaHandle = DMA4;
	mMicDacTimerHandle = TM0;

  TIMER_Open(mMicDacTimerHandle);
	TIMER_SetFreq(mMicDacTimerHandle, (12000 * 4));	               				// Sample rate: 10000 x 4 = 40000(Hz), for H/W 4x upsampling

  // if((SACM_MIXER_Ch1_GetStatus() & MIXER_CH1_PLAY_FLAG) == 0)
	// {
    DAC_Open();
    DAC_Scale_Enable();
    SET_BIT(DAC->CTRL, DAC_CTRL_CH0_POSTWAVE_OUT_CH0 + DAC_CTRL_CH1_POSTWAVE_OUT_CH1);
    DAC_AudioPwm_Open();
    DAC_SetAudPwmGain(31);      // 1x Gain.
  // }
 	DAC_CH0_Init(DAC_TRG_SEL_TM0);

	if((ModeCtrl & MIC_EN)!= 0)
	{
    /*
     * DS-ADC(MIC) init
     */
    mSampleRate = DSADC_Init(12000);
    DSADC_SetBoostGain(3);
    DSADC_SetPgaGain(18);
    //DSADC_AutoMuteInit(0x180);
    DSADC_AutoMuteInit(0x0000);

    /*
     * Digital AGC init
     */
    DAGC_SetCenterBoundary(0x20, 0x1000);
    DAGC_SetSensitivity(0x1E, 0x01, 0xFE);
    DAGC_SetAttackScale(ATTACK_X1);
    DAGC_SetReleaseScale(RELEASE_X1);
    DAGC_ZeroCross_Enable();
    DAGC_Enable();
  }
}

/**
 * @brief
 *   Callback function. To be implemented by user.
 *   When user call API function SACM_MIC2DAC_Play, this function is called by MIC Library.
 * @param
 *  *MixerCh0WorkingRam [in]: MIC2DAC working RAM address.
 * @return
 *   None.
 */
void MIXER_Ch0_CB_StartPlay(const SACM_MIXER_CH0_WORKING_RAM *MixerCh0WorkingRam)
{
	mSkipFrameNum = MIXER_CH0_SKIP_FRAME;
  //if((SACM_MIXER_Ch1_GetStatus() & MIXER_CH1_PLAY_FLAG) == 0)
	//{
       if(AudioOutType == AUD_OUT_PWM)                                 // For AUD_OUT_PWM
      {
        DAC_VoltageDAC_CH0_Disable();
        DAC_AudioPwm_IP_Enable();
      }
      else                                                          // For AUD_OUT_DAC
      {
        DAC_AudioPwm_IP_Disable();
        DAC_VoltageDAC_CH0_Enable();
      }
  //}
	DacAutoRampUp(SACM_MIXER_Ch0_GetStatus());                  						// DAC auto ramp up

  if((ModeCtrl & MIC_EN)!= 0)
	{
    #if (MIXER_CH0_BITS == MIXER_CH0_MIC_ON)
      DMA_Init(mMicDsAdcDmaHandle, DMA_REQSEL_DSADC, DMA_SRC_DATA_16B, DMA_SRC_ADDR_FIX, DMA_DST_DATA_16B, DMA_DST_ADDR_INC);
      SET_BIT(mMicDsAdcDmaHandle->CTRL,DMA_CTRL_DMA_CONTINUE_ENABLE);    // DMA continuous mode enabled.
    #endif
    DMA_InstallIsrService(mMicDsAdcDmaHandle, SACM_MIXER_Ch0_GetAdc_DmaIsrService);
    DMA_EnableInt(mMicDsAdcDmaHandle);
    NVIC_EnableIRQ(DMA3_IRQn);
    DMA_Trigger(mMicDsAdcDmaHandle, (uint32_t)&DS_ADC->DATA, (uint32_t)MixerCh0WorkingRam->AdcInBufPtr, MIXER_CH0_FRAME_SIZE);
  }

	if((SACM_MIXER_Ch0_GetStatus() & MIXER_CH0_ENABLE_DAC_CH0_FLAG) != 0)
	{
	  DMA_Init(mMicDacCh0DmaHandle, DMA_REQSEL_DAC_CH0, DMA_SRC_DATA_16B, DMA_SRC_ADDR_INC, DMA_DST_DATA_16B, DMA_DST_ADDR_FIX);
	  SET_BIT(mMicDacCh0DmaHandle->CTRL,DMA_CTRL_DMA_CONTINUE_ENABLE);    // DMA continuous mode enabled.
	  DMA_InstallIsrService(mMicDacCh0DmaHandle, SACM_MIXER_Ch0_SendDac_DmaIsrService);
	  DMA_EnableInt(mMicDacCh0DmaHandle);
	  NVIC_EnableIRQ(DMA4_IRQn);
	  DMA_Trigger(mMicDacCh0DmaHandle, (uint32_t)MixerCh0WorkingRam->DacOutBufPtr, (uint32_t)&DAC->DAC_CH0_DMA_DATA0, MIXER_CH0_FRAME_SIZE);
	}
}

/**
 * @brief
 *   Callback function.
 * @param
 *  *MixerCh0WorkingRam [in]: MIXER_Ch0 working RAM address.
 * @return
 *   None.
 */
void MIXER_Ch0_CB_StopPlay(const SACM_MIXER_CH0_WORKING_RAM *MixerCh0WorkingRam)
{
	if((ModeCtrl & MIC_EN)!= 0)
	{
    DMA_Close(mMicDsAdcDmaHandle);
  }

	if((SACM_MIXER_Ch0_GetStatus() & MIXER_CH0_PLAY_FLAG) != 0)
  {
	  if((SACM_MIXER_Ch0_GetStatus() & MIXER_CH0_ENABLE_DAC_CH0_FLAG) != 0)
	  {
		  DMA_Close(mMicDacCh0DmaHandle);
		}
		DacAutoRampDown(SACM_MIXER_Ch0_GetStatus());	                				// DAC auto ramp down

		//if((SACM_MIXER_Ch1_GetStatus() & MIXER_CH1_PLAY_FLAG) == 0)
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
 *  *MixerCh0WorkingRam [in]: MIC2DAC library working RAM address.
 * @return
 *   None.
 */
void MIXER_Ch0_CB_Pause(const SACM_MIXER_CH0_WORKING_RAM *MixerCh0WorkingRam)
{
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When user call API function SACM_MIC2DAC_Resume, this function is Called by MIC Library.
 * @param
 *  *MixerCh0WorkingRam [in]: MIC2DAC library working RAM address.
 * @return
 *   None.
 */
void MIXER_Ch0_CB_Resume(const SACM_MIXER_CH0_WORKING_RAM *MixerCh0WorkingRam)
{
	if((ModeCtrl & MIC_EN)!= 0)
	{
    DMA_Trigger(mMicDsAdcDmaHandle, (uint32_t)&DS_ADC->DATA, (uint32_t)MixerCh0WorkingRam->AdcInBufPtr, MIXER_CH0_FRAME_SIZE);
  }

	if((SACM_MIXER_Ch0_GetStatus() & MIXER_CH0_ENABLE_DAC_CH0_FLAG) != 0)
	{
    DMA_Trigger(mMicDacCh0DmaHandle, (uint32_t)MixerCh0WorkingRam->DacOutBufPtr, (uint32_t)&DAC->DAC_CH0_DMA_DATA0, MIXER_CH0_FRAME_SIZE);
	}
}
/**
 * @brief
 *   Callback function. Implemented by user.
 *   When playing, this function is called by MIC Library to trigger DS-ADC DMA.
 * @param
 *  *MixerCh0WorkingRam [in]: MIC2DAC working RAM address.
 *  *DstBufAddr [out]: ADC input buffer address pointer.
 *   DataLen [in]: Data length (halfword)
 * @return
 *   None.
 */
void MIXER_Ch0_CB_GetAdc_DmaIsr(const SACM_MIXER_CH0_WORKING_RAM *MixerCh0WorkingRam, int8_t *DstBufAddr, uint16_t DataLen)
{
	if((ModeCtrl & MIC_EN)!= 0)
	{
    DMA_Trigger(mMicDsAdcDmaHandle, (uint32_t)&DS_ADC->DATA, (uint32_t)DstBufAddr, DataLen);
  }
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When playing, this function is called by MIC Library to trigger DAC DMA.
 * @param
 *  *MixerCh0WorkingRam [in]: MIC2DAC working RAM address.
 *  *SrcBufAddr [in]: DAC Output Buffer address pointer.
 *   DataLen [in]: Data length (halfword)
 * @return
 *   None.
 */
void MIXER_Ch0_CB_SendDac_DmaIsr(const SACM_MIXER_CH0_WORKING_RAM *MixerCh0WorkingRam, int16_t *SrcBufAddr, uint16_t DataLen)
{
	if((SACM_MIXER_Ch0_GetStatus() & MIXER_CH0_ENABLE_DAC_CH0_FLAG) != 0)
	{
		while(DMA_CheckBusy(mMicDacCh0DmaHandle) != 0);        							// Check DMAx busy
		DMA_Trigger(mMicDacCh0DmaHandle, (uint32_t)SrcBufAddr, (uint32_t)&DAC->DAC_CH0_DMA_DATA0, DataLen);
	}
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When playing, this function is called by MIC library to encode data.
 * @param
 *  *MixerCh0WorkingRam [in]: MIXER_Ch0 working RAM address.
 *  *DstBufAddr [out]: Destination buffer address pointer.
 *  *SrcBufAddr [in]: Source buffer address pointer.
 * @return
 *   None.
 */
void MIXER_Ch0_CB_VoiceProcess(const SACM_MIXER_CH0_WORKING_RAM *MixerCh0WorkingRam, int16_t *DstBufAddr, int8_t *SrcBufAddr)
{
	uint32_t iCount;
  int16_t *SwMixerCh1Ptr;
  int16_t *SwMixerCh2Ptr;
  int16_t A3400ProBuffer[MIXER_CH0_FRAME_SIZE];

	int16_t *pSrcBufAddr;
	pSrcBufAddr = (int16_t *)SrcBufAddr;

	/*
	 * There will be a pop sound the first time, Skip the first 30 frames.
	 */
	if(mSkipFrameNum != 0)
	{
		mSkipFrameNum -= 1;
		return;
	}

  if((ModeCtrl & MIC_EN)!= 0)
	{
    for(iCount = 0; iCount < MIXER_CH0_FRAME_SIZE; iCount++)
    {
      DstBufAddr[iCount] = pSrcBufAddr[iCount];
    }
    APP_SwVolCtrl_VolProcess(0, DstBufAddr, MIXER_CH0_FRAME_SIZE);
  }
  else
  {
    for(iCount = 0; iCount < MIXER_CH0_FRAME_SIZE; iCount++)
    {
      DstBufAddr[iCount] = 0;
    }
  }

  if(((SACM_A3400Pro_GetStatus() & A3400PRO_PLAY_FLAG) != 0) && (SACM_A3400Pro_GetStatus() & A3400PRO_PAUSE_FLAG) == 0)
	{
    if((SACM_A3400Pro_GetStatus() & A3400PRO_DECODE_END_FLAG) != 0)
    {
      SACM_A3400Pro_Stop();
      mA3400ProStatus = 0;
      SwMixerCh1Ptr = 0x0000;
    }
    else
    {
      mA3400ProStatus |= MIXER_CH0_DECODE_WORK_FLAG;
      A3400ProWorkingRam.A3400ProDecodeBufPtr = &A3400ProBuffer[0];
      SACM_A3400Pro_ServiceLoop();

      for(iCount = 0; iCount < MIXER_CH0_FRAME_SIZE; iCount++)
      {
        DstBufAddr[iCount] = A3400ProBuffer[iCount];
      }
      SwMixerCh1Ptr = &DstBufAddr[0];
      APP_SwVolCtrl_VolProcess(0, SwMixerCh1Ptr, MIXER_CH0_FRAME_SIZE);
    }
	}
	else
	{
    SwMixerCh1Ptr = 0x0000;
	}

	if(((SACM_A3400Pro_CH2_GetStatus() & A3400PRO_PLAY_FLAG) != 0) && ((SACM_A3400Pro_CH2_GetStatus() & A3400PRO_PAUSE_FLAG) == 0))
	{
    if((SACM_A3400Pro_CH2_GetStatus() & A3400PRO_DECODE_END_FLAG) != 0)
    {
      SACM_A3400Pro_CH2_Stop();
      mA3400ProCh2Status = 0;
      SwMixerCh2Ptr = 0x0000;
    }
    else
    {
      mA3400ProCh2Status |= MIXER_CH0_DECODE_WORK_FLAG;
      A3400ProCh2WorkingRam.A3400ProDecodeBufPtr = &A3400ProBuffer[0];
      SACM_A3400Pro_CH2_ServiceLoop();

      SwMixerCh2Ptr = &A3400ProBuffer[0];
      APP_SwVolCtrl_VolProcess(1, SwMixerCh2Ptr, MIXER_CH0_FRAME_SIZE);
    }
	}
	else
	{
    SwMixerCh2Ptr = 0x0000;
	}

  APP_SwMixer_3ch(SwMixerCh1Ptr, SwMixerCh2Ptr, DstBufAddr, DstBufAddr, MIXER_CH0_FRAME_SIZE);
}

