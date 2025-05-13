/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   SACM_MIXER_Ch0_User.c
 * @Version:
 *   V0.9.0
 * @Date:
 *   03 March, 2025
 * @Abstract:
 *
 **************************************************************************************************/


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "GPCM3_FM1.h"
#include "SACM_MIXER_Ch0_User.h"
#include "SACM_A1801_User.h"
#include "SACM_A1801_CH2_User.h"
#include "SACM_A1801_CH3_User.h"
#include "SACM_A1801_CH4_User.h"
#include "APP_SwVolumeControl.h"

#if defined(__CC_ARM)
#define Aligned4B __align(4)
#elif defined(__GNUC__)
#define Aligned4B __attribute__ ((aligned (4)))
#endif


/*---------------------------------------------------------------------------------------
 * Gobal Variable Declaration Area
 *---------------------------------------------------------------------------------------*/
DMA_TYPE_DEF *mMicDsAdcDmaHandle;
TIMER_TYPE_DEF *mMicDacTimerHandle;
DMA_TYPE_DEF *mMicDacCh0DmaHandle;
DMA_TYPE_DEF *mMicDacCh1DmaHandle;
static int16_t mSkipFrameNum;
uint16_t mSampleRate;


/*---------------------------------------------------------------------------------------
 * Extern Variable Declaration Area
 *---------------------------------------------------------------------------------------*/
extern SACM_MIXER_CH0_WORKING_RAM *mMixerCh0WorkingRamPtr;
extern uint8_t AudOutType;

extern uint16_t mA1801Status;
extern uint16_t mA1801Ch2Status;
extern uint16_t mA1801Ch3Status;
extern uint16_t mA1801Ch4Status;
extern SACM_A1801_WORKING_RAM A1801WorkingRam;
extern SACM_A1801_WORKING_RAM A1801Ch2WorkingRam;
extern SACM_A1801_WORKING_RAM A1801Ch3WorkingRam;
extern SACM_A1801_WORKING_RAM A1801Ch4WorkingRam;
extern SACM_A1801_PCM_BUFFER A1801PcmBuffer;
extern SACM_A1801_PCM_BUFFER A1801Ch2PcmBuffer;
extern SACM_A1801_PCM_BUFFER A1801Ch3PcmBuffer;
extern SACM_A1801_PCM_BUFFER A1801Ch4PcmBuffer;


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
	  CLEAR_BIT(DAC->CTRL, DAC_CTRL_DAC_CH0_DMA_ENABLE);             // DAC_Ch0 DMA request disabled.
		TIMER_Close(mMicDacCh0DmaHandle);
	  DMA_Init(mMicDacCh0DmaHandle, DMA_REQSEL_DAC_CH0, DMA_SRC_DATA_16B, DMA_SRC_ADDR_INC, DMA_DST_DATA_16B, DMA_DST_ADDR_FIX);
	  SET_BIT(mMicDacCh0DmaHandle->CTRL,DMA_CTRL_DMA_CONTINUE_ENABLE);    // DMA continuous mode enabled.
	  DMA_Trigger(mMicDacCh0DmaHandle, (uint32_t)&DacDataBuf[0], (uint32_t)&DAC->DAC_CH0_DMA_DATA0, DataLen);
	  TIMER_Open(mMicDacCh0DmaHandle);
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
	  CLEAR_BIT(DAC->CTRL, DAC_CTRL_DAC_CH0_DMA_ENABLE);             // DAC_Ch0 DMA request disabled.
		TIMER_Close(mMicDacCh1DmaHandle);
		DMA_Init(mMicDacCh1DmaHandle, DMA_REQSEL_DAC_CH1, DMA_SRC_DATA_16B, DMA_SRC_ADDR_INC, DMA_DST_DATA_16B, DMA_DST_ADDR_FIX);
	  SET_BIT(mMicDacCh1DmaHandle->CTRL,DMA_CTRL_DMA_CONTINUE_ENABLE);    // DMA continuous mode enabled.
		DMA_Trigger(mMicDacCh1DmaHandle, (uint32_t)&DacDataBuf[0], (uint32_t)&DAC->DAC_CH1_DMA_DATA0, DataLen);
	  TIMER_Open(mMicDacCh1DmaHandle);
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
	mMicDsAdcDmaHandle = DMA2;
	mMicDacCh0DmaHandle = DMA3;
	mMicDacCh1DmaHandle = DMA4;
	mMicDacTimerHandle = TM0;

  TIMER_Open(mMicDacTimerHandle);
	TIMER_SetFreq(mMicDacTimerHandle, (16000*4));	               				// Sample rate: 16K(Hz), for H/W 4x upsampling

  DAC_Open();
	DAC_Scale_Enable();
	DAC_CH0_Init(DAC_TRG_SEL_TM0);
	DAC_CH1_Init(DAC_TRG_SEL_TM0);
  SET_BIT(DAC->CTRL, DAC_CTRL_CH0_POSTWAVE_OUT_CH0 + DAC_CTRL_CH1_POSTWAVE_OUT_CH1);
	DAC_AudioPwm_Open();
	DAC_SetAudPwmGain(31);      // 1x Gain.

	if(AudOutType == AUD_OUT_DAC)                                 // For AUD_OUT_DAC
	{
    DAC_AudioPwm_IP_Disable();
    DAC_VoltageDAC_CH0_Enable();
    DAC_VoltageDAC_CH1_Enable();
	}
}

void F_Delay(int32_t	CNT)
{
	int32_t	x;

	do
	{
	x+=CNT;
	x=x*x;
	CNT--;
	}
	while(CNT!=0);

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
	if(AudOutType == AUD_OUT_PWM)                                 // For AUD_OUT_PWM
	{
    DAC_VoltageDAC_CH0_Disable();
    DAC_VoltageDAC_CH1_Disable();
    DAC_AudioPwm_IP_Enable();
	}
	DacAutoRampUp(SACM_MIXER_Ch0_GetStatus());                   						// DAC auto ramp up

	if((SACM_MIXER_Ch0_GetStatus() & MIXER_CH0_ENABLE_DAC_CH0_FLAG) != 0)
	{
    CLEAR_BIT(DAC->CTRL, DAC_CTRL_DAC_CH0_DMA_ENABLE);                // DAC_Ch0 DMA request disabled.
	  DMA_Init(mMicDacCh0DmaHandle, DMA_REQSEL_DAC_CH0, DMA_SRC_DATA_16B, DMA_SRC_ADDR_INC, DMA_DST_DATA_16B, DMA_DST_ADDR_FIX);
	  SET_BIT(mMicDacCh0DmaHandle->CTRL,DMA_CTRL_DMA_CONTINUE_ENABLE);    // DMA continuous mode enabled.
	  DMA_InstallIsrService(mMicDacCh0DmaHandle, SACM_MIXER_Ch0_SendDac_DmaIsrService);
	  DMA_EnableInt(mMicDacCh0DmaHandle);
	  NVIC_EnableIRQ(DMA3_IRQn);
	  DMA_Trigger(mMicDacCh0DmaHandle, (uint32_t)MixerCh0WorkingRam->DacOutBufPtr, (uint32_t)&DAC->DAC_CH0_DMA_DATA0, MIXER_CH0_FRAME_SIZE);
	}

	if((SACM_MIXER_Ch0_GetStatus() & MIXER_CH0_ENABLE_DAC_CH1_FLAG) != 0)
	{
    CLEAR_BIT(DAC->CTRL, DAC_CTRL_DAC_CH1_DMA_ENABLE);                // DAC_Ch1 DMA request disabled.
		DMA_Init(mMicDacCh1DmaHandle, DMA_REQSEL_DAC_CH1, DMA_SRC_DATA_16B, DMA_SRC_ADDR_INC, DMA_DST_DATA_16B, DMA_DST_ADDR_FIX);
		SET_BIT(mMicDacCh1DmaHandle->CTRL,DMA_CTRL_DMA_CONTINUE_ENABLE);    // DMA continuous mode enabled.
    if((SACM_MIXER_Ch0_GetStatus() & MIXER_CH0_ENABLE_DAC_CH0_FLAG) == 0)
		{
	    DMA_InstallIsrService(mMicDacCh1DmaHandle, SACM_MIXER_Ch0_SendDac_DmaIsrService);
	    DMA_EnableInt(mMicDacCh1DmaHandle);
	    NVIC_EnableIRQ(DMA4_IRQn);
		}
		DMA_Trigger(mMicDacCh1DmaHandle, (uint32_t)MixerCh0WorkingRam->DacOutBufPtr, (uint32_t)&DAC->DAC_CH1_DMA_DATA0, MIXER_CH0_FRAME_SIZE);
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
	if((SACM_MIXER_Ch0_GetStatus() & MIXER_CH0_PLAY_FLAG) != 0)
  {
	  if((SACM_MIXER_Ch0_GetStatus() & MIXER_CH0_ENABLE_DAC_CH0_FLAG) != 0)
	  {
		  DMA_Close(mMicDacCh0DmaHandle);
		}

	  if((SACM_MIXER_Ch0_GetStatus() & MIXER_CH0_ENABLE_DAC_CH1_FLAG) != 0)
	  {
		  DMA_Close(mMicDacCh1DmaHandle);
		}
		DacAutoRampDown(SACM_MIXER_Ch0_GetStatus());	                				// DAC auto ramp down
		DAC_AudioPwm_IP_Disable();
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
	// DMA_Trigger(mMicDsAdcDmaHandle, (uint32_t)&DS_ADC->DATA, (uint32_t)MixerCh0WorkingRam->AdcInBufPtr, MIXER_CH0_FRAME_SIZE);

	if((SACM_MIXER_Ch0_GetStatus() & MIXER_CH0_ENABLE_DAC_CH0_FLAG) != 0)
	{
    DMA_Trigger(mMicDacCh0DmaHandle, (uint32_t)MixerCh0WorkingRam->DacOutBufPtr, (uint32_t)&DAC->DAC_CH0_DMA_DATA0, MIXER_CH0_FRAME_SIZE);
	}

	if((SACM_MIXER_Ch0_GetStatus() & MIXER_CH0_ENABLE_DAC_CH1_FLAG) != 0)
	{
    DMA_Trigger(mMicDacCh1DmaHandle, (uint32_t)MixerCh0WorkingRam->DacOutBufPtr, (uint32_t)&DAC->DAC_CH1_DMA_DATA0, MIXER_CH0_FRAME_SIZE);
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
	// DMA_Trigger(mMicDsAdcDmaHandle, (uint32_t)&DS_ADC->DATA, (uint32_t)DstBufAddr, DataLen);
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

	if((SACM_MIXER_Ch0_GetStatus() & MIXER_CH0_ENABLE_DAC_CH1_FLAG) != 0)
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
  int16_t *SwMixerCh3Ptr;
  int16_t *SwMixerCh4Ptr;
  int16_t *SwMixerCh5Ptr = 0x0000;

  if(((SACM_A1801_GetStatus() & A1801_PLAY_FLAG) != 0) && (SACM_A1801_GetStatus() & A1801_PAUSE_FLAG) == 0)
	{
    if((mA1801Status & A1801_DECODE_END_FLAG)!= 0)
    {
      SACM_A1801_Stop();
      SwMixerCh1Ptr = 0x0000;
    }
    else
    {
      mA1801Status |= A1801_DECODE_WORK_FLAG;
      A1801WorkingRam.A1801DecodeBufPtr = A1801PcmBuffer.A1801PcmBuffer;
      SACM_A1801_ServiceLoop();
      SwMixerCh1Ptr = &A1801PcmBuffer.A1801PcmBuffer[0];
      APP_SwVolCtrl_VolProcess(0, SwMixerCh1Ptr, MIXER_CH0_FRAME_SIZE);
    }
	}
	else
	{
    SwMixerCh1Ptr = 0x0000;
	}

	// Ch2 Mixer
	if(((SACM_A1801_CH2_GetStatus() & A1801_PLAY_FLAG) != 0) && (SACM_A1801_CH2_GetStatus() & A1801_PAUSE_FLAG) == 0)
	{
    if((mA1801Ch2Status & A1801_DECODE_END_FLAG)!= 0)
    {
      SACM_A1801_CH2_Stop();
      SwMixerCh2Ptr = 0x0000;
    }
    else
    {
      mA1801Ch2Status |= A1801_DECODE_WORK_FLAG;
      A1801Ch2WorkingRam.A1801DecodeBufPtr = A1801Ch2PcmBuffer.A1801PcmBuffer;
      SACM_A1801_CH2_ServiceLoop();
      SwMixerCh2Ptr = &A1801Ch2PcmBuffer.A1801PcmBuffer[0];
      APP_SwVolCtrl_VolProcess(1, SwMixerCh2Ptr, MIXER_CH0_FRAME_SIZE);
    }
	}
	else
	{
    SwMixerCh2Ptr = 0x0000;
	}

  // Ch3 Mixer
	if(((SACM_A1801_CH3_GetStatus() & A1801_PLAY_FLAG) != 0) && (SACM_A1801_CH3_GetStatus() & A1801_PAUSE_FLAG) == 0)
	{
    if((mA1801Ch3Status & A1801_DECODE_END_FLAG)!= 0)
    {
      SACM_A1801_CH3_Stop();
      SwMixerCh3Ptr = 0x0000;
    }
    else
    {
      mA1801Ch3Status |= A1801_DECODE_WORK_FLAG;
      A1801Ch3WorkingRam.A1801DecodeBufPtr = A1801Ch3PcmBuffer.A1801PcmBuffer;
      SACM_A1801_CH3_ServiceLoop();
      SwMixerCh3Ptr = &A1801Ch3PcmBuffer.A1801PcmBuffer[0];
      APP_SwVolCtrl_VolProcess(2, SwMixerCh3Ptr, MIXER_CH0_FRAME_SIZE);
    }
	}
	else
	{
    SwMixerCh3Ptr = 0x0000;
	}

  // Ch4 Mixer
	if(((SACM_A1801_CH4_GetStatus() & A1801_PLAY_FLAG) != 0) && (SACM_A1801_CH4_GetStatus() & A1801_PAUSE_FLAG) == 0)
	{
    if((mA1801Ch4Status & A1801_DECODE_END_FLAG)!= 0)
    {
      SACM_A1801_CH4_Stop();
      SwMixerCh4Ptr = 0x0000;
    }
    else
    {
      mA1801Ch4Status |= A1801_DECODE_WORK_FLAG;
      A1801Ch4WorkingRam.A1801DecodeBufPtr = A1801Ch4PcmBuffer.A1801PcmBuffer;
      SACM_A1801_CH4_ServiceLoop();
      SwMixerCh4Ptr = &A1801Ch4PcmBuffer.A1801PcmBuffer[0];
      APP_SwVolCtrl_VolProcess(3, SwMixerCh4Ptr, MIXER_CH0_FRAME_SIZE);
    }
	}
	else
	{
    SwMixerCh4Ptr = 0x0000;
	}
  APP_SwMixer_5ch(SwMixerCh1Ptr, SwMixerCh2Ptr, SwMixerCh3Ptr, SwMixerCh4Ptr, SwMixerCh5Ptr, DstBufAddr, MIXER_CH0_FRAME_SIZE);
}



