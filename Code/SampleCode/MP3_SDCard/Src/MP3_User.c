/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   MP3_User.c
 * @Version:
 *   V1.0.2
 * @Date:
 *   December 14, 2021
 * @Abstract:
 *
 **************************************************************************************************/


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "GPCM3_FM1.h"
#include "MP3_User.h"
#include "MP3_FileMerger.h"
extern uint32_t Fi_size;
extern uint8_t PlaySD;
extern void GetMP3Data(uint8_t *DstBufAddr, uint8_t *SrcDataAddr, uint16_t DataLen);
/*---------------------------------------------------------------------------------------
 * Gobal Variable Declaration Area
 *---------------------------------------------------------------------------------------*/
TIMER_TYPE_DEF *mMp3DacTimerHandle;
DMA_TYPE_DEF *mMp3DacCh0DmaHandle;
DMA_TYPE_DEF *mMp3DacCh1DmaHandle;


/*---------------------------------------------------------------------------------------
 * User Subroutine Area
 *---------------------------------------------------------------------------------------*/

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   MP3 number: The number of MP3 in SPI flash
 */
uint16_t GetMp3Num()
{
	return *(int32_t*) (SEC_START_ADDR);
}

/**
 * @brief
 *   Get MP3 data start address
 * @param
 *   Mp3Index [in]: MP3 index
 * @return
 *   MP3 start address
 */
uint8_t* GetMp3StartAddr(uint16_t Mp3Index)
{
	//return (uint8_t*) ((*((uint32_t *)(SEC_START_ADDR) + Mp3Index)) + SEC_START_ADDR);
	if(PlaySD)
	{
		return 0;
	}
	else
	{
		return (uint8_t*) ((*((uint32_t *)(SEC_START_ADDR) + Mp3Index)) + SEC_START_ADDR);
	}
}

/**
 * @brief
 *   Get MP3 data length
 * @param
 *   Mp3Index [in]: MP3 index
 * @return
 *   MP3 file length
 */
int32_t GetMp3Length(uint16_t Mp3Index)
{
	//return ((*((uint32_t *)(SEC_START_ADDR) + Mp3Index + 1)) + SEC_START_ADDR) - ((*((uint32_t *)(SEC_START_ADDR) + Mp3Index)) + SEC_START_ADDR);
			if(PlaySD)
	{
		return Fi_size;
	}
	else
	{
		return ((*((uint32_t *)(SEC_START_ADDR) + Mp3Index + 1)) + SEC_START_ADDR) - ((*((uint32_t *)(SEC_START_ADDR) + Mp3Index)) + SEC_START_ADDR);
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
void static RampUpDacCh0()
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
	  CLEAR_BIT(DAC->CTRL, DAC_CTRL_DAC_CH0_DMA_ENABLE);                 // DAC_Ch0 DMA request disabled.
    TIMER_Close(mMp3DacTimerHandle);
	  DMA_Init(mMp3DacCh0DmaHandle, DMA_REQSEL_DAC_CH0, DMA_SRC_DATA_16B, DMA_SRC_ADDR_INC, DMA_DST_DATA_16B, DMA_DST_ADDR_FIX);
	  SET_BIT(mMp3DacCh0DmaHandle->CTRL,DMA_CTRL_DMA_CONTINUE_ENABLE);    // DMA continuous mode enabled.
	  DMA_Trigger(mMp3DacCh0DmaHandle, (uint32_t)&DacDataBuf[0], (uint32_t)&DAC->DAC_CH0_DMA_DATA0, DataLen);
		TIMER_Open(mMp3DacTimerHandle);
		while(DMA_CheckBusy(mMp3DacCh0DmaHandle) != 0);                // Wait DMAx busy
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
	  CLEAR_BIT(DAC->CTRL, DAC_CTRL_DAC_CH1_DMA_ENABLE);                 // DAC_Ch1 DMA request disabled.
    TIMER_Close(mMp3DacTimerHandle);
		DMA_Init(mMp3DacCh1DmaHandle, DMA_REQSEL_DAC_CH1, DMA_SRC_DATA_16B, DMA_SRC_ADDR_INC, DMA_DST_DATA_16B, DMA_DST_ADDR_FIX);
		SET_BIT(mMp3DacCh1DmaHandle->CTRL,DMA_CTRL_DMA_CONTINUE_ENABLE);    // DMA continuous mode enabled.
		DMA_Trigger(mMp3DacCh1DmaHandle, (uint32_t)&DacDataBuf[0], (uint32_t)&DAC->DAC_CH1_DMA_DATA0, DataLen);
		TIMER_Open(mMp3DacTimerHandle);
		while(DMA_CheckBusy(mMp3DacCh1DmaHandle) != 0);                // Wait DMAx busy
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
 *   Mp3Status [in]: MP3 status
 * @return
 *   None.
 */
void static DacAutoRampUp(uint16_t Mp3Status)
{
  if((Mp3Status & MP3_AUTO_RAMP_UP_ENABLE_FLAG) != 0)
	{
		if((Mp3Status & MP3_ENABLE_DAC_CH0_FLAG) != 0)
		{
			RampUpDacCh0();
		}

		if((Mp3Status & MP3_ENABLE_DAC_CH1_FLAG) != 0)
		{
			RampUpDacCh1();
		}
	}
}

/**
 * @brief
 *
 * @param
 *   Mp3Status [in]: MP3 status
 * @return
 *   None.
 */
void static DacAutoRampDown(uint16_t Mp3Status)
{
  if((Mp3Status & MP3_AUTO_RAMP_DOWN_ENABLE_FLAG) != 0)
	{
		if((Mp3Status & MP3_ENABLE_DAC_CH0_FLAG) != 0)
		{
			RampDownDacCh0();
		}

		if((Mp3Status & MP3_ENABLE_DAC_CH1_FLAG) != 0)
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
 *   When user call API function MP3_Initial, this function is called by MP3 Library.
 * @param
 *  *Mp3LibWorkingRam [in]: MP3 library working RAM address.
 *   SampleRate [in]: MP3 sample rate
 * @return
 *   None.
 */
void MP3_CB_Init(const MP3_LIB_WORKING_RAM *Mp3LibWorkingRam)
{
	mMp3DacCh0DmaHandle = DMA3;
	mMp3DacCh1DmaHandle = DMA4;
	mMp3DacTimerHandle = TM0;

  // DAC init
	DAC_Open();
	DAC_Scale_Enable();
	DAC_CH0_Init(DAC_TRG_SEL_TM0);
	DAC_CH1_Init(DAC_TRG_SEL_TM0);
	SET_BIT(DAC->CTRL, DAC_CTRL_CH0_POSTWAVE_OUT_CH0 + DAC_CTRL_CH1_POSTWAVE_OUT_CH1);

	// PWM init
	DAC_AudioPwm_Open();
	DAC_SetAudPwmGain(31);
}

/**
 * @brief
 *   Callback function. To be implemented by user.
 *   This function is called by MP3 Library.
 * @param
 *  *Mp3LibWorkingRam [in]: MP3 library working RAM address.
 * @return
 *   None.
 */
void MP3_CB_StartPlay(const MP3_LIB_WORKING_RAM *Mp3LibWorkingRam, const int32_t SampleRate)
{
  TIMER_Open(mMp3DacTimerHandle);
	TIMER_SetFreq(mMp3DacTimerHandle, (SampleRate * 4));	             // DAC sample rate: MP3 sample rate x 4, for H/W 4x upsampling

	DAC_AudioPwm_IP_Enable();
	DacAutoRampUp(MP3_GetStatus());                                    // DAC auto ramp up

	if((MP3_GetStatus() & MP3_ENABLE_DAC_CH0_FLAG) != 0)
	{
    CLEAR_BIT(DAC->CTRL, DAC_CTRL_DAC_CH0_DMA_ENABLE);                // DAC_Ch0 DMA request disabled.
	  DMA_Init(mMp3DacCh0DmaHandle, DMA_REQSEL_DAC_CH0, DMA_SRC_DATA_16B, DMA_SRC_ADDR_INC, DMA_DST_DATA_16B, DMA_DST_ADDR_FIX);
	  SET_BIT(mMp3DacCh0DmaHandle->CTRL,DMA_CTRL_DMA_CONTINUE_ENABLE);    // DMA continuous mode enabled.
	  DMA_InstallIsrService(mMp3DacCh0DmaHandle, MP3_DmaIsrService);
	  DMA_EnableInt(mMp3DacCh0DmaHandle);
	  NVIC_EnableIRQ(DMA3_IRQn);
		DMA_Trigger(mMp3DacCh0DmaHandle, (uint32_t)&Mp3LibWorkingRam->PcmRingBuffer[0][0], (uint32_t)&DAC->DAC_CH0_DMA_DATA0, MP3_PCM_BUF_LEN);
	}

	if((MP3_GetStatus() & MP3_ENABLE_DAC_CH1_FLAG) != 0)
	{
    CLEAR_BIT(DAC->CTRL, DAC_CTRL_DAC_CH1_DMA_ENABLE);                // DAC_Ch1 DMA request disabled.
		DMA_Init(mMp3DacCh1DmaHandle, DMA_REQSEL_DAC_CH1, DMA_SRC_DATA_16B, DMA_SRC_ADDR_INC, DMA_DST_DATA_16B, DMA_DST_ADDR_FIX);
		SET_BIT(mMp3DacCh1DmaHandle->CTRL,DMA_CTRL_DMA_CONTINUE_ENABLE);    // DMA continuous mode enabled.
    if((MP3_GetStatus() & MP3_ENABLE_DAC_CH0_FLAG) == 0)
		{
	    DMA_InstallIsrService(mMp3DacCh1DmaHandle, MP3_DmaIsrService);
	    DMA_EnableInt(mMp3DacCh1DmaHandle);
	    NVIC_EnableIRQ(DMA4_IRQn);
		}
		DMA_Trigger(mMp3DacCh1DmaHandle, (uint32_t)&Mp3LibWorkingRam->PcmRingBuffer[0][0], (uint32_t)&DAC->DAC_CH1_DMA_DATA0, MP3_PCM_BUF_LEN);
	}
}

/**
 * @brief
 *
 * @param
 *  *Mp3LibWorkingRam [in]: MP3 library working RAM pointer
 * @return
 *   None.
 */
void MP3_CB_StopPlay(const MP3_LIB_WORKING_RAM *Mp3LibWorkingRam)
{
	if((MP3_GetStatus() & MP3_PLAY_FLAG) != 0)
  {
	  DacAutoRampDown(MP3_GetStatus());	                               // DAC auto ramp down

		if((MP3_GetStatus() & MP3_ENABLE_DAC_CH0_FLAG) != 0)
	  {
		  DMA_Close(mMp3DacCh0DmaHandle);
		}

	  if((MP3_GetStatus() & MP3_ENABLE_DAC_CH1_FLAG) != 0)
	  {
		  DMA_Close(mMp3DacCh1DmaHandle);
		}

		DAC_AudioPwm_IP_Disable();
  }
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When user call API function MP3_Pause, this function is called by MP3 Library.
 * @param
 *  *Mp3LibWorkingRam [in]: MP3 library working RAM pointer
 * @return
 *   None.
 */
void MP3_CB_Pause(const MP3_LIB_WORKING_RAM *Mp3LibWorkingRam)
{
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When user call API function MP3_Resume, this function is Called by MP3 Library.
 * @param
 *  *Mp3LibWorkingRam [in]: MP3 library working RAM pointer
 * @return
 *   None.
 */
void MP3_CB_Resume(const MP3_LIB_WORKING_RAM *Mp3LibWorkingRam)
{
	if((MP3_GetStatus() & MP3_ENABLE_DAC_CH0_FLAG) != 0)
	{
		DMA_Trigger(mMp3DacCh0DmaHandle, (uint32_t)(uint32_t)&Mp3LibWorkingRam->PcmRingBuffer[Mp3LibWorkingRam->RdPcmRingIdx][0], (uint32_t)&DAC->DAC_CH0_DMA_DATA0, MP3_PCM_BUF_LEN);
	}

	if((MP3_GetStatus() & MP3_ENABLE_DAC_CH1_FLAG) != 0)
	{
		DMA_Trigger(mMp3DacCh1DmaHandle, (uint32_t)(uint32_t)&Mp3LibWorkingRam->PcmRingBuffer[Mp3LibWorkingRam->RdPcmRingIdx][0], (uint32_t)&DAC->DAC_CH1_DMA_DATA0, MP3_PCM_BUF_LEN);
	}
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When playing, this function is called by MP3 Library to get mp3 data.
 * @param
 *  *Mp3LibWorkingRam [in]: MP3 library working RAM pointer
 *  *SrcDataAddr [in]: Source data address pointer.
 *  *DstBufAddr [out]: Destination buffer address pointer.
 *   DataLen [in]: Data length (halfword)
 * @return
 *   None.
 */
void MP3_CB_GetData(const MP3_LIB_WORKING_RAM *Mp3LibWorkingRam, uint8_t *DstBufAddr, uint8_t *SrcDataAddr, uint16_t DataLen)
{
	uint32_t iCount;
	if(PlaySD)
	{
		SET_BIT(GPIOA->OBUF,BIT16);
		GetMP3Data(DstBufAddr,SrcDataAddr,DataLen);
		CLEAR_BIT(GPIOA->OBUF,BIT16);
	}
	else
	{
		for(iCount = 0; iCount < DataLen; iCount++)
		{
			DstBufAddr[iCount] = SrcDataAddr[iCount];
		}
	}
//	for(iCount = 0; iCount < DataLen; iCount++)
//  {
//    DstBufAddr[iCount] = SrcDataAddr[iCount];
//	}
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When playing, this function is called by MP3 Library to trigger DAC DMA.
 * @param
 *  *Mp3LibWorkingRam [in]: MP3 library working RAM pointer
 *  *SrcBufAddr [in]: Source buffer address pointer.
 *   DataLen [in]: Data length (halfword)
 * @return
 *   None.
 */
void MP3_CB_SendDac_DmaIsr(const MP3_LIB_WORKING_RAM *Mp3LibWorkingRam, int16_t *SrcBufAddr, uint16_t DataLen)
{
	if((MP3_GetStatus() & MP3_ENABLE_DAC_CH0_FLAG) != 0)
	{
		while(DMA_CheckBusy(mMp3DacCh0DmaHandle) != 0);                  // Check DMAx busy
		DMA_Trigger(mMp3DacCh0DmaHandle, (uint32_t)SrcBufAddr, (uint32_t)&DAC->DAC_CH0_DMA_DATA0, DataLen);
	}

	if((MP3_GetStatus() & MP3_ENABLE_DAC_CH1_FLAG) != 0)
	{
		while(DMA_CheckBusy(mMp3DacCh1DmaHandle) != 0);                  // Check DMAx busy
    DMA_Trigger(mMp3DacCh1DmaHandle, (uint32_t)SrcBufAddr, (uint32_t)&DAC->DAC_CH1_DMA_DATA0, DataLen);
	}
}
