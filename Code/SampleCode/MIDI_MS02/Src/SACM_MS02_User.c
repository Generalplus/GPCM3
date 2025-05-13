/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   SACM_MS02_User.c
 * @Version:
 *   V1.0.0
 * @Date:
 *   March 28, 2022
 * @Abstract:
 *
 **************************************************************************************************/


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "GPCM3_FM1.h"
#include "SACM_MS02_User.h"
#include "MS02_FileMerger.h"
#include "APP_SwVolumeControl.h"


/*---------------------------------------------------------------------------------------
 * Gobal Variable Declaration Area
 *---------------------------------------------------------------------------------------*/
TIMER_TYPE_DEF *mMS02DacTimerHandle;
DMA_TYPE_DEF   *mMS02DacCh0DmaHandle;
DMA_TYPE_DEF   *mMS02DacCh1DmaHandle;

uint8_t AUD_OUT_TYPE;

/*---------------------------------------------------------------------------------------
 * User Subroutine Area
 *---------------------------------------------------------------------------------------*/

/**
 * @brief
 *   Get Midi data start address
 * @param
 *   MidiIdx [in]: Midi index
 * @return
 *   None.
 */
int16_t* GetMidiStartAddr(uint16_t MidiIdx)
{
	return (int16_t*) ((*((uint32_t *)(SEC_START_ADDR) + MidiIdx)) + SEC_START_ADDR);
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   Midi number: The number of Midi in SPI flash
 */
uint16_t GetMidiNum()
{
	return *(int32_t*) (SEC_START_ADDR);
}

/**
 * @brief
 *   Get MS02 Library start address
 * @param
 *   LibIdx [in]: MIDI Lib index
 * @return
 *   None.
 */
int16_t* GetMS02LibStartAddr(uint16_t LibIdx)
{
	return (int16_t*) ((*((uint32_t *)(SEC_START_ADDR) + LibIdx)) + SEC_START_ADDR);
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
	  TIMER_Close(mMS02DacTimerHandle);
	  DMA_Init(mMS02DacCh0DmaHandle, DMA_REQSEL_DAC_CH0, DMA_SRC_DATA_16B, DMA_SRC_ADDR_INC, DMA_DST_DATA_16B, DMA_DST_ADDR_FIX);
	  SET_BIT(mMS02DacCh0DmaHandle->CTRL,DMA_CTRL_DMA_CONTINUE_ENABLE);    // DMA continuous mode enabled.
	  DMA_Trigger(mMS02DacCh0DmaHandle, (uint32_t)&DacDataBuf[0], (uint32_t)&DAC->DAC_CH0_DMA_DATA0, DataLen);
	  TIMER_Open(mMS02DacTimerHandle);
		while(DMA_CheckBusy(mMS02DacCh0DmaHandle) != 0);                // Wait DMAx busy
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
	  TIMER_Close(mMS02DacTimerHandle);
		DMA_Init(mMS02DacCh1DmaHandle, DMA_REQSEL_DAC_CH1, DMA_SRC_DATA_16B, DMA_SRC_ADDR_INC, DMA_DST_DATA_16B, DMA_DST_ADDR_FIX);
		SET_BIT(mMS02DacCh1DmaHandle->CTRL,DMA_CTRL_DMA_CONTINUE_ENABLE);    // DMA continuous mode enabled.
		DMA_Trigger(mMS02DacCh1DmaHandle, (uint32_t)&DacDataBuf[0], (uint32_t)&DAC->DAC_CH1_DMA_DATA0, DataLen);
		TIMER_Open(mMS02DacTimerHandle);
		while(DMA_CheckBusy(mMS02DacCh1DmaHandle) != 0);                // Wait DMAx busy
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
 *   MS02Status [in]: MS02 status
 * @return
 *   None.
 */
void static DacAutoRampUp(uint16_t MS02Status)
{
  if((MS02Status & MS02_AUTO_RAMP_UP_ENABLE_FLAG) != 0)
	{
		if((MS02Status & MS02_ENABLE_DAC_CH0_FLAG) != 0)
		{
			RampUpDacCh0();
		}

		if((MS02Status & MS02_ENABLE_DAC_CH1_FLAG) != 0)
		{
			RampUpDacCh1();
		}
	}
}

/**
 * @brief
 *
 * @param
 *   MS02Status [in]: MS02 status
 * @return
 *   None.
 */
void static DacAutoRampDown(uint16_t MS02Status)
{
  if((MS02Status & MS02_AUTO_RAMP_DOWN_ENABLE_FLAG) != 0)
	{
		if((MS02Status & MS02_ENABLE_DAC_CH0_FLAG) != 0)
		{
			RampDownDacCh0();
		}

		if((MS02Status & MS02_ENABLE_DAC_CH1_FLAG) != 0)
		{
			RampDownDacCh1();
		}
	}
}

/**
 * @brief
 *
 * @param
 *   MS02Status [in]: MS02 status
 * @return
 *   Return 0 if Ramp Up/ down finished
 */

uint8_t MS02AutoRampUpDown(uint16_t MS02Status)
{
	DacAutoRampDown(MS02Status);

	return 0;
}


/*---------------------------------------------------------------------------------------
 * Callback Function Area
 *---------------------------------------------------------------------------------------*/
/**
 * @brief
 *   Callback function. Implemented by user.
 *   When user call API function SACM_MS02_Initial, this function is called by MS02 Library.
 * @param
 *  *MS02WorkingRam [in]: MS02 library working RAM address.
 *  DacChannelNo    [in]: MS02_CH0/ MS02_CH1
 *  Timer_SR        [in]:
 * @return
 *   None.
 */
void MS02_CB_Init(const SACM_MS02_WORKING_RAM *MS02WorkingRam, uint8_t AudOutType, uint8_t DacChannelNo, uint32_t Timer_SR)
{
	mMS02DacCh0DmaHandle = DMA3;
	mMS02DacCh1DmaHandle = DMA4;
	mMS02DacTimerHandle = TM0;

  TIMER_Open(mMS02DacTimerHandle);
	TIMER_SetFreq(mMS02DacTimerHandle, Timer_SR);

  // DAC init
	DAC_Open();
	DAC_Scale_Enable();
  SET_BIT(DAC->CTRL, DAC_CTRL_CH0_POSTWAVE_OUT_CH0 + DAC_CTRL_CH1_POSTWAVE_OUT_CH1);
  DAC_AudioPwm_Open();

  if(DacChannelNo == MS02_CH0)
	{
	  DAC_CH0_Init(DAC_TRG_SEL_TM0);
	}
	else
	{
    DAC_CH1_Init(DAC_TRG_SEL_TM0);
	}

	if (AudOutType == AUD_OUT_PWM)
	{
    AUD_OUT_TYPE =  AUD_OUT_PWM;
	}
	else
	{
    AUD_OUT_TYPE =  AUD_OUT_DAC;
	}

}

/**
 * @brief
 *   Callback function. To be implemented by user.
 *   When user call API function SACM_MS02_Play, this function is called by MS02 Library.
 * @param
 *  *MS02WorkingRam [in]: MS02 library working RAM address.
 * @return
 *   None.
 */
void MS02_CB_StartPlay(const SACM_MS02_WORKING_RAM *MS02WorkingRam)
{
	if(AUD_OUT_TYPE == AUD_OUT_PWM)                                   // For AUD_OUT_PWM, Kim 2020.12.02
	{
    DAC_VoltageDAC_CH0_Disable();
    DAC_VoltageDAC_CH1_Disable();
    DAC_AudioPwm_IP_Enable();
	}
	else                                                              // For AUD_OUT_DAC, Kim 2020.12.02
	{
    DAC_AudioPwm_IP_Disable();
    DAC_VoltageDAC_CH0_Enable();
    DAC_VoltageDAC_CH1_Enable();
	}
  DacAutoRampUp(SACM_MS02_GetStatus());                             // DAC auto ramp up

	if((SACM_MS02_GetStatus() & MS02_ENABLE_DAC_CH0_FLAG) != 0)
	{
	  CLEAR_BIT(DAC->CTRL, DAC_CTRL_DAC_CH0_DMA_ENABLE);              // DAC_Ch0 DMA request disabled.
	  TIMER_Close(mMS02DacTimerHandle);
    while(DMA_CheckBusy(mMS02DacCh0DmaHandle) != 0);                // Check DMAx busy
	  DMA_Init(mMS02DacCh0DmaHandle, DMA_REQSEL_DAC_CH0, DMA_SRC_DATA_16B, DMA_SRC_ADDR_INC, DMA_DST_DATA_16B, DMA_DST_ADDR_FIX);
	  SET_BIT(mMS02DacCh0DmaHandle->CTRL,DMA_CTRL_DMA_CONTINUE_ENABLE);    // DMA continuous mode enabled.
	  DMA_InstallIsrService(mMS02DacCh0DmaHandle, SACM_MS02_DmaIsrService);
	  DMA_EnableInt(mMS02DacCh0DmaHandle);
	  NVIC_EnableIRQ(DMA3_IRQn);
	  DMA_Trigger(mMS02DacCh0DmaHandle, (uint32_t)MS02WorkingRam->DecodeOutBuf2Ptr, (uint32_t)&DAC->DAC_CH0_DMA_DATA0, MS02_OUT_BUFFER_SIZE);
	  TIMER_Open(mMS02DacTimerHandle);
	}

	if((SACM_MS02_GetStatus() & MS02_ENABLE_DAC_CH1_FLAG) != 0)
	{
	  CLEAR_BIT(DAC->CTRL, DAC_CTRL_DAC_CH1_DMA_ENABLE);              // DAC_Ch1 DMA request disabled.
	  TIMER_Close(mMS02DacTimerHandle);
    while(DMA_CheckBusy(mMS02DacCh1DmaHandle) != 0);                // Check DMAx busy
		DMA_Init(mMS02DacCh1DmaHandle, DMA_REQSEL_DAC_CH1, DMA_SRC_DATA_16B, DMA_SRC_ADDR_INC, DMA_DST_DATA_16B, DMA_DST_ADDR_FIX);
		SET_BIT(mMS02DacCh1DmaHandle->CTRL,DMA_CTRL_DMA_CONTINUE_ENABLE);    // DMA continuous mode enabled.
    if((SACM_MS02_GetStatus() & MS02_ENABLE_DAC_CH0_FLAG) == 0)
		{
	    DMA_InstallIsrService(mMS02DacCh1DmaHandle, SACM_MS02_DmaIsrService);
	    DMA_EnableInt(mMS02DacCh1DmaHandle);
	    NVIC_EnableIRQ(DMA4_IRQn);
		}
		DMA_Trigger(mMS02DacCh1DmaHandle, (uint32_t)MS02WorkingRam->DecodeOutBuf2Ptr, (uint32_t)&DAC->DAC_CH1_DMA_DATA0, MS02_OUT_BUFFER_SIZE);
		TIMER_Open(mMS02DacTimerHandle);
	}
}

/**
 * @brief
 *
 * @param
 *  *MS02WorkingRam [in]: MS02 library working RAM address.
 * @return
 *   None.
 */
uint8_t MS02_CB_StopPlay(const SACM_MS02_WORKING_RAM *MS02WorkingRam)
{
	if((SACM_MS02_GetStatus() & MS02_PLAY_FLAG) != 0)
  {
	  if((SACM_MS02_GetStatus() & MS02_ENABLE_DAC_CH0_FLAG) != 0)
	  {
		  DMA_Close(mMS02DacCh0DmaHandle);
		}

	  if((SACM_MS02_GetStatus() & MS02_ENABLE_DAC_CH1_FLAG) != 0)
	  {
		  DMA_Close(mMS02DacCh1DmaHandle);
		}

		DacAutoRampDown(SACM_MS02_GetStatus());	                       // DAC auto ramp down
    if(AUD_OUT_TYPE == AUD_OUT_PWM)                                // For AUD_OUT_PWM, Kim 2020.12.02
    {
      DAC_AudioPwm_IP_Disable();
    }
  }

	return 0;
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When user call API function SACM_MS02_Pause, this function is called by MS02 Library.
 * @param
 *  *MS02WorkingRam [in]: MS02 library working RAM address.
 * @return
 *   None.
 */
void MS02_CB_Pause(const SACM_MS02_WORKING_RAM *MS02WorkingRam)
{
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When user call API function SACM_MS02_Resume, this function is Called by MS02 Library.
 * @param
 *  *MS02WorkingRam [in]: MS02 library working RAM address.
 * @return
 *   None.
 */
void MS02_CB_Resume(const SACM_MS02_WORKING_RAM *MS02WorkingRam)
{
	if((SACM_MS02_GetStatus() & MS02_ENABLE_DAC_CH0_FLAG) != 0)
	{
    DMA_Trigger(mMS02DacCh0DmaHandle, (uint32_t)MS02WorkingRam->DecodeOutBuf1Ptr, (uint32_t)&DAC->DAC_CH0_DMA_DATA0, MS02_OUT_BUFFER_SIZE);
	}

	if((SACM_MS02_GetStatus() & MS02_ENABLE_DAC_CH1_FLAG) != 0)
	{
		DMA_Trigger(mMS02DacCh1DmaHandle, (uint32_t)MS02WorkingRam->DecodeOutBuf1Ptr, (uint32_t)&DAC->DAC_CH1_DMA_DATA0, MS02_OUT_BUFFER_SIZE);
	}
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When playing, this function is called by MS02 Library to get encoded data.
 * @param
 *  *MS02WorkingRam [in]: MS02 library working RAM address.
 *  *SrcDataAddr [in]: Source data address pointer.
 *  *DstBufAddr [out]: Destination buffer address pointer.
 *   DataLen [in]: Data length (halfword)
 * @return
 *   None.
 */
void MS02_CB_GetData(const SACM_MS02_WORKING_RAM *MS02WorkingRam, int16_t *DstBufAddr, int16_t *SrcDataAddr, uint16_t DataLen)
{
	uint16_t iCount;

	for(iCount = 0; iCount < DataLen; iCount++)
  {
    DstBufAddr[iCount] = SrcDataAddr[iCount];
	}
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When playing, this function is called by MS02 Library to trigger DAC DMA.
 * @param
 *  *MS02WorkingRam [in]: MS02 working RAM pointer
 *  *SrcBufAddr [in]: Source buffer address pointer.
 *   DataLen [in]: Data length (halfword)
 * @return
 *   None.
 */
void MS02_CB_SendDac_DmaIsr(const SACM_MS02_WORKING_RAM *MS02WorkingRam, int16_t *SrcBufAddr, uint16_t DataLen)
{
	if((SACM_MS02_GetStatus() & MS02_ENABLE_DAC_CH0_FLAG) != 0)
	{
		while(DMA_CheckBusy(mMS02DacCh0DmaHandle) != 0);                // Check DMAx busy
		DMA_Trigger(mMS02DacCh0DmaHandle, (uint32_t)SrcBufAddr, (uint32_t)&DAC->DAC_CH0_DMA_DATA0, DataLen);
	}

	if((SACM_MS02_GetStatus() & MS02_ENABLE_DAC_CH1_FLAG) != 0)
	{
		while(DMA_CheckBusy(mMS02DacCh1DmaHandle) != 0);                // Check DMAx busy
		DMA_Trigger(mMS02DacCh1DmaHandle, (uint32_t)SrcBufAddr, (uint32_t)&DAC->DAC_CH1_DMA_DATA0, DataLen);
	}
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When playing, this function is called by MS02 library to decode data.
 * @param
 *   *DstBufAddr [out]: Destination buffer address pointer.
 *    OutBufSize  [in]: MS02_OUT_BUFFER_SIZE = 320(Don't modify)
 * @return
 *   None.
 */
void MS02_CB_DecodeProcess(int16_t* DstBufAddr, int16_t OutBufSize)
{
  SACM_MS02_DecodeProcess(DstBufAddr, OutBufSize);
	APP_SwVolCtrl_VolProcess(0, DstBufAddr, MS02_OUT_BUFFER_SIZE);
}

/**
 * @brief
 *   Call back function for User event
 *
 * @param
 *    R_CallBackEvent: bit[15:12] = 0x9(Fix)
 *    R_CMD_Code: parameter of the callback event
 *                bit[15:8] = Main-Index
 *                bit[7:0]  = sub-Index
 * @return
 *   None.
 */
void MS02_CB_SongEvent(short R_CallBack_Event, short R_CMD_Code)
{
  //-----------------------
  //CallBack Event processed by user
  //-----------------------
  short mSongEvent;
  mSongEvent = R_CMD_Code;
}

/**
 * @brief
 *   Call back function for Note on events
 *
 * @param
 *    R_CMD_Code:
 *		         bit[15:12] = 0x1 : Note Event
 * 		         bit[11:8] Channel 0~15
 *		         bit[7:0]  Pitch
 *    R_Duration:
 * @return
 *   None.
 */
void MS02_CB_NoteOnEvent(short R_CMD_Code, short R_Duration)
{
  //-----------------------
  // Note on Event processed by user
  //-----------------------
  short mNoteEventCMD;
  if((R_CMD_Code & 0x0F00) == 0x0900)   // Ex. Only check Ch.9 note. (Note:User can change according to your needs.)
  {
    mNoteEventCMD = R_CMD_Code;
  }
  if(R_CMD_Code == 0x1048)
  {
    mNoteEventCMD = R_Duration;
  }


}
