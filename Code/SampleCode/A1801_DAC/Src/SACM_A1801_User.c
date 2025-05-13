/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   SACM_A1801_User.c
 * @Version:
 *   V1.0.3.1
 * @Date:
 *   April 15, 2022
 * @Abstract:
 *
 **************************************************************************************************/


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "GPCM3_FM1.h"
#include "SACM_A1801_User.h"
#include "A1801_FileMerger.h"
#include "APP_SwVolumeControl.h"


/*---------------------------------------------------------------------------------------
 * Gobal Variable Declaration Area
 *---------------------------------------------------------------------------------------*/
TIMER_TYPE_DEF *mA1801DacTimerHandle;
DMA_TYPE_DEF *mA1801DacCh0DmaHandle;
DMA_TYPE_DEF *mA1801DacCh1DmaHandle;

extern uint8_t AudOutType;


/*---------------------------------------------------------------------------------------
 * User Subroutine Area
 *---------------------------------------------------------------------------------------*/

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   Speech number: The number of speech in SPI flash
 */
uint16_t A1801_User_GetA1801Num()
{
	return *(int32_t*) (SEC_START_ADDR);
}

/**
 * @brief
 *   Get speech data start address
 * @param
 *   SpeechIdx [in]: Speech index
 * @return
 *   Speech data start address
 */
int16_t* A1801_User_GetA1801StartAddr(uint16_t SpeechIdx)
{
	return (int16_t*) ((*((uint32_t *)(SEC_START_ADDR) + SpeechIdx)) + SEC_START_ADDR);
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
	  CLEAR_BIT(DAC->CTRL, DAC_CTRL_DAC_CH0_DMA_ENABLE);                // DAC_Ch0 DMA request disabled.
		TIMER_Close(mA1801DacTimerHandle);
	  DMA_Init(mA1801DacCh0DmaHandle, DMA_REQSEL_DAC_CH0, DMA_SRC_DATA_16B, DMA_SRC_ADDR_INC, DMA_DST_DATA_16B, DMA_DST_ADDR_FIX);
	  SET_BIT(mA1801DacCh0DmaHandle->CTRL,DMA_CTRL_DMA_CONTINUE_ENABLE);    // DMA continuous mode enabled.
	  DMA_Trigger(mA1801DacCh0DmaHandle, (uint32_t)&DacDataBuf[0], (uint32_t)&DAC->DAC_CH0_DMA_DATA0, DataLen);
		TIMER_Open(mA1801DacTimerHandle);
		while(DMA_CheckBusy(mA1801DacCh0DmaHandle) != 0);                // Wait DMAx busy
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
	  CLEAR_BIT(DAC->CTRL, DAC_CTRL_DAC_CH1_DMA_ENABLE);                // DAC_Ch1 DMA request disabled.
		TIMER_Close(mA1801DacTimerHandle);
		DMA_Init(mA1801DacCh1DmaHandle, DMA_REQSEL_DAC_CH1, DMA_SRC_DATA_16B, DMA_SRC_ADDR_INC, DMA_DST_DATA_16B, DMA_DST_ADDR_FIX);
		SET_BIT(mA1801DacCh1DmaHandle->CTRL,DMA_CTRL_DMA_CONTINUE_ENABLE);    // DMA continuous mode enabled.
		DMA_Trigger(mA1801DacCh1DmaHandle, (uint32_t)&DacDataBuf[0], (uint32_t)&DAC->DAC_CH1_DMA_DATA0, DataLen);
		TIMER_Open(mA1801DacTimerHandle);
		while(DMA_CheckBusy(mA1801DacCh1DmaHandle) != 0);                // Wait DMAx busy
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
 *   A1801Status [in]: A1801 status
 * @return
 *   None.
 */
void static DacAutoRampUp(uint16_t A1801Status)
{
  if((A1801Status & A1801_AUTO_RAMP_UP_ENABLE_FLAG) != 0)
	{
		if((A1801Status & A1801_ENABLE_DAC_CH0_FLAG) != 0)
		{
			RampUpDacCh0();
		}

		if((A1801Status & A1801_ENABLE_DAC_CH1_FLAG) != 0)
		{
			RampUpDacCh1();
		}
	}
}

/**
 * @brief
 *
 * @param
 *   A1801Status [in]: A1801 status
 * @return
 *   None.
 */
void static DacAutoRampDown(uint16_t A1801Status)
{
  if((A1801Status & A1801_AUTO_RAMP_DOWN_ENABLE_FLAG) != 0)
	{
		if((A1801Status & A1801_ENABLE_DAC_CH0_FLAG) != 0)
		{
			RampDownDacCh0();
		}

		if((A1801Status & A1801_ENABLE_DAC_CH1_FLAG) != 0)
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
 *   When user call API function SACM_A1801_Initial, this function is called by A1801 Library.
 * @param
 *  *A1801WorkingRam [in]: A1801 library working RAM address.
 * @return
 *   None.
 */
void A1801_CB_Init(const SACM_A1801_WORKING_RAM *A1801WorkingRam)
{
	mA1801DacCh0DmaHandle = DMA3;
	mA1801DacCh1DmaHandle = DMA4;
	mA1801DacTimerHandle = TM0;

  TIMER_Open(mA1801DacTimerHandle);
	TIMER_SetFreq(mA1801DacTimerHandle, (16000 * 4));	                 // Sample rate: 16000 x 4 = 64000(Hz), for H/W 4x upsampling

  // DAC init
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

/**
 * @brief
 *   Callback function. To be implemented by user.
 *   When user call API function SACM_A1801_Play, this function is called by A1801 Library.
 * @param
 *  *A1801WorkingRam [in]: A1801 library working RAM address.
 * @return
 *   None.
 */
void A1801_CB_StartPlay(const SACM_A1801_WORKING_RAM *A1801WorkingRam)
{
	if(AudOutType == AUD_OUT_PWM)                                 // For AUD_OUT_PWM
	{
    DAC_VoltageDAC_CH0_Disable();
    DAC_VoltageDAC_CH1_Disable();
    DAC_AudioPwm_IP_Enable();
	}
	DacAutoRampUp(SACM_A1801_GetStatus());                             // DAC auto ramp up

	if((SACM_A1801_GetStatus() & A1801_ENABLE_DAC_CH0_FLAG) != 0)
	{
    CLEAR_BIT(DAC->CTRL, DAC_CTRL_DAC_CH0_DMA_ENABLE);                // DAC_Ch0 DMA request disabled.
    TIMER_Close(mA1801DacTimerHandle);
    while(DMA_CheckBusy(mA1801DacCh0DmaHandle) != 0);
	  DMA_Init(mA1801DacCh0DmaHandle, DMA_REQSEL_DAC_CH0, DMA_SRC_DATA_16B, DMA_SRC_ADDR_INC, DMA_DST_DATA_16B, DMA_DST_ADDR_FIX);
	  SET_BIT(mA1801DacCh0DmaHandle->CTRL,DMA_CTRL_DMA_CONTINUE_ENABLE);    // DMA continuous mode enabled.
	  DMA_InstallIsrService(mA1801DacCh0DmaHandle, SACM_A1801_DmaIsrService);
	  DMA_EnableInt(mA1801DacCh0DmaHandle);
	  NVIC_EnableIRQ(DMA3_IRQn);
	  DMA_Trigger(mA1801DacCh0DmaHandle, (uint32_t)A1801WorkingRam->PcmBufPtr, (uint32_t)&DAC->DAC_CH0_DMA_DATA0, A1801_FRAME_SIZE);
	  TIMER_Open(mA1801DacTimerHandle);
	}

	if((SACM_A1801_GetStatus() & A1801_ENABLE_DAC_CH1_FLAG) != 0)
	{
    CLEAR_BIT(DAC->CTRL, DAC_CTRL_DAC_CH1_DMA_ENABLE);                // DAC_Ch1 DMA request disabled.
    TIMER_Close(mA1801DacTimerHandle);
    while(DMA_CheckBusy(mA1801DacCh1DmaHandle) != 0);
		DMA_Init(mA1801DacCh1DmaHandle, DMA_REQSEL_DAC_CH1, DMA_SRC_DATA_16B, DMA_SRC_ADDR_INC, DMA_DST_DATA_16B, DMA_DST_ADDR_FIX);
		SET_BIT(mA1801DacCh1DmaHandle->CTRL,DMA_CTRL_DMA_CONTINUE_ENABLE);    // DMA continuous mode enabled.
    if((SACM_A1801_GetStatus() & A1801_ENABLE_DAC_CH0_FLAG) == 0)
		{
	    DMA_InstallIsrService(mA1801DacCh1DmaHandle, SACM_A1801_DmaIsrService);
	    DMA_EnableInt(mA1801DacCh1DmaHandle);
		}
		NVIC_EnableIRQ(DMA4_IRQn);
		DMA_Trigger(mA1801DacCh1DmaHandle, (uint32_t)A1801WorkingRam->PcmBufPtr, (uint32_t)&DAC->DAC_CH1_DMA_DATA0, A1801_FRAME_SIZE);
		TIMER_Open(mA1801DacTimerHandle);
	}
}

/**
 * @brief
 *
 * @param
 *  *A1801WorkingRam [in]: A1801 library working RAM address.
 * @return
 *   None.
 */
void A1801_CB_Play_Con(const SACM_A1801_WORKING_RAM *A1801WorkingRam)
{
}

/**
 * @brief
 *
 * @param
 *  *A1801WorkingRam [in]: A1801 library working RAM address.
 * @return
 *   None.
 */
void A1801_CB_StopPlay(const SACM_A1801_WORKING_RAM *A1801WorkingRam)
{
	if((SACM_A1801_GetStatus() & A1801_PLAY_FLAG) != 0)
  {
	  DacAutoRampDown(SACM_A1801_GetStatus());	                       // DAC auto ramp down

		if((SACM_A1801_GetStatus() & A1801_ENABLE_DAC_CH0_FLAG) != 0)
	  {
		  DMA_Close(mA1801DacCh0DmaHandle);
		}

	  if((SACM_A1801_GetStatus() & A1801_ENABLE_DAC_CH1_FLAG) != 0)
	  {
		  DMA_Close(mA1801DacCh1DmaHandle);
		}

    if(AudOutType == AUD_OUT_PWM)                                // For AUD_OUT_PWM, Kim 2020.12.02
    {
      DAC_AudioPwm_IP_Disable();
    }
    else                                                           // For AUD_OUT_DAC, Kim 2020.12.02
    {
      // DAC_VoltageDac_Disable();
    }
  }
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When user call API function SACM_A1801_Pause, this function is called by A1801 Library.
 * @param
 *  *A1801WorkingRam [in]: A1801 library working RAM address.
 * @return
 *   None.
 */
void A1801_CB_Pause(const SACM_A1801_WORKING_RAM *A1801WorkingRam)
{
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When user call API function SACM_A1801_Resume, this function is Called by A1801 Library.
 * @param
 *  *A1801WorkingRam [in]: A1801 library working RAM address.
 * @return
 *   None.
 */
void A1801_CB_Resume(const SACM_A1801_WORKING_RAM *A1801WorkingRam)
{
	if((SACM_A1801_GetStatus() & A1801_ENABLE_DAC_CH0_FLAG) != 0)
	{
    DMA_Trigger(mA1801DacCh0DmaHandle, (uint32_t)A1801WorkingRam->PcmBufPtr, (uint32_t)&DAC->DAC_CH0_DMA_DATA0, A1801_FRAME_SIZE);
	}

	if((SACM_A1801_GetStatus() & A1801_ENABLE_DAC_CH1_FLAG) != 0)
	{
    DMA_Trigger(mA1801DacCh1DmaHandle, (uint32_t)A1801WorkingRam->PcmBufPtr, (uint32_t)&DAC->DAC_CH1_DMA_DATA0, A1801_FRAME_SIZE);
	}
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When playing, this function is called by A1801 Library to get encoded data.
 * @param
 *  *A1801WorkingRam [in]: A1801 library working RAM address.
 *  *SrcDataAddr [in]: Source data address pointer.
 *  *DstBufAddr [out]: Destination buffer address pointer.
 *   DataLen [in]: Data length (halfword)
 * @return
 *   None.
 */
void A1801_CB_GetData(const SACM_A1801_WORKING_RAM *A1801WorkingRam, int16_t *DstBufAddr, int16_t *SrcDataAddr, uint16_t DataLen)
{
	uint32_t iCount;

	for(iCount = 0; iCount < DataLen; iCount++)
  {
    DstBufAddr[iCount] = SrcDataAddr[iCount];
	}
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When playing, this function is called by A1801 Library to trigger DAC DMA.
 * @param
 *  *A1801WorkingRam [in]: A1801 working RAM pointer
 *  *SrcBufAddr [in]: Source buffer address pointer.
 *   DataLen [in]: Data length (halfword)
 * @return
 *   None.
 */
void A1801_CB_SendDac_DmaIsr(const SACM_A1801_WORKING_RAM *A1801WorkingRam, int16_t *SrcBufAddr, uint16_t DataLen)
{
	if((SACM_A1801_GetStatus() & A1801_ENABLE_DAC_CH0_FLAG) != 0)
	{
		while(DMA_CheckBusy(mA1801DacCh0DmaHandle) != 0);                // Wait DMAx busy
		DMA_Trigger(mA1801DacCh0DmaHandle, (uint32_t)SrcBufAddr, (uint32_t)&DAC->DAC_CH0_DMA_DATA0, DataLen);
	}

	if((SACM_A1801_GetStatus() & A1801_ENABLE_DAC_CH1_FLAG) != 0)
	{
		while(DMA_CheckBusy(mA1801DacCh1DmaHandle) != 0);                // Wait DMAx busy
    DMA_Trigger(mA1801DacCh1DmaHandle, (uint32_t)SrcBufAddr, (uint32_t)&DAC->DAC_CH1_DMA_DATA0, DataLen);
	}
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When playing, this function is called by A1801 library to decode data.
 * @param
 *  *A1801WorkingRam [in]: A1801 working RAM pointer
 *  *DstBufAddr [out]: Destination buffer address pointer.
 *  *SrcBufAddr [in]: Source buffer address pointer.
 * @return
 *   None.
 */
void A1801_CB_DecodeProcess(const SACM_A1801_WORKING_RAM *A1801WorkingRam, int16_t *DstBufAddr, int16_t *SrcBufAddr)
{
  SACM_A1801_DecodeProcess(DstBufAddr, SrcBufAddr);
	APP_SwVolCtrl_VolProcess(0, DstBufAddr, A1801_FRAME_SIZE);
}
