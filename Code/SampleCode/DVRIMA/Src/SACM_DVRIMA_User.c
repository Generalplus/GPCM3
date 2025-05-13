/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   SACM_DVRIMA_User.c
 * @Version:
 *   V0.9
 * @Date:
 *   April 16, 2025
 * @Abstract:
 *
 **************************************************************************************************/


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "GPCM3_FM1.h"
#include "SACM_DVRIMA_User.h"
#include "DVRIMA_FileMerger.h"
#include "APP_SwVolumeControl.h"


/*---------------------------------------------------------------------------------------
 * Gobal Variable Declaration Area
 *---------------------------------------------------------------------------------------*/
TIMER_TYPE_DEF *mDVRIMADacTimerHandle;
DMA_TYPE_DEF *mDVRIMADacCh0DmaHandle;
DMA_TYPE_DEF *mDVRIMADacCh1DmaHandle;


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
uint16_t DVRIMA_User_GetDVRIMANum()
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
int16_t* DVRIMA_User_GetDVRIMAStartAddr(uint16_t SpeechIdx)
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
		TIMER_Close(mDVRIMADacTimerHandle);
	  DMA_Init(mDVRIMADacCh0DmaHandle, DMA_REQSEL_DAC_CH0, DMA_SRC_DATA_16B, DMA_SRC_ADDR_INC, DMA_DST_DATA_16B, DMA_DST_ADDR_FIX);
	  DMA_Trigger(mDVRIMADacCh0DmaHandle, (uint32_t)&DacDataBuf[0], (uint32_t)&DAC->DAC_CH0_DMA_DATA0, DataLen);
		TIMER_Open(mDVRIMADacTimerHandle);
		while(DMA_CheckBusy(mDVRIMADacCh0DmaHandle) != 0);                // Wait DMAx busy
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
		TIMER_Close(mDVRIMADacTimerHandle);
		DMA_Init(mDVRIMADacCh1DmaHandle, DMA_REQSEL_DAC_CH1, DMA_SRC_DATA_16B, DMA_SRC_ADDR_INC, DMA_DST_DATA_16B, DMA_DST_ADDR_FIX);
		DMA_Trigger(mDVRIMADacCh1DmaHandle, (uint32_t)&DacDataBuf[0], (uint32_t)&DAC->DAC_CH1_DMA_DATA0, DataLen);
		TIMER_Open(mDVRIMADacTimerHandle);
		while(DMA_CheckBusy(mDVRIMADacCh1DmaHandle) != 0);                // Wait DMAx busy
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
 *   DVRIMAStatus [in]: DVRIMA status
 * @return
 *   None.
 */
void static DacAutoRampUp(uint16_t DVRIMAStatus)
{
  if((DVRIMAStatus & DVRIMA_AUTO_RAMP_UP_ENABLE_FLAG) != 0)
	{
		if((DVRIMAStatus & DVRIMA_ENABLE_DAC_CH0_FLAG) != 0)
		{
			RampUpDacCh0();
		}

		if((DVRIMAStatus & DVRIMA_ENABLE_DAC_CH1_FLAG) != 0)
		{
			RampUpDacCh1();
		}
	}
}

/**
 * @brief
 *
 * @param
 *   DVRIMAStatus [in]: DVRIMA status
 * @return
 *   None.
 */
void static DacAutoRampDown(uint16_t DVRIMAStatus)
{
  if((DVRIMAStatus & DVRIMA_AUTO_RAMP_DOWN_ENABLE_FLAG) != 0)
	{
		if((DVRIMAStatus & DVRIMA_ENABLE_DAC_CH0_FLAG) != 0)
		{
			RampDownDacCh0();
		}

		if((DVRIMAStatus & DVRIMA_ENABLE_DAC_CH1_FLAG) != 0)
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
 *   When user call API function SACM_DVRIMA_Initial, this function is called by DVRIMA Library.
 * @param
 *  *DVRIMAWorkingRam [in]: DVRIMA library working RAM address.
 * @return
 *   None.
 */
void DVRIMA_CB_Init(const SACM_DVRIMA_WORKING_RAM *DVRIMAWorkingRam)
{
	mDVRIMADacCh0DmaHandle = DMA3;
	mDVRIMADacCh1DmaHandle = DMA4;
	mDVRIMADacTimerHandle = TM0;

  TIMER_Open(mDVRIMADacTimerHandle);
	TIMER_SetFreq(mDVRIMADacTimerHandle, (16000 * 4));	                 // Sample rate: 16000 x 4 = 64000(Hz), for H/W 4x upsampling

  // DAC init
	DAC_Open();
	DAC_Scale_Enable();
	DAC_CH0_Init(DAC_TRG_SEL_TM0);
	DAC_CH1_Init(DAC_TRG_SEL_TM0);
	SET_BIT(DAC->CTRL, DAC_CTRL_CH0_POSTWAVE_OUT_CH0 + DAC_CTRL_CH1_POSTWAVE_OUT_CH1);

	// PWM init
	DAC_AudioPwm_Open();
	DAC_SetAudPwmGain(31);      // 1x Gain.
}

/**
 * @brief
 *   Callback function. To be implemented by user.
 *   When user call API function SACM_DVRIMA_Play, this function is called by DVRIMA Library.
 * @param
 *  *DVRIMAWorkingRam [in]: DVRIMA library working RAM address.
 * @return
 *   None.
 */
void DVRIMA_CB_StartPlay(const SACM_DVRIMA_WORKING_RAM *DVRIMAWorkingRam)
{
	DAC_AudioPwm_IP_Enable();
	DacAutoRampUp(SACM_DVRIMA_GetStatus());                             // DAC auto ramp up

	if((SACM_DVRIMA_GetStatus() & DVRIMA_ENABLE_DAC_CH0_FLAG) != 0)
	{
    CLEAR_BIT(DAC->CTRL, DAC_CTRL_DAC_CH0_DMA_ENABLE);                // DAC_Ch0 DMA request disabled.
    TIMER_Close(mDVRIMADacTimerHandle);
    while(DMA_CheckBusy(mDVRIMADacCh0DmaHandle) != 0);
	  DMA_Init(mDVRIMADacCh0DmaHandle, DMA_REQSEL_DAC_CH0, DMA_SRC_DATA_16B, DMA_SRC_ADDR_INC, DMA_DST_DATA_16B, DMA_DST_ADDR_FIX);
	  DMA_InstallIsrService(mDVRIMADacCh0DmaHandle, SACM_DVRIMA_DmaIsrService);
	  DMA_EnableInt(mDVRIMADacCh0DmaHandle);
	  NVIC_EnableIRQ(DMA3_IRQn);
	  DMA_Trigger(mDVRIMADacCh0DmaHandle, (uint32_t)DVRIMAWorkingRam->PcmBufPtr, (uint32_t)&DAC->DAC_CH0_DMA_DATA0, DVRIMA_FRAME_SIZE);
	  TIMER_Open(mDVRIMADacTimerHandle);
	}

	if((SACM_DVRIMA_GetStatus() & DVRIMA_ENABLE_DAC_CH1_FLAG) != 0)
	{
    CLEAR_BIT(DAC->CTRL, DAC_CTRL_DAC_CH1_DMA_ENABLE);                // DAC_Ch1 DMA request disabled.
    TIMER_Close(mDVRIMADacTimerHandle);
    while(DMA_CheckBusy(mDVRIMADacCh1DmaHandle) != 0);
		DMA_Init(mDVRIMADacCh1DmaHandle, DMA_REQSEL_DAC_CH1, DMA_SRC_DATA_16B, DMA_SRC_ADDR_INC, DMA_DST_DATA_16B, DMA_DST_ADDR_FIX);
    if((SACM_DVRIMA_GetStatus() & DVRIMA_ENABLE_DAC_CH0_FLAG) == 0)
		{
	    DMA_InstallIsrService(mDVRIMADacCh1DmaHandle, SACM_DVRIMA_DmaIsrService);
	    DMA_EnableInt(mDVRIMADacCh1DmaHandle);
		}
		NVIC_EnableIRQ(DMA4_IRQn);
		DMA_Trigger(mDVRIMADacCh1DmaHandle, (uint32_t)DVRIMAWorkingRam->PcmBufPtr, (uint32_t)&DAC->DAC_CH1_DMA_DATA0, DVRIMA_FRAME_SIZE);
		TIMER_Open(mDVRIMADacTimerHandle);
	}
}

/**
 * @brief
 *
 * @param
 *  *DVRIMAWorkingRam [in]: DVRIMA library working RAM address.
 * @return
 *   None.
 */
void DVRIMA_CB_Play_Con(const SACM_DVRIMA_WORKING_RAM *DVRIMAWorkingRam)
{
}

/**
 * @brief
 *
 * @param
 *  *DVRIMAWorkingRam [in]: DVRIMA library working RAM address.
 * @return
 *   None.
 */
void DVRIMA_CB_StopPlay(const SACM_DVRIMA_WORKING_RAM *DVRIMAWorkingRam)
{
	if((SACM_DVRIMA_GetStatus() & DVRIMA_PLAY_FLAG) != 0)
  {
	  DacAutoRampDown(SACM_DVRIMA_GetStatus());	                       // DAC auto ramp down

		if((SACM_DVRIMA_GetStatus() & DVRIMA_ENABLE_DAC_CH0_FLAG) != 0)
	  {
		  DMA_Close(mDVRIMADacCh0DmaHandle);
		}

	  if((SACM_DVRIMA_GetStatus() & DVRIMA_ENABLE_DAC_CH1_FLAG) != 0)
	  {
		  DMA_Close(mDVRIMADacCh1DmaHandle);
		}

		DAC_AudioPwm_IP_Disable();
  }
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When user call API function SACM_DVRIMA_Pause, this function is called by DVRIMA Library.
 * @param
 *  *DVRIMAWorkingRam [in]: DVRIMA library working RAM address.
 * @return
 *   None.
 */
void DVRIMA_CB_Pause(const SACM_DVRIMA_WORKING_RAM *DVRIMAWorkingRam)
{
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When user call API function SACM_DVRIMA_Resume, this function is Called by DVRIMA Library.
 * @param
 *  *DVRIMAWorkingRam [in]: DVRIMA library working RAM address.
 * @return
 *   None.
 */
void DVRIMA_CB_Resume(const SACM_DVRIMA_WORKING_RAM *DVRIMAWorkingRam)
{
	if((SACM_DVRIMA_GetStatus() & DVRIMA_ENABLE_DAC_CH0_FLAG) != 0)
	{
    DMA_Trigger(mDVRIMADacCh0DmaHandle, (uint32_t)DVRIMAWorkingRam->PcmBufPtr, (uint32_t)&DAC->DAC_CH0_DMA_DATA0, DVRIMA_FRAME_SIZE);
	}

	if((SACM_DVRIMA_GetStatus() & DVRIMA_ENABLE_DAC_CH1_FLAG) != 0)
	{
    DMA_Trigger(mDVRIMADacCh1DmaHandle, (uint32_t)DVRIMAWorkingRam->PcmBufPtr, (uint32_t)&DAC->DAC_CH1_DMA_DATA0, DVRIMA_FRAME_SIZE);
	}
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When playing, this function is called by DVRIMA Library to get encoded data.
 * @param
 *  *DVRIMAWorkingRam [in]: DVRIMA library working RAM address.
 *  *SrcDataAddr [in]: Source data address pointer.
 *  *DstBufAddr [out]: Destination buffer address pointer.
 *   DataLen [in]: Data length (halfword)
 * @return
 *   None.
 */
void DVRIMA_CB_GetData(const SACM_DVRIMA_WORKING_RAM *DVRIMAWorkingRam, int16_t *DstBufAddr, int16_t *SrcDataAddr, uint16_t DataLen)
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
 *   When playing, this function is called by DVRIMA Library to trigger DAC DMA.
 * @param
 *  *DVRIMAWorkingRam [in]: DVRIMA working RAM pointer
 *  *SrcBufAddr [in]: Source buffer address pointer.
 *   DataLen [in]: Data length (halfword)
 * @return
 *   None.
 */
void DVRIMA_CB_SendDac_DmaIsr(const SACM_DVRIMA_WORKING_RAM *DVRIMAWorkingRam, int16_t *SrcBufAddr, uint16_t DataLen)
{
	if((SACM_DVRIMA_GetStatus() & DVRIMA_ENABLE_DAC_CH0_FLAG) != 0)
	{
		while(DMA_CheckBusy(mDVRIMADacCh0DmaHandle) != 0);                // Wait DMAx busy
		DMA_Trigger(mDVRIMADacCh0DmaHandle, (uint32_t)SrcBufAddr, (uint32_t)&DAC->DAC_CH0_DMA_DATA0, DataLen);
	}

	if((SACM_DVRIMA_GetStatus() & DVRIMA_ENABLE_DAC_CH1_FLAG) != 0)
	{
		while(DMA_CheckBusy(mDVRIMADacCh1DmaHandle) != 0);                // Wait DMAx busy
    DMA_Trigger(mDVRIMADacCh1DmaHandle, (uint32_t)SrcBufAddr, (uint32_t)&DAC->DAC_CH1_DMA_DATA0, DataLen);
	}
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When playing, this function is called by DVRIMA library to decode data.
 * @param
 *  *DVRIMAWorkingRam [in]: DVRIMA working RAM pointer
 *  *DstBufAddr [out]: Destination buffer address pointer.
 *  *SrcBufAddr [in]: Source buffer address pointer.
 * @return
 *   None.
 */
void DVRIMA_CB_DecodeProcess(const SACM_DVRIMA_WORKING_RAM *DVRIMAWorkingRam, int16_t *DstBufAddr, int16_t *SrcBufAddr)
{
  SACM_DVRIMA_DecodeProcess(DstBufAddr, SrcBufAddr);
	APP_SwVolCtrl_VolProcess(0, DstBufAddr, DVRIMA_FRAME_SIZE);
}
