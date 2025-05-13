/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   SACM_DVR1801_User.c
 * @Version:
 *   V1.0.4.1
 * @Date:
 *   April 18, 2022
 * @Abstract:
 *
 **************************************************************************************************/


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "GPCM3_FM1.h"
#include "SACM_DVR1801_User.h"
#include "A1801_FileMerger.h"
#include "APP_SwVolumeControl.h"


/*---------------------------------------------------------------------------------------
 * Gobal Variable Declaration Area
 *---------------------------------------------------------------------------------------*/
DMA_TYPE_DEF *mDvr1801DsAdcDmaHandle;
TIMER_TYPE_DEF *mDvr1801DacTimerHandle;
DMA_TYPE_DEF *mDvr1801DacCh0DmaHandle;
DMA_TYPE_DEF *mDvr1801DacCh1DmaHandle;

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
uint16_t DVR1801_User_GetA1801Num()
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
int16_t* DVR1801_User_GetA1801StartAddr(uint16_t SpeechIdx)
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
	  CLEAR_BIT(DAC->CTRL, DAC_CTRL_DAC_CH0_DMA_ENABLE);                 // DAC_Ch0 DMA request disabled.
    TIMER_Close(mDvr1801DacTimerHandle);
	  DMA_Init(mDvr1801DacCh0DmaHandle, DMA_REQSEL_DAC_CH0, DMA_SRC_DATA_16B, DMA_SRC_ADDR_INC, DMA_DST_DATA_16B, DMA_DST_ADDR_FIX);
	  DMA_Trigger(mDvr1801DacCh0DmaHandle, (uint32_t)&DacDataBuf[0], (uint32_t)&DAC->DAC_CH0_DMA_DATA0, DataLen);
		TIMER_Open(mDvr1801DacTimerHandle);
		while(DMA_CheckBusy(mDvr1801DacCh0DmaHandle) != 0);                // Wait DMAx busy
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
    TIMER_Close(mDvr1801DacTimerHandle);
		DMA_Init(mDvr1801DacCh1DmaHandle, DMA_REQSEL_DAC_CH1, DMA_SRC_DATA_16B, DMA_SRC_ADDR_INC, DMA_DST_DATA_16B, DMA_DST_ADDR_FIX);
		DMA_Trigger(mDvr1801DacCh1DmaHandle, (uint32_t)&DacDataBuf[0], (uint32_t)&DAC->DAC_CH1_DMA_DATA0, DataLen);
		TIMER_Open(mDvr1801DacTimerHandle);
		while(DMA_CheckBusy(mDvr1801DacCh1DmaHandle) != 0);                // Wait DMAx busy
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
 *   Dvr1801Status [in]: DVR1801 status
 * @return
 *   None.
 */
void static DacAutoRampUp(uint16_t Dvr1801Status)
{
  if((Dvr1801Status & DVR1801_AUTO_RAMP_UP_ENABLE_FLAG) != 0)
	{
		if((Dvr1801Status & DVR1801_ENABLE_DAC_CH0_FLAG) != 0)
		{
			RampUpDacCh0();
		}

		if((Dvr1801Status & DVR1801_ENABLE_DAC_CH1_FLAG) != 0)
		{
			RampUpDacCh1();
		}
	}
}

/**
 * @brief
 *
 * @param
 *   Dvr1801Status [in]: DVR1801 status
 * @return
 *   None.
 */
void static DacAutoRampDown(uint16_t Dvr1801Status)
{
  if((Dvr1801Status & DVR1801_AUTO_RAMP_DOWN_ENABLE_FLAG) != 0)
	{
		if((Dvr1801Status & DVR1801_ENABLE_DAC_CH0_FLAG) != 0)
		{
			RampDownDacCh0();
		}

		if((Dvr1801Status & DVR1801_ENABLE_DAC_CH1_FLAG) != 0)
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
 *   When user call API function SACM_DVR1801_Initial, this function is called by DVR1801 Library.
 * @param
 *  *Dvr1801WorkingRam [in]: DVR1801 library working RAM address.
 * @return
 *   None.
 */
void DVR1801_CB_Init(const SACM_DVR1801_WORKING_RAM *Dvr1801WorkingRam)
{
	mDvr1801DsAdcDmaHandle = DMA3;
	mDvr1801DacCh0DmaHandle = DMA3;
	mDvr1801DacCh1DmaHandle = DMA4;
	mDvr1801DacTimerHandle = TM0;

  TIMER_Open(mDvr1801DacTimerHandle);
	TIMER_SetFreq(mDvr1801DacTimerHandle, (16000 * 4));	                // Sample rate: 16000 x 4 = 64000(Hz), for H/W 4x upsampling

	// DAC init
	DAC_Open();
	DAC_Scale_Enable();
	DAC_CH0_Init(DAC_TRG_SEL_TM0);
	// DAC_CH1_Init(DAC_TRG_SEL_TM0);
	SET_BIT(DAC->CTRL, DAC_CTRL_CH0_POSTWAVE_OUT_CH0 + DAC_CTRL_CH1_POSTWAVE_OUT_CH1);
	// SET_BIT(DAC->CTRL, DAC_CTRL_CH0_POSTWAVE_OUT_CH0);
	DAC_AudioPwm_Open();
	DAC_SetAudPwmGain(0x1F);

  // DS-ADC init
  DSADC_Init(16000);
  DSADC_SetBoostGain(2);
  DSADC_SetPgaGain(18);
	//DSADC_AutoMuteInit(0x180);

	/*
	 * Digital AGC init
	 */
  /*
	DAGC_SetCenterBoundary(0x20, 0x1000);
	DAGC_SetSensitivity(0x1E, 0x01, 0xFE);
	DAGC_SetAttackScale(ATTACK_X1);
	DAGC_SetReleaseScale(RELEASE_X1);
	DAGC_ZeroCross_Enable();
	DAGC_Enable();
	*/
}

/**
 * @brief
 *   Callback function. To be implemented by user.
 *   When user call API function SACM_DVR1801_Play, this function is called by DVR1801 Library.
 * @param
 *  *Dvr1801WorkingRam [in]: DVR1801 library working RAM address.
 * @return
 *   None.
 */
void DVR1801_CB_StartPlay(const SACM_DVR1801_WORKING_RAM *Dvr1801WorkingRam)
{
	if(AudOutType == AUD_OUT_PWM)                                 // For AUD_OUT_PWM
	{
    DAC_VoltageDAC_CH0_Disable();
    // DAC_VoltageDAC_CH1_Disable();
    DAC_AudioPwm_IP_Enable();
	}
	else                                                          // For AUD_OUT_DAC
	{
    DAC_AudioPwm_IP_Disable();
    DAC_VoltageDAC_CH0_Enable();
    // DAC_VoltageDAC_CH1_Enable();
	}
  DacAutoRampUp(SACM_DVR1801_GetStatus());                      // DAC auto ramp up

	if((SACM_DVR1801_GetStatus() & DVR1801_ENABLE_DAC_CH0_FLAG) != 0)
	{
    CLEAR_BIT(DAC->CTRL, DAC_CTRL_DAC_CH0_DMA_ENABLE);          // DAC_Ch0 DMA request disabled.
    TIMER_Close(mDvr1801DacTimerHandle);
    while(DMA_CheckBusy(mDvr1801DacCh0DmaHandle) != 0);
	  DMA_Init(mDvr1801DacCh0DmaHandle, DMA_REQSEL_DAC_CH0, DMA_SRC_DATA_16B, DMA_SRC_ADDR_INC, DMA_DST_DATA_16B, DMA_DST_ADDR_FIX);
	  DMA_InstallIsrService(mDvr1801DacCh0DmaHandle, SACM_DVR1801_DmaIsrService);
	  DMA_EnableInt(mDvr1801DacCh0DmaHandle);
	  NVIC_EnableIRQ(DMA3_IRQn);
	  DMA_Trigger(mDvr1801DacCh0DmaHandle, (uint32_t)Dvr1801WorkingRam->PcmBufPtr, (uint32_t)&DAC->DAC_CH0_DMA_DATA0, DVR1801_FRAME_SIZE);
	  TIMER_Open(mDvr1801DacTimerHandle);
	}

	/*
	if((SACM_DVR1801_GetStatus() & DVR1801_ENABLE_DAC_CH1_FLAG) != 0)
	{
    CLEAR_BIT(DAC->CTRL, DAC_CTRL_DAC_CH1_DMA_ENABLE);                // DAC_Ch1 DMA request disabled.
    TIMER_Close(mDvr1801DacTimerHandle);
    while(DMA_CheckBusy(mDvr1801DacCh1DmaHandle) != 0);
		DMA_Init(mDvr1801DacCh1DmaHandle, DMA_REQSEL_DAC_CH1, DMA_SRC_DATA_16B, DMA_SRC_ADDR_INC, DMA_DST_DATA_16B, DMA_DST_ADDR_FIX);
    if((SACM_DVR1801_GetStatus() & DVR1801_ENABLE_DAC_CH0_FLAG) == 0)
		{
	    DMA_InstallIsrService(mDvr1801DacCh1DmaHandle, SACM_DVR1801_DmaIsrService);
	    DMA_EnableInt(mDvr1801DacCh1DmaHandle);
	    NVIC_EnableIRQ(DMA4_IRQn);
		}
		DMA_Trigger(mDvr1801DacCh1DmaHandle, (uint32_t)Dvr1801WorkingRam->PcmBufPtr, (uint32_t)&DAC->DAC_CH1_DMA_DATA0, DVR1801_FRAME_SIZE);
		TIMER_Open(mDvr1801DacTimerHandle);
	}
	*/
}

/**
 * @brief
 *
 * @param
 *  *A1801WorkingRam [in]: A1801 library working RAM address.
 * @return
 *   None.
 */
void DVR1801_CB_Play_Con(const SACM_DVR1801_WORKING_RAM *Dvr1801WorkingRam)
{
}

/**
 * @brief
 *
 * @param
 *  *Dvr1801WorkingRam [in]: DVR1801 library working RAM address.
 * @return
 *   None.
 */
void DVR1801_CB_StopPlay(const SACM_DVR1801_WORKING_RAM *Dvr1801WorkingRam)
{
	if((SACM_DVR1801_GetStatus() & DVR1801_PLAY_FLAG) != 0)
  {
    DacAutoRampDown(SACM_DVR1801_GetStatus());	                // DAC auto ramp down

	  if((SACM_DVR1801_GetStatus() & DVR1801_ENABLE_DAC_CH0_FLAG) != 0)
	  {
		  DMA_Close(mDvr1801DacCh0DmaHandle);
		}

	  if((SACM_DVR1801_GetStatus() & DVR1801_ENABLE_DAC_CH1_FLAG) != 0)
	  {
		  DMA_Close(mDvr1801DacCh1DmaHandle);
		}

    if(AudOutType == AUD_OUT_PWM)                                // For AUD_OUT_PWM, Kim 2020.12.02
    {
      DAC_AudioPwm_IP_Disable();
    }
    else                                                           // For AUD_OUT_DAC, Kim 2020.12.02
    {
      DAC_VoltageDac_Disable();
    }
  }
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When user call API function SACM_DVR1801_Pause, this function is called by DVR1801 Library.
 * @param
 *  *Dvr1801WorkingRam [in]: DVR1801 library working RAM address.
 * @return
 *   None.
 */
void DVR1801_CB_Pause(const SACM_DVR1801_WORKING_RAM *Dvr1801WorkingRam)
{
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When user call API function SACM_DVR1801_Resume, this function is Called by DVR1801 Library.
 * @param
 *  *Dvr1801WorkingRam [in]: DVR1801 library working RAM address.
 * @return
 *   None.
 */
void DVR1801_CB_Resume(const SACM_DVR1801_WORKING_RAM *Dvr1801WorkingRam)
{
	if((SACM_DVR1801_GetStatus() & DVR1801_ENABLE_DAC_CH0_FLAG) != 0)
	{
    DMA_Trigger(mDvr1801DacCh0DmaHandle, (uint32_t)Dvr1801WorkingRam->PcmBufPtr, (uint32_t)&DAC->DAC_CH0_DMA_DATA0, DVR1801_FRAME_SIZE);
	}

	if((SACM_DVR1801_GetStatus() & DVR1801_ENABLE_DAC_CH1_FLAG) != 0)
	{
    DMA_Trigger(mDvr1801DacCh1DmaHandle, (uint32_t)Dvr1801WorkingRam->PcmBufPtr, (uint32_t)&DAC->DAC_CH1_DMA_DATA0, DVR1801_FRAME_SIZE);
	}
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When user call API function SACM_DVR1801_Rec, this function is called by DVR1801 Library.
 * @param
 *  *Dvr1801WorkingRam [in]: DVR1801 library working RAM address.
 * @return
 *   None.
 */
void DVR1801_CB_StartRecord(const SACM_DVR1801_WORKING_RAM *Dvr1801WorkingRam)
{
	DMA_Init(mDvr1801DsAdcDmaHandle, DMA_REQSEL_DSADC, DMA_SRC_DATA_16B, DMA_SRC_ADDR_FIX, DMA_DST_DATA_16B, DMA_DST_ADDR_INC);
	DMA_InstallIsrService(mDvr1801DsAdcDmaHandle, SACM_DVR1801_DmaIsrService);
	DMA_EnableInt(mDvr1801DsAdcDmaHandle);
	NVIC_EnableIRQ(DMA3_IRQn);
	DMA_Trigger(mDvr1801DsAdcDmaHandle, (uint32_t)&DS_ADC->DATA, (uint32_t)Dvr1801WorkingRam->PcmBufPtr, DVR1801_FRAME_SIZE);
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When user call API function SACM_DVR1801_Stop to stop recording, this function is called by DVR1801 Library to writeback record data length.
 * @param
 *  *Dvr1801WorkingRam [in]: DVR1801 library working RAM address.
 *   RecDataLen [in]: Record data length (byte).
 * @return
 *   None.
 */
void DVR1801_CB_StopRecord(const SACM_DVR1801_WORKING_RAM *Dvr1801WorkingRam, uint32_t RecDataLen)
{
	DSADC_Close();
	//SPI_Flash_SendNBytes((uint32_t)Dvr1801WorkingRam->SpeechDataAddr, ((uint8_t*)&RecDataLen), 4);
	SPIFC_WriteNBytes((uint32_t)Dvr1801WorkingRam->SpeechDataAddr, (uint8_t*)&RecDataLen, 4);
	DMA_Close(mDvr1801DsAdcDmaHandle);
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When playing, this function is called by DVR1801 Library to get encoded data.
 * @param
 *  *Dvr1801WorkingRam [in]: DVR1801 library working RAM address.
 *  *SrcDataAddr [in]: Source data address pointer.
 *  *DstBufAddr [out]: Destination buffer address pointer.
 *   DataLen [in]: Data length (halfword)
 * @return
 *   None.
 */
void DVR1801_CB_GetData(const SACM_DVR1801_WORKING_RAM *Dvr1801WorkingRam, int16_t *DstBufAddr, int16_t *SrcDataAddr, uint16_t DataLen)
{
  ///*
	uint32_t iCount;

	for(iCount = 0; iCount < DataLen; iCount++)
  {
    DstBufAddr[iCount] = SrcDataAddr[iCount];
	}
  //*/

	/*
  DMA_Init(DMA2, DMA_REQSEL_MEM, DMA_SRC_DATA_16B, DMA_SRC_ADDR_INC, DMA_DST_DATA_16B, DMA_DST_ADDR_INC);
	DMA_Trigger(DMA2, (uint32_t)SrcDataAddr, (uint32_t)DstBufAddr, DataLen);
	while(DMA_CheckBusy(DMA2) != 0);
	DMA_Close(DMA2);
	*/

  //SPI_Flash_ReadNBytes((uint32_t)SrcDataAddr, (uint8_t *)DstBufAddr, (DataLen << 1));
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When recording, this function is called by DVR1801 Library to write encoded data.
 * @param
 *  *Dvr1801WorkingRam [in]: DVR1801 library working RAM address.
 *  *DstDataAddr [out]: Destination data address pointer.
 *  *SrcBufAddr [in]: Source buffer address pointer.
 *   DataLen [in]: Data length (halfword)
 * @return
 *   None.
 */
void DVR1801_CB_WriteData(const SACM_DVR1801_WORKING_RAM *Dvr1801WorkingRam, int16_t *DstDataAddr, int16_t *SrcBufAddr, uint16_t DataLen)
{
	//SPI_Flash_SendNBytes((uint32_t)DstDataAddr, (uint8_t *)SrcBufAddr, (DataLen << 1));
	SPIFC_WriteNBytes((uint32_t)DstDataAddr, (uint8_t *)SrcBufAddr, (DataLen << 1));
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When playing, this function is called by DVR1801 Library to trigger DAC DMA.
 * @param
 *  *Dvr1801WorkingRam [in]: Dvr1801 working RAM pointer
 *  *SrcBufAddr [in]: Source buffer address pointer.
 *   DataLen [in]: Data length (halfword)
 * @return
 *   None.
 */
void DVR1801_CB_SendDac_DmaIsr(const SACM_DVR1801_WORKING_RAM *Dvr1801WorkingRam, int16_t *SrcBufAddr, uint16_t DataLen)
{
	if((SACM_DVR1801_GetStatus() & DVR1801_ENABLE_DAC_CH0_FLAG) != 0)
	{
		while(DMA_CheckBusy(mDvr1801DacCh0DmaHandle) != 0);         // Check DMAx busy
		DMA_Trigger(mDvr1801DacCh0DmaHandle, (uint32_t)SrcBufAddr, (uint32_t)&DAC->DAC_CH0_DMA_DATA0, DataLen);
	}
	/*
	if((SACM_DVR1801_GetStatus() & DVR1801_ENABLE_DAC_CH1_FLAG) != 0)
	{
		while(DMA_CheckBusy(mDvr1801DacCh1DmaHandle) != 0);         // Check DMAx busy
    DMA_Trigger(mDvr1801DacCh1DmaHandle, (uint32_t)SrcBufAddr, (uint32_t)&DAC->DAC_CH1_DMA_DATA0, DataLen);
	}
	*/
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When recording, this function is called by DVR1801 Library to trigger DS-ADC DMA.
 * @param
 *  *Dvr1801WorkingRam [in]: Dvr1801 working RAM pointer
 *  *DstBufAddr [out]: Destination buffer address pointer.
 *   DataLen [in]: Data length (halfword)
 * @return
 *   None.
 */
void DVR1801_CB_GetAdc_DmaIsr(const SACM_DVR1801_WORKING_RAM *Dvr1801WorkingRam, int16_t *DstBufAddr, uint16_t DataLen)
{
	DMA_Trigger(mDvr1801DsAdcDmaHandle, (uint32_t)&DS_ADC->DATA, (uint32_t)DstBufAddr, DataLen);
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When playing, this function is called by DVR1801 library to decode data.
 * @param
 *  *Dvr1801WorkingRam [in]: Dvr1801 working RAM pointer
 *  *DstBufAddr [out]: Destination buffer address pointer.
 *  *SrcBufAddr [in]: Source buffer address pointer.
 * @return
 *   None.
 */
void DVR1801_CB_DecodeProcess(const SACM_DVR1801_WORKING_RAM *Dvr1801WorkingRam, int16_t *DstBufAddr, int16_t *SrcBufAddr)
{
  SACM_DVR1801_DecodeProcess(DstBufAddr, SrcBufAddr);
  APP_SwVolCtrl_VolProcess(0, DstBufAddr, DVR1801_FRAME_SIZE);
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When recording, this function is called by DVR1801 library to encode data.
 * @param
 *  *Dvr1801WorkingRam [in]: Dvr1801 working RAM pointer
 *  *DstBufAddr [out]: Destination buffer address pointer.
 *  *SrcBufAddr [in]: Source buffer address pointer.
 * @return
 *   None.
 */
void DVR1801_CB_EncodeProcess(const SACM_DVR1801_WORKING_RAM *Dvr1801WorkingRam, int16_t *DstBufAddr, int16_t *SrcBufAddr)
{
  SACM_DVR1801_EncodeProcess(DstBufAddr, SrcBufAddr);
}
