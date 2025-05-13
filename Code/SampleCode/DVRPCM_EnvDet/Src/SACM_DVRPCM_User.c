/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   SACM_DVRPCM_User.c
 * @Version:
 *   V1.0.0
 * @Date:
 *   March 08, 2023
 * @Abstract:
 *
 **************************************************************************************************/


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include <stdio.h>
#include "GPCM3_FM1.h"
#include "FileMerger.h"
#include "EnvDet_User.h"
#include "SACM_DVRPCM_User.h"
#include "APP_SwVolumeControl.h"


/*---------------------------------------------------------------------------------------
 * Definition Area
 *---------------------------------------------------------------------------------------*/
#define D_ClickTime		(10000*0.1)     // For DVRPCM recording

// For Envelope Detect
#define D_AttackLevel   0x0800
#define D_ReleaseLevel  0x0400
#define D_AttacTime     0x007F
#define D_ReleaseTime   0x1000


/*---------------------------------------------------------------------------------------
 * Gobal Variable Declaration Area
 *---------------------------------------------------------------------------------------*/
#if defined(__CC_ARM)
#define Aligned4B __align(4)
#elif defined(__GNUC__)
#define Aligned4B __attribute__ ((aligned (4)))
#endif

DMA_TYPE_DEF *mDvrPcmDsAdcDmaHandle;
TIMER_TYPE_DEF *mDvrPcmDacTimerHandle;
DMA_TYPE_DEF *mDvrPcmDacCh0DmaHandle;
DMA_TYPE_DEF *mDvrPcmDacCh1DmaHandle;

// For Envelope Detect
uint8_t R_ActiveFlag = 0;
uint8_t R_ReleaseFlag = 0;
__attribute__ ((aligned (4))) ENVDET_WORKING_RAM          EnvDetWorkingRam;

SACM_DVRPCM_WORKING_RAM *mDvrPcmWorkingRamPointer;

extern SACM_DVRPCM_WORKING_RAM *mDvrPcmWorkingRamPtr;
extern int16_t *mSpeechRecDataAddr;
extern uint16_t mDvrPcmStatus;


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
uint16_t GetPcmNum()
{
	return *(int32_t*) (SEC_START_ADDR);
}

/**
 * @brief
 *   Get speech data start address
 * @param
 *   SpeechIdx [in]: Speech index
 * @return
 *   None.
 */
int16_t* GetPcmStartAddr(uint16_t SpeechIdx)
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
	  DMA_Init(mDvrPcmDacCh0DmaHandle, DMA_REQSEL_DAC_CH0, DMA_SRC_DATA_16B, DMA_SRC_ADDR_INC, DMA_DST_DATA_16B, DMA_DST_ADDR_FIX);
	  SET_BIT(mDvrPcmDacCh0DmaHandle->CTRL,DMA_CTRL_DMA_CONTINUE_ENABLE);    // DMA continuous mode enabled.
	  DMA_Trigger(mDvrPcmDacCh0DmaHandle, (uint32_t)&DacDataBuf[0], (uint32_t)&DAC->DAC_CH0_DMA_DATA0, DataLen);
		while(DMA_CheckBusy(mDvrPcmDacCh0DmaHandle) != 0);                // Wait DMAx busy
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
		DMA_Init(mDvrPcmDacCh1DmaHandle, DMA_REQSEL_DAC_CH1, DMA_SRC_DATA_16B, DMA_SRC_ADDR_INC, DMA_DST_DATA_16B, DMA_DST_ADDR_FIX);
		SET_BIT(mDvrPcmDacCh1DmaHandle->CTRL,DMA_CTRL_DMA_CONTINUE_ENABLE);    // DMA continuous mode enabled.
		DMA_Trigger(mDvrPcmDacCh1DmaHandle, (uint32_t)&DacDataBuf[0], (uint32_t)&DAC->DAC_CH1_DMA_DATA0, DataLen);
		while(DMA_CheckBusy(mDvrPcmDacCh1DmaHandle) != 0);                // Wait DMAx busy
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
 *   DvrPcmStatus [in]: DVRPCM status
 * @return
 *   None.
 */
void static DacAutoRampUp(uint16_t DvrPcmStatus)
{
  if((DvrPcmStatus & DVRPCM_AUTO_RAMP_UP_ENABLE_FLAG) != 0)
	{
		if((DvrPcmStatus & DVRPCM_ENABLE_DAC_CH0_FLAG) != 0)
		{
			RampUpDacCh0();
		}

		if((DvrPcmStatus & DVRPCM_ENABLE_DAC_CH1_FLAG) != 0)
		{
			RampUpDacCh1();
		}
	}
}

/**
 * @brief
 *
 * @param
 *   DvrPcmStatus [in]: DVRPCM status
 * @return
 *   None.
 */
void static DacAutoRampDown(uint16_t DvrPcmStatus)
{
  if((DvrPcmStatus & DVRPCM_AUTO_RAMP_DOWN_ENABLE_FLAG) != 0)
	{
		if((DvrPcmStatus & DVRPCM_ENABLE_DAC_CH0_FLAG) != 0)
		{
			RampDownDacCh0();
		}

		if((DvrPcmStatus & DVRPCM_ENABLE_DAC_CH1_FLAG) != 0)
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
 *   When user call API function SACM_DVRPCM_Initial, this function is called by DVRPCM Library.
 * @param
 *  *DvrPcmWorkingRam [in]: DVRPCM library working RAM address.
 * @return
 *   None.
 */
void DVRPCM_CB_Init(const SACM_DVRPCM_WORKING_RAM *DvrPcmWorkingRam)
{
	mDvrPcmDsAdcDmaHandle = DMA3;
	mDvrPcmDacCh0DmaHandle = DMA3;
	mDvrPcmDacCh1DmaHandle = DMA4;
	mDvrPcmDacTimerHandle = TM0;

  TIMER_Open(mDvrPcmDacTimerHandle);
	TIMER_SetFreq(mDvrPcmDacTimerHandle, (16000 * 4));	                // Sample rate: 16000 x 4 = 64000(Hz), for H/W 4x upsampling

	DAC_Open();
	DAC_Scale_Enable();
	DAC_CH0_Init(DAC_TRG_SEL_TM0);
	DAC_CH1_Init(DAC_TRG_SEL_TM0);
	SET_BIT(DAC->CTRL, DAC_CTRL_CH0_POSTWAVE_OUT_CH0 + DAC_CTRL_CH1_POSTWAVE_OUT_CH1);

	// PWM init
	DAC_AudioPwm_Open();
	DAC_SetAudPwmGain(30);
	DAC_AudioPwm_IP_Enable();

  DSADC_Init(16000);
  DSADC_SetBoostGain(2);
  DSADC_SetPgaGain(18);
	//DSADC_AutoMuteInit(0x180);

	//DAGC_Disable();
  /*
	 * Digital AGC init
	 */
	DAGC_SetCenterBoundary(0x20, 0x1000);
	DAGC_SetSensitivity(0x1E, 0x01, 0xFE);
	DAGC_SetAttackScale(ATTACK_X1);
	DAGC_SetReleaseScale(RELEASE_X1);
	DAGC_ZeroCross_Enable();
	DAGC_Enable();

	EnvDet_Initial(&EnvDetWorkingRam);
	EnvDet_SetAttackLevel(D_AttackLevel);
	EnvDet_SetReleaseLevel(D_ReleaseLevel);
	EnvDet_SetAttackTime(D_AttacTime);
	EnvDet_SetReleaseTime(D_ReleaseTime);
}


/**
 * @brief
 *   Callback function. To be implemented by user.
 *   When user call API function SACM_DVRPCM_Play, this function is called by DVRPCM Library.
 * @param
 *  *DvrPcmWorkingRam [in]: DVRPCM library working RAM address.
 * @return
 *   None.
 */
void DVRPCM_CB_StartPlay(const SACM_DVRPCM_WORKING_RAM *DvrPcmWorkingRam)
{
	DacAutoRampUp(SACM_DVRPCM_GetStatus());                      // DAC auto ramp up

	if((SACM_DVRPCM_GetStatus() & DVRPCM_ENABLE_DAC_CH0_FLAG) != 0)
	{
	  DMA_Init(mDvrPcmDacCh0DmaHandle, DMA_REQSEL_DAC_CH0, DMA_SRC_DATA_16B, DMA_SRC_ADDR_INC, DMA_DST_DATA_16B, DMA_DST_ADDR_FIX);
	  SET_BIT(mDvrPcmDacCh0DmaHandle->CTRL,DMA_CTRL_DMA_CONTINUE_ENABLE);    // DMA continuous mode enabled.
	  DMA_InstallIsrService(mDvrPcmDacCh0DmaHandle, SACM_DVRPCM_DmaIsrService);
	  DMA_EnableInt(mDvrPcmDacCh0DmaHandle);
	  NVIC_EnableIRQ(DMA3_IRQn);
	  DMA_Trigger(mDvrPcmDacCh0DmaHandle, (uint32_t)DvrPcmWorkingRam->PcmBufPtr, (uint32_t)&DAC->DAC_CH0_DMA_DATA0, DVRPCM_FRAME_SIZE);
	}

	if((SACM_DVRPCM_GetStatus() & DVRPCM_ENABLE_DAC_CH1_FLAG) != 0)
	{
		DMA_Init(mDvrPcmDacCh1DmaHandle, DMA_REQSEL_DAC_CH1, DMA_SRC_DATA_16B, DMA_SRC_ADDR_INC, DMA_DST_DATA_16B, DMA_DST_ADDR_FIX);
		SET_BIT(mDvrPcmDacCh1DmaHandle->CTRL,DMA_CTRL_DMA_CONTINUE_ENABLE);    // DMA continuous mode enabled.
    if((SACM_DVRPCM_GetStatus() & DVRPCM_ENABLE_DAC_CH0_FLAG) == 0)
		{
	    DMA_InstallIsrService(mDvrPcmDacCh1DmaHandle, SACM_DVRPCM_DmaIsrService);
	    DMA_EnableInt(mDvrPcmDacCh1DmaHandle);
	    NVIC_EnableIRQ(DMA4_IRQn);
		}
		DMA_Trigger(mDvrPcmDacCh1DmaHandle, (uint32_t)DvrPcmWorkingRam->PcmBufPtr, (uint32_t)&DAC->DAC_CH1_DMA_DATA0, DVRPCM_FRAME_SIZE);
	}
}

/**
 * @brief
 *
 * @param
 *  *DvrPcmWorkingRam [in]: DVRPCM library working RAM address.
 * @return
 *   None.
 */
void DVRPCM_CB_StopPlay(const SACM_DVRPCM_WORKING_RAM *DvrPcmWorkingRam)
{
	if((SACM_DVRPCM_GetStatus() & DVRPCM_PLAY_FLAG) != 0)
  {
    DacAutoRampDown(SACM_DVRPCM_GetStatus());	                // DAC auto ramp down

	  if((SACM_DVRPCM_GetStatus() & DVRPCM_ENABLE_DAC_CH0_FLAG) != 0)
	  {
		  DMA_Close(mDvrPcmDacCh0DmaHandle);
		}

	  if((SACM_DVRPCM_GetStatus() & DVRPCM_ENABLE_DAC_CH1_FLAG) != 0)
	  {
		  DMA_Close(mDvrPcmDacCh1DmaHandle);
		}
  }
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When user call API function SACM_DVRPCM_Pause, this function is called by DVRPCM Library.
 * @param
 *  *DvrPcmWorkingRam [in]: DVRPCM library working RAM address.
 * @return
 *   None.
 */
void DVRPCM_CB_Pause(const SACM_DVRPCM_WORKING_RAM *DvrPcmWorkingRam)
{
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When user call API function SACM_DVRPCM_Resume, this function is Called by DVRPCM Library.
 * @param
 *  *DvrPcmWorkingRam [in]: DVRPCM library working RAM address.
 * @return
 *   None.
 */
void DVRPCM_CB_Resume(const SACM_DVRPCM_WORKING_RAM *DvrPcmWorkingRam)
{
	if((SACM_DVRPCM_GetStatus() & DVRPCM_ENABLE_DAC_CH0_FLAG) != 0)
	{
    DMA_Trigger(mDvrPcmDacCh0DmaHandle, (uint32_t)DvrPcmWorkingRam->PcmBufPtr, (uint32_t)&DAC->DAC_CH0_DMA_DATA0, DVRPCM_FRAME_SIZE);
	}

	if((SACM_DVRPCM_GetStatus() & DVRPCM_ENABLE_DAC_CH1_FLAG) != 0)
	{
    DMA_Trigger(mDvrPcmDacCh1DmaHandle, (uint32_t)DvrPcmWorkingRam->PcmBufPtr, (uint32_t)&DAC->DAC_CH1_DMA_DATA0, DVRPCM_FRAME_SIZE);
	}
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When user call API function SACM_DVRPCM_Rec, this function is called by DVRPCM Library.
 * @param
 *  *DvrPcmWorkingRam [in]: DVRPCM library working RAM address.
 * @return
 *   None.
 */
void DVRPCM_CB_StartRecord(const SACM_DVRPCM_WORKING_RAM *DvrPcmWorkingRam)
{
  R_ActiveFlag = 0;
  R_ReleaseFlag = 0;
	EnvDet_Start();

	DMA_Init(mDvrPcmDsAdcDmaHandle, DMA_REQSEL_DSADC, DMA_SRC_DATA_16B, DMA_SRC_ADDR_FIX, DMA_DST_DATA_16B, DMA_DST_ADDR_INC);
	SET_BIT(mDvrPcmDsAdcDmaHandle->CTRL,DMA_CTRL_DMA_CONTINUE_ENABLE);    // DMA continuous mode enabled.
	DMA_InstallIsrService(mDvrPcmDsAdcDmaHandle, SACM_DVRPCM_DmaIsrService);
	DMA_EnableInt(mDvrPcmDsAdcDmaHandle);
	NVIC_EnableIRQ(DMA3_IRQn);
	DMA_Trigger(mDvrPcmDsAdcDmaHandle, (uint32_t)&DS_ADC->DATA, (uint32_t)DvrPcmWorkingRam->PcmBufPtr, DVRPCM_FRAME_SIZE);
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When user call API function SACM_DVRPCM_Stop to stop recording, this function is called by DVRPCM Library to writeback record data length.
 * @param
 *  *DvrPcmWorkingRam [in]: DVRPCM library working RAM address.
 *   RecDataLen [in]: Record data length (byte).
 * @return
 *   None.
 */
void DVRPCM_CB_StopRecord(const SACM_DVRPCM_WORKING_RAM *DvrPcmWorkingRam, uint32_t RecDataLen)
{
	if(RecDataLen > D_ClickTime)
	{
		RecDataLen -= D_ClickTime;
	}
  SPIFC_WriteNBytes((uint32_t)DvrPcmWorkingRam->SpeechDataAddr, (uint8_t*)&RecDataLen, 4);
	DMA_Close(mDvrPcmDsAdcDmaHandle);
	EnvDet_Stop();
  GPIOA_OBIT->OBIT15 = 0x00000000;      // IOA.15 = 0
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When playing, this function is called by DVRPCM Library to get encoded data.
 * @param
 *  *DvrPcmWorkingRam [in]: DVRPCM library working RAM address.
 *  *SrcDataAddr [in]: Source data address pointer.
 *  *DstBufAddr [out]: Destination buffer address pointer.
 *   DataLen [in]: Data length (halfword)
 * @return
 *   None.
 */
void DVRPCM_CB_GetData(const SACM_DVRPCM_WORKING_RAM *DvrPcmWorkingRam, int16_t *DstBufAddr, int16_t *SrcDataAddr, uint16_t DataLen)
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
 *   When recording, this function is called by DVRPCM Library to write encoded data.
 * @param
 *  *DvrPcmWorkingRam [in]: DVRPCM library working RAM address.
 *  *DstDataAddr [out]: Destination data address pointer.
 *  *SrcBufAddr [in]: Source buffer address pointer.
 *   DataLen [in]: Data length (halfword)
 * @return
 *   1: Notify the kernel to update the count value
 *   0: Notify the kernel not to the count value
 */
uint8_t DVRPCM_CB_WriteData(const SACM_DVRPCM_WORKING_RAM *DvrPcmWorkingRam, int16_t *DstDataAddr, int16_t *SrcBufAddr, uint16_t DataLen)
{
  if(R_ActiveFlag == 1)
  {
    GPIOA_OBIT->OBIT15 = 0xFFFFFFFF;      // IOA.15 = 1
    SPIFC_WriteNBytes((uint32_t)DstDataAddr, (uint8_t *)SrcBufAddr, (DataLen << 1));
    return 1;
  }
  else
  {
    GPIOA_OBIT->OBIT15 = 0x00000000;      // IOA.15 = 0
    return 0;
  }
}


/**
 * @brief
 *   Callback function. Implemented by user.
 *   When playing, this function is called by DVRPCM Library to trigger DAC DMA.
 * @param
 *  *DvrPcmWorkingRam [in]: DvrPcm working RAM pointer
 *  *SrcBufAddr [in]: Source buffer address pointer.
 *   DataLen [in]: Data length (halfword)
 * @return
 *   None.
 */
void DVRPCM_CB_SendDac_DmaIsr(const SACM_DVRPCM_WORKING_RAM *DvrPcmWorkingRam, int16_t *SrcBufAddr, uint16_t DataLen)
{
	if((SACM_DVRPCM_GetStatus() & DVRPCM_ENABLE_DAC_CH0_FLAG) != 0)
	{
		while(DMA_CheckBusy(mDvrPcmDacCh0DmaHandle) != 0);         // Check DMAx busy
		DMA_Trigger(mDvrPcmDacCh0DmaHandle, (uint32_t)SrcBufAddr, (uint32_t)&DAC->DAC_CH0_DMA_DATA0, DataLen);
	}

	if((SACM_DVRPCM_GetStatus() & DVRPCM_ENABLE_DAC_CH1_FLAG) != 0)
	{
		while(DMA_CheckBusy(mDvrPcmDacCh1DmaHandle) != 0);         // Check DMAx busy
    DMA_Trigger(mDvrPcmDacCh1DmaHandle, (uint32_t)SrcBufAddr, (uint32_t)&DAC->DAC_CH1_DMA_DATA0, DataLen);
	}
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When recording, this function is called by DVRPCM Library to trigger DS-ADC DMA.
 * @param
 *  *DvrPcmWorkingRam [in]: DvrPcm working RAM pointer
 *  *DstBufAddr [out]: Destination buffer address pointer.
 *   DataLen [in]: Data length (halfword)
 * @return
 *   None.
 */
void DVRPCM_CB_GetAdc_DmaIsr(const SACM_DVRPCM_WORKING_RAM *DvrPcmWorkingRam, int16_t *DstBufAddr, uint16_t DataLen)
{
	DMA_Trigger(mDvrPcmDsAdcDmaHandle, (uint32_t)&DS_ADC->DATA, (uint32_t)DstBufAddr, DataLen);
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When playing, this function is called by A1801 library to decode data.
 * @param
 *  *DstBufAddr [out]: Destination buffer address pointer.
 *  *SrcBufAddr [in]: Source buffer address pointer.
 * @return
 *   None.
 */
void DVRPCM_CB_DecodeProcess(const SACM_DVRPCM_WORKING_RAM *DvrPcmWorkingRam, int16_t *DstBufAddr, int16_t *SrcBufAddr)
{
	int32_t iCount;
	for(iCount = 0; iCount < DVRPCM_FRAME_SIZE; iCount++)
	{
		DstBufAddr[iCount] = SrcBufAddr[iCount];
	}
	APP_SwVolCtrl_VolProcess(0, DstBufAddr, DVRPCM_FRAME_SIZE);
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When recording, this function is called by DVR1801 library to encode data.
 * @param
 *  *DstBufAddr [out]: Destination buffer address pointer.
 *  *SrcBufAddr [in]: Source buffer address pointer.
 * @return
 *   None.
 */
void DVRPCM_CB_EncodeProcess(const SACM_DVRPCM_WORKING_RAM *DvrPcmWorkingRam, int16_t *DstBufAddr, int16_t *SrcBufAddr)
{
	uint16_t temp;

	EnvDet_ServiceLoop(DstBufAddr, DVRPCM_FRAME_SIZE);

  temp = EnvDet_GetStatus();
  if(READ_BIT (temp, ENV_DET_ATTACK_ACTIVE))
  {
      R_ActiveFlag = 1;
  }

  if(READ_BIT (temp, ENV_DET_RELEASE_ACTIVE))
  {
      R_ReleaseFlag = 1;
  }
}

