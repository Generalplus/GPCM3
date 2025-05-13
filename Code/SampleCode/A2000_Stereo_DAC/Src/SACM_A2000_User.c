/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   SACM_A2000_User.c
 * @Version:
 *   V1.0.0
 * @Date:
 *   June 12, 2024
 * @Abstract:
 *
 **************************************************************************************************/


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "GPCM3_FM1.h"
#include "SACM_A2000_User.h"
#include "GP_Eventor_User.h"
#include "A2000_FileMerger.h"
#include "APP_SwVolumeControl.h"

extern uint8_t AudioOutType;


/*---------------------------------------------------------------------------------------
 * Gobal Variable Declaration Area
 *---------------------------------------------------------------------------------------*/
TIMER_TYPE_DEF *mA2000DacTimerHandle;
DMA_TYPE_DEF *mA2000DacCh0DmaHandle;
DMA_TYPE_DEF *mA2000DacCh1DmaHandle;

__attribute__ ((aligned (4))) GP_EVENTOR_WORKING_RAM GpEventorCh0WorkingRam;
__attribute__ ((aligned (4))) GP_EVENTOR_CH0_SWPWM_WORKING_RAM GpEventorCh0SwPwmWorkingRam;
// extern uint8_t SR_32K_Ctrl;

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
uint16_t A2000_User_GetA2000Num()
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
int16_t* A2000_User_GetA2000StartAddr(uint16_t SpeechIdx)
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
void A2000_User_A2000WithGpEventServerLoop()
{
  SACM_A2000_ServiceLoop();
  GP_EVENTOR_ServiceLoop(EVENTOR_CH0);
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
		TIMER_Close(mA2000DacTimerHandle);
	  DMA_Init(mA2000DacCh0DmaHandle, DMA_REQSEL_DAC_CH0, DMA_SRC_DATA_16B, DMA_SRC_ADDR_INC, DMA_DST_DATA_16B, DMA_DST_ADDR_FIX);
	  DMA_Trigger(mA2000DacCh0DmaHandle, (uint32_t)&DacDataBuf[0], (uint32_t)&DAC->DAC_CH0_DMA_DATA0, DataLen);
		TIMER_Open(mA2000DacTimerHandle);
		while(DMA_CheckBusy(mA2000DacCh0DmaHandle) != 0);                // Wait DMAx busy
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
		TIMER_Close(mA2000DacTimerHandle);
		DMA_Init(mA2000DacCh1DmaHandle, DMA_REQSEL_DAC_CH1, DMA_SRC_DATA_16B, DMA_SRC_ADDR_INC, DMA_DST_DATA_16B, DMA_DST_ADDR_FIX);
		DMA_Trigger(mA2000DacCh1DmaHandle, (uint32_t)&DacDataBuf[0], (uint32_t)&DAC->DAC_CH1_DMA_DATA0, DataLen);
		TIMER_Open(mA2000DacTimerHandle);
		while(DMA_CheckBusy(mA2000DacCh1DmaHandle) != 0);                // Wait DMAx busy
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
 *   A2000Status [in]: A2000 status
 * @return
 *   None.
 */
void static DacAutoRampUp(uint16_t A2000Status)
{
  if((A2000Status & A2000_AUTO_RAMP_UP_ENABLE_FLAG) != 0)
	{
		if((A2000Status & A2000_ENABLE_DAC_CH0_FLAG) != 0)
		{
			RampUpDacCh0();
		}

		if((A2000Status & A2000_ENABLE_DAC_CH1_FLAG) != 0)
		{
			RampUpDacCh1();
		}
	}
}

/**
 * @brief
 *
 * @param
 *   A2000Status [in]: A2000 status
 * @return
 *   None.
 */
void static DacAutoRampDown(uint16_t A2000Status)
{
  if((A2000Status & A2000_AUTO_RAMP_DOWN_ENABLE_FLAG) != 0)
	{
		if((A2000Status & A2000_ENABLE_DAC_CH0_FLAG) != 0)
		{
			RampDownDacCh0();
		}

		if((A2000Status & A2000_ENABLE_DAC_CH1_FLAG) != 0)
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
 *   When user call API function SACM_A2000_Initial, this function is called by A2000 Library.
 * @param
 *  *A2000WorkingRam [in]: A2000 library working RAM address.
 * @return
 *   None.
 */
void A2000_CB_Init(const SACM_A2000_WORKING_RAM *A2000WorkingRam)
{
	mA2000DacCh0DmaHandle = DMA0;
	mA2000DacCh1DmaHandle = DMA1;
	mA2000DacTimerHandle = TM0;

  TIMER_Open(mA2000DacTimerHandle);
	TIMER_SetFreq(mA2000DacTimerHandle, (32000 * 4));	                 // Sample rate: 32000 x 4 = 128000(Hz), for H/W 4x upsampling

	/*
	if(SR_32K_Ctrl == 1)
	{
    TIMER_SetFreq(mA2000DacTimerHandle, (32000 * 4));
	}
	else    // 20K S.R
	{
    TIMER_SetFreq(mA2000DacTimerHandle, (20000 * 4));
	}
	*/

  // DAC init
	DAC_Open();
	DAC_Scale_Enable();
	DAC_CH0_Init(DAC_TRG_SEL_TM0);
	// DAC_CH1_Init(DAC_TRG_SEL_TM0);
	SET_BIT(DAC->CTRL, DAC_CTRL_CH0_POSTWAVE_OUT_CH0 + DAC_CTRL_CH1_POSTWAVE_OUT_CH1);
	SET_BIT(DAC->CTRL, DAC_CTRL_CH0_SRC_ONLY_CH0 + DAC_CTRL_CH1_SRC_ONLY_CH1);


	// PWM init
	DAC_AudioPwm_Open();
	DAC_SetAudPwmGain(31);      // 1x Gain.

  // GP_EVENTOR_Initial(EVENTOR_CH0, &GpEventorCh0WorkingRam, (int16_t*)&GpEventorCh0SwPwmWorkingRam, EVENTOR_CH0_IO_NUM, EVENTOR_CH0_SW_PWM_LEVEL);

}

void A2000_SR_Change(void)
{
	mA2000DacTimerHandle = TM0;

  TIMER_Open(mA2000DacTimerHandle);

  /*
	if(SR_32K_Ctrl == 1)
	{
    TIMER_SetFreq(mA2000DacTimerHandle, (32000 * 4));	                 // Sample rate: 16000 x 4 = 64000(Hz), for H/W 4x upsampling
	}
	else    // 20K S.R
	{
    TIMER_SetFreq(mA2000DacTimerHandle, (20000 * 4));
	}
	*/
}


/**
 * @brief
 *   Callback function. To be implemented by user.
 *   When user call API function SACM_A2000_Play, this function is called by A2000 Library.
 * @param
 *  *A2000WorkingRam [in]: A2000 library working RAM address.
 * @return
 *   None.
 */
void A2000_CB_StartPlay(const SACM_A2000_WORKING_RAM *A2000WorkingRam)
{
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
	DacAutoRampUp(SACM_A2000_GetStatus());                             // DAC auto ramp up

	if((SACM_A2000_GetStatus() & A2000_ENABLE_DAC_CH0_FLAG) != 0)
	{
    CLEAR_BIT(DAC->CTRL, DAC_CTRL_DAC_CH0_DMA_ENABLE);                // DAC_Ch0 DMA request disabled.
    TIMER_Close(mA2000DacTimerHandle);
    while(DMA_CheckBusy(mA2000DacCh0DmaHandle) != 0);
	  DMA_Init(mA2000DacCh0DmaHandle, DMA_REQSEL_DAC_CH0, DMA_SRC_DATA_16B, DMA_SRC_ADDR_INC, DMA_DST_DATA_16B, DMA_DST_ADDR_FIX);
	  DMA_InstallIsrService(mA2000DacCh0DmaHandle, SACM_A2000_DmaIsrService);
	  DMA_EnableInt(mA2000DacCh0DmaHandle);
	  NVIC_EnableIRQ(DMA0_IRQn);
	  DMA_Trigger(mA2000DacCh0DmaHandle, (uint32_t)A2000WorkingRam->PcmBufPtr, (uint32_t)&DAC->DAC_CH0_DMA_DATA0, A2000_FRAME_SIZE);
	  TIMER_Open(mA2000DacTimerHandle);
	}

	if((SACM_A2000_GetStatus() & A2000_ENABLE_DAC_CH1_FLAG) != 0)
	{
    CLEAR_BIT(DAC->CTRL, DAC_CTRL_DAC_CH1_DMA_ENABLE);                // DAC_Ch1 DMA request disabled.
    TIMER_Close(mA2000DacTimerHandle);
    while(DMA_CheckBusy(mA2000DacCh1DmaHandle) != 0);
		DMA_Init(mA2000DacCh1DmaHandle, DMA_REQSEL_DAC_CH1, DMA_SRC_DATA_16B, DMA_SRC_ADDR_INC, DMA_DST_DATA_16B, DMA_DST_ADDR_FIX);
    if((SACM_A2000_GetStatus() & A2000_ENABLE_DAC_CH0_FLAG) == 0)
		{
	    DMA_InstallIsrService(mA2000DacCh1DmaHandle, SACM_A2000_DmaIsrService);
	    DMA_EnableInt(mA2000DacCh1DmaHandle);
		}
		NVIC_EnableIRQ(DMA1_IRQn);
		DMA_Trigger(mA2000DacCh1DmaHandle, (uint32_t)A2000WorkingRam->PcmBufPtr, (uint32_t)&DAC->DAC_CH1_DMA_DATA0, A2000_FRAME_SIZE);
		TIMER_Open(mA2000DacTimerHandle);
	}

	// GP_EVENTOR_Start(EVENTOR_CH0, (uint8_t *)(&A2000WorkingRam->SpeechDataAddr[A2000WorkingRam->DataLen]));
}

/**
 * @brief
 *
 * @param
 *  *A2000WorkingRam [in]: A2000 library working RAM address.
 * @return
 *   None.
 */
void A2000_CB_Play_Con(const SACM_A2000_WORKING_RAM *A2000WorkingRam)
{
	// GP_EVENTOR_Start(EVENTOR_CH0, (uint8_t *)(&A2000WorkingRam->SpeechDataAddr[A2000WorkingRam->DataLen]));
}

/**
 * @brief
 *
 * @param
 *  *A2000WorkingRam [in]: A2000 library working RAM address.
 * @return
 *   None.
 */
void A2000_CB_StopPlay(const SACM_A2000_WORKING_RAM *A2000WorkingRam)
{
	if((SACM_A2000_GetStatus() & A2000_PLAY_FLAG) != 0)
  {
	  DacAutoRampDown(SACM_A2000_GetStatus());	                       // DAC auto ramp down

		if((SACM_A2000_GetStatus() & A2000_ENABLE_DAC_CH0_FLAG) != 0)
	  {
		  DMA_Close(mA2000DacCh0DmaHandle);
		}
	  if((SACM_A2000_GetStatus() & A2000_ENABLE_DAC_CH1_FLAG) != 0)
	  {
		  DMA_Close(mA2000DacCh1DmaHandle);
		}
		/*
		DAC_AudioPwm_IP_Disable();
    GP_EVENTOR_Stop(EVENTOR_CH0);
    */
  }
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When user call API function SACM_A2000_Pause, this function is called by A2000 Library.
 * @param
 *  *A2000WorkingRam [in]: A2000 library working RAM address.
 * @return
 *   None.
 */
void A2000_CB_Pause(const SACM_A2000_WORKING_RAM *A2000WorkingRam)
{
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When user call API function SACM_A2000_Resume, this function is Called by A2000 Library.
 * @param
 *  *A2000WorkingRam [in]: A2000 library working RAM address.
 * @return
 *   None.
 */
void A2000_CB_Resume(const SACM_A2000_WORKING_RAM *A2000WorkingRam)
{
	if((SACM_A2000_GetStatus() & A2000_ENABLE_DAC_CH0_FLAG) != 0)
	{
    DMA_Trigger(mA2000DacCh0DmaHandle, (uint32_t)A2000WorkingRam->PcmBufPtr, (uint32_t)&DAC->DAC_CH0_DMA_DATA0, A2000_FRAME_SIZE);
	}

	if((SACM_A2000_GetStatus() & A2000_ENABLE_DAC_CH1_FLAG) != 0)
	{
    DMA_Trigger(mA2000DacCh1DmaHandle, (uint32_t)A2000WorkingRam->PcmBufPtr, (uint32_t)&DAC->DAC_CH1_DMA_DATA0, A2000_FRAME_SIZE);
	}
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When playing, this function is called by A2000 Library to get encoded data.
 * @param
 *  *A2000WorkingRam [in]: A2000 library working RAM address.
 *  *SrcDataAddr [in]: Source data address pointer.
 *  *DstBufAddr [out]: Destination buffer address pointer.
 *   DataLen [in]: Data length (halfword)
 * @return
 *   None.
 */
void A2000_CB_GetData(const SACM_A2000_WORKING_RAM *A2000WorkingRam, int16_t *DstBufAddr, int16_t *SrcDataAddr, uint16_t DataLen)
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
 *   When playing, this function is called by A2000 Library to trigger DAC DMA.
 * @param
 *  *A2000WorkingRam [in]: A2000 working RAM pointer
 *  *SrcBufAddr [in]: Source buffer address pointer.
 *   DataLen [in]: Data length (halfword)
 * @return
 *   None.
 */
void A2000_CB_SendDac_DmaIsr(const SACM_A2000_WORKING_RAM *A2000WorkingRam, int16_t *SrcBufAddr, uint16_t DataLen)
{
	if((SACM_A2000_GetStatus() & A2000_ENABLE_DAC_CH0_FLAG) != 0)
	{
		while(DMA_CheckBusy(mA2000DacCh0DmaHandle) != 0);                // Wait DMAx busy
		DMA_Trigger(mA2000DacCh0DmaHandle, (uint32_t)SrcBufAddr, (uint32_t)&DAC->DAC_CH0_DMA_DATA0, DataLen);
	}

	if((SACM_A2000_GetStatus() & A2000_ENABLE_DAC_CH1_FLAG) != 0)
	{
		while(DMA_CheckBusy(mA2000DacCh1DmaHandle) != 0);                // Wait DMAx busy
    DMA_Trigger(mA2000DacCh1DmaHandle, (uint32_t)SrcBufAddr, (uint32_t)&DAC->DAC_CH1_DMA_DATA0, DataLen);
	}

}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When playing, this function is called by A2000 library to decode data.
 * @param
 *  *A2000WorkingRam [in]: A2000 working RAM pointer
 *  *DstBufAddr [out]: Destination buffer address pointer.
 *  *SrcBufAddr [in]: Source buffer address pointer.
 * @return
 *   None.
 */
void A2000_CB_DecodeProcess(const SACM_A2000_WORKING_RAM *A2000WorkingRam, int16_t *DstBufAddr, int16_t *SrcBufAddr)
{
	uint32_t GpEventFlag;

  GpEventFlag = SACM_A2000_DecodeProcess(DstBufAddr, SrcBufAddr);

  APP_SwVolCtrl_VolProcess(0, DstBufAddr, A2000_FRAME_SIZE);

  if(GpEventFlag != 0)
	{
		GP_EVENTOR_AddEvent(EVENTOR_CH0);
	}
}
