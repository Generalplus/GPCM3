/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   SACM_A3400Pro_User.c
 * @Version:
 *   V1.0.3
 * @Date:
 *   May 02th, 2023
 * @Abstract:
 *
 **************************************************************************************************/

/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "GPCM3_FM1.h"
#include "SACM_A3400Pro_User.h"
#include "APP_SwVolumeControl.h"
#include "A34_FileMerger.h"


/*---------------------------------------------------------------------------------------
 * Gobal Variable Declaration Area
 *---------------------------------------------------------------------------------------*/
#if defined(__CC_ARM)
#define Aligned4B __align(4)
#elif defined(__GNUC__)
#define Aligned4B __attribute__ ((aligned (4)))
#endif

TIMER_TYPE_DEF *mA3400ProDacTimerHandle;
DMA_TYPE_DEF *mA3400ProDacCh0DmaHandle;
DMA_TYPE_DEF *mA3400ProDacCh1DmaHandle;

// Aligned4B	GP_EVENTOR_WORKING_RAM 						GpEventorCh0WorkingRam;
// Aligned4B	GP_EVENTOR_CH0_SWPWM_WORKING_RAM 	GpEventorCh0SwPwmWorkingRam;

uint32_t R_DAC_Data;
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
uint16_t GetA3400ProNum()
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
int16_t* GetA3400ProStartAddr(uint16_t SpeechIdx)
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
	  TIMER_Close(mA3400ProDacTimerHandle);
	  DMA_Init(mA3400ProDacCh0DmaHandle, DMA_REQSEL_DAC_CH0, DMA_SRC_DATA_16B, DMA_SRC_ADDR_INC, DMA_DST_DATA_16B, DMA_DST_ADDR_FIX);
    SET_BIT(mA3400ProDacCh0DmaHandle->CTRL,DMA_CTRL_DMA_CONTINUE_ENABLE);    // DMA continuous mode enabled.
	  DMA_Trigger(mA3400ProDacCh0DmaHandle, (uint32_t)&DacDataBuf[0], (uint32_t)&DAC->DAC_CH0_DMA_DATA0, DataLen);
		TIMER_Open(mA3400ProDacTimerHandle);
		while(DMA_CheckBusy(mA3400ProDacCh0DmaHandle) != 0);              // Wait DMAx busy
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
	  TIMER_Close(mA3400ProDacTimerHandle);
		DMA_Init(mA3400ProDacCh1DmaHandle, DMA_REQSEL_DAC_CH1, DMA_SRC_DATA_16B, DMA_SRC_ADDR_INC, DMA_DST_DATA_16B, DMA_DST_ADDR_FIX);
    SET_BIT(mA3400ProDacCh1DmaHandle->CTRL,DMA_CTRL_DMA_CONTINUE_ENABLE);    // DMA continuous mode enabled.
		DMA_Trigger(mA3400ProDacCh1DmaHandle, (uint32_t)&DacDataBuf[0], (uint32_t)&DAC->DAC_CH1_DMA_DATA0, DataLen);
		TIMER_Open(mA3400ProDacTimerHandle);
		while(DMA_CheckBusy(mA3400ProDacCh1DmaHandle) != 0);              // Wait DMAx busy
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
 *   A3400ProStatus [in]: A3400Pro status
 * @return
 *   None.
 */
void static DacAutoRampUp(uint16_t A3400ProStatus)
{
  if((A3400ProStatus & A3400PRO_AUTO_RAMP_UP_ENABLE_FLAG) != 0)
	{
		if((A3400ProStatus & A3400PRO_ENABLE_DAC_CH0_FLAG) != 0)
		{
			RampUpDacCh0();
		}

		if((A3400ProStatus & A3400PRO_ENABLE_DAC_CH1_FLAG) != 0)
		{
			RampUpDacCh1();
		}
	}
}

/**
 * @brief
 *
 * @param
 *   A3400ProStatus [in]: A3400Pro status
 * @return
 *   None.
 */
void static DacAutoRampDown(uint16_t A3400ProStatus)
{
  if((A3400ProStatus & A3400PRO_AUTO_RAMP_DOWN_ENABLE_FLAG) != 0)
	{
		if((A3400ProStatus & A3400PRO_ENABLE_DAC_CH0_FLAG) != 0)
		{
			RampDownDacCh0();
		}

		if((A3400ProStatus & A3400PRO_ENABLE_DAC_CH1_FLAG) != 0)
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
 *   When user call API function SACM_A3400Pro_Initial, this function is called by A3400Pro Library.
 * @param
 *  *A3400ProWorkingRam [in]: A3400Pro library working RAM address.
 * @return
 *   None.
 */
void A3400Pro_CB_Init(const SACM_A3400PRO_WORKING_RAM *A3400ProWorkingRam)
{
	mA3400ProDacCh0DmaHandle = DMA3;
	mA3400ProDacCh1DmaHandle = DMA4;
	mA3400ProDacTimerHandle = TM0;

  TIMER_Open(mA3400ProDacTimerHandle);
	TIMER_SetFreq(mA3400ProDacTimerHandle, (12000 * 4));	              // Sample rate: 12000 x 4 = 48000(Hz), for H/W 4x upsampling

 	// DAC init
	DAC_Open();
	DAC_Scale_Enable();
	DAC_CH0_Init(DAC_TRG_SEL_TM0);
	DAC_CH1_Init(DAC_TRG_SEL_TM0);
	SET_BIT(DAC->CTRL, DAC_CTRL_CH0_POSTWAVE_OUT_CH0 + DAC_CTRL_CH1_POSTWAVE_OUT_CH1);
	DAC_AudioPwm_Open();
	DAC_SetAudPwmGain(31);      // 1x Gain.

	// GP_EVENTOR_Initial(EVENTOR_CH0, &GpEventorCh0WorkingRam, (int16_t*)&GpEventorCh0SwPwmWorkingRam, EVENTOR_CH0_IO_NUM, EVENTOR_CH0_SW_PWM_LEVEL);
}

/**
 * @brief
 *   Callback function. To be implemented by user.
 *   When user call API function SACM_A3400Pro_Play, this function is called by A3400Pro Library.
 * @param
 *  *A3400ProWorkingRam [in]: A3400Pro library working RAM address.
 * @return
 *   None.
 */
void A3400Pro_CB_StartPlay(const SACM_A3400PRO_WORKING_RAM *A3400ProWorkingRam)
{
	uint32_t DelayCount;

	#if AUDIO_PWM
    DAC_VoltageDAC_CH0_Disable();
    DAC_VoltageDAC_CH1_Disable();
    DAC_AudioPwm_IP_Enable();
	#endif

	#if CURRENT_DAC
    DAC_AudioPwm_IP_Disable();
    DAC_VoltageDAC_CH0_Enable();
    DAC_VoltageDAC_CH1_Enable();
	#endif
	DacAutoRampUp(SACM_A3400Pro_GetStatus());                           // DAC auto ramp up

	if((SACM_A3400Pro_GetStatus() & A3400PRO_ENABLE_DAC_CH0_FLAG) != 0)
	{
    CLEAR_BIT(DAC->CTRL, DAC_CTRL_DAC_CH0_DMA_ENABLE);                // DAC_Ch0 DMA request disabled.
    TIMER_Close(mA3400ProDacTimerHandle);
    while(DMA_CheckBusy(mA3400ProDacCh0DmaHandle) != 0);
	  DMA_Init(mA3400ProDacCh0DmaHandle, DMA_REQSEL_DAC_CH0, DMA_SRC_DATA_16B, DMA_SRC_ADDR_INC, DMA_DST_DATA_16B, DMA_DST_ADDR_FIX);
    SET_BIT(mA3400ProDacCh0DmaHandle->CTRL,DMA_CTRL_DMA_CONTINUE_ENABLE);    // DMA continuous mode enabled.
	  DMA_InstallIsrService(mA3400ProDacCh0DmaHandle, SACM_A3400Pro_DmaIsrService);
	  DMA_EnableInt(mA3400ProDacCh0DmaHandle);
	  NVIC_EnableIRQ(DMA3_IRQn);
	  DMA_Trigger(mA3400ProDacCh0DmaHandle, (uint32_t)A3400ProWorkingRam->PcmBufPtr, (uint32_t)&DAC->DAC_CH0_DMA_DATA0, A3400ProWorkingRam->FrameSize);
    TIMER_Open(mA3400ProDacTimerHandle);
	}

	if((SACM_A3400Pro_GetStatus() & A3400PRO_ENABLE_DAC_CH1_FLAG) != 0)
	{
    CLEAR_BIT(DAC->CTRL, DAC_CTRL_DAC_CH1_DMA_ENABLE);                // DAC_Ch1 DMA request disabled.
    TIMER_Close(mA3400ProDacTimerHandle);
    while(DMA_CheckBusy(mA3400ProDacCh1DmaHandle) != 0);
		DMA_Init(mA3400ProDacCh1DmaHandle, DMA_REQSEL_DAC_CH1, DMA_SRC_DATA_16B, DMA_SRC_ADDR_INC, DMA_DST_DATA_16B, DMA_DST_ADDR_FIX);
    SET_BIT(mA3400ProDacCh1DmaHandle->CTRL,DMA_CTRL_DMA_CONTINUE_ENABLE);    // DMA continuous mode enabled.
    if((SACM_A3400Pro_GetStatus() & A3400PRO_ENABLE_DAC_CH0_FLAG) == 0)
		{
	    DMA_InstallIsrService(mA3400ProDacCh1DmaHandle, SACM_A3400Pro_DmaIsrService);
	    DMA_EnableInt(mA3400ProDacCh1DmaHandle);
	    NVIC_EnableIRQ(DMA4_IRQn);
		}
		DMA_Trigger(mA3400ProDacCh1DmaHandle, (uint32_t)A3400ProWorkingRam->PcmBufPtr, (uint32_t)&DAC->DAC_CH1_DMA_DATA0, A3400ProWorkingRam->FrameSize);
    TIMER_Open(mA3400ProDacTimerHandle);
	}

	// GP_EVENTOR_Start(EVENTOR_CH0, (uint8_t *)A3400ProWorkingRam->EventDataAddr);
}

/**
 * @brief
 *   Callback function. To be implemented by user.
 *   When user call API function SACM_A3400Pro_Play_Con, this function is called by A3400Pro Library.
 * @param
 *  *A3400ProWorkingRam [in]: A3400Pro library working RAM address.
 * @return
 *   None.
 */
void A3400Pro_CB_StartPlay_Con(const SACM_A3400PRO_WORKING_RAM *A3400ProWorkingRam)
{
	// GP_EVENTOR_Start(EVENTOR_CH0, (uint8_t *)A3400ProWorkingRam->EventDataAddr);
}


/**
 * @brief
 *
 * @param
 *  *A3400ProWorkingRam [in]: A3400Pro library working RAM address.
 * @return
 *   None.
 */
void A3400Pro_CB_StopPlay(const SACM_A3400PRO_WORKING_RAM *A3400ProWorkingRam)
{
	if((SACM_A3400Pro_GetStatus() & A3400PRO_PLAY_FLAG) != 0)
  {
	  if((SACM_A3400Pro_GetStatus() & A3400PRO_ENABLE_DAC_CH0_FLAG) != 0)
	  {
		  DMA_Close(mA3400ProDacCh0DmaHandle);
		}

	  if((SACM_A3400Pro_GetStatus() & A3400PRO_ENABLE_DAC_CH1_FLAG) != 0)
	  {
		  DMA_Close(mA3400ProDacCh1DmaHandle);
		}

    DacAutoRampDown(SACM_A3400Pro_GetStatus());	                      // DAC auto ramp down

	#if AUDIO_PWM
    DAC_AudioPwm_IP_Disable();
	#endif

	#if CURRENT_DAC
    DAC_VoltageDac_Disable();
	#endif

		// GP_EVENTOR_Stop(EVENTOR_CH0);
  }
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When user call API function SACM_A3400Pro_Pause, this function is called by A3400Pro Library.
 * @param
 *  *A3400ProWorkingRam [in]: A3400Pro library working RAM address.
 * @return
 *   None.
 */
void A3400Pro_CB_Pause(const SACM_A3400PRO_WORKING_RAM *A3400ProWorkingRam)
{
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When user call API function SACM_A3400Pro_Resume, this function is Called by A3400Pro Library.
 * @param
 *  *A3400ProWorkingRam [in]: A3400Pro library working RAM address.
 * @return
 *   None.
 */
void A3400Pro_CB_Resume(const SACM_A3400PRO_WORKING_RAM *A3400ProWorkingRam)
{
	if((SACM_A3400Pro_GetStatus() & A3400PRO_ENABLE_DAC_CH0_FLAG) != 0)
	{
    DMA_Trigger(mA3400ProDacCh0DmaHandle, (uint32_t)A3400ProWorkingRam->PcmBufPtr, (uint32_t)&DAC->DAC_CH0_DMA_DATA0, A3400ProWorkingRam->FrameSize);
	}

	if((SACM_A3400Pro_GetStatus() & A3400PRO_ENABLE_DAC_CH1_FLAG) != 0)
	{
    DMA_Trigger(mA3400ProDacCh1DmaHandle, (uint32_t)A3400ProWorkingRam->PcmBufPtr, (uint32_t)&DAC->DAC_CH1_DMA_DATA0, A3400ProWorkingRam->FrameSize);
	}
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When playing, this function is called by A3400Pro Library to get encoded data.
 * @param
 *  *A3400ProWorkingRam [in]: A3400Pro library working RAM address.
 *  *DstBufAddr [out]: Destination buffer address pointer.
 *  *SrcDataAddr [in]: Source data address pointer.
 *   DataLen [in]: Data length (halfword)
 * @return
 *   None.
 */
void A3400Pro_CB_GetData(const SACM_A3400PRO_WORKING_RAM *A3400ProWorkingRam, uint8_t *DstBufAddr, uint8_t *SrcDataAddr, uint16_t DataLen)
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
 *   When playing, this function is called by A3400Pro Library to trigger DAC DMA.
 * @param
 *  *A3400ProWorkingRam [in]: A3400Pro working RAM pointer
 *  *SrcBufAddr [in]: Source buffer address pointer.
 *   DataLen [in]: Data length (halfword)
 * @return
 *   None.
 */
void A3400Pro_CB_SendDac_DmaIsr(const SACM_A3400PRO_WORKING_RAM *A3400ProWorkingRam, int16_t *SrcBufAddr, uint16_t DataLen)
{

	if((SACM_A3400Pro_GetStatus() & A3400PRO_ENABLE_DAC_CH0_FLAG) != 0)
	{
		while(DMA_CheckBusy(mA3400ProDacCh0DmaHandle) != 0);              // Check DMAx busy
		DMA_Trigger(mA3400ProDacCh0DmaHandle, (uint32_t)SrcBufAddr, (uint32_t)&DAC->DAC_CH0_DMA_DATA0, DataLen);
		R_DAC_Data = DAC->DAC_CH0_DMA_DATA0;
	}

	if((SACM_A3400Pro_GetStatus() & A3400PRO_ENABLE_DAC_CH1_FLAG) != 0)
	{
		while(DMA_CheckBusy(mA3400ProDacCh1DmaHandle) != 0);              // Check DMAx busy
    DMA_Trigger(mA3400ProDacCh1DmaHandle, (uint32_t)SrcBufAddr, (uint32_t)&DAC->DAC_CH1_DMA_DATA0, DataLen);
	}
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When playing, this function is called by A3400Pro library to decode data.
 * @param
 *  *DstBufAddr [out]: Destination buffer address pointer.
 *  *SrcBufAddr [in]: Source buffer address pointer.
 * @return
 *   None.
 */
void A3400Pro_CB_DecodeProcess(const SACM_A3400PRO_WORKING_RAM *A3400ProWorkingRam, int16_t *DstBufAddr, int16_t *SrcBufAddr)
{
  SACM_A3400Pro_DecodeProcess(DstBufAddr, SrcBufAddr);
/*
	uint32_t GpEventFlag;

  GpEventFlag = SACM_A3400Pro_DecodeProcess(DstBufAddr, SrcBufAddr);
  if(GpEventFlag != 0)
	{
		GP_EVENTOR_AddEvent(EVENTOR_CH0);
	}
	GP_EVENTOR_ServiceLoop(EVENTOR_CH0);
	*/
	APP_SwVolCtrl_VolProcess(0, DstBufAddr, A3400ProWorkingRam->FrameSize);
}

