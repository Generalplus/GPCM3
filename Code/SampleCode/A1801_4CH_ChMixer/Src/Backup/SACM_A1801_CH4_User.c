/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   SACM_A1801_CH4_User.c
 * @Version:
 *   V1.0.4
 * @Date:
 *   March 10, 2025
 * @Abstract:
 *
 **************************************************************************************************/


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "GPCM3_FM1.h"
#include "SACM_A1801_User.h"
#include "SACM_A1801_CH4_User.h"
// #include "GP_Eventor_User.h"
#include "A1801_FileMerger.h"
#include "APP_SwVolumeControl.h"


/*---------------------------------------------------------------------------------------
 * Gobal Variable Declaration Area
 *---------------------------------------------------------------------------------------*/
TIMER_TYPE_DEF *mA1801Ch4DacTimerHandle;
DMA_TYPE_DEF *mA1801Ch4DacCh0DmaHandle;
DMA_TYPE_DEF *mA1801Ch4DacCh1DmaHandle;

/*
#if defined(__CC_ARM)
__align(4) GP_EVENTOR_WORKING_RAM GpEventorCh1WorkingRam;
__align(4) GP_EVENTOR_CH1_SWPWM_WORKING_RAM GpEventorCh1SwPwmWorkingRam;
#elif defined(__GNUC__)
__attribute__ ((aligned (4))) GP_EVENTOR_WORKING_RAM GpEventorCh3WorkingRam;
__attribute__ ((aligned (4))) GP_EVENTOR_CH3_SWPWM_WORKING_RAM GpEventorCh3SwPwmWorkingRam;
#endif
*/

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
void A1801_CH4_User_A1801Ch4WithGpEvent_ServerLoop()
{
  SACM_A1801_CH4_ServiceLoop();
  // GP_EVENTOR_ServiceLoop(EVENTOR_CH3);
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
    TIMER_Close(mA1801Ch4DacTimerHandle);
	  DMA_Init(mA1801Ch4DacCh0DmaHandle, DMA_REQSEL_DAC_CH0, DMA_SRC_DATA_16B, DMA_SRC_ADDR_INC, DMA_DST_DATA_16B, DMA_DST_ADDR_FIX);
	  DMA_Trigger(mA1801Ch4DacCh0DmaHandle, (uint32_t)&DacDataBuf[0], (uint32_t)&DAC->DAC_CH0_DMA_DATA0, DataLen);
		TIMER_Open(mA1801Ch4DacTimerHandle);
		while(DMA_CheckBusy(mA1801Ch4DacCh0DmaHandle) != 0);                // Wait DMAx busy
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
    TIMER_Close(mA1801Ch4DacTimerHandle);
		DMA_Init(mA1801Ch4DacCh1DmaHandle, DMA_REQSEL_DAC_CH1, DMA_SRC_DATA_16B, DMA_SRC_ADDR_INC, DMA_DST_DATA_16B, DMA_DST_ADDR_FIX);
		DMA_Trigger(mA1801Ch4DacCh1DmaHandle, (uint32_t)&DacDataBuf[0], (uint32_t)&DAC->DAC_CH1_DMA_DATA0, DataLen);
		TIMER_Open(mA1801Ch4DacTimerHandle);
		while(DMA_CheckBusy(mA1801Ch4DacCh1DmaHandle) != 0);              // Wait DMAx busy
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
 *   A1801Ch4Status [in]: A1801 ch2 status
 * @return
 *   None.
 */
void static DacAutoRampUp(uint16_t A1801Ch4Status)
{
  if((A1801Ch4Status & A1801_AUTO_RAMP_UP_ENABLE_FLAG) != 0)
	{
		if((A1801Ch4Status & A1801_ENABLE_DAC_CH0_FLAG) != 0)
		{
			RampUpDacCh0();
		}

		if((A1801Ch4Status & A1801_ENABLE_DAC_CH1_FLAG) != 0)
		{
			RampUpDacCh1();
		}
	}
}

/**
 * @brief
 *
 * @param
 *   A1801Ch4Status [in]: A1801 ch2 status
 * @return
 *   None.
 */
void static DacAutoRampDown(uint16_t A1801Ch4Status)
{
  if((A1801Ch4Status & A1801_AUTO_RAMP_DOWN_ENABLE_FLAG) != 0)
	{
		if((A1801Ch4Status & A1801_ENABLE_DAC_CH0_FLAG) != 0)
		{
			RampDownDacCh0();
		}

		if((A1801Ch4Status & A1801_ENABLE_DAC_CH1_FLAG) != 0)
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
 *   When user call API function SACM_A1801_CH4_Initial, this function is called by A1801_CH4 Library.
 * @param
 *  *A1801Ch4WorkingRam [in]: A1801_CH4 library working RAM address.
 * @return
 *   None.
 */
void A1801_CH4_CB_Init(const SACM_A1801_WORKING_RAM *A1801Ch4WorkingRam)
{
	mA1801Ch4DacCh0DmaHandle = DMA1;
	mA1801Ch4DacCh1DmaHandle = DMA2;
	mA1801Ch4DacTimerHandle = TM0;

  TIMER_Open(mA1801Ch4DacTimerHandle);
	TIMER_SetFreq(mA1801Ch4DacTimerHandle, (16000 * 4));	                 // Sample rate: 16000 x 4 = 64000(Hz), for H/W 4x upsampling

	DAC_Open();
	DAC_Scale_Enable();
	DAC_CH0_Init(DAC_TRG_SEL_TM0);
	DAC_CH1_Init(DAC_TRG_SEL_TM1);
	SET_BIT(DAC->CTRL, DAC_CTRL_CH0_POSTWAVE_OUT_CH0 + DAC_CTRL_CH1_POSTWAVE_OUT_CH1);

	// PWM init
	DAC_AudioPwm_Open();
	DAC_SetAudPwmGain(31);      // 1x Gain.

	// GP_EVENTOR_Initial(EVENTOR_CH3, &GpEventorCh3WorkingRam, (int16_t*)&GpEventorCh3SwPwmWorkingRam, EVENTOR_CH3_IO_NUM, EVENTOR_CH3_SW_PWM_LEVEL);
}

/**
 * @brief
 *   Callback function. To be implemented by user.
 *   When user call API function SACM_A1801_CH4_Play, this function is called by A1801_CH4 Library.
 * @param
 *  *A1801Ch4WorkingRam [in]: A1801_CH4 library working RAM address.
 * @return
 *   None.
 */
void A1801_CH4_CB_StartPlay(const SACM_A1801_WORKING_RAM *A1801Ch4WorkingRam)
{
	if((SACM_A1801_GetStatus() & A1801_PLAY_FLAG) == 0)
	{
		DAC_AudioPwm_IP_Enable();
	}
	DacAutoRampUp(SACM_A1801_CH4_GetStatus());                            // DAC auto ramp up

	if((SACM_A1801_CH4_GetStatus() & A1801_ENABLE_DAC_CH0_FLAG) != 0)
	{
    CLEAR_BIT(DAC->CTRL, DAC_CTRL_DAC_CH0_DMA_ENABLE);                // DAC_Ch0 DMA request disabled.
    TIMER_Close(mA1801Ch4DacTimerHandle);
    while(DMA_CheckBusy(mA1801Ch4DacCh0DmaHandle) != 0);                // Wait DMAx busy
	  DMA_Init(mA1801Ch4DacCh0DmaHandle, DMA_REQSEL_DAC_CH0, DMA_SRC_DATA_16B, DMA_SRC_ADDR_INC, DMA_DST_DATA_16B, DMA_DST_ADDR_FIX);
	  DMA_InstallIsrService(mA1801Ch4DacCh0DmaHandle, SACM_A1801_CH4_DmaIsrService);
	  DMA_EnableInt(mA1801Ch4DacCh0DmaHandle);
	  NVIC_EnableIRQ(DMA1_IRQn);
	  DMA_Trigger(mA1801Ch4DacCh0DmaHandle, (uint32_t)A1801Ch4WorkingRam->PcmBufPtr, (uint32_t)&DAC->DAC_CH0_DMA_DATA0, A1801_FRAME_SIZE);
 	  TIMER_Open(mA1801Ch4DacTimerHandle);
	}

	if((SACM_A1801_CH4_GetStatus() & A1801_ENABLE_DAC_CH1_FLAG) != 0)
	{
    CLEAR_BIT(DAC->CTRL, DAC_CTRL_DAC_CH1_DMA_ENABLE);                // DAC_Ch1 DMA request disabled.
    TIMER_Close(mA1801Ch4DacTimerHandle);
    while(DMA_CheckBusy(mA1801Ch4DacCh1DmaHandle) != 0);                // Wait DMAx busy
		DMA_Init(mA1801Ch4DacCh1DmaHandle, DMA_REQSEL_DAC_CH1, DMA_SRC_DATA_16B, DMA_SRC_ADDR_INC, DMA_DST_DATA_16B, DMA_DST_ADDR_FIX);
    if((SACM_A1801_CH4_GetStatus() & A1801_ENABLE_DAC_CH0_FLAG) == 0)
		{
	    DMA_InstallIsrService(mA1801Ch4DacCh1DmaHandle, SACM_A1801_CH4_DmaIsrService);
	    DMA_EnableInt(mA1801Ch4DacCh1DmaHandle);
	    NVIC_EnableIRQ(DMA2_IRQn);
		}
		DMA_Trigger(mA1801Ch4DacCh1DmaHandle, (uint32_t)A1801Ch4WorkingRam->PcmBufPtr, (uint32_t)&DAC->DAC_CH1_DMA_DATA0, A1801_FRAME_SIZE);
    TIMER_Open(mA1801Ch4DacTimerHandle);
	}

	// GP_EVENTOR_Start(EVENTOR_CH3, (uint8_t *)(&A1801Ch4WorkingRam->SpeechDataAddr[A1801Ch4WorkingRam->DataLen]));
}

/**
 * @brief
 *
 * @param
 *  *A1801WorkingRam [in]: A1801 library working RAM address.
 * @return
 *   None.
 */
void A1801_CH4_CB_Play_Con(const SACM_A1801_WORKING_RAM *A1801Ch4WorkingRam)
{
	// GP_EVENTOR_Start(EVENTOR_CH3, (uint8_t *)(&A1801Ch4WorkingRam->SpeechDataAddr[A1801Ch4WorkingRam->DataLen]));
}

/**
 * @brief
 *
 * @param
 *  *A1801Ch4WorkingRam [in]: A1801_CH4 library working RAM address.
 * @return
 *   None.
 */
void A1801_CH4_CB_StopPlay(const SACM_A1801_WORKING_RAM *A1801Ch4WorkingRam)
{
	if((SACM_A1801_CH4_GetStatus() & A1801_PLAY_FLAG) != 0)
  {
	  DacAutoRampDown(SACM_A1801_CH4_GetStatus());	                      // DAC auto ramp down

		if((SACM_A1801_CH4_GetStatus() & A1801_ENABLE_DAC_CH0_FLAG) != 0)
	  {
		  DMA_Close(mA1801Ch4DacCh0DmaHandle);
		}

	  if((SACM_A1801_CH4_GetStatus() & A1801_ENABLE_DAC_CH1_FLAG) != 0)
	  {
		  DMA_Close(mA1801Ch4DacCh1DmaHandle);
		}

	  if((SACM_A1801_GetStatus() & A1801_PLAY_FLAG) == 0)
	  {
		  DAC_AudioPwm_IP_Disable();
	  }
  }

	// GP_EVENTOR_Stop(EVENTOR_CH3);
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When user call API function SACM_A1801_CH4_Pause, this function is called by A1801_CH4 Library.
 * @param
 *  *A1801Ch4WorkingRam [in]: A1801_CH4 library working RAM address.
 * @return
 *   None.
 */
void A1801_CH4_CB_Pause(const SACM_A1801_WORKING_RAM *A1801Ch4WorkingRam)
{
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When user call API function SACM_A1801_CH4_Resume, this function is Called by A1801_CH4 Library.
 * @param
 *  *A1801Ch4WorkingRam [in]: A1801_CH4 library working RAM address.
 * @return
 *   None.
 */
void A1801_CH4_CB_Resume(const SACM_A1801_WORKING_RAM *A1801Ch4WorkingRam)
{
	if((SACM_A1801_CH4_GetStatus() & A1801_ENABLE_DAC_CH0_FLAG) != 0)
	{
    DMA_Trigger(mA1801Ch4DacCh0DmaHandle, (uint32_t)A1801Ch4WorkingRam->PcmBufPtr, (uint32_t)&DAC->DAC_CH0_DMA_DATA0, A1801_FRAME_SIZE);
	}

	if((SACM_A1801_CH4_GetStatus() & A1801_ENABLE_DAC_CH1_FLAG) != 0)
	{
    DMA_Trigger(mA1801Ch4DacCh1DmaHandle, (uint32_t)A1801Ch4WorkingRam->PcmBufPtr, (uint32_t)&DAC->DAC_CH1_DMA_DATA0, A1801_FRAME_SIZE);
	}
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When playing, this function is called by A1801_CH4 Library to get encoded data.
 * @param
 *  *A1801Ch4WorkingRam [in]: A1801_CH4 library working RAM address.
 *  *DstBufAddr [out]: Destination buffer address pointer.
 *  *SrcDataAddr [in]: Source data address pointer.
 *   DataLen [in]: Data length (halfword)
 * @return
 *   None.
 */
void A1801_CH4_CB_GetData(const SACM_A1801_WORKING_RAM *A1801Ch4WorkingRam, int16_t *DstBufAddr, int16_t *SrcDataAddr, uint16_t DataLen)
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
 *   When playing, this function is called by A1801_CH4 Library to trigger DAC DMA.
 * @param
 *  *A1801Ch4WorkingRam [in]: A1801_CH4 working RAM pointer
 *  *SrcBufAddr [in]: Source buffer address pointer.
 *   DataLen [in]: Data length (halfword)
 * @return
 *   None.
 */
void A1801_CH4_CB_SendDac_DmaIsr(const SACM_A1801_WORKING_RAM *A1801Ch4WorkingRam, int16_t *SrcBufAddr, uint16_t DataLen)
{
	if((SACM_A1801_CH4_GetStatus() & A1801_ENABLE_DAC_CH0_FLAG) != 0)
	{
		while(DMA_CheckBusy(mA1801Ch4DacCh0DmaHandle) != 0);                // Wait DMAx busy
		DMA_Trigger(mA1801Ch4DacCh0DmaHandle, (uint32_t)SrcBufAddr, (uint32_t)&DAC->DAC_CH0_DMA_DATA0, DataLen);
	}

	if((SACM_A1801_CH4_GetStatus() & A1801_ENABLE_DAC_CH1_FLAG) != 0)
	{
		while(DMA_CheckBusy(mA1801Ch4DacCh1DmaHandle) != 0);                // Wait DMAx busy
    DMA_Trigger(mA1801Ch4DacCh1DmaHandle, (uint32_t)SrcBufAddr, (uint32_t)&DAC->DAC_CH1_DMA_DATA0, DataLen);
	}
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When playing, this function is called by A1801_CH4 library to decode data.
 * @param
 *  *A1801Ch4WorkingRam [in]: A1801_CH4 working RAM pointer
 *  *DstBufAddr [out]: Destination buffer address pointer.
 *  *SrcBufAddr [in]: Source buffer address pointer.
 * @return
 *   None.
 */
void A1801_CH4_CB_DecodeProcess(const SACM_A1801_WORKING_RAM *A1801Ch4WorkingRam, int16_t *DstBufAddr, int16_t *SrcBufAddr)
{
	uint32_t GpEventFlag;

  GpEventFlag = SACM_A1801_CH4_DecodeProcess(DstBufAddr, SrcBufAddr);
	APP_SwVolCtrl_VolProcess(3, DstBufAddr, A1801_FRAME_SIZE);

  if(GpEventFlag != 0)
	{
		// GP_EVENTOR_AddEvent(EVENTOR_CH3);
	}
}
