/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   SACM_VC3_User.c
 * @Version:
 *   V1.0.3.1
 * @Date:
 *   April 18, 2022
 * @Abstract:
 *
 **************************************************************************************************/


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "GPCM3_FM1.h"
#include "SACM_VC3_User.h"
#include "APP_SwVolumeControl.h"
#include "AEC_User.h"

#define AEC_ON								(1)
extern SACM_VC3_API_WORKING_RAM *mVc3ApiWorkingRamPtr;


/*---------------------------------------------------------------------------------------
 * VC3 Shift Pitch Filter Table
 *---------------------------------------------------------------------------------------*/
#if defined(__CC_ARM)
#pragma diag_suppress 1296
#endif
const uint32_t c_au32Bupdn[24] =
{
  (uint32_t) &VC3_ShiftPitchFilterTable1[0],               // shift pitch -12
  (uint32_t) &VC3_ShiftPitchFilterTable1[0],               // shift pitch -11
  (uint32_t) &VC3_ShiftPitchFilterTable1[0],               // shift pitch -10
  (uint32_t) &VC3_ShiftPitchFilterTable1[0],               // shift pitch -9
  (uint32_t) &VC3_ShiftPitchFilterTable1[0],               // shift pitch -8
  (uint32_t) &VC3_ShiftPitchFilterTable1[0],               // shift pitch -7
  (uint32_t) &VC3_ShiftPitchFilterTable1[0],               // shift pitch -6
  (uint32_t) &VC3_ShiftPitchFilterTable1[0],               // shift pitch -5
  (uint32_t) &VC3_ShiftPitchFilterTable1[0],               // shift pitch -4
  (uint32_t) &VC3_ShiftPitchFilterTable1[0],               // shift pitch -3
  (uint32_t) &VC3_ShiftPitchFilterTable1[0],               // shift pitch -2
  (uint32_t) &VC3_ShiftPitchFilterTable1[0],               // shift pitch -1
  (uint32_t) &VC3_ShiftPitchFilterTable1[0],               // shift pitch +1
  (uint32_t) &VC3_ShiftPitchFilterTable2[0],               // shift pitch +2
  (uint32_t) &VC3_ShiftPitchFilterTable3[0],               // shift pitch +3
  (uint32_t) &VC3_ShiftPitchFilterTable4[0],               // shift pitch +4
  (uint32_t) &VC3_ShiftPitchFilterTable5[0],               // shift pitch +5
  (uint32_t) &VC3_ShiftPitchFilterTable6[0],               // shift pitch +6
  (uint32_t) &VC3_ShiftPitchFilterTable7[0],               // shift pitch +7
  (uint32_t) &VC3_ShiftPitchFilterTable8[0],               // shift pitch +8
  (uint32_t) &VC3_ShiftPitchFilterTable9[0],               // shift pitch +9
  (uint32_t) &VC3_ShiftPitchFilterTable10[0],              // shift pitch +10
  (uint32_t) &VC3_ShiftPitchFilterTable11[0],              // shift pitch +11
  (uint32_t) &VC3_ShiftPitchFilterTable12[0],              // shift pitch +12
};


/*---------------------------------------------------------------------------------------
 * Gobal Variable Declaration Area
 *---------------------------------------------------------------------------------------*/
 #if defined(__CC_ARM)
#define Aligned4B __align(4)
#elif defined(__GNUC__)
#define Aligned4B __attribute__ ((aligned (4)))
#endif

DMA_TYPE_DEF *mVc3DsAdcDmaHandle;
TIMER_TYPE_DEF *mVc3DacTimerHandle;
DMA_TYPE_DEF *mVc3DacCh0DmaHandle;
DMA_TYPE_DEF *mVc3DacCh1DmaHandle;

// AEC usage
Aligned4B AEC_MEMORY_BLOCK	AecWorkingRam;
Aligned4B AEC_ADC_RAM 			AecAdcRam;
AEC_ADC_RAM                *mAecAdcRam;

uint32_t R_AEC_WorkFlag = 1;
// Used to avoid "pop" noise
uint8_t DAC_ReadyFlag = 0;

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
	  CLEAR_BIT(DAC->CTRL, DAC_CTRL_DAC_CH0_DMA_ENABLE);             // DAC_Ch0 DMA request disabled.
		TIMER_Close(mVc3DacTimerHandle);
	  DMA_Init(mVc3DacCh0DmaHandle, DMA_REQSEL_DAC_CH0, DMA_SRC_DATA_16B, DMA_SRC_ADDR_INC, DMA_DST_DATA_16B, DMA_DST_ADDR_FIX);
	  SET_BIT(mVc3DacCh0DmaHandle->CTRL,DMA_CTRL_DMA_CONTINUE_ENABLE);    // DMA continuous mode enabled.
	  DMA_Trigger(mVc3DacCh0DmaHandle, (uint32_t)&DacDataBuf[0], (uint32_t)&DAC->DAC_CH0_DMA_DATA0, DataLen);
	  TIMER_Open(mVc3DacTimerHandle);
		while(DMA_CheckBusy(mVc3DacCh0DmaHandle) != 0);                // Wait DMAx busy
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
		TIMER_Close(mVc3DacTimerHandle);
		DMA_Init(mVc3DacCh1DmaHandle, DMA_REQSEL_DAC_CH1, DMA_SRC_DATA_16B, DMA_SRC_ADDR_INC, DMA_DST_DATA_16B, DMA_DST_ADDR_FIX);
		SET_BIT(mVc3DacCh1DmaHandle->CTRL,DMA_CTRL_DMA_CONTINUE_ENABLE);    // DMA continuous mode enabled.
		DMA_Trigger(mVc3DacCh1DmaHandle, (uint32_t)&DacDataBuf[0], (uint32_t)&DAC->DAC_CH1_DMA_DATA0, DataLen);
		TIMER_Open(mVc3DacTimerHandle);
		while(DMA_CheckBusy(mVc3DacCh1DmaHandle) != 0);                // Wait DMAx busy
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
 *   Vc3Status [in]: VC3 status
 * @return
 *   None.
 */
void static DacAutoRampUp(uint16_t Vc3Status)
{
  if((Vc3Status & VC3_AUTO_RAMP_UP_ENABLE_FLAG) != 0)
	{
		if((Vc3Status & VC3_ENABLE_DAC_CH0_FLAG) != 0)
		{
			RampUpDacCh0();
		}

		if((Vc3Status & VC3_ENABLE_DAC_CH1_FLAG) != 0)
		{
			RampUpDacCh1();
		}
	}
}

/**
 * @brief
 *
 * @param
 *   Vc3Status [in]: VC3 status
 * @return
 *   None.
 */
void static DacAutoRampDown(uint16_t Vc3Status)
{
  if((Vc3Status & VC3_AUTO_RAMP_DOWN_ENABLE_FLAG) != 0)
	{
		if((Vc3Status & VC3_ENABLE_DAC_CH0_FLAG) != 0)
		{
			RampDownDacCh0();
		}

		if((Vc3Status & VC3_ENABLE_DAC_CH1_FLAG) != 0)
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
 *   When user call API function SACM_VC3_Initial, this function is called by VC3 Library.
 * @param
 *  *Vc3ApiWorkingRam [in]: VC3 API working RAM address.
 * @return
 *   None.
 */
void VC3_CB_Init(const SACM_VC3_API_WORKING_RAM *Vc3ApiWorkingRam)
{
	mVc3DsAdcDmaHandle = DMA2;
	mVc3DacCh0DmaHandle = DMA3;
	mVc3DacCh1DmaHandle = DMA4;

	mVc3DacTimerHandle = TM0;

  TIMER_Open(mVc3DacTimerHandle);
	TIMER_SetFreq(mVc3DacTimerHandle, (16000 * 4));	         // Sample rate: 16000 x 4 = 64000(Hz), for H/W 4x upsampling

	// DAC init
	DAC_Open();
	DAC_Scale_Enable();
	DAC_CH0_Init(DAC_TRG_SEL_TM0);
	DAC_CH1_Init(DAC_TRG_SEL_TM0);
	SET_BIT(DAC->CTRL, DAC_CTRL_CH0_POSTWAVE_OUT_CH0 + DAC_CTRL_CH1_POSTWAVE_OUT_CH1);
	DAC_AudioPwm_Open();
	DAC_SetAudPwmGain(31);      // 1x Gain.

	DSADC_Init(16000);
	DSADC_SetBoostGain(3);
	DSADC_SetPgaGain(11);
	//DSADC_AutoMuteInit(0x180);
	//DSADC_SetSampleRate(4000);
	//DSADC_SetSampleRate(16000);

  /*
	 * Digital AGC init
	 */
	#if 1
	DAGC_SetCenterBoundary(0x20, 0x1000);
	DAGC_SetSensitivity(0x1E, 0x01, 0xFE);
	DAGC_SetAttackScale(ATTACK_X1);
	DAGC_SetReleaseScale(RELEASE_X1);
	DAGC_ZeroCross_Enable();
	DAGC_Enable();
	#endif
}

/**
 * @brief
 *   Callback function. To be implemented by user.
 *   When user call API function SACM_VC3_Play, this function is called by VC3 Library.
 * @param
 *  *Vc3ApiWorkingRam [in]: VC3 API working RAM address.
 * @return
 *   None.
 */
void VC3_CB_StartPlay(const SACM_VC3_API_WORKING_RAM *Vc3ApiWorkingRam)
{
  DAC_ReadyFlag = 0;
	DAC_AudioPwm_IP_Enable();
	DacAutoRampUp(SACM_VC3_GetStatus());                     // DAC auto ramp up

 /*
  * DMA setting
  */
  #if AEC_ON
    /*
     * 1. Change DMA as 32-bit format which is (16-bit ADC) + (16-bit DAC data)
     * 2. Change CSP16_DmaIsrService() to AEC_DmaIsrService() for preprocess
     * 3. AEC_DmaIsrService() at the bottom of CSP16_User.c
     * 4. Initialize AEC pointer
     */
		EXT_INT_Initial();	// set low priority interrupt.

    mAecAdcRam = &AecAdcRam;
    mAecAdcRam->AecPcmBufPtr = mAecAdcRam->AecPcmBuf[0];
    mAecAdcRam->AecInBufPtr  = mAecAdcRam->AecPcmBuf[1];
    mAecAdcRam->AEC_CQBuf_WtPtr = 0;
    F_AEC_InitProc((AEC_MEMORY_BLOCK*)&AecWorkingRam, (uint16_t)C_STEP, (uint16_t)C_BIAS, 1, 100000);

    DS_ADC->MUTE_CTRL0	|= DS_ADC_MUTE_CTRL0_ECHO_CANCEL_ENABLE;
    DMA_Init(mVc3DsAdcDmaHandle, DMA_REQSEL_DSADC, DMA_SRC_DATA_32B, DMA_SRC_ADDR_FIX, DMA_DST_DATA_32B, DMA_DST_ADDR_INC);
    SET_BIT(mVc3DsAdcDmaHandle->CTRL,DMA_CTRL_DMA_CONTINUE_ENABLE);    // DMA continuous mode enabled.
    DMA_InstallIsrService(mVc3DsAdcDmaHandle, AEC_DmaIsrService);
    DMA_EnableInt(mVc3DsAdcDmaHandle);
    NVIC_EnableIRQ(DMA2_IRQn);
    AEC_GetAdc_DmaTrigger();
  #else
    DS_ADC->MUTE_CTRL0	&= ~DS_ADC_MUTE_CTRL0_ECHO_CANCEL_ENABLE;

    DMA_Init(mVc3DsAdcDmaHandle, DMA_REQSEL_DSADC, DMA_SRC_DATA_16B, DMA_SRC_ADDR_FIX, DMA_DST_DATA_16B, DMA_DST_ADDR_INC);
    DMA_InstallIsrService(mVc3DsAdcDmaHandle, SACM_VC3_GetAdc_DmaIsrService);
    DMA_EnableInt(mVc3DsAdcDmaHandle);
    NVIC_EnableIRQ(DMA2_IRQn);
    DMA_Trigger(mVc3DsAdcDmaHandle, (uint32_t)&DS_ADC->DATA, (uint32_t)Vc3ApiWorkingRam->pAdcInBuf, VC3_FRAME_SIZE);
  #endif

	//delay_ms(1);	// to guarantee DAC after ADC.
	while(DAC_ReadyFlag < 10);    // Wait until ADC ready & stable to avoid "pop" noise by Kim
	if((SACM_VC3_GetStatus() & VC3_ENABLE_DAC_CH0_FLAG) != 0)
	{
    CLEAR_BIT(DAC->CTRL, DAC_CTRL_DAC_CH0_DMA_ENABLE);                // DAC_Ch0 DMA request disabled.
	  DMA_Init(mVc3DacCh0DmaHandle, DMA_REQSEL_DAC_CH0, DMA_SRC_DATA_16B, DMA_SRC_ADDR_INC, DMA_DST_DATA_16B, DMA_DST_ADDR_FIX);
	  SET_BIT(mVc3DacCh0DmaHandle->CTRL,DMA_CTRL_DMA_CONTINUE_ENABLE);    // DMA continuous mode enabled.
	  DMA_InstallIsrService(mVc3DacCh0DmaHandle, SACM_VC3_SendDac_DmaIsrService);
	  DMA_EnableInt(mVc3DacCh0DmaHandle);
	  NVIC_EnableIRQ(DMA3_IRQn);
	  DMA_Trigger(mVc3DacCh0DmaHandle, (uint32_t)Vc3ApiWorkingRam->pVc3OutBuf, (uint32_t)&DAC->DAC_CH0_DMA_DATA0, VC3_FRAME_SIZE);
	}

	if((SACM_VC3_GetStatus() & VC3_ENABLE_DAC_CH1_FLAG) != 0)
	{
    CLEAR_BIT(DAC->CTRL, DAC_CTRL_DAC_CH1_DMA_ENABLE);                // DAC_Ch1 DMA request disabled.
		DMA_Init(mVc3DacCh1DmaHandle, DMA_REQSEL_DAC_CH1, DMA_SRC_DATA_16B, DMA_SRC_ADDR_INC, DMA_DST_DATA_16B, DMA_DST_ADDR_FIX);
		SET_BIT(mVc3DacCh1DmaHandle->CTRL,DMA_CTRL_DMA_CONTINUE_ENABLE);    // DMA continuous mode enabled.
    if((SACM_VC3_GetStatus() & VC3_ENABLE_DAC_CH0_FLAG) == 0)
		{
	    DMA_InstallIsrService(mVc3DacCh1DmaHandle, SACM_VC3_SendDac_DmaIsrService);
	    DMA_EnableInt(mVc3DacCh1DmaHandle);
	    NVIC_EnableIRQ(DMA4_IRQn);
		}
		DMA_Trigger(mVc3DacCh1DmaHandle, (uint32_t)Vc3ApiWorkingRam->pVc3OutBuf, (uint32_t)&DAC->DAC_CH1_DMA_DATA0, VC3_FRAME_SIZE);
	}
}

/**
 * @brief
 *   Callback function.
 * @param
 *  *Vc3ApiWorkingRam [in]: VC3 API working RAM address.
 * @return
 *   None.
 */
void VC3_CB_StopPlay(const SACM_VC3_API_WORKING_RAM *Vc3ApiWorkingRam)
{
	if((SACM_VC3_GetStatus() & VC3_PLAY_FLAG) != 0)
  {
	  if((SACM_VC3_GetStatus() & VC3_ENABLE_DAC_CH0_FLAG) != 0)
	  {
		  DMA_Close(mVc3DacCh0DmaHandle);
		}

	  if((SACM_VC3_GetStatus() & VC3_ENABLE_DAC_CH1_FLAG) != 0)
	  {
		  DMA_Close(mVc3DacCh1DmaHandle);
		}

		DacAutoRampDown(SACM_VC3_GetStatus());	               // DAC auto ramp down
		DAC_AudioPwm_IP_Disable();
  }
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When playing, this function is called by VC3 Library to trigger DS-ADC DMA.
 * @param
 *  *Vc3ApiWorkingRam [in]: VC3 API working RAM address.
 *  *AdcInBufAddr [out]: ADC input buffer address pointer.
 *   DataLen [in]: Data length (halfword)
 * @return
 *   None.
 */
void VC3_CB_GetAdc_DmaIsr(const SACM_VC3_API_WORKING_RAM *Vc3ApiWorkingRam, int16_t *AdcInBufAddr, uint16_t DataLen)
{
  DAC_ReadyFlag += 1;
	#if !AEC_ON
	DMA_Trigger(mVc3DsAdcDmaHandle, (uint32_t)&DS_ADC->DATA, (uint32_t)AdcInBufAddr, DataLen);
	#endif
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When playing, this function is called by VC3 Library to trigger DAC DMA.
 * @param
 *  *Vc3ApiWorkingRam [in]: VC3 API working RAM address.
 *  *DacOutBufAddr [in]: DAC Output Buffer address pointer.
 *   DataLen [in]: Data length (halfword)
 * @return
 *   None.
 */
void VC3_CB_SendDac_DmaIsr(const SACM_VC3_API_WORKING_RAM *Vc3ApiWorkingRam, int16_t *DacOutBufAddr, uint16_t DataLen)
{
	if((SACM_VC3_GetStatus() & VC3_ENABLE_DAC_CH0_FLAG) != 0)
	{
		while(DMA_CheckBusy(mVc3DacCh0DmaHandle) != 0);        // Check DMAx busy
		DMA_Trigger(mVc3DacCh0DmaHandle, (uint32_t)DacOutBufAddr, (uint32_t)&DAC->DAC_CH0_DMA_DATA0, DataLen);
	}

	if((SACM_VC3_GetStatus() & VC3_ENABLE_DAC_CH1_FLAG) != 0)
	{
		while(DMA_CheckBusy(mVc3DacCh1DmaHandle) != 0);        // Check DMAx busy
    DMA_Trigger(mVc3DacCh1DmaHandle, (uint32_t)DacOutBufAddr, (uint32_t)&DAC->DAC_CH1_DMA_DATA0, DataLen);
	}
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When playing, this function is called by VC3 library to encode data.
 * @param
 *  *Vc3ApiWorkingRam [in]: VC3 API working RAM address.
 *  *DstBufAddr [out]: Destination buffer address pointer.
 *  *SrcBufAddr [in]: Source buffer address pointer.
 * @return
 *   None.
 */
void VC3_CB_VoiceProcess(const SACM_VC3_API_WORKING_RAM *Vc3ApiWorkingRam, int16_t *SrcBufAddr, int16_t *DstBufAddr, uint8_t Vc3Mode)
{
	uint32_t iCount;

	switch(Vc3Mode)
	{
		case VC3_SHIFT_PITCH_MODE:
      GPIOA_OBIT->OBIT08 = 0xFFFFFFFF;
			SACM_VC3_ShiftPitchProcess(SrcBufAddr, DstBufAddr);
      GPIOA_OBIT->OBIT08 = 0;
			break;

		case VC3_CONST_PITCH_MODE:
			SACM_VC3_ConstPitchProcess(SrcBufAddr, DstBufAddr);
			break;

		case VC3_ECHO_MODE:
			SACM_VC3_EchoProcess(SrcBufAddr, DstBufAddr);
			break;

		case VC3_VM_ROBOT_EFFECT1_MODE:
      SACM_VC3_RobotEffect1(SrcBufAddr, DstBufAddr, VC3_FRAME_SIZE);
			break;

		case VC3_VM_ROBOT_EFFECT2_MODE:
			SACM_VC3_RobotEffect2(SrcBufAddr, DstBufAddr, VC3_FRAME_SIZE);
			break;

		case VC3_VM_STRANGE_TONE_MODE:
			SACM_VC3_StrangeTone(SrcBufAddr, DstBufAddr, VC3_FRAME_SIZE);
			break;

		case VC3_VM_DJ_EFFECT_MODE:
			SACM_VC3_DJEffect(SrcBufAddr, DstBufAddr, VC3_FRAME_SIZE);
			break;

		case VC3_VM_VIBRATION_MODE:
			SACM_VC3_Vibration(SrcBufAddr, DstBufAddr, VC3_FRAME_SIZE);
			break;

		case VC3_VM_JET_PLANE_MODE:
			SACM_VC3_JetPlaneEffect(SrcBufAddr, DstBufAddr, VC3_FRAME_SIZE);
			break;

		case VC3_VM_DOUBLING_MODE:
			SACM_VC3_Doubling(SrcBufAddr, DstBufAddr, VC3_FRAME_SIZE);
			break;

		case VC3_VM_UNDERWATER_MODE:
			SACM_VC3_UnderWater(SrcBufAddr, DstBufAddr, VC3_FRAME_SIZE);
			break;

		case VC3_USER_DEFINE_MODE1:
	    for(iCount = 0; iCount < VC3_FRAME_SIZE; iCount++)
      {
        DstBufAddr[iCount] = SrcBufAddr[iCount];
	    }
			break;

		case VC3_USER_DEFINE_MODE2:
		  SACM_VC3_ShiftPitchProcess(SrcBufAddr, SrcBufAddr);
		  SACM_VC3_EchoProcess(SrcBufAddr, DstBufAddr);
	    break;
	}

	APP_SwVolCtrl_VolProcess(0, DstBufAddr, VC3_FRAME_SIZE);
}


/*---------------------------------------------------------------------------------------
 * AEC Function Area
 *---------------------------------------------------------------------------------------*/
/**
 * @brief
 *   Callback function. Implemented by user.
 *   When CSP16 working, this function is called by AEC Library to trigger DS-ADC DMA.
 * @param
 *	 None.
 * @return
 *   None.
 * @note
 *   main() & AEC_DmaIsrService() call this function.
 */
void AEC_GetAdc_DmaTrigger(void)
{
	#if AEC_ON
		DMA_Trigger(mVc3DsAdcDmaHandle, (uint32_t)&DS_ADC->DATA, (uint32_t)mAecAdcRam->AecPcmBufPtr, AEC_FRAME_SIZE);
	#endif
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   2.5ms: 40/16000
 */
void AEC_DmaIsrService(void)
{
	/*
	 * change A/B buffer
	 */
  {
	  if((mAecAdcRam->AEC_CQBuf_WtPtr & 0x01) == 0)
		{
			mAecAdcRam->AecPcmBufPtr = mAecAdcRam->AecPcmBuf[1];
			mAecAdcRam->AecInBufPtr  = mAecAdcRam->AecPcmBuf[0];
	  }
		else
		{
			mAecAdcRam->AecPcmBufPtr = mAecAdcRam->AecPcmBuf[0];
			mAecAdcRam->AecInBufPtr  = mAecAdcRam->AecPcmBuf[1];
	  }
	}
	AEC_GetAdc_DmaTrigger();

	NVIC_SetPendingIRQ(EXTI_IRQn);
	NVIC_SetPriority(EXTI_IRQn, 1);
}

/*
 * @brief
 *
 * @param
 *          None.
 * @return
 *          None.
 */
void EXT_INT_Initial(void)
{
	NVIC_EnableIRQ(EXTI_IRQn);														//	Enable NVIC for EXTI0:
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   mVc31800Status
 */
void F_Copy_DS_ADC_Memory(int32_t	*StereoADC, int16_t	*monoADC,	int32_t Length)
{
	int32_t	temp32 = 0;
	do
	{
		temp32		 = 		*StereoADC++;
		*monoADC++ =	  temp32&0x0000FFFF;
		Length--;
	}
	while(Length!=0);
}

/*
 * @brief
 *
 * @param
 *          None.
 * @return
 *          None.
 */
void EXTI_IRQHandler(void)
{
	volatile int32_t	WrPtr = mAecAdcRam->AEC_CQBuf_WtPtr;
	NVIC_ClearPendingIRQ(EXTI_IRQn);

	if(R_AEC_WorkFlag == 1)
	{
		GPIOA_OBIT->OBIT16 = 0xFFFFFFFF;
		// AEC Process
		while( (MAC->CTRL&MAC_CTRL_START)!=0 )
		{
			WDT_Clear();
		}
		SET_BIT(CLOCK->AHBCKEN, CLOCK_AHBCKEN_MAC_CLK_ENABLE);
		SET_BIT(MAC->CTRL, MAC_CTRL_RESET);
		F_AEC_FrameProc(&AecWorkingRam, mAecAdcRam->AecInBufPtr, &mVc3ApiWorkingRamPtr->pAdcInBuf[WrPtr*AEC_FRAME_SIZE]);
		GPIOA_OBIT->OBIT16 = 0;
	}
	else
	{
		GPIOA_OBIT->OBIT16 ^= 0xFFFFFFFF;
		F_Copy_DS_ADC_Memory(mAecAdcRam->AecInBufPtr, &mVc3ApiWorkingRamPtr->pAdcInBuf[WrPtr*AEC_FRAME_SIZE], AEC_FRAME_SIZE);
	}

	WrPtr++;
	if(WrPtr>=CQBuf_size)
	{
		WrPtr-= CQBuf_size;
		SACM_VC3_GetAdc_DmaIsrService();															// set work flag
		mVc3ApiWorkingRamPtr->pVc3InBuf = mVc3ApiWorkingRamPtr->pAdcInBuf;
	}
	mAecAdcRam->AEC_CQBuf_WtPtr = WrPtr;
}


