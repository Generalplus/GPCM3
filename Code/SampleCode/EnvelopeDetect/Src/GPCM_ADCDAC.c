/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 **************************************************************************************************/

/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "GPCM3_FM1.h"


/*---------------------------------------------------------------------------------------
 * Constant Area
 *---------------------------------------------------------------------------------------*/
#define SACM_SamplingRate			16000
#define SACM_FrameSize				160
#define D_DUAL_MODE           1


/*---------------------------------------------------------------------------------------
 * Variable Area
 *---------------------------------------------------------------------------------------*/
TIMER_TYPE_DEF 	*mDacTimerHandle;
DMA_TYPE_DEF 		*mDsAdcDmaHandle;
DMA_TYPE_DEF 		*mDacDmaHandle;

#if defined(__CC_ARM)
#define Aligned4B __align(4)
#elif defined(__GNUC__)
#define Aligned4B __attribute__ ((aligned (4)))
#endif

Aligned4B int16_t	R_AdcDmaBuffer0[SACM_FrameSize*2];  //ADC Dual Buffer
Aligned4B int16_t	R_DacDmaBuffer0[SACM_FrameSize*2];  //DAC Dual Buffer

int16_t	*DMA_AdcRxAddr;	                              // hardware DMA handle address
int16_t	*DMA_DacTxAddr;	                              // hardware DMA handle address
int16_t	*AdcIn_Frame_Addr;	                          // software FrameWork Buffer Address
int16_t	*DacOut_Frame_Addr;	                          // software FrameWork Buffer Address

volatile int16_t Flag_FrameWork;	 		                // software FrameWork : check flag

/*---------------------------------------------------------------------------------------
 * SACM Framework Function Area
 *---------------------------------------------------------------------------------------*/

/**
 * @brief
 *   None.
 * @param
 *   None.
 * @return
 *   None.
 */
void fn_SendDac_DmaIsrService()
{
  // GPIOA_OBIT->OBIT09 ^= 0xFFFFFFFF;

  if(DMA_DacTxAddr==R_DacDmaBuffer0)
  {
    DacOut_Frame_Addr=R_DacDmaBuffer0;
    DMA_DacTxAddr = R_DacDmaBuffer0+SACM_FrameSize;
  }
  else
  {
    DacOut_Frame_Addr=R_DacDmaBuffer0+SACM_FrameSize;
    DMA_DacTxAddr = R_DacDmaBuffer0;
  }

#if D_DUAL_MODE
#else
  while(DMA_CheckBusy(mDacDmaHandle) != 0);        // Check DMAx busy
  DMA_Trigger(mDacDmaHandle, (uint32_t)DMA_DacTxAddr, (uint32_t)&DAC->DAC_CH0_DMA_DATA0, SACM_FrameSize);
#endif

  Flag_FrameWork |= 0x0100;
}

/**
 * @brief
 *   None.
 * @param
 *   None.
 * @return
 *   None.
 */
void fn_GetAdc_DmaIsrService()
{
  // GPIOA_OBIT->OBIT08 ^= 0xFFFFFFFF;

  if(DMA_AdcRxAddr==R_AdcDmaBuffer0)
  {
    AdcIn_Frame_Addr=R_AdcDmaBuffer0;
    DMA_AdcRxAddr = R_AdcDmaBuffer0+SACM_FrameSize;
  }
  else
  {
    AdcIn_Frame_Addr=R_AdcDmaBuffer0+SACM_FrameSize;
    DMA_AdcRxAddr = R_AdcDmaBuffer0;
  }

#if D_DUAL_MODE
#else
  while(DMA_CheckBusy(mDsAdcDmaHandle) != 0);        // Check DMAx busy
	DMA_Trigger(mDsAdcDmaHandle, (uint32_t)&DS_ADC->DATA, (uint32_t) DMA_AdcRxAddr, SACM_FrameSize);
#endif

	Flag_FrameWork |= 0x0080;
}

/**
 * @brief
 *   None.
 * @param
 *   None.
 * @return
 *   None.
 */void F_Delay(int32_t	CNT)
{
	int32_t	x;

	do
	{
    x+=CNT;
    x=x*x;
    CNT--;
	}
	while(CNT!=0);
}

/**
 * @brief
 *   None.
 * @param
 *   None.
 * @return
 *   None.
 */
void F_SACM_AdcDac_Init(void)
{
	mDacTimerHandle  = TM0;
	mDsAdcDmaHandle  = DMA1;
	mDacDmaHandle 	 = DMA2;

	Flag_FrameWork = 0;

  TIMER_Open(mDacTimerHandle);
	TIMER_SetFreq(mDacTimerHandle, SACM_SamplingRate*4);	        // DAC H/W : 4x upsampling

	DAC_Open();											                              // ENABLE DAC_Reg
	DAC_Scale_Enable();
	DAC_CH0_Init(DAC_TRG_SEL_TM0);	                              // DAC_CH0 trigger = Timer0
	DAC_CH1_Init(DAC_TRG_SEL_TM0);
  SET_BIT(DAC->CTRL, DAC_CTRL_CH0_POSTWAVE_OUT_CH0 + DAC_CTRL_CH1_POSTWAVE_OUT_CH1);

	DAC_AudioPwm_Open();
	DAC_SetAudPwmGain(30);
	DAC_AudioPwm_IP_Enable();

	DSADC_Init(SACM_SamplingRate);
	DSADC_SetBoostGain(2);
	DSADC_SetPgaGain(22);
	// DSADC_AutoMuteInit(0x180);			                              // disable DS-ADC AutoMute function

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

  //================================================================//
  DMA_Init(mDsAdcDmaHandle, DMA_REQSEL_DSADC, DMA_SRC_DATA_16B, DMA_SRC_ADDR_FIX, DMA_DST_DATA_16B, DMA_DST_ADDR_INC);
#if D_DUAL_MODE
SET_BIT(mDsAdcDmaHandle->CTRL,DMA_CTRL_DMA_DUAL_ENABLE);      // DMA Dual Buffer Mode Enabled.
SET_BIT(mDsAdcDmaHandle->CTRL,DMA_CTRL_DMA_CIRC_ENABLE);      // DMA circular mode enabled.
#else
CLEAR_BIT(mDsAdcDmaHandle->CTRL, DMA_CTRL_DMA_DUAL_ENABLE);      // DMA Dual Buffer Mode Enabled.
CLEAR_BIT(mDsAdcDmaHandle->CTRL, DMA_CTRL_DMA_CIRC_ENABLE);      // DMA circular mode enabled.
#endif
  DMA_InstallIsrService(mDsAdcDmaHandle, fn_GetAdc_DmaIsrService);
  DMA_EnableInt(mDsAdcDmaHandle);
  NVIC_EnableIRQ(DMA1_IRQn);

	DMA_AdcRxAddr = R_AdcDmaBuffer0;
	DMA_Trigger(mDsAdcDmaHandle, (uint32_t)&DS_ADC->DATA, (uint32_t) DMA_AdcRxAddr, SACM_FrameSize);			 // trigger DMA :Start

  //================================================================//
  DMA_Init(mDacDmaHandle, DMA_REQSEL_DAC_CH0, DMA_SRC_DATA_16B, DMA_SRC_ADDR_INC, DMA_DST_DATA_16B, DMA_DST_ADDR_FIX);
#if D_DUAL_MODE
SET_BIT(DAC->CTRL,0x60000);                        // DAC Ch0 & Ch1 DMA continue mode enabled
SET_BIT(mDacDmaHandle->CTRL,DMA_CTRL_DMA_DUAL_ENABLE);      // DMA Dual Buffer Mode Enabled.
SET_BIT(mDacDmaHandle->CTRL,DMA_CTRL_DMA_CIRC_ENABLE);      // DMA circular mode enabled.
#endif
  DMA_InstallIsrService(mDacDmaHandle, fn_SendDac_DmaIsrService);
  DMA_EnableInt(mDacDmaHandle);
  NVIC_EnableIRQ(DMA2_IRQn);

	DMA_DacTxAddr= R_DacDmaBuffer0;
	DMA_Trigger(mDacDmaHandle, (uint32_t)DMA_DacTxAddr, (uint32_t)&DAC->DAC_CH0_DMA_DATA0, SACM_FrameSize); // trigger DMA :Start
}

/**
 * @brief
 *   None.
 * @param
 *   None.
 * @return
 *   None.
 */
void SACM_AdcDac_ServiceLoop(void)
{
  uint32_t iCount;

	if (Flag_FrameWork == 0x0180)
	{
    for(iCount=0;iCount<SACM_FrameSize;iCount++)
    {
      DacOut_Frame_Addr[iCount] = AdcIn_Frame_Addr[iCount];
    }
    Flag_FrameWork=0;
	}
}
