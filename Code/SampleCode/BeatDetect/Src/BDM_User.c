/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   BDM_User.c for GPCM3
 * @Version:
 *   V0.9.1
 * @Date:
 *   April 23, 2024
 * @Abstract:
 *
 **************************************************************************************************/


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "GPCM3_FM1.h"
#include "BDM_User.h"


/*---------------------------------------------------------------------------------------
 * Gobal Variable Declaration Area
 *---------------------------------------------------------------------------------------*/
DMA_TYPE_DEF *mDsAdcDmaHandle;


/*---------------------------------------------------------------------------------------
 * Callback Function Area
 *---------------------------------------------------------------------------------------*/
/**
 * @brief
 *   Callback function. Implemented by user.
 *   When user call API function BeatDetMic_Init, this function is called by BeatDetMic Library.
 * @param
 *  *BeatDetMicWorkingRam [in]: BeatDetMic working RAM pointer
 * @return
 *
 */
void BDM_CB_Init(void)
{
	// PWMIO_Open();

	/*
	 * DS-ADC init
	 */
	DSADC_Init(SamplingRate);
	DSADC_SetBoostGain(2);
	DSADC_SetPgaGain(22);
	// DSADC_AutoMuteInit(0x180);			// disable DS-ADC AutoMute function

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

	mDsAdcDmaHandle = DMA4;
	DMA_Init(mDsAdcDmaHandle, DMA_REQSEL_DSADC, DMA_SRC_DATA_16B, DMA_SRC_ADDR_FIX, DMA_DST_DATA_16B, DMA_DST_ADDR_INC);
  SET_BIT(mDsAdcDmaHandle->CTRL,DMA_CTRL_DMA_CONTINUE_ENABLE);    // DMA continuous mode enabled.
	DMA_InstallIsrService(mDsAdcDmaHandle, BDM_DmaIsrService);
	DMA_EnableInt(mDsAdcDmaHandle);
	NVIC_EnableIRQ(DMA4_IRQn);

	return;
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When CSP16 working, this function is called by CSP16 Library to trigger DS-ADC DMA.
 * @param
 *  *DstPcmBufAddr [in]: Destination PCM buffer address pointer.
 *   DataLen [in]: Data length (halfword)
 * @return
 *   None.
 */
void BDM_CB_Start(int16_t *DstPcmBufAddr, uint16_t DataLen)
{
  DMA_Trigger(mDsAdcDmaHandle, (uint32_t)&DS_ADC->DATA, (uint32_t)DstPcmBufAddr, DataLen);
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void BDM_CB_Stop()
{
	DMA_Close(mDsAdcDmaHandle);
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When CSP16 working, this function is called by CSP16 Library to trigger DS-ADC DMA.
 * @param
 *  *DstPcmBufAddr [in]: Destination PCM buffer address pointer.
 *   DataLen [in]: Data length (halfword)
 * @return
 *   None.
 */
void BDM_CB_GetAdc_DmaIsr(int16_t *DstPcmBufAddr, uint16_t DataLen)
{
	DMA_Trigger(mDsAdcDmaHandle, (uint32_t)&DS_ADC->DATA, (uint32_t)DstPcmBufAddr, DataLen);
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
void BDM_CB_DecodeProcess(int16_t *SrcBufAddr)
{
	static uint16_t TriColor_Index;
	int16_t	Output, VAD;

	Output = BDM_DecodeProcess(SrcBufAddr);

	VAD	=	ReadVAD1();
	//---------------------------------//
	if (VAD)
	{
		GPIOA_OBIT->OBIT15 = 0xFFFFFFFF;
	}
	else
	{
		GPIOA_OBIT->OBIT15 = 0;
	}

	//---------------------------------//

	if (Output)
	{
		GPIOA_OBIT->OBIT14 = 0xFFFFFFFF;
	}
	else
	{
		GPIOA_OBIT->OBIT14 = 0;
	}
}





