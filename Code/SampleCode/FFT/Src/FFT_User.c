/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   FFT_User.c
 * @Version:
 *   V0.9.0
 * @Date:
 *   February 18, 2025
 * @Abstract:
 *
 **************************************************************************************************/


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "GPCM3_FM1.h"
#include "FFT_User.h"
#include "stdio.h"
/*---------------------------------------------------------------------------------------
 * Gobal Variable Declaration Area
 *---------------------------------------------------------------------------------------*/
DMA_TYPE_DEF *mDsAdcDmaHandle;

#define Aligned4B __attribute__ ((aligned (4)))
#if FFT_FRAME_SIZE == 128
	Aligned4B int16_t 	Magnitude_Exp=1; 				// 2^1=2;
	Aligned4B uint16_t 	Magnitude[64];
#elif	FFT_FRAME_SIZE == 256
	Aligned4B int16_t 	Magnitude_Exp=2; 				// 2^2=4;
	Aligned4B uint16_t 	Magnitude[128];
#elif FFT_FRAME_SIZE == 512
	Aligned4B int16_t 	Magnitude_Exp=3; 				// 2^3=8;
	Aligned4B uint16_t 	Magnitude[256];
#endif

/*---------------------------------------------------------------------------------------
 * Subroutine Area
 *---------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
 * Callback Function Area
 *---------------------------------------------------------------------------------------*/

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When user call API function FFT_Initial, this function is called by FFT Library.
 * @param
 *  *CspWorkingRam [in]: CSP working RAM pointer
 * @return
 *	 None
 */
void FFT_CB_Init(FFT_WORKING_RAM *FftWorkingRam)
{
	/*
	 * DS-ADC init
	 */
	DSADC_Init(16000);
	DSADC_SetBoostGain(3);
	DSADC_SetPgaGain(20);
	DSADC_AutoMuteInit(0x0000);

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
	DMA_InstallIsrService(mDsAdcDmaHandle, FFT_DmaIsrService);
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
void FFT_CB_Start(int16_t *DstPcmBufAddr, uint16_t DataLen)
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
void FFT_CB_Stop()
{
	DMA_Close(mDsAdcDmaHandle);
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When FFT working, this function is called by FFT Library to trigger DS-ADC DMA.
 * @param
 *  *DstPcmBufAddr [in]: Destination PCM buffer address pointer.
 *   DataLen [in]: Data length (halfword)
 * @return
 *   None.
 */
void FFT_CB_GetAdc_DmaIsr(int16_t *DstPcmBufAddr, uint16_t DataLen)
{
  DMA_Trigger(mDsAdcDmaHandle, (uint32_t)&DS_ADC->DATA, (uint32_t)DstPcmBufAddr, DataLen);
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When playing, this function is called by FFT library to process data.
 * @param
 *  *FFT_WORKING_RAM [in]: FFT API working RAM address.
 *  *SrcBufAddr [in]: Source buffer address pointer.
 *   DataLen [in]: Data length (uint16_t)
 * @return
 *   None.
 */
void FFT_CB_VoiceProcess(const FFT_WORKING_RAM *FftWorkingRam, int16_t *SrcBufAddr, uint16_t DataLen)
{
	uint32_t iCount;
	uint16_t Max, MaxIndex;

	GPIOA_OBIT->OBIT08 = 0xFFFFFFFF;

	#if FFT_FRAME_SIZE == 128
		GPLib_RFFT128(SrcBufAddr, Magnitude, Magnitude_Exp);		// spectrum value = magnitude(OUT) * 2^Magnitude_Exp(IN)
	#elif FFT_FRAME_SIZE == 256
		GPLib_RFFT256(SrcBufAddr, Magnitude, Magnitude_Exp);		// spectrum value = magnitude(OUT) * 2^Magnitude_Exp(IN)
	#elif FFT_FRAME_SIZE == 512
		GPLib_RFFT512(SrcBufAddr, Magnitude, Magnitude_Exp);		// spectrum value = magnitude(OUT) * 2^Magnitude_Exp(IN)
	#endif
	GPIOA_OBIT->OBIT08 = 0;

	Max = 0;
	MaxIndex = 0;
	for(iCount = 2;iCount<(DataLen>>1);iCount++)	// 31.25Hz/bin, don't care 60Hz below.
	{
		if(Max < Magnitude[iCount])
		{
			Max = Magnitude[iCount];
			MaxIndex = iCount;
		}
	}
	printf("Index = %d Val=%d\r\n", MaxIndex, Max);
}
