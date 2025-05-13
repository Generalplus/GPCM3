/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *
 * @Version:
 *    V0.9
 * @Date:
 *    12th, February 2025
 * @Abstract:
 *
 **************************************************************************************************/

/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "GPCM3_FM1.h"
#include "EnvDet_User.h"

/*---------------------------------------------------------------------------------------
 * Definition Area
 *---------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
 * Gobal Variable Declaration Area
 *---------------------------------------------------------------------------------------*/
DMA_TYPE_DEF *mEnvDetDsAdcDmaHandle;

ENVDET_ADC_RAM   EnvDetAdcRam;
ENVDET_ADC_RAM *mEnvDetAdcRam;


/*---------------------------------------------------------------------------------------
 * Callback Function Area
 *---------------------------------------------------------------------------------------*/
/**
 * @brief
 *   Callback function. Implemented by user.
 *   When user call API function EnvDet_Initial, this function is Called by EnvDet Library.
 * @param
 *  *EnvDetWorkingRam [out]: EnvDet library working RAM address.
 * @return
 *   None.
 */
void EnvDet_CB_Init(const ENVDET_WORKING_RAM *EnvDetWorkingRam)
{
	mEnvDetDsAdcDmaHandle = DMA4;

  DSADC_Init(8000);
	DSADC_SetBoostGain(3);
	DSADC_SetPgaGain(11);

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

	mEnvDetAdcRam = &EnvDetAdcRam;
	mEnvDetAdcRam->EnvDetPcmBufPtr = mEnvDetAdcRam->EnvDetPcmBuf[0];
	mEnvDetAdcRam->EnvDetInBufPtr  = mEnvDetAdcRam->EnvDetPcmBuf[1];
	mEnvDetAdcRam->EnvDet_WtPtr = 0;

	DMA_Init(mEnvDetDsAdcDmaHandle, DMA_REQSEL_DSADC, DMA_SRC_DATA_16B, DMA_SRC_ADDR_FIX, DMA_DST_DATA_16B, DMA_DST_ADDR_INC);
	DMA_InstallIsrService(mEnvDetDsAdcDmaHandle, EnvDet_DmaIsrService);
	DMA_EnableInt(mEnvDetDsAdcDmaHandle);
	NVIC_EnableIRQ(DMA4_IRQn);
	DMA_Trigger(mEnvDetDsAdcDmaHandle, (uint32_t)&DS_ADC->DATA, (uint32_t)mEnvDetAdcRam->EnvDetPcmBufPtr, ENVDET_FRAME_SIZE);
}

/**
 * @brief
 *
 * @param
 *  *EnvDetWorkingRam [out]: EnvDet library working RAM address.
 * @return
 *   None.
 */
void EnvDet_CB_Stop(const ENVDET_WORKING_RAM *EnvDetWorkingRam)
{
	DMA_Close(mEnvDetDsAdcDmaHandle);
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   Signed ADC Data
 */
int16_t EnvDet_CB_GetAdc(void)
{
	return 0;
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void EnvDet_DmaIsrService(void)
{
	//GPIOA_OBIT->OBIT16 = 0xFFFFFFFF;
	if((mEnvDetAdcRam->EnvDet_WtPtr & ENVDET_BUF_EVEN_FLAG) == 0)
	{
		mEnvDetAdcRam->EnvDetPcmBufPtr = mEnvDetAdcRam->EnvDetPcmBuf[1];
		mEnvDetAdcRam->EnvDetInBufPtr  = mEnvDetAdcRam->EnvDetPcmBuf[0];
	}
	else
	{
		mEnvDetAdcRam->EnvDetPcmBufPtr = mEnvDetAdcRam->EnvDetPcmBuf[0];
		mEnvDetAdcRam->EnvDetInBufPtr  = mEnvDetAdcRam->EnvDetPcmBuf[1];
	}

	DMA_Trigger(mEnvDetDsAdcDmaHandle, (uint32_t)&DS_ADC->DATA, (uint32_t)mEnvDetAdcRam->EnvDetPcmBufPtr, ENVDET_FRAME_SIZE);

	EnvDet_ServiceLoop(mEnvDetAdcRam->EnvDetInBufPtr, ENVDET_FRAME_SIZE);

	mEnvDetAdcRam->EnvDet_WtPtr ^= ENVDET_BUF_EVEN_FLAG;
	//GPIOA_OBIT->OBIT16 = 0;
}
