/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   SACM_VC3_User.c
 * @Version:
 *   V1.0.0
 * @Date:
 *   June 11, 2020
 * @Abstract:
 *
 **************************************************************************************************/


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "GPCM3_FM1.h"
#include "SACM_VC3_User.h"
#include "APP_SwVolumeControl.h"


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
DMA_TYPE_DEF *mVc3DsAdcDmaHandle;
TIMER_TYPE_DEF *mVc3DacTimerHandle;
DMA_TYPE_DEF *mVc3DacCh0DmaHandle;
DMA_TYPE_DEF *mVc3DacCh1DmaHandle;


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
	// RampUpDacCh0();
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
	// RampUpDacCh1();
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
	mVc3DsAdcDmaHandle = DMA0;
	mVc3DacCh0DmaHandle = DMA1;
	mVc3DacCh1DmaHandle = DMA2;
	mVc3DacTimerHandle = TM0;

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
	// DMA_Trigger(mVc3DsAdcDmaHandle, (uint32_t)&DS_ADC->DATA, (uint32_t)AdcInBufAddr, DataLen);
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

}
