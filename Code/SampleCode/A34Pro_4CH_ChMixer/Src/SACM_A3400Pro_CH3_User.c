/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   SACM_A3400Pro_CH3_User.c
 * @Version:
 *   V1.0.4
 * @Date:
 *   January 13th, 2024
 * @Abstract:
 *
 *************************************************************************************************/

/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "GPCM3_FM1.h"
#include "SACM_A3400Pro_User.h"
#include "SACM_A3400Pro_Ch3_User.h"
#include "A34_FileMerger.h"
#include "APP_SwVolumeControl.h"


/*---------------------------------------------------------------------------------------
 * Gobal Variable Declaration Area
 *---------------------------------------------------------------------------------------*/
#if defined(__CC_ARM)
#define Aligned4B __align(4)
#elif defined(__GNUC__)
#define Aligned4B __attribute__ ((aligned (4)))
#endif

TIMER_TYPE_DEF *mA3400ProCh3DacTimerHandle;
DMA_TYPE_DEF *mA3400ProCh3DacCh0DmaHandle;
DMA_TYPE_DEF *mA3400ProCh3DacCh1DmaHandle;


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
uint16_t GetA3400ProCh3Num()
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
int16_t* GetA3400ProCh3StartAddr(uint16_t SpeechIdx)
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
void A3400Pro_CH3_CB_Init(const SACM_A3400PRO_WORKING_RAM *A3400ProWorkingRam)
{
	mA3400ProCh3DacCh0DmaHandle = DMA3;
	mA3400ProCh3DacCh1DmaHandle = DMA4;
	mA3400ProCh3DacTimerHandle = TM0;

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
void A3400Pro_CH3_CB_StartPlay(const SACM_A3400PRO_WORKING_RAM *A3400ProCh3WorkingRam)
{
}

/**
 * @brief
 *   Callback function. To be implemented by user.
 *   When user call API function SACM_A3400Pro_Play_Con, this function is called by A3400Pro_CH3 Library.
 * @param
 *  *A3400ProCh3WorkingRam [in]: A3400Pro_CH3 library working RAM address.
 * @return
 *   None.
 */
void A3400Pro_CH3_CB_StartPlay_Con(const SACM_A3400PRO_WORKING_RAM *A3400ProCh3WorkingRam)
{

}

/**
 * @brief
 *
 * @param
 *  *A3400ProWorkingRam [in]: A3400Pro library working RAM address.
 * @return
 *   None.
 */
void A3400Pro_CH3_CB_StopPlay(const SACM_A3400PRO_WORKING_RAM *A3400ProCh3WorkingRam)
{

}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When user call API function SACM_A3400Pro_CH3_Pause, this function is called by A3400Pro Library.
 * @param
 *  *A3400ProWorkingRam [in]: A3400Pro library working RAM address.
 * @return
 *   None.
 */
void A3400Pro_CH3_CB_Pause(const SACM_A3400PRO_WORKING_RAM *A3400ProCh3WorkingRam)
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
void A3400Pro_CH3_CB_Resume(const SACM_A3400PRO_WORKING_RAM *A3400ProCh3WorkingRam)
{
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When playing, this function is called by A3400Pro Library to get encoded data.
 * @param
 *  *A3400ProWorkingRam [in]: A3400Pro library working RAM address.
 *  *SrcDataAddr [in]: Source data address pointer.
 *  *DstBufAddr [out]: Destination buffer address pointer.
 *   DataLen [in]: Data length (halfword)
 * @return
 *   None.
 */
void A3400Pro_CH3_CB_GetData(const SACM_A3400PRO_WORKING_RAM *A3400ProCh3WorkingRam, uint8_t *DstBufAddr, uint8_t *SrcDataAddr, uint16_t DataLen)
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
 *   When playing, this function is called by A3400Pro Library to trigger DAC DMA.
 * @param
 *  *A3400ProWorkingRam [in]: A3400Pro working RAM pointer
 *  *SrcBufAddr [in]: Source buffer address pointer.
 *   DataLen [in]: Data length (halfword)
 * @return
 *   None.
 */
void A3400Pro_CH3_CB_SendDac_DmaIsr(const SACM_A3400PRO_WORKING_RAM *A3400ProCh3WorkingRam, int16_t *SrcBufAddr, uint16_t DataLen)
{
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
void A3400Pro_CH3_CB_DecodeProcess(const SACM_A3400PRO_WORKING_RAM *A3400ProCh3WorkingRam, int16_t *DstBufAddr, int16_t *SrcBufAddr)
{
  SACM_A3400Pro_CH3_DecodeProcess(DstBufAddr, SrcBufAddr);
}
