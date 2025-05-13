/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   SACM_MS02PN_User.c
 * @Version:
 *   V1.0.2
 * @Date:
 *   June 02, 2023
 * @Abstract:
 *
 **************************************************************************************************/


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "GPCM3_FM1.h"
#include "SACM_MS02PN_User.h"
#include "SACM_MS02PN_API.h"
#include "MS02_FileMerger.h"
#include "APP_SwVolumeControl.h"


/*---------------------------------------------------------------------------------------
 * Gobal Variable Declaration Area
 *---------------------------------------------------------------------------------------*/
TIMER_TYPE_DEF *mMS02PNDacTimerHandle;
DMA_TYPE_DEF   *mMS02PNDacCh0DmaHandle;
DMA_TYPE_DEF   *mMS02PNDacCh1DmaHandle;

uint8_t PN_AUD_OUT_TYPE;

/*---------------------------------------------------------------------------------------
 * User Subroutine Area
 *---------------------------------------------------------------------------------------*/

/**
 * @brief
 *   Get MS02 Library start address
 * @param
 *   LibIdx [in]: MIDI Lib index
 * @return
 *   None.
 */
int16_t* GetMS02PNLibStartAddr(uint16_t LibIdx)
{
	return (int16_t*) ((*((uint32_t *)(SEC_START_ADDR) + LibIdx)) + SEC_START_ADDR);
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
	  DMA_Init(mMS02PNDacCh0DmaHandle, DMA_REQSEL_DAC_CH0, DMA_SRC_DATA_16B, DMA_SRC_ADDR_INC, DMA_DST_DATA_16B, DMA_DST_ADDR_FIX);
	  SET_BIT(mMS02PNDacCh0DmaHandle->CTRL,DMA_CTRL_DMA_CONTINUE_ENABLE);    // DMA continuous mode enabled.
	  DMA_Trigger(mMS02PNDacCh0DmaHandle, (uint32_t)&DacDataBuf[0], (uint32_t)&DAC->DAC_CH0_DMA_DATA0, DataLen);
		while(DMA_CheckBusy(mMS02PNDacCh0DmaHandle) != 0);                // Wait DMAx busy
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
		DMA_Init(mMS02PNDacCh1DmaHandle, DMA_REQSEL_DAC_CH1, DMA_SRC_DATA_16B, DMA_SRC_ADDR_INC, DMA_DST_DATA_16B, DMA_DST_ADDR_FIX);
		SET_BIT(mMS02PNDacCh1DmaHandle->CTRL,DMA_CTRL_DMA_CONTINUE_ENABLE);    // DMA continuous mode enabled.
		DMA_Trigger(mMS02PNDacCh1DmaHandle, (uint32_t)&DacDataBuf[0], (uint32_t)&DAC->DAC_CH1_DMA_DATA0, DataLen);
		while(DMA_CheckBusy(mMS02PNDacCh1DmaHandle) != 0);                // Wait DMAx busy
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
 *   mMS02PNStatus [in]: MS02 status
 * @return
 *   None.
 */
void static DacAutoRampUp(uint16_t mMS02PNStatus)
{
  if((mMS02PNStatus & MS02_AUTO_RAMP_UP_ENABLE_FLAG) != 0)
	{
		if((mMS02PNStatus & MS02_ENABLE_DAC_CH0_FLAG) != 0)
		{
			RampUpDacCh0();
		}

		if((mMS02PNStatus & MS02_ENABLE_DAC_CH1_FLAG) != 0)
		{
			RampUpDacCh1();
		}
	}
}

/**
 * @brief
 *
 * @param
 *   mMS02PNStatus [in]: MS02 status
 * @return
 *   None.
 */
void static DacAutoRampDown(uint16_t mMS02PNStatus)
{
  if((mMS02PNStatus & MS02_AUTO_RAMP_DOWN_ENABLE_FLAG) != 0)
	{
		if((mMS02PNStatus & MS02_ENABLE_DAC_CH0_FLAG) != 0)
		{
			RampDownDacCh0();
		}

		if((mMS02PNStatus & MS02_ENABLE_DAC_CH1_FLAG) != 0)
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
 *   When user call API function SACM_MS02_Initial, this function is called by MS02 Library.
 * @param
 *  *MS02WorkingRam [in]: MS02 library working RAM address.
 *  AudOutType      [in]: AUD_OUT_DAC/ AUD_OUT_PWM
 *  DacChannelNo    [in]: MS02_CH0/ MS02_CH1
 *  Timer_SR        [in]:
 * @return
 *   None.
 */
void MS02PN_CB_Init(const SACM_MS02PN_WORKING_RAM *MS02PNWorkingRam, uint8_t AudOutType, uint8_t DacChannelNo, uint32_t Timer_SR)
{
	mMS02PNDacCh0DmaHandle = DMA3;
	mMS02PNDacCh1DmaHandle = DMA4;
	mMS02PNDacTimerHandle = TM1;
}

/**
 * @brief
 *   Callback function. To be implemented by user.
 *   When user call API function SACM_MS02_Play, this function is called by MS02 Library.
 * @param
 *  *MS02WorkingRam [in]: MS02 library working RAM address.
 * @return
 *   None.
 */
void MS02PN_CB_StartPlay(const SACM_MS02PN_WORKING_RAM *MS02PNWorkingRam)
{
}

/**
 * @brief
 *
 * @param
 *  *MS02WorkingRam [in]: MS02 library working RAM address.
 * @return
 *   None.
 */
uint8_t MS02PN_CB_StopPlay(const SACM_MS02PN_WORKING_RAM *MS02PNWorkingRam)
{
	return 0;
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When user call API function SACM_MS02_Pause, this function is called by MS02 Library.
 * @param
 *  *MS02WorkingRam [in]: MS02 library working RAM address.
 * @return
 *   None.
 */
void MS02PN_CB_Pause(const SACM_MS02PN_WORKING_RAM *MS02PNWorkingRam)
{
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When user call API function SACM_MS02_Resume, this function is Called by MS02 Library.
 * @param
 *  *MS02WorkingRam [in]: MS02 library working RAM address.
 * @return
 *   None.
 */
void MS02PN_CB_Resume(const SACM_MS02PN_WORKING_RAM *MS02PNWorkingRam)
{
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When playing, this function is called by MS02 Library to get encoded data.
 * @param
 *  *MS02WorkingRam [in]: MS02 library working RAM address.
 *  *SrcDataAddr [in]: Source data address pointer.
 *  *DstBufAddr [out]: Destination buffer address pointer.
 *   DataLen [in]: Data length (halfword)
 * @return
 *   None.
 */
void MS02PN_CB_GetData(const SACM_MS02PN_WORKING_RAM *MS02PNWorkingRam, int16_t *DstBufAddr, int16_t *SrcDataAddr, uint16_t DataLen)
{
	uint16_t iCount;

	for(iCount = 0; iCount < DataLen; iCount++)
  {
    DstBufAddr[iCount] = SrcDataAddr[iCount];
	}
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When playing, this function is called by MS02 Library to trigger DAC DMA.
 * @param
 *  *MS02WorkingRam [in]: MS02 working RAM pointer
 *  *SrcBufAddr [in]: Source buffer address pointer.
 *   DataLen [in]: Data length (halfword)
 * @return
 *   None.
 */
void MS02PN_CB_SendDac_DmaIsr(const SACM_MS02PN_WORKING_RAM *MS02PNWorkingRam, int16_t *SrcBufAddr, uint16_t DataLen)
{
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When playing, this function is called by MS02 library to decode data.
 * @param
 *   *DstBufAddr [out]: Destination buffer address pointer.
 *    OutBufSize  [in]: MS02_OUT_BUFFER_SIZE = 320(Don't modify)
 * @return
 *   None.
 */
void MS02PN_CB_DecodeProcess(int16_t* DstBufAddr, int16_t OutBufSize)
{
  SACM_MS02PN_DecodeProcess(DstBufAddr, OutBufSize);
}
