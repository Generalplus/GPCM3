/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   SACM_A1801_User.c
 * @Version:
 *   V1.0.3
 * @Date:
 *   December 17, 2021
 * @Abstract:
 *
 **************************************************************************************************/


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "GPCM3_FM1.h"
#include "SACM_A1801_User.h"
#include "SACM_A1801_CH2_User.h"
#include "A1801_FileMerger.h"
#include "APP_SwVolumeControl.h"


/*---------------------------------------------------------------------------------------
 * Gobal Variable Declaration Area
 *---------------------------------------------------------------------------------------*/


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
uint16_t A1801_User_GetA1801Num()
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
int16_t* A1801_User_GetA1801StartAddr(uint16_t SpeechIdx)
{
	return (int16_t*) ((*((uint32_t *)(SEC_START_ADDR) + SpeechIdx)) + SEC_START_ADDR);
}


/*---------------------------------------------------------------------------------------
 * Callback Function Area
 *---------------------------------------------------------------------------------------*/
/**
 * @brief
 *   Callback function. Implemented by user.
 *   When user call API function SACM_A1801_Initial, this function is called by A1801 Library.
 * @param
 *  *A1801WorkingRam [in]: A1801 library working RAM address.
 * @return
 *   None.
 */
void A1801_CB_Init(const SACM_A1801_WORKING_RAM *A1801WorkingRam)
{
}

/**
 * @brief
 *   Callback function. To be implemented by user.
 *   When user call API function SACM_A1801_Play, this function is called by A1801 Library.
 * @param
 *  *A1801WorkingRam [in]: A1801 library working RAM address.
 * @return
 *   None.
 */
void A1801_CB_StartPlay(const SACM_A1801_WORKING_RAM *A1801WorkingRam)
{
}

/**
 * @brief
 *
 * @param
 *  *A1801WorkingRam [in]: A1801 library working RAM address.
 * @return
 *   None.
 */
void A1801_CB_Play_Con(const SACM_A1801_WORKING_RAM *A1801WorkingRam)
{
}

/**
 * @brief
 *
 * @param
 *  *A1801WorkingRam [in]: A1801 library working RAM address.
 * @return
 *   None.
 */
void A1801_CB_StopPlay(const SACM_A1801_WORKING_RAM *A1801WorkingRam)
{
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When user call API function SACM_A1801_Pause, this function is called by A1801 Library.
 * @param
 *  *A1801WorkingRam [in]: A1801 library working RAM address.
 * @return
 *   None.
 */
void A1801_CB_Pause(const SACM_A1801_WORKING_RAM *A1801WorkingRam)
{
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When user call API function SACM_A1801_Resume, this function is Called by A1801 Library.
 * @param
 *  *A1801WorkingRam [in]: A1801 library working RAM address.
 * @return
 *   None.
 */
void A1801_CB_Resume(const SACM_A1801_WORKING_RAM *A1801WorkingRam)
{
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When playing, this function is called by A1801 Library to get encoded data.
 * @param
 *  *A1801WorkingRam [in]: A1801 library working RAM address.
 *  *DstBufAddr [out]: Destination buffer address pointer.
 *  *SrcDataAddr [in]: Source data address pointer.
 *   DataLen [in]: Data length (halfword)
 * @return
 *   None.
 */
void A1801_CB_GetData(const SACM_A1801_WORKING_RAM *A1801WorkingRam, int16_t *DstBufAddr, int16_t *SrcDataAddr, uint16_t DataLen)
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
 *   When playing, this function is called by A1801 Library to trigger DAC DMA.
 * @param
 *  *A1801WorkingRam [in]: A1801 working RAM pointer
 *  *SrcBufAddr [in]: Source buffer address pointer.
 *   DataLen [in]: Data length (halfword)
 * @return
 *   None.
 */
void A1801_CB_SendDac_DmaIsr(const SACM_A1801_WORKING_RAM *A1801WorkingRam, int16_t *SrcBufAddr, uint16_t DataLen)
{
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
void A1801_CB_DecodeProcess(const SACM_A1801_WORKING_RAM *A1801WorkingRam, int16_t *DstBufAddr, int16_t *SrcBufAddr)
{
	SACM_A1801_DecodeProcess(DstBufAddr, SrcBufAddr);
	//APP_SwVolCtrl_VolProcess(0, DstBufAddr, A1801_FRAME_SIZE);
}
