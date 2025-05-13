/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   APP_SwVolumeControl.c
 * @Version:
 *   V1.0.2
 * @Date:
 *   October 7, 2020
 * @Abstract:
 *
 **************************************************************************************************/


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "APP_SwVolumeControl.h"


/*---------------------------------------------------------------------------------------
 * Table Declaration Area
 *---------------------------------------------------------------------------------------*/
static uint16_t const SW_Volume_Gain_Table[] =
{
  0x0000, 0x0250, 0x0500, 0x1000,
  0x1500, 0x2000, 0x2500, 0x3000,
  0x3500, 0x4000, 0x5000, 0x6500,
  0x7D00, 0x9C00, 0xC400, 0xF500
};


/*---------------------------------------------------------------------------------------
 * Gobal Variable Declaration Area
 *---------------------------------------------------------------------------------------*/
static uint8_t mVolGain[VOL_CONTROL_NUM] = {9, 9, 9, 9};


/*---------------------------------------------------------------------------------------
 * Subroutine Area
 *---------------------------------------------------------------------------------------*/

/**
 * @brief
 *   Initial software volume control
 * @param
 *   None.
 * @return
 *   None.
 */
void APP_SwVolCtrl_Init()
{
	uint32_t iCount;

	for(iCount = 0 ; iCount < sizeof(mVolGain); iCount++)
	{
    mVolGain[iCount] = 9;
	}
}

/**
 * @brief
 *   Set the gain of software volume control
 * @param
 *   VolCtrlChannel [in]:
 *   VolGain [in]:
 *    - 0 ~ 15
 * @return
 *   None.
 */
void APP_SwVolCtrl_SetVolGain(int32_t VolCtrlChannel, int32_t VolGain)
{
	mVolGain[VolCtrlChannel] = VolGain;
}

/**
 * @brief
 *
 * @param
 *  *PcmBuffer [in out]: PCM buffer pointer.
 *   DataLen [in]: Data length (unit: halfword)
 * @return
 *   None.
 */
void APP_SwVolCtrl_VolProcess(int32_t VolCtrlChannel, int16_t *PcmBuffer, uint16_t DataLen)
{
	uint32_t PcmBufferIdx;
	int32_t VolumeGain;
  int32_t PcmData;

	VolumeGain = SW_Volume_Gain_Table[mVolGain[VolCtrlChannel]];
	for(PcmBufferIdx = 0; PcmBufferIdx < DataLen; PcmBufferIdx++)
	{
		PcmData = PcmBuffer[PcmBufferIdx];
		if(PcmData < 0)
		{
			PcmData = ((PcmData * VolumeGain) >> 14);
			if(PcmData < -32750)
			{
				PcmData = -32750;
			}
		}
		else
		{
			PcmData = ((PcmData * VolumeGain) >> 14);
			if(PcmData > 32750)
			{
				PcmData = 32750;
			}
		}

		PcmBuffer[PcmBufferIdx] = (int16_t)PcmData;
	}
}
