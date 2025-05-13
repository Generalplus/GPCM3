/**************************************************************************************************
 * Copyright(c) 2021 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   APP_SwMixer.c
 * @Version:
 *   V1.0.0
 * @Date:
 *   March 26, 2021
 * @Abstract:
 *
 **************************************************************************************************/


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "APP_SwMixer.h"


/*---------------------------------------------------------------------------------------
 * Table Declaration Area
 *---------------------------------------------------------------------------------------*/
static const uint16_t SW_Mixer_2ch_Table[] =
{
  0,     0,     0, 0,
  2048,  2048,  0, 0,
  4096,  4096,  0, 0,
  6144,  6144,  0, 0,
	8192,  8192,  0, 0,
	10240, 10240, 0, 0,
	12288, 12288, 1, 0,
	14336, 13312, 1, 0,
	16384, 14336, 1, 0,
	18432, 15361, 2, 0,
	20480, 15874, 3, 0,
	22528, 16131, 4, 0,
  24576, 16260,	5, 0,
  26624, 16325, 6, 0,
  28672, 16358, 7, 0,
  30720, 16375, 8, 0,
  32768, 16384, 0, 0,
};

static const uint16_t SW_Mixer_3Ch_Table[] =
{
  0,     0,    0, 0,
  2048,  2048, 0, 0,
  4096,  4096, 0, 0,
  6144,  6144, 1, 0,
	8192,  7168, 2, 0,
	10240, 7680, 3, 0,
	12288, 7936, 4, 0,
	14336, 8064, 5, 0,
	16384, 8128, 6, 0,
	18432, 8160, 7, 0,
	20480, 8176, 8, 0,
	22528, 8184, 8, 0,
  24576, 8192, 0, 0,
};


/*---------------------------------------------------------------------------------------
 * Subroutine Area
 *---------------------------------------------------------------------------------------*/

/**
 * @brief
 *
 * @param
 *  *PcmInBuffer1 [in]: 1st PCM buffer
 *  *PcmInBuffer2 [in]: 2nd PCM buffer
 *  *DstBufAddr [out]: Destination buffer address
 *   DataLen [in]: Data length (unit: halfword)
 * @return
 *   None.
 */
 /*

void APP_SwMixer_2ch(int16_t *PcmInBuffer1, int16_t *PcmInBuffer2, int16_t *DstBufAddr, uint16_t DataLen)
{
  uint32_t SignFlag;
	uint32_t BuffIdx;
  int32_t PcmData;
	uint32_t SwMixerTableIdx;
	uint32_t Boundary;
	uint32_t ReMappingBoundary;
	uint32_t Divisor;

	for(BuffIdx = 0; BuffIdx < DataLen; BuffIdx++)
	{
		PcmData = PcmInBuffer1[BuffIdx] +	PcmInBuffer2[BuffIdx];

		if(PcmData > 0)
		{
			SignFlag = 0;
			PcmData >>= 1;
		}
		else
		{
			SignFlag = 1;
			PcmData *= -1;
			PcmData >>= 1;
		}

		SwMixerTableIdx = PcmData >> 11;
		if(SwMixerTableIdx != 0)
		{
			SwMixerTableIdx <<= 2;
			Boundary = SW_Mixer_2ch_Table[SwMixerTableIdx++];
			ReMappingBoundary = SW_Mixer_2ch_Table[SwMixerTableIdx++];
			Divisor = SW_Mixer_2ch_Table[SwMixerTableIdx];
			PcmData = ReMappingBoundary + ((PcmData - Boundary) >> Divisor);
		}

		if(SignFlag != 0)
		{
		  PcmData *= -1;
		}

		DstBufAddr[BuffIdx] = (int16_t)(PcmData << 1);
	}
}
*/
/**
 * @brief
 *
 * @param
 *  *PcmInBuffer1 [in]: 1st PCM buffer
 *  *PcmInBuffer2 [in]: 2nd PCM buffer
 *  *PcmInBuffer3 [in]: 3rd PCM buffer
 *  *DstBufAddr [out]: Destination buffer address
 *   DataLen [in]: Data length (unit: halfword)
 * @return
 *   None.
 */
void APP_SwMixer_3ch(int16_t *PcmInBuffer1, int16_t *PcmInBuffer2, int16_t *PcmInBuffer3, int16_t *DstBufAddr, uint16_t DataLen)
{
  uint32_t SignFlag;
	uint32_t BuffIdx;
  int32_t PcmData;
	uint32_t SwMixerTableIdx;
	uint32_t Boundary;
	uint32_t ReMappingBoundary;
	uint32_t Divisor;

	for(BuffIdx = 0; BuffIdx < DataLen; BuffIdx++)
	{
    PcmData = 0;
    if(PcmInBuffer1 != 0)
    {
      PcmData += PcmInBuffer1[BuffIdx];
    }
    if(PcmInBuffer2 != 0)
    {
      PcmData += PcmInBuffer2[BuffIdx];
    }
    if(PcmInBuffer3 != 0)
    {
      PcmData += PcmInBuffer3[BuffIdx];
    }

    PcmData >>= 2;  // Added by Ray
		if(PcmData > 0)
		{
			SignFlag = 0;
		}
		else
		{
			SignFlag = 1;
			PcmData *= -1;
		}

//		SwMixerTableIdx = PcmData >> 12;
		SwMixerTableIdx = PcmData >> 11; // Modified by Ray
		if(SwMixerTableIdx != 0)
		{
			SwMixerTableIdx <<= 2;
			Boundary = SW_Mixer_3Ch_Table[SwMixerTableIdx++];
			ReMappingBoundary = SW_Mixer_3Ch_Table[SwMixerTableIdx++];
			Divisor = SW_Mixer_3Ch_Table[SwMixerTableIdx];
			PcmData = ReMappingBoundary + ((PcmData - Boundary) >> Divisor);
		}

    PcmData = PcmData << 2;   // Added by Ray
		if(SignFlag != 0)
		{
		  PcmData *= -1;
		}

//		DstBufAddr[BuffIdx] = (int16_t)(PcmData << 1);
		DstBufAddr[BuffIdx] = (int16_t)PcmData;   // Modified by Ray
	}
}



