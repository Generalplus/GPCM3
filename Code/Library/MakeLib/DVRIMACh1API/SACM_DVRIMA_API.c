/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   SACM_DVRIMA_API.c
 * @Version:
 *   V1.0.3
 * @Date:
 *   April 16, 2025
 * @Abstract:
 *
 **************************************************************************************************/


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "GPCM3_FM1.h"
#include "DVRIMADec.h"
#include "SACM_DVRIMA_API.h"
#include "SACM_DVRIMA_User.h"


/*---------------------------------------------------------------------------------------
 * Module Variable Declaration Area
 *---------------------------------------------------------------------------------------*/
uint16_t volatile mDVRIMAStatus = 0;
uint8_t mDVRIMABackgroundServiceFlag = 0x00;
SACM_DVRIMA_WORKING_RAM *mDVRIMAWorkingRamPtr;

// For DVRIMA
/* For decode */
int16_t R_DVRIMA_PrevSample = 0;
int8_t  R_DVRIMA_Index = 0;
/* For encode */
int16_t R_EnCodeIndex = 0;
int32_t R_DVRIMA_predsample = 0;

uint16_t R_UINT_Sample_1;    // = diff
uint16_t R_UINT_Sample_0;    // = PrevSample weight
int16_t R_ADD_Sample_1;      // = sign by decode bit.3
int16_t R_ADD_Sample_0;      // = R_DVRIMA_PrevSample
// int16_t sign;        // = R_ADD_Sample[1]

/*---------------------------------------------------------------------------------------
 * Table Definition Area
 *---------------------------------------------------------------------------------------*/
/*  DVR IMA ADPCM Index Adjust Table */
static int8_t IndexTable[16] = {
-1, -1, -1, -1, 2, 4, 6, 8,
-1, -1, -1, -1, 2, 4, 6, 8,
};

// DVR IMA ADPCM Step Size Table
const uint16_t StepSizeTable[89]={7,8,9,10,11,12,13,14,16,17,
                            19,21,23,25,28,31,34,37,41,45,
                            50,55,60,66,73,80,88,97,107,118,
                            130,143,157,173,190,209,230,253,279,307,
                            337,371,408,449,494,544,598,658,724,796,
                            876,963,1060,1166,1282,1411,1552,1707,1878,2066,
                            2272,2499,2749,3024,3327,3660,4026,4428,4871,5358,
                            5894,6484,7132,7845,8630,9493,10442,11487,12635,13899,
                            15289,16818,18500,20350,22385,24623,27086,29794,32767};


/*---------------------------------------------------------------------------------------
 * DVRIMA Library Code Section
 *---------------------------------------------------------------------------------------*/
 /**
 * @brief ADPCM_DVRIMA Encode(by SampleBase)
 *
 * @param
 *   code: 16-bit data convert as 4-bit IMA ADPCM sample.
 * @return
 *
 * @Note by Kim 2025.05.12
 */
uint8_t ADPCM_Encode(int16_t sample)
{
  uint8_t code=0;
  uint16_t tmpstep=0;
  int32_t diff=0;
  int32_t diffq=0;
  uint16_t step=0;

  // Get the current step length
  step = StepSizeTable[R_EnCodeIndex];

  diff = sample-R_DVRIMA_predsample;
  if (diff < 0)
  {
    code=8;
    diff = -diff;
  }

  tmpstep = step;
  diffq = (step >> 3);

  if (diff >= tmpstep)
  {
    code |= 0x04;
    diff -= tmpstep;
    diffq += step;
  }

  tmpstep = tmpstep >> 1;

  if (diff >= tmpstep)
  {
    code |= 0x02;
    diff -= tmpstep;
    diffq+=(step >> 1);
  }

  tmpstep = tmpstep >> 1;

  if (diff >= tmpstep)
  {
    code |=0x01;
    diffq+=(step >> 2);
  }

  if (code & 8)
  {
    R_DVRIMA_predsample -= diffq;
  }
  else
  {
    R_DVRIMA_predsample += diffq;
  }

  if (R_DVRIMA_predsample > 32767)
  {
    R_DVRIMA_predsample = 32767;
  }
  else if (R_DVRIMA_predsample < -32768)
  {
    R_DVRIMA_predsample = -32768;
  }

  R_EnCodeIndex += IndexTable[code];
  if (R_EnCodeIndex <0)
  {
    R_EnCodeIndex = 0;
  }
  else if (R_EnCodeIndex > 88)
  {
    R_EnCodeIndex = 88;
  }

  return (code & 0x0f);
}



/**
 * @brief ADPCM_DVRIMA Decode(by SampleBase)
 *
 * @param
 *   code: A byte containing a 4-bit ADPCM sample. (Only the lower 4 bits data are valid)
 * @return
 *   16-bit ADPCM sample
 * @Note by Kim 2025.04.24
 */
int16_t ADPCM_Decode(uint8_t code)
{
  uint16_t step=0;
  int32_t diffq=0;

  // Get the current step length
  step = StepSizeTable[R_DVRIMA_Index];

  // First, a fixed difference of 1/8(=0.125)
  diffq = step>> 3;

  // 3rd, 1x step
  if (code&4)
  {
    diffq += step;
  }
  // 2nd, 0.5x step
  if (code&2)
  {
    diffq += step>>1;
  }
  // 1st, 0.25x step
  if (code&1)
  {
    diffq += step>>2;
  }

  // The highest bit represents positive or negative: 0: +,1:-
  if (code&8)
  {
    R_DVRIMA_PrevSample -= diffq;
  }
  else
  {
    R_DVRIMA_PrevSample += diffq;
  }

  // Cross-border protection
  if (R_DVRIMA_PrevSample > 32767)
  {
    R_DVRIMA_PrevSample = 32767;
  }
  else if (R_DVRIMA_PrevSample < -32768)
  {
    R_DVRIMA_PrevSample = -32768;
  }

  R_DVRIMA_Index += IndexTable [code];
  // Cross-border protection
  if (R_DVRIMA_Index < 0)
  {
    R_DVRIMA_Index = 0;
  }
  if (R_DVRIMA_Index > 88)
  {
    R_DVRIMA_Index = 88;
  }

  return ((int16_t)R_DVRIMA_PrevSample);
}


/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 * @Note
 */
void IMADecode_Run( int16_t *OutPCM, int16_t *InDecode, int in_len)
{
  uint16_t iCount;
  uint8_t decCount;

  for(iCount = 0; iCount < IMA_DEC_FRAMESIZE; iCount++)
	{
    for(decCount =0; decCount<4; decCount++)
    {
      switch(decCount)
      {
        case 0:
          OutPCM[(iCount*4)+decCount] = ADPCM_Decode((int8_t)(InDecode[iCount] & 0x000F));
          break;
        case 1:
          OutPCM[(iCount*4)+decCount] = ADPCM_Decode((int8_t)((InDecode[iCount] >> 4) & 0x000F));
          break;
        case 2:
          OutPCM[(iCount*4)+decCount] = ADPCM_Decode((int8_t)((InDecode[iCount] >> 8) & 0x000F));
          break;
        case 3:
          OutPCM[(iCount*4)+decCount] = ADPCM_Decode((int8_t)((InDecode[iCount] >> 12) & 0x000F));
          break;
      }
    }
	}
}


/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void DVRIMA_LinkLibTag()
{

}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   Error Code = 0 : init pass
 *              = not zero : init fail
 */
int8_t DVRIMA_DecodeInit()
{
	int16_t TempData[3];
	int32_t Temp;

	mDVRIMAWorkingRamPtr->DecodeCount = 0;                                        // Reset decode counter
	DVRIMA_CB_GetData(mDVRIMAWorkingRamPtr, TempData, &mDVRIMAWorkingRamPtr->SpeechDataAddr[mDVRIMAWorkingRamPtr->DecodeCount], 2);	 // callback function:DVRIMA_CB_GetData in SACM_DVRIMA_User.c
	mDVRIMAWorkingRamPtr->DecodeCount += 2;       // Kim modify by DVRIMA
	Temp = (*(uint32_t*)TempData);
	mDVRIMAWorkingRamPtr->DataLen = (((*(uint32_t*)TempData) + 4) >> 1);          // file data length. Unit: halfword
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
void DVRIMA_ReInitial(void)
{
	mDVRIMAStatus &= ~(DVRIMA_AUTO_RAMP_DOWN_ENABLE_FLAG | DVRIMA_AUTO_RAMP_UP_ENABLE_FLAG | DVRIMA_ENABLE_DAC_CH0_FLAG | DVRIMA_ENABLE_DAC_CH1_FLAG);
  mDVRIMAStatus |= mDVRIMAWorkingRamPtr->NextStatus;
	mDVRIMAWorkingRamPtr->SpeechDataAddr = mDVRIMAWorkingRamPtr->NextSpeechDataAddr;
	mDVRIMAWorkingRamPtr->NextSpeechDataAddr = NULL;

  if(DVRIMA_DecodeInit() != 0)
	{
    mDVRIMAStatus = 0;
    return;
	}

	DVRIMA_CB_Play_Con(mDVRIMAWorkingRamPtr);
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   Status
 */
uint16_t DVRIMA_InitStatus(uint8_t DacChannelNo, uint8_t AutoRampUpDownCtrl, uint16_t InitState)
{
  if((DacChannelNo & DVRIMA_DAC_CH0) != 0)
	{
	  InitState |= DVRIMA_ENABLE_DAC_CH0_FLAG;
	}

  if((DacChannelNo & DVRIMA_DAC_CH1) != 0)
	{
	  InitState |= DVRIMA_ENABLE_DAC_CH1_FLAG;
	}

	if((AutoRampUpDownCtrl & DVRIMA_AUTO_RAMP_UP) != 0)
	{
	  InitState |= DVRIMA_AUTO_RAMP_UP_ENABLE_FLAG;
	}

	if((AutoRampUpDownCtrl & DVRIMA_AUTO_RAMP_DOWN) != 0)
	{
	  InitState |= DVRIMA_AUTO_RAMP_DOWN_ENABLE_FLAG;
	}

	return InitState;
}


/*---------------------------------------------------------------------------------------
 * API Function Area
 *---------------------------------------------------------------------------------------*/

/**
 * @brief
 *
 * @param
 *  *DVRIMAWorkingRam [in]: DVRIMA working RAM address
 *  *DVRIMATempBuffer [in]: DVRIMA temp buffer address
 * @return
 *   None.
 */
void SACM_DVRIMA_Initial(SACM_DVRIMA_WORKING_RAM *DVRIMAWorkingRam, SACM_DVRIMA_TEMP_RAM *pDVRIMATempBuffer, void *DVRIMAPcmBuffer)
{
	// DVRIMA_LinkLibTag();
	mDVRIMAStatus = 0x0000;
  R_DVRIMA_PrevSample = 0;      // Kim add 2025.04.21
  R_DVRIMA_Index = 0;           // Kim add 2025.04.21

	mDVRIMAWorkingRamPtr = DVRIMAWorkingRam;
	mDVRIMAWorkingRamPtr->DVRIMATempBufPtr = pDVRIMATempBuffer;
	mDVRIMAWorkingRamPtr->PcmBufStartAddress = DVRIMAPcmBuffer;
	mDVRIMAWorkingRamPtr->HalfWordsPerFrame = IMA_DEC_FRAMESIZE;

	mDVRIMAWorkingRamPtr->NextSpeechDataAddr = NULL;
	mDVRIMABackgroundServiceFlag = 0x00;
	mDVRIMAWorkingRamPtr->VolumeGain = 0x4000;                                    // default volume
  DVRIMA_CB_Init(mDVRIMAWorkingRamPtr);                                          // callback function: DVRIMA_CB_Init in SACM_DVRIMA_User.c
}

/**
 * @brief
 *
 * @param
 *  *SpeechDataStartAddr [in]: Speech data start address
 *   DacChannelNo [in]:
 *    - 1 => DAC1 enable
 *    - 2 => DAC2 enable
 *    - 3 => DAC1, DAC2 enable
 *   AutoRampUpDownCtrl [in]:
 *    - 0 => Ramp Up disable/Ramp Dn disable
 *    - 1 => Ramp Up enable/Ramp Dn disable
 *    - 2 => Ramp Up disable/Ramp Dn enable
 *    - 3 => Ramp Up enable/Ramp Dn enable
 * @return
 *   None.
 */
void SACM_DVRIMA_Play(int16_t *SpeechDataStartAddr, uint8_t DacChannelNo, uint8_t AutoRampUpDownCtrl)
{
  int16_t iCount;
  R_DVRIMA_PrevSample = 0;      // Kim add 2025.04.21
  R_DVRIMA_Index = 0;           // Kim add 2025.04.21

	// DVRIMA_LinkLibTag();
  mDVRIMAStatus = DVRIMA_InitStatus(DacChannelNo, AutoRampUpDownCtrl, (DVRIMA_PLAY_FLAG | DVRIMA_DECODE_WORK_FLAG | DVRIMA_ISR_SERVICE_ENABLE_FLAG | DVRIMA_BUF_EVEN_FLAG));
	mDVRIMABackgroundServiceFlag = 0x00;
	mDVRIMAWorkingRamPtr->SpeechDataAddr = SpeechDataStartAddr;
	mDVRIMAWorkingRamPtr->NextSpeechDataAddr = NULL;

  /*
   * Initial Play Buffer Pointer & Codec Buffer Pointer
   */
	mDVRIMAWorkingRamPtr->PcmBufPtr = mDVRIMAWorkingRamPtr->PcmBufStartAddress;
	mDVRIMAWorkingRamPtr->DVRIMADecodeBufPtr = mDVRIMAWorkingRamPtr->PcmBufStartAddress + DVRIMA_FRAME_SIZE;

  /*
   * Clear DacOut PCM buffer
   */
	for(iCount = 0; iCount < DVRIMA_FRAME_SIZE; iCount++)
  {
		mDVRIMAWorkingRamPtr->PcmBufPtr[iCount] = 0x0000;
	}

  if(DVRIMA_DecodeInit() != 0)
	{
    mDVRIMAStatus = 0;
    return;
	}

	mDVRIMABackgroundServiceFlag = 1;                                             // enable background service loop
	DVRIMA_CB_StartPlay(mDVRIMAWorkingRamPtr);                                     // callback function: DVRIMA_CB_StartPlay in SACM_DVRIMA_User.asm
}

/**
 * @brief
 *
 * @param
 *  *SpeechDataStartAddr [in]: Speech data start address
 *   DacChannelNo [in]:
 *    - 1 => DAC1 enable
 *    - 2 => DAC2 enable
 *    - 3 => DAC1, DAC2 enable
 *   AutoRampUpDownCtrl [in]:
 *    - 0 => Ramp Up disable/Ramp Dn disable
 *    - 1 => Ramp Up enable/Ramp Dn disable
 *    - 2 => Ramp Up disable/Ramp Dn enable
 *    - 3 => Ramp Up enable/Ramp Dn enable
 * @return
 *   None.
 */
void SACM_DVRIMA_Play_Con(int16_t *SpeechDataStartAddr, uint8_t DacChannelNo, uint8_t AutoRampUpDownCtrl)
{
	if(((mDVRIMAStatus & DVRIMA_PLAY_FLAG) != 0) && (mDVRIMAWorkingRamPtr->NextSpeechDataAddr == NULL))
	{
		mDVRIMAWorkingRamPtr->NextStatus = DVRIMA_InitStatus(DacChannelNo, AutoRampUpDownCtrl, 0);
		mDVRIMAWorkingRamPtr->NextSpeechDataAddr = SpeechDataStartAddr;
	}
	else
	{
	  SACM_DVRIMA_Play(SpeechDataStartAddr, DacChannelNo, AutoRampUpDownCtrl);
	}
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *
 */
uint16_t SACM_DVRIMA_Check_Con()
{
  if(mDVRIMAWorkingRamPtr->NextSpeechDataAddr == NULL)
	{
	  return 0;
	}
	else
	{
		return 1;
	}
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void SACM_DVRIMA_Stop()
{
	DVRIMA_CB_StopPlay(mDVRIMAWorkingRamPtr);                                      // callback function: DVRIMA_CB_StopPlay in SACM_DVRIMA_User.asm
	mDVRIMABackgroundServiceFlag = 0x00;
	mDVRIMAWorkingRamPtr->NextStatus = 0x0000;
	mDVRIMAWorkingRamPtr->NextSpeechDataAddr = NULL;
  mDVRIMAStatus = 0x0000;
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   mDVRIMAStatus
 */
uint16_t SACM_DVRIMA_GetStatus()
{
  return mDVRIMAStatus;
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void SACM_DVRIMA_ServiceLoop()
{
	if(((mDVRIMAStatus & DVRIMA_PLAY_FLAG) == 0) || ((mDVRIMAStatus & DVRIMA_DECODE_WORK_FLAG) == 0) || ((mDVRIMAStatus & DVRIMA_PAUSE_FLAG) != 0))
	{
		return;
	}

	if((mDVRIMAStatus & DVRIMA_STOP_FLAG) != 0)
	{
		SACM_DVRIMA_Stop();
		return;
	}

	mDVRIMABackgroundServiceFlag = 0;

	if((mDVRIMAStatus & DVRIMA_DECODE_END_FLAG) == 0)
	{
	  DVRIMA_CB_GetData(
									   mDVRIMAWorkingRamPtr,
									   mDVRIMAWorkingRamPtr->DVRIMATempBufPtr,                   // Kim 2025.04.23
									   &mDVRIMAWorkingRamPtr->SpeechDataAddr[mDVRIMAWorkingRamPtr->DecodeCount],
									   mDVRIMAWorkingRamPtr->HalfWordsPerFrame
									  );  		                                                   // callback function: DVRIMA_CB_GetData in SACM_DVRIMA_User.asm
	  mDVRIMAWorkingRamPtr->DecodeCount += mDVRIMAWorkingRamPtr->HalfWordsPerFrame;

    DVRIMA_CB_DecodeProcess(mDVRIMAWorkingRamPtr, mDVRIMAWorkingRamPtr->DVRIMADecodeBufPtr, mDVRIMAWorkingRamPtr->DVRIMATempBufPtr);      // Kim 2025.04.23

	  if((mDVRIMAWorkingRamPtr->DecodeCount >= mDVRIMAWorkingRamPtr->DataLen) && (mDVRIMAWorkingRamPtr->DecodeCount != 0xFFFFFFFF))
	  {
		  if(mDVRIMAWorkingRamPtr->NextSpeechDataAddr == NULL)
		  {
			  mDVRIMAStatus |= DVRIMA_DECODE_END_FLAG;
			  //mDVRIMAStatus &= ~DVRIMA_DECODE_WORK_FLAG;                                 // clear decode work flag
			  //return;
		  }
		  else
		  {
			  DVRIMA_ReInitial();
		  }
	  }
	}

	mDVRIMAStatus &= ~DVRIMA_DECODE_WORK_FLAG;                                   // clear decode work flag
	mDVRIMABackgroundServiceFlag = 0x01;
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void SACM_DVRIMA_Pause()
{
	if((mDVRIMAStatus & DVRIMA_PLAY_FLAG) != 0)
	{
	  mDVRIMAStatus |= DVRIMA_PAUSE_FLAG;
		mDVRIMAStatus &= ~DVRIMA_ISR_SERVICE_ENABLE_FLAG;
		DVRIMA_CB_Pause(mDVRIMAWorkingRamPtr);                                       // callback function: DVRIMA_CB_Pause in SACM_DVRIMA_User.asm
	}
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void SACM_DVRIMA_Resume()
{
	if((mDVRIMAStatus & DVRIMA_PAUSE_FLAG) != 0)
	{
	  mDVRIMAStatus &= ~DVRIMA_PAUSE_FLAG;
		mDVRIMAStatus |= DVRIMA_ISR_SERVICE_ENABLE_FLAG;
		DVRIMA_CB_Resume(mDVRIMAWorkingRamPtr);                                      // callback function: DVRIMA_CB_Resume in SACM_DVRIMA_User.c
	}
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   1 => CPU overload
 *   0 => CPU non-overload
 */
uint32_t SACM_DVRIMA_CheckCpuOverload()
{
  if((mDVRIMAStatus & DVRIMA_CPU_OVERLOAD_FLAG) == 0)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void SACM_DVRIMA_ClearCpuOverload()
{
  mDVRIMAStatus &= ~DVRIMA_CPU_OVERLOAD_FLAG;
}


/*---------------------------------------------------------------------------------------
 * DVRIMA ISR Service Fucntion Code Section
 *---------------------------------------------------------------------------------------*/

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void SACM_DVRIMA_DmaIsrService()
{
  if((mDVRIMAStatus & DVRIMA_ISR_SERVICE_ENABLE_FLAG)	== 0)
	{
		return;
	}

	if(((mDVRIMAStatus & DVRIMA_DECODE_WORK_FLAG) != 0) && (mDVRIMAWorkingRamPtr->DecodeCount > 3))
	{
		mDVRIMAStatus |= DVRIMA_CPU_OVERLOAD_FLAG;
	}

	if((mDVRIMAStatus & DVRIMA_PLAY_FLAG)	!= 0)
  {
		if((mDVRIMAStatus & DVRIMA_DECODE_WORK_FLAG) == 0)
		{
      mDVRIMAStatus |= DVRIMA_DECODE_WORK_FLAG;

      if((mDVRIMAStatus & DVRIMA_PLAY_LAST_FRAME_FLAG) != 0)
	    {
        mDVRIMAStatus |= DVRIMA_STOP_FLAG;
        return;
      }

	    if((mDVRIMAStatus & DVRIMA_BUF_EVEN_FLAG) != 0)
		  {
			  mDVRIMAWorkingRamPtr->PcmBufPtr = mDVRIMAWorkingRamPtr->PcmBufStartAddress + DVRIMA_FRAME_SIZE;
			  mDVRIMAWorkingRamPtr->DVRIMADecodeBufPtr = mDVRIMAWorkingRamPtr->PcmBufStartAddress;
	    }
		  else
		  {
        mDVRIMAWorkingRamPtr->PcmBufPtr = mDVRIMAWorkingRamPtr->PcmBufStartAddress;
			  mDVRIMAWorkingRamPtr->DVRIMADecodeBufPtr = mDVRIMAWorkingRamPtr->PcmBufStartAddress + DVRIMA_FRAME_SIZE;
	    }
	    mDVRIMAStatus ^= DVRIMA_BUF_EVEN_FLAG;

	    if((mDVRIMAStatus & DVRIMA_DECODE_END_FLAG) != 0)
		  {
			  mDVRIMAStatus |= DVRIMA_PLAY_LAST_FRAME_FLAG;
	    }
		}

		if((mDVRIMAStatus & DVRIMA_PAUSE_FLAG) == 0)
		{
			DVRIMA_CB_SendDac_DmaIsr(mDVRIMAWorkingRamPtr, mDVRIMAWorkingRamPtr->PcmBufPtr, DVRIMA_FRAME_SIZE);                            // callback function: DVRIMA_CB_SendDac_DmaIsr in SACM_DVRIMA_User.c
		}
	}
}


/**
 * @brief
 *
 * @param
 *  *DstBufAddr [out]: Destination buffer address pointer.
 *  *SrcBufAddr [in]: Source buffer address pointer.
 * @return
 *   GpEventFlag
 */
uint32_t SACM_DVRIMA_DecodeProcess(int16_t *DstBufAddr, int16_t *SrcBufAddr)
{
  IMADecode_Run( DstBufAddr, SrcBufAddr, IMA_DEC_FRAMESIZE);    // Kim 2025.04.28
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void SACM_DVRIMA_IsrService()
{
}
