/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   KeyScan.c
 * @Version:
 *   V0.9.1
 * @Date:
 *   March 10, 2023
 * @Abstract:
 *    2023.03.10 Support IOA[31:24](8-bit)/IOA[31:16](16-bit) input keys
 *
 **************************************************************************************************/


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "GPCM3_FM1.h"
#include "KeyScan.h"


/*---------------------------------------------------------------------------------------
 * Gobal Variable Declaration Area
 *---------------------------------------------------------------------------------------*/
KEYSCAN_WORKING_RAM *mKeyScanWorkingRamPtr;


/*---------------------------------------------------------------------------------------
 * Subroutine Area
 *---------------------------------------------------------------------------------------*/

/**
 * @brief
 *   Key scan initialzation
 * @param
 *   None.
 * @return
 *   None.
 */
void KeyScan_Initial(KEYSCAN_WORKING_RAM *KeyScanWorkingRam)
{
	mKeyScanWorkingRamPtr = KeyScanWorkingRam;

	#if SCAN_8_BITS
    #if INPUT_IOA24_31
      GPIO_SetMode(GPIOA, 0xFF000000, GPIO_MODE_INPUT);     // IOA[31:24]
    #else
      GPIO_SetMode(GPIOA, 0x000000FF, GPIO_MODE_INPUT);     // IOA[7:0]
    #endif
	#else
    #if INPUT_IOA24_31
      GPIO_SetMode(GPIOA, 0xFFFF0000, GPIO_MODE_INPUT);     // IOA[31:16]
    #else
      GPIO_SetMode(GPIOA, 0x0000FFFF, GPIO_MODE_INPUT);     // IOA[15:0]
    #endif

	#endif

	mKeyScanWorkingRamPtr->KeyDebounceReg = 0;
	mKeyScanWorkingRamPtr->KeyBuf = 0;
	mKeyScanWorkingRamPtr->KeyStrobe = 0;
	mKeyScanWorkingRamPtr->KeyDebounceCnt = DEBOUNCE_CNT;
}

/**
 * @brief
 *   Get Key code from key pad
 * @param
 *   None.
 * @return
 *   None.
 */
void KeyScan_ServiceLoop(void)
{
	uint32_t R_PrevData;

	if(mKeyScanWorkingRamPtr->KeyDebounceCnt != 0)
	{
		mKeyScanWorkingRamPtr->KeyDebounceCnt -= 1;
	}

	R_PrevData = mKeyScanWorkingRamPtr->KeyDebounceReg;                          // get prev I/O

	/*
   *   SCAN_8_BITS = 1, ScanKey 8bits
   *   SCAN_8_BITS = 0, ScanKey 16bits
	 */
	#if SCAN_8_BITS
    #if INPUT_IOA24_31
      mKeyScanWorkingRamPtr->KeyDebounceReg = (GPIOA->IDATA & 0xFF000000);     // IOA[31:24]
    #else
      mKeyScanWorkingRamPtr->KeyDebounceReg = (GPIOA->IDATA & 0x000000FF);     // IOA[7:0]
    #endif

	#else
    #if INPUT_IOA24_31
      mKeyScanWorkingRamPtr->KeyDebounceReg = (GPIOA->IDATA & 0xFFFF0000);     // IOA[31:16]
    #else
      mKeyScanWorkingRamPtr->KeyDebounceReg = (GPIOA->IDATA & 0x0000FFFF);     // IOA[15:0]
    #endif
	#endif

	if(R_PrevData != mKeyScanWorkingRamPtr->KeyDebounceReg)                      // if prev != curr
	{
		mKeyScanWorkingRamPtr->KeyDebounceCnt = DEBOUNCE_CNT;                      // debounce counter reset
		// mKeyScanWorkingRamPtr->KeyDebounceReg = mKeyScanWorkingRamPtr->KeyDebounceReg; //Kim: 將目前Data回存至KeyDebounceReg
		return;
	}

	if(mKeyScanWorkingRamPtr->KeyDebounceCnt != 0)
	{
		return;
	}

	R_PrevData = mKeyScanWorkingRamPtr->KeyBuf;															          // prev buf

	#if SCAN_8_BITS
    #if INPUT_IOA24_31
      mKeyScanWorkingRamPtr->KeyBuf = (mKeyScanWorkingRamPtr->KeyDebounceReg >> 24);  // IOA[31:24]
    #else
      mKeyScanWorkingRamPtr->KeyBuf = mKeyScanWorkingRamPtr->KeyDebounceReg;	        // IOA[7:0]
    #endif

		R_PrevData ^= 0x00FF;
		R_PrevData &= mKeyScanWorkingRamPtr->KeyBuf;
		R_PrevData &= 0x00FF;

	#else
    #if INPUT_IOA24_31
      mKeyScanWorkingRamPtr->KeyBuf = (mKeyScanWorkingRamPtr->KeyDebounceReg >> 24);  // IOA[31:16]
    #else
      mKeyScanWorkingRamPtr->KeyBuf = mKeyScanWorkingRamPtr->KeyDebounceReg;	        // IOA[15:0]
    #endif

		R_PrevData ^= 0xFFFF;
		R_PrevData &= mKeyScanWorkingRamPtr->KeyBuf;
		R_PrevData &= 0xFFFF;
	#endif

	mKeyScanWorkingRamPtr->KeyStrobe |= R_PrevData;

	return;
}

/**
 * @brief
 *   Get Key code
 * @param
 *   None.
 * @return
 *   None.
 */
uint32_t KeyScan_GetCh(void)
{
	uint32_t temp;

	temp = mKeyScanWorkingRamPtr->KeyStrobe;                                     // Get Key code
	mKeyScanWorkingRamPtr->KeyStrobe = 0x0000;                                   // Clear KeyStrobe for next key

	return temp;
}
