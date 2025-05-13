/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   KeyScan.h
 * @Version:
 *   V0.9.1
 * @Date:
 *   March 10, 2023
 * @Abstract:
 *    2023.03.10 Support the IOA[31:24](8-bit)/IOA[31:16](16-bit) input keys
 *
 **************************************************************************************************/
#ifndef _KEYSCAN_H_
#define _KEYSCAN_H_


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include <stdint.h>


/*---------------------------------------------------------------------------------------
 * Definition Area
 *---------------------------------------------------------------------------------------*/
#define DEBOUNCE_CNT			  (0x0080)
#define SCAN_8_BITS         (1) 							// 0: 16-Bit/ 1: 8-bit
#define INPUT_IOA24_31      (1) 							// 0: 8-bit for IOA[7:0], 16-bit for IOA[15:0]/ 1: 16-bit for IOA[31:16]

/*---------------------------------------------------------------------------------------
 * Structure Definition Area
 *---------------------------------------------------------------------------------------*/
typedef struct _KEYSCAN_WORKING_RAM
{
	uint32_t KeyDebounceReg;
	uint32_t KeyBuf;
	uint32_t KeyStrobe;
	uint16_t KeyDebounceCnt;
} KEYSCAN_WORKING_RAM;


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C"{
#endif

void KeyScan_Initial(KEYSCAN_WORKING_RAM*);
void KeyScan_ServiceLoop(void);
uint32_t KeyScan_GetCh(void);


#ifdef __cplusplus
}
#endif

#endif





