/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   FW_Update.c
 * @Version:
 *   V0.9
 * @Date:
 *   November 11, 2024
 * @Abstract:
 *
 *
 **************************************************************************************************/


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "GPCM3_FM1.h"


/*---------------------------------------------------------------------------------------
 * Subroutine Area
 *---------------------------------------------------------------------------------------*/

/**
 * @brief
 *
 * @param

 * @return
 *   None.
 */
uint16_t FW_Open(void)
{
	uint16_t iCount;

	for(iCount=0;iCount<256;iCount++)
	{
    asm("NOP");
	}
	return iCount;
}

