/************************************************************************************
 *  File:     main.c
 *  Purpose:  GPCM3/GPFM1 Envelope Detect Example Code
 *            Replace with your code.
 *  Date:     12 February 2025
 *  Info:     Output:
 *            Envelope detect: IOA.14
 *
 *
 ************************************************************************************/

/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include <stdio.h>
#include "GPCM3_FM1.h"
#include "EnvDet_User.h"
#include "KeyScan.h"


/*---------------------------------------------------------------------------------------
 * Variable Declaration Area
 *---------------------------------------------------------------------------------------*/
#if defined(__CC_ARM)
#define Aligned4B __align(4)
#elif defined(__GNUC__)
extern unsigned int __boot_table_start;
#define Aligned4B __attribute__ ((aligned (4)))
#endif

Aligned4B KEYSCAN_WORKING_RAM     KeyScanWorkingRam;
Aligned4B ENVDET_WORKING_RAM 	EnvDetWorkingRam;
uint32_t ScanedKey;

/*---------------------------------------------------------------------------------------
 * Subroutine Area
 *---------------------------------------------------------------------------------------*/
  uint32_t SWTrim_CheckBusy(void)
  {
    return (CLOCK->SWTRIM & CLOCK_SWTRIM_COUNTER_RDY);
  }


/*---------------------------------------------------------------------------------------
 * Main Function
 *---------------------------------------------------------------------------------------*/
int main()
{
	uint16_t temp;

  // GPIO Initialize
  GPIO_SetMode(GPIOA, 0x000000FF, GPIO_MODE_INPUT);     //IOA[7:0] Input pull-low
  GPIO_SetMode(GPIOA, 0x0000FF00, GPIO_MODE_OUTPUT);     //IOA[15:8] Output low
  MODIFY_REG(GPIOA->OBUF, 0xFFFFFFFF, 0);

  GPIO_SetMode(GPIOB, 0xFFFFFFFF, GPIO_MODE_INPUT);     //IOB Input pull-low
  MODIFY_REG(GPIOB->OBUF, 0xFFFFFFFF, 0);

  GPIO_SetMode(GPIOC, 0xFFFFFFFF, GPIO_MODE_INPUT);     //IOC Input pull-low
  MODIFY_REG(GPIOC->OBUF, 0xFFFFFFFF, 0);

  GPIO_SetMode(GPIOD, 0xFFFFFFFF, GPIO_MODE_INPUT);     //IOD Input pull-low
  MODIFY_REG(GPIOD->OBUF, 0xFFFFFFFF, 0);

	/*
	 * EnvDet Init
	 */
	EnvDet_Initial(&EnvDetWorkingRam);
	EnvDet_SetAttackLevel(0x0800);
	EnvDet_SetReleaseLevel(0x0600);
	EnvDet_SetAttackTime(0x007F);
	EnvDet_SetReleaseTime(0x1000);
	EnvDet_Start();

  while(1)
	{
		WDT_Clear();
		temp = EnvDet_GetStatus();
		if(READ_BIT (temp, C_EnvDet_AttackActive))
		{
				GPIOA_OBIT->OBIT14 = 0xFFFFFFFF;
		}
		else if(READ_BIT(temp, C_EnvDet_ReleaseActive))
		{
				GPIOA_OBIT->OBIT14 = 0x00000000;
		}
	}

  return 0;
}


/*---------------------------------------------------------------------------------------
 * IRQ Handler
 *---------------------------------------------------------------------------------------*/



