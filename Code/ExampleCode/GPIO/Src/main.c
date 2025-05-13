/************************************************************************************
 *  File:     main.c
 *  Purpose:  GPCM3/GPFM1 Example Code
 *            Replace with your code.
 *  Date:     28 December 2022
 *  Info:
 *
 *
 ************************************************************************************/

/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include <stdio.h>
#include "GPCM3_FM1.h"
#include "KeyScan.h"


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#if defined(__CC_ARM)
#define Aligned4B __align(4)
#elif defined(__GNUC__)
extern unsigned int __boot_table_start; //in GPCM3Test.ld
#define Aligned4B __attribute__ ((aligned (4)))
#endif

KEYSCAN_WORKING_RAM     KeyScanWorkingRam;
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
  // GPIO initialize
  GPIO_SetMode(GPIOA, 0xFFFFFFFF, GPIO_MODE_INPUT);         //IOA[31:0] Input pull-low, bypass IOA[14:13] UART & IOA.8 Output
  MODIFY_REG(GPIOA->OBUF, 0xFFFFFFFF, 0);
  GPIO_SetMode(GPIOB, 0xFFFFFFFF, GPIO_MODE_INPUT);         //IOB Input pull-low
  MODIFY_REG(GPIOB->OBUF, 0xFFFFFFFF, 0);
  GPIO_SetMode(GPIOC, 0xFFFFFFFF, GPIO_MODE_INPUT);         //IOC Input pull-low
  MODIFY_REG(GPIOC->OBUF, 0xFFFFFFFF, 0);
  GPIO_SetMode(GPIOD, 0xFFFFFFFF, GPIO_MODE_INPUT);         //IOD Input pull-low
  MODIFY_REG(GPIOD->OBUF, 0xFFFFFFFF, 0);

  GPIO_SetMode(GPIOA, 0x00FF0000, GPIO_MODE_OUTPUT);
  MODIFY_REG(GPIOA->OBUF, 0x00FF0000, 0x00000000);

  KeyScan_Initial(&KeyScanWorkingRam);                     // key scan init.

  while(1)
	{
		KeyScan_ServiceLoop();
		WDT_Clear();
		ScanedKey = KeyScan_GetCh();
	  switch(ScanedKey)
		{
			case 0x01:                                 						// IOA24+VDD:
        XOR_BIT(GPIOA->OBUF, BIT16);
			  break;

			case 0x02:                                 						// IOA25+VDD:
        XOR_BIT(GPIOA->OBUF, BIT17);
			  break;

			case 0x04:                                 						// IOA24+VDD:
        XOR_BIT(GPIOA->OBUF, BIT18);
			  break;

			case 0x08:                                 						// IOA25+VDD:
        XOR_BIT(GPIOA->OBUF, BIT19);
			  break;

			case 0x10:                                 						// IOA24+VDD:
        XOR_BIT(GPIOA->OBUF, BIT20);
			  break;

			case 0x20:                                 						// IOA25+VDD:
        XOR_BIT(GPIOA->OBUF, BIT21);
			  break;

			case 0x40:                                 						// IOA24+VDD:
        XOR_BIT(GPIOA->OBUF, BIT22);
			  break;

			case 0x80:                                 						// IOA25+VDD:
        XOR_BIT(GPIOA->OBUF, BIT23);
			  break;
    }
  }
  return 0;
}


