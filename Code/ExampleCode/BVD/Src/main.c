/************************************************************************************
 *  File:     main.c
 *  Purpose:  GPCM3/GPFM1 Example Code
 *  Note: The BVD is to detect the voltage of the VDDIO pin
 *  IOA[15:12] Output pins and to show the BVD result.
 *        0000: VDD≤2.0V
 *        0001: 2.0V<VDD<2.2V
 *        0010: 2.2V<VDD<2.4V
 *        0011: 2.4V<VDD<2.6V
 *        0100: 2.6V<VDD<2.8V
 *        0101: 2.8V<VDD<3.0V
 *        0110: 3.0V<VDD<3.3V
 *        0111: 3.3V<VDD<3.6V
 *        1000: VDD>3.6V
 *  Date: 17 April 2023
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
uint32_t BVDResult;


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
  // GPIO Initialize
  GPIO_SetMode(GPIOA, 0xFFFFFFFF, GPIO_MODE_INPUT);     //IOA[31:0] Input pull-low
  MODIFY_REG(GPIOA->OBUF, 0xFFFFFFFF, 0);
  GPIO_SetMode(GPIOC, 0xFFFFFFFF, GPIO_MODE_INPUT);     //IOC Input pull-low
  MODIFY_REG(GPIOC->OBUF, 0xFFFFFFFF, 0);
  GPIO_SetMode(GPIOD, 0xFFFFFFFF, GPIO_MODE_INPUT);     //IOD Input pull-low
  MODIFY_REG(GPIOD->OBUF, 0xFFFFFFFF, 0);

  KeyScan_Initial(&KeyScanWorkingRam);                  // Key scan init

  /*
	 * BVD init
	 */
  SET_BIT(ACU->BVD_CTRL, ACU_BVD_CTRL_ENABLE);

  /*
	 * IOA[15:12] output low
	 */
  GPIO_SetMode(GPIOA, 0x0000F000, GPIO_MODE_OUTPUT);
  MODIFY_REG(GPIOA->OBUF, 0x0000F000, 0);

  while(1)
  {
    WDT_Clear();

		BVDResult = READ_REG(ACU->BVD_CTRL);
		BVDResult &= 0x0000000F;
		BVDResult = BVDResult << 12;
		MODIFY_REG(GPIOA->OBUF, 0x0000F000, BVDResult);

		KeyScan_ServiceLoop();

		ScanedKey = KeyScan_GetCh();
		switch(ScanedKey)
		{
      case 0x01:                                                                              // IOA0+VDD: Open TB. Clock source from IOSC32K.

        break;

      case 0x02:
        break;
		  case 0x04:                                                                              // IOA2+VDD: Enable TB 2Hz interrupt.
        break;

		  case 0x08:                                                                              // IOA3+VDD: Enable TB 16Hz/64Hz interrupt.
        break;

		  case 0x10:                                                                              // IOA4+VDD: Enable TB 512Hz/2KHz/4KHz interrupt.
        break;

      case 0x20:                                                                              // IOA5+VDD: Disable TB all interrupt.
        break;

			case 0x40:                                                                              // IOA6+VDD: Reset TB count.
			  break;

      case 0x80:                                                                              // IOA7+VDD: Close TB.
        break;

    }

  }
  return 0;
}

/*---------------------------------------------------------------------------------------
 * IRQ Handler
 *---------------------------------------------------------------------------------------*/


