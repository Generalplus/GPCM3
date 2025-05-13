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
  // GPIO Initialize
  GPIO_SetMode(GPIOA, 0xFFFFFFFF, GPIO_MODE_INPUT);     //IOA[31:0] Input pull-low, bypass IOA[14:13] UART & IOA.8 Output
  MODIFY_REG(GPIOA->OBUF, 0xFFFFFFFF, 0);

  GPIO_SetMode(GPIOB, 0xFFFFFFFF, GPIO_MODE_INPUT);     //IOB Input pull-low
  MODIFY_REG(GPIOB->OBUF, 0xFFFFFFFF, 0);

  GPIO_SetMode(GPIOC, 0xFFFFFFFF, GPIO_MODE_INPUT);     //IOC Input pull-low
  MODIFY_REG(GPIOC->OBUF, 0xFFFFFFFF, 0);

  GPIO_SetMode(GPIOD, 0xFFFFFFFF, GPIO_MODE_INPUT);     //IOD Input pull-low
  MODIFY_REG(GPIOD->OBUF, 0xFFFFFFFF, 0);
  KeyScan_Initial(&KeyScanWorkingRam);                  // Key scan init

  /*
   * IOA[18:9] output low
	 */
  GPIO_SetMode(GPIOA, 0x0007FE00, GPIO_MODE_OUTPUT);
  MODIFY_REG(GPIOA->OBUF, 0x0007FE00, 0);

  // TIMEBASE_Open(TB_CLK_SEL_IOSC32K);
  TIMEBASE_Open(TB_CLK_SEL_XTAL32K);
	TIMEBASE_EnableInt(TB_INT_2HZ);
	NVIC_EnableIRQ(TIMEBASE_IRQn);

  while(1)
  {
    WDT_Clear();
		KeyScan_ServiceLoop();

		ScanedKey = KeyScan_GetCh();
		switch(ScanedKey)
		{
      case 0x01:                                                                              // IOA0+VDD: Open TB. Clock source from IOSC32K.
			  TIMEBASE_Open(TB_CLK_SEL_IOSC32K);
        break;

      case 0x02:
        TIMEBASE_Open(TB_CLK_SEL_XTAL32K);
        break;
		  case 0x04:                                                                              // IOA2+VDD: Enable TB 2Hz interrupt.
			  TIMEBASE_EnableInt(TB_INT_2HZ);
        break;

		  case 0x08:                                                                              // IOA3+VDD: Enable TB 16Hz/64Hz interrupt.
				TIMEBASE_EnableInt(TB_INT_16HZ | TB_INT_64HZ);
        break;

		  case 0x10:                                                                              // IOA4+VDD: Enable TB 512Hz/2KHz/4KHz interrupt.
				TIMEBASE_EnableInt(TB_INT_512HZ | TB_INT_2KHZ | TB_INT_4KHZ);
        break;

      case 0x20:                                                                              // IOA5+VDD: Disable TB all interrupt.
				TIMEBASE_DisableInt(TB_INT_2HZ | TB_INT_16HZ | TB_INT_64HZ | TB_INT_512HZ | TB_INT_2KHZ | TB_INT_4KHZ);
        break;

			case 0x40:                                                                              // IOA6+VDD: Reset TB count.
			  TIMEBASE_Reset();
			  break;

      case 0x80:                                                                              // IOA7+VDD: Close TB.
				TIMEBASE_Close();
        break;

    }

  }
  return 0;
}

/*---------------------------------------------------------------------------------------
 * IRQ Handler
 *---------------------------------------------------------------------------------------*/
/**
 * @brief
 *   TimeBase IRQ Handler
 * @param
 *   None.
 * @return
 *   None.
 */
void TIMEBASE_IRQHandler(void)
{
	if((TIMEBASE->STS & TIMEBASE_STS_0D125HZ_INTF_FLAG) != 0)       // TB_0.125Hz
	{
		XOR_BIT(GPIOA->OBUF, BIT9);                                   // Toggle IOA[9]
		TIMEBASE->STS = TIMEBASE_STS_0D125HZ_INTF_FLAG;               // Clear TB INT flag
	}

	if((TIMEBASE->STS & TIMEBASE_STS_0D25HZ_INTF_FLAG) != 0)        // TB_0.25Hz
	{
		XOR_BIT(GPIOA->OBUF, BIT10);                                   // Toggle IOA[10]
		TIMEBASE->STS = TIMEBASE_STS_0D25HZ_INTF_FLAG;                // Clear TB INT flag
	}

	if((TIMEBASE->STS & TIMEBASE_STS_0D5HZ_INTF_FLAG) != 0)         // TB_0.5Hz
	{
		XOR_BIT(GPIOA->OBUF, BIT11);                                  // Toggle IOA[11]
		TIMEBASE->STS = TIMEBASE_STS_0D5HZ_INTF_FLAG;                 // Clear TB INT flag
	}

	if((TIMEBASE->STS & TIMEBASE_STS_1HZ_INTF_FLAG) != 0)           // TB_1Hz
	{
		XOR_BIT(GPIOA->OBUF, BIT12);                                  // Toggle IOA[12]
		TIMEBASE->STS = TIMEBASE_STS_1HZ_INTF_FLAG;                   // Clear TB INT flag
	}

	if((TIMEBASE->STS & TIMEBASE_STS_2HZ_INTF_FLAG) != 0)           // TB_2Hz
	{
		XOR_BIT(GPIOA->OBUF, BIT13);                                  // Toggle IOA[13]
		TIMEBASE->STS = TIMEBASE_STS_2HZ_INTF_FLAG;                   // Clear TB INT flag
	}

	if((TIMEBASE->STS & TIMEBASE_STS_16HZ_INTF_FLAG) != 0)          // TB_16Hz
	{
		XOR_BIT(GPIOA->OBUF, BIT14);                                  // Toggle IOA[14]
		TIMEBASE->STS = TIMEBASE_STS_16HZ_INTF_FLAG;                  // Clear TB INT flag
	}

	if((TIMEBASE->STS & TIMEBASE_STS_64HZ_INTF_FLAG) != 0)          // TB_64Hz
	{
		XOR_BIT(GPIOA->OBUF, BIT15);                                  // Toggle IOA[15]
		TIMEBASE->STS = TIMEBASE_STS_64HZ_INTF_FLAG;                  // Clear TB INT flag
	}

	if((TIMEBASE->STS & TIMEBASE_STS_512HZ_INTF_FLAG) != 0)          // TB_512Hz
	{
		XOR_BIT(GPIOA->OBUF, BIT16);                                   // Toggle IOA[16]
		TIMEBASE->STS = TIMEBASE_STS_512HZ_INTF_FLAG;                  // Clear TB INT flag
	}

	if((TIMEBASE->STS & TIMEBASE_STS_2KHZ_INTF_FLAG) != 0)           // TB_2KHz
	{
		XOR_BIT(GPIOA->OBUF, BIT17);                                   // Toggle IOA[17]
		TIMEBASE->STS = TIMEBASE_STS_2KHZ_INTF_FLAG;                   // Clear TB INT flag
	}

	if((TIMEBASE->STS & TIMEBASE_STS_4KHZ_INTF_FLAG) != 0)          // TB_4KHz
	{
		XOR_BIT(GPIOA->OBUF, BIT18);                                   // Toggle IOA[18]
		TIMEBASE->STS = TIMEBASE_STS_4KHZ_INTF_FLAG;                   // Clear TB INT flag
	}
}


