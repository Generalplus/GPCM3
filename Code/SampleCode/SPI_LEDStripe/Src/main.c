/************************************************************************************
 *  File:     main.c
 *  Purpose:  GPCM3/GPFM1 LED stripe demo
 *            Replace with your code.
 *  Date:     13 May 2024
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
 * Definition Area
 *---------------------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#if defined(__CC_ARM)
#define Aligned4B __align(4)
__align(4) KEYSCAN_WORKING_RAM KeyScanWorkingRam;
#elif defined(__GNUC__)
extern unsigned int __boot_table_start; //in GPCM2_CM3.ld
#define Aligned4B __attribute__ ((aligned (4)))
__attribute__ ((aligned (4))) KEYSCAN_WORKING_RAM KeyScanWorkingRam;
#endif
KEYSCAN_WORKING_RAM 										KeyScanWorkingRam;

uint32_t ScanedKey;

extern uint8_t __user_spifc_load_addr;
extern uint8_t __user_spifc_start;
extern uint32_t __user_spifc_size;


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
	WRITE_REG(RCU->STS,0x0000FFFF);                             // Clear Reset status
  MoveSpifcRamCode();

  // GPIO initialize
  GPIO_SetMode(GPIOA, 0xFFFFFFFF, GPIO_MODE_INPUT);     //IOA[31:0] Input pull-low, bypass IOA[14:13] UART & IOA.8 Output
  MODIFY_REG(GPIOA->OBUF, 0xFFFFFFFF, 0);

  GPIO_SetMode(GPIOB, 0xFFFFFFFF, GPIO_MODE_INPUT);     //IOB Input pull-low
  MODIFY_REG(GPIOB->OBUF, 0xFFFFFFFF, 0);

  GPIO_SetMode(GPIOC, 0xFFFFFFFF, GPIO_MODE_INPUT);     //IOC Input pull-low
  MODIFY_REG(GPIOC->OBUF, 0xFFFFFFFF, 0);

  GPIO_SetMode(GPIOD, 0xFFFFFFFF, GPIO_MODE_INPUT);     //IOD Input pull-low
  MODIFY_REG(GPIOD->OBUF, 0xFFFFFFFF, 0);

	KeyScan_Initial(&KeyScanWorkingRam);                    // key scan init.

	/*
	 * LED stripe init
	 */
	LEDStripe_SPI_Initial();
	LEDStripe_SPI1_Start();

  while(1)
	{
    LEDStripe_SPI_MainLoopService();
		KeyScan_ServiceLoop();
		WDT_Clear();

		ScanedKey = KeyScan_GetCh();
	  switch(ScanedKey)
		{
		  case 0x01:                                 						// IOA0+VDD:
        TIMEBASE_EnableInt(TB_INT_512HZ);
				LEDStripe_SPI1_Start();
        break;

			case 0x02:                                 						// IOA1+VDD:
        TIMEBASE_DisableInt(TB_INT_512HZ);
				LEDStripe_SPI1_Stop();
        LEDStripe_SPI1_ClearAll();
				break;

			case 0x04:                                 						// IOA2+VDD:

			  break;

			case 0x08:                                 						// IOA3+VDD: Playback recording voice
				break;

			case 0x10:                                 						// IOA4+VDD: Play Midi Note
				break;

      case 0x20:                                 						// IOA5+VDD: Stop
				break;

			case 0x40:                                 						// IOA6+VDD: Volume Up
			  break;

			case 0x80:                                 						// IOA7+VDD: Volume Dn
				break;
	  }
	}
  return 0;
}


void MoveSpifcRamCode(void)
{
    uint8_t *pCode_LDA = &__user_spifc_load_addr;
    uint8_t *pCode_VMA = &__user_spifc_start;
    uint32_t l_CodeSize = (uint32_t)&__user_spifc_size;
    uint32_t iCount;

    for(iCount=0;iCount<l_CodeSize;iCount++)
    {
      pCode_VMA[iCount] = pCode_LDA[iCount];
    }

    return;
}

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
	if((TIMEBASE->STS & TIMEBASE_STS_2HZ_INTF_FLAG) != 0)
	{
    TIMEBASE->STS = TIMEBASE_STS_2HZ_INTF_FLAG;	                                              // Clear TimeBase 2Hz interrupt flag
	}

	if((TIMEBASE->STS & TIMEBASE_STS_16HZ_INTF_FLAG) != 0)
	{
		TIMEBASE->STS = TIMEBASE_STS_16HZ_INTF_FLAG;                                              // Clear TimeBase 16Hz interrupt flag
	}

	if((TIMEBASE->STS & TIMEBASE_STS_64HZ_INTF_FLAG) != 0)
	{
		TIMEBASE->STS = TIMEBASE_STS_64HZ_INTF_FLAG;                                              // Clear TimeBase 64Hz interrupt flag
	}

	if((TIMEBASE->STS & TIMEBASE_STS_512HZ_INTF_FLAG) != 0)
	{
    LEDStripe_SPI_FrameServiceIsr();
		TIMEBASE->STS = TIMEBASE_STS_512HZ_INTF_FLAG;	                                            // Clear TimeBase 64Hz interrupt flag
	}

	if((TIMEBASE->STS & TIMEBASE_STS_2KHZ_INTF_FLAG) != 0)
	{
		TIMEBASE->STS = TIMEBASE_STS_2KHZ_INTF_FLAG;                                              // Clear TimeBase 2KHz interrupt flag
	}

	if((TIMEBASE->STS & TIMEBASE_STS_4KHZ_INTF_FLAG) != 0)
	{
		TIMEBASE->STS = TIMEBASE_STS_4KHZ_INTF_FLAG;                                              // Clear TimeBase 4KHz interrupt flag
	}
}
