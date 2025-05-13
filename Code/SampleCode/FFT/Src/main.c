/************************************************************************************
 *  File:     main.c
 *  Purpose:  GPCM3/GPFM1 1.	FFT(Fast Fourier Transform) Example Code
 *            Replace with your code.
 *  Date:     18 February 2025
 *  Info:     Output:
 *            UART IOA[14:13]
 *
 *
 *
 ************************************************************************************/

/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include <stdio.h>
#include "GPCM3_FM1.h"
#include "FFT_User.h"
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

Aligned4B FFT_WORKING_RAM         FftWorkingRam;
Aligned4B int16_t  					FftBuffer[2 * FFT_FRAME_SIZE];

Aligned4B KEYSCAN_WORKING_RAM     KeyScanWorkingRam;
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
  GPIO_SetMode(GPIOA, 0x000000FF, GPIO_MODE_INPUT);     //IOA[7:0] Input pull-low
  GPIO_SetMode(GPIOA, 0x00000F00, GPIO_MODE_OUTPUT);     //IOA[11:8] Output low
  MODIFY_REG(GPIOA->OBUF, 0xFFFFFFFF, 0);

  GPIO_SetMode(GPIOB, 0xFFFFFFFF, GPIO_MODE_INPUT);     //IOB Input pull-low
  MODIFY_REG(GPIOB->OBUF, 0xFFFFFFFF, 0);

  GPIO_SetMode(GPIOC, 0xFFFFFFFF, GPIO_MODE_INPUT);     //IOC Input pull-low
  MODIFY_REG(GPIOC->OBUF, 0xFFFFFFFF, 0);

  GPIO_SetMode(GPIOD, 0xFFFFFFFF, GPIO_MODE_INPUT);     //IOD Input pull-low
  MODIFY_REG(GPIOD->OBUF, 0xFFFFFFFF, 0);

  // FFT initialize
	UART_Init(UART0, UART0_IOA13_14, 115200);
	FFT_Initial((FFT_WORKING_RAM*)&FftWorkingRam, FftBuffer, FFT_FRAME_SIZE);
  FFT_Start();

  while(1)
	{
		WDT_Clear();
	  FFT_ServiceLoop();

		if(FFT_CheckCpuOverload() != 0)
		{
			FFT_ClearCpuOverloadFlag();
		}
	}

  return 0;
}


/*---------------------------------------------------------------------------------------
 * IRQ Handler
 *---------------------------------------------------------------------------------------*/



