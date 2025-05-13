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
	 * IOA[12:8] output low
	 */
  GPIO_SetMode(GPIOA, 0x00001F00, GPIO_MODE_OUTPUT);
  MODIFY_REG(GPIOA->OBUF, 0x00001F00, 0);

  while(1)
  {
    WDT_Clear();
		KeyScan_ServiceLoop();

		ScanedKey = KeyScan_GetCh();
		switch(ScanedKey)
		{
		  case 0x01:                                 // IOA0+VDD: Setting Timer0 interrupt frequenct = 8Khz
			  TIMER_Open(TM0);
			  TIMER_SetFreq(TM0, 8000);
			  TIMER_EnableInt(TM0);
			  NVIC_EnableIRQ(TIMER0_IRQn);
        break;

		  case 0x02:                                 // IOA1+VDD: Setting Timer1 interrupt frequenct = 16Khz
			  TIMER_Open(TM1);
			  TIMER_SetFreq(TM1, 16000);
			  TIMER_EnableInt(TM1);
			  NVIC_EnableIRQ(TIMER1_IRQn);
        break;

		  case 0x04:                                 // IOA2+VDD: Setting Timer2 interrupt frequenct = 10hz
			  TIMER_Open(TM2);
			  TIMER_SetClkIn(TM2, TM_CLK_SEL_32K);
			  TIMER_SetFreq(TM2, 10);
			  TIMER_EnableInt(TM2);
			  NVIC_EnableIRQ(TIMER2_IRQn);
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
 *   Timer0 IRQ handler
 * @param
 *   None.
 * @return
 *   None.
 */
void TIMER0_IRQHandler(void)
{
	XOR_BIT(GPIOA->OBUF, BIT8);                    // Toggle IOA.8
	TM_INT->INTSTS = TIMER_INTSTS_TM0_INT_FLAG;    //	Clear Timer0 intterupt flag
}

/**
 * @brief
 *   Timer1 IRQ handler
 * @param
 *   None.
 * @return
 *   None.
 */
void TIMER1_IRQHandler(void)
{
	XOR_BIT(GPIOA->OBUF, BIT9);                    // Toggle IOA.9
	TM_INT->INTSTS = TIMER_INTSTS_TM1_INT_FLAG;    //	Clear Timer1 intterupt flag
}

/**
 * @brief
 *   Timer2 IRQ handler
 * @param
 *   None.
 * @return
 *   None.
 */
void TIMER2_IRQHandler(void)
{
	XOR_BIT(GPIOA->OBUF, BIT10);                   // Toggle IOA.10
	TM_INT->INTSTS = TIMER_INTSTS_TM2_INT_FLAG;    //	Clear Timer2 intterupt flag
}


