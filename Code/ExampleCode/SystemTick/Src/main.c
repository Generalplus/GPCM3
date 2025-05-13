/************************************************************************************
 *  File:     main.c
 *  Purpose:  GPCM3/GPFM1 Example Code
 *            Replace with your code.
 *  Date:     03 December 2024
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


  //System Tick initialize
  SET_BIT(CLOCK->AHBCKEN, CLOCK_AHBCKEN_SYSTICK_CLK_ENABLE);                             // System Tick clock enable
  SYSTICK->CTRL = SYSTICK_CTRL_CLK_SEL_HCLK | SYSTICK_CTRL_INT_ENABLE | SYSTICK_CTRL_CNT_ENABLE;
  SYSTICK->RELOAD = 0xFFF;
	NVIC_EnableIRQ(SysTick_IRQn);

  while(1)
  {
    WDT_Clear();
		KeyScan_ServiceLoop();

		ScanedKey = KeyScan_GetCh();
		switch(ScanedKey)
		{
      case 0x01:    // IOA24+VDD:
        break;

      case 0x02:    // IOA25+VDD:
        break;

		  case 0x04:    // IOA26+VDD:
        break;

		  case 0x08:    // IOA27+VDD:
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
 *   SysTick IRQ Handler
 * @param
 *   None.
 * @return
 *   None.
 */
void SysTick_Handler(void)
{
		XOR_BIT(GPIOA->OBUF, BIT9);                                   // Toggle IOA[9]
		SYSTICK->CTRL |= SYSTICK_CTRL_INT_FLAG;                        // Clear SysTick INT flag
}


