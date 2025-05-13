/************************************************************************************
 *  File:     main.c
 *  Purpose:  GPCM3/GPFM1 Example Code
 *            Replace with your code.
 *  Date:     01 February 2023
 *  Info:
 *
 *
 ************************************************************************************/

/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include <stdio.h>
#include "GPCM3_FM1.h"
#include "CCP_GPCM3_FM1.h"
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
uint16_t PWMIODuty = 0;


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

  // EXT0/1/2/3 INT enabled.
	GPIO_ExtInterrupt_Init(EXT_INT0, EXT_INT_RISING_EDGE_TRIG, EXT_INT_SEL_IOA0);
	GPIO_ExtInterrupt_Init(EXT_INT1, EXT_INT_RISING_EDGE_TRIG, EXT_INT_SEL_IOA1);
  GPIO_ExtInterrupt_Init(EXT_INT2, EXT_INT_RISING_EDGE_TRIG, EXT_INT_SEL_IOA2);
	GPIO_ExtInterrupt_Init(EXT_INT3, EXT_INT_RISING_EDGE_TRIG, EXT_INT_SEL_IOA3);

	GPIO_ExtInterrupt_Enable(EXT_INT0);
	GPIO_ExtInterrupt_Enable(EXT_INT1);
	GPIO_ExtInterrupt_Enable(EXT_INT2);
	GPIO_ExtInterrupt_Enable(EXT_INT3);

	NVIC_EnableIRQ(EXTI_IRQn);

  while(1)
  {
    WDT_Clear();

  }
  return 0;
}

/*---------------------------------------------------------------------------------------
 * IRQ Handler
 *---------------------------------------------------------------------------------------*/
/*
 * @brief
 *  				Cortex-M0 Processor Interrupt & Exception Handlers
 * @param
 *          None.
 * @return
 *          None.
 */
void EXTI_IRQHandler(void)
{
	if((ITU->EXTIFLG & ITU_EXTIFLG_EXT0_INT_FLAG) != 0)
	{
    XOR_BIT(GPIOA->OBUF, BIT8);	                              // Toggle IOA[8]
    ITU->EXTIFLG = ITU_EXTIFLG_EXT0_INT_FLAG;                 // Clear EXT0 INT flag
	}

	if((ITU->EXTIFLG & ITU_EXTIFLG_EXT1_INT_FLAG) != 0)
	{
    XOR_BIT(GPIOA->OBUF, BIT9);	                              // Toggle IOA[9]
    ITU->EXTIFLG = ITU_EXTIFLG_EXT1_INT_FLAG;                 // Clear EXT1 INT flag
	}

	if((ITU->EXTIFLG & ITU_EXTIFLG_EXT2_INT_FLAG) != 0)
	{
    XOR_BIT(GPIOA->OBUF, BIT10);	                            // Toggle IOA[10]
    ITU->EXTIFLG = ITU_EXTIFLG_EXT2_INT_FLAG;                 // Clear EXT2 INT flag
	}

	if((ITU->EXTIFLG & ITU_EXTIFLG_EXT3_INT_FLAG) != 0)
	{
    XOR_BIT(GPIOA->OBUF, BIT11);	                            // Toggle IOA[11]
    ITU->EXTIFLG = ITU_EXTIFLG_EXT3_INT_FLAG;                 // Clear EXT3 INT flag
	}
}

