/************************************************************************************
 *  File:     main.c
 *  Purpose:  GPCM3/GPFM1 Example Code
 *            Replace with your code.
 *  Date:     22 July 2024
 *  Info:     CTS Timer is used as a general timer
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
 * Constant Definitions
 *---------------------------------------------------------------------------------------*/
 //#define CTS_TMADATA				0xFF00
 #define CTS_TMADATA				  0x00FF
 #define CTS_TMBDATA				  0x8000


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

  void CTS_Timer_Initial(void)
  {
    SET_BIT(CLOCK->APBCKEN, CLOCK_APBCKEN_CTS_CLK_ENABLE);

    WRITE_REG(CTS->TMAPLOAD, CTS_TMADATA);
    WRITE_REG(CTS->TMACOUNT, CTS_TMADATA);
    WRITE_REG(CTS->TMBPLOAD, CTS_TMBDATA);
    WRITE_REG(CTS->TMBCOUNT, CTS_TMBDATA);

    NVIC_EnableIRQ(CTS_TM0_IRQn);               // Enable CTS_TMA INT
    NVIC_EnableIRQ(CTS_TM1_IRQn);               // Enable CTS_TMB INT

    WRITE_REG(CTS->CTRL, (CTS_CTRL_CTSEN_ENABLE | CTS_CTRL_AUTOSTOP_ENABLE | CTS_CTRL_TMA_INTEN_SEL_ENABLE | CTS_CTRL_TMB_INTEN_SEL_ENABLE	| CTS_CTRL_TMA_MODE_GENERALTMR | CTS_CTRL_TMB_MODE_GENERALTMR
                  | CTS_CTRL_TMA_CLK_SEL_HCLK_DIV_8 | CTS_CTRL_TMB_CLK_SEL_HCLK_DIV_4));

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

  CTS_Timer_Initial();

  while(1)
  {
    WDT_Clear();
		KeyScan_ServiceLoop();

		ScanedKey = KeyScan_GetCh();
		switch(ScanedKey)
		{
		  case 0x01:                                 // IOA0+VDD: Setting Timer0 interrupt frequenct = 8Khz
        break;

		  case 0x02:                                 // IOA1+VDD: Setting Timer1 interrupt frequenct = 16Khz

        break;

		  case 0x04:                                 // IOA2+VDD: Setting Timer2 interrupt frequenct = 10hz

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
 *
 * @param
 *          None.
 * @return
 *          None.
 */
void CTS_TM0_IRQHandler(void)
{
		WRITE_REG(CTS->TMBCOUNT, CTS_TMBDATA);										 // Re-load TMB Data
    XOR_BIT(GPIOA->OBUF, BIT9);

		CLEAR_FLAG(CTS->STS, CTS_STS_TMA_INTF_MSK, CTS_STS_TMA_INTF_FLAG);
}

/**
 * @brief
 *
 * @param
 *          None.
 * @return
 *          None.
 */
void CTS_TM1_IRQHandler(void)
{
    XOR_BIT(GPIOA->OBUF, BIT10);
		CLEAR_FLAG(CTS->STS, CTS_STS_TMB_INTF_MSK, CTS_STS_TMB_INTF_FLAG);
}



