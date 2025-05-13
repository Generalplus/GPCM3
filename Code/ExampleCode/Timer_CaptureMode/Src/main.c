/************************************************************************************
 *  File:     main.c
 *  Purpose:  GPCM3/GPFM1 Example Code
 *            Replace with your code.
 *  Date:     19 July 2024
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
uint16_t TimerCaptureValue;

/*---------------------------------------------------------------------------------------
 * Subroutine Area
 *---------------------------------------------------------------------------------------*/
  uint32_t SWTrim_CheckBusy(void)
  {
    return (CLOCK->SWTRIM & CLOCK_SWTRIM_COUNTER_RDY);
  }

   /**
   * @brief
   *    Enable TMx Capture interrupt
   * @param
   *   *TimerHandle [in]: The base address of Timer module
   *    - TM0, TM1, TM2
   * @return
   *   None.
   */
  void TIMER_Capture_EnableInt(TIMER_TYPE_DEF *TimerHandle)
  {
    if(TimerHandle == TM0)
    {
      TM_INT->INTEN |= TIMER_INTEN_TM0_CAPINT_ENABLE;
    }
    else if(TimerHandle == TM1)
    {
      TM_INT->INTEN |= TIMER_INTEN_TM1_CAPINT_ENABLE;
    }
    else if(TimerHandle == TM2)
    {
      TM_INT->INTEN |= TIMER_INTEN_TM2_CAPINT_ENABLE;
    }
  }


  void TimerCaptureInt(TIMER_TYPE_DEF *TimerHandler)
  {
    GPIO_SetMode(GPIOA, 0x00000080, GPIO_MODE_INPUT);                       // IOA[7] Input pull-low
    MODIFY_REG(GPIOA->OBUF, 0x00000080, 0);

    TIMER_DisableInt(TimerHandler);
    SET_BIT(CLOCK->APBCKEN, CLOCK_APBCKEN_TIMER_CLK_ENABLE);                // TM clk enable
    SET_BIT(TimerHandler->CTRL, TIMER_CTRL_TM_ENABLE);	                    // TMx Enabled
    SET_BIT(TimerHandler->CTRL, TIMER_CTRL_CAPMODE_ENABLE);	                // Enable capture mode
    CLEAR_BIT(TimerHandler->CTRL, TIMER_CTRL_CAP_ENDSW_RISE_EDGE);	        //	Capture end edge switch: Rising
    MODIFY_REG(TimerHandler->CTRL, TIMER_CTRL_CAP_LPF_SEL_MSK, TIMER_CTRL_CAP_LPF_DISABLE);  // LPF:
    MODIFY_REG(TimerHandler->CAPSRC, TIMER_CAPSRC_IOSEL_MSK, TIMER_CAPSRC_IOSEL_IOA7);
    TIMER_Capture_EnableInt(TimerHandler);
    if(TimerHandler == TM0)
    {
      NVIC_EnableIRQ(TIMER0_IRQn);
    }
    else if(TimerHandler == TM1)
    {
      NVIC_EnableIRQ(TIMER1_IRQn);
    }
    else
    {
      NVIC_EnableIRQ(TIMER2_IRQn);
    }

    SET_BIT(TimerHandler->CTRL, TIMER_CTRL_CAP_START);	                   //	Capture Start
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

  // Timer Capture mode Initialize
	TimerCaptureInt(TM0);
	TIMER_SetFreq(TM0, 8000);
  SET_BIT(TM0->CTRL, TIMER_CTRL_CAP_START);

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
		  case 0x01:                                 // IOA0+VDD:
        break;

		  case 0x02:                                 // IOA1+VDD:

        break;

		  case 0x04:                                 // IOA2+VDD:
        break;

		  case 0x80:                                 // IOA7+VDD:
        // SET_BIT(TM0->CTRL, TIMER_CTRL_CAP_START);	                   //	Capture Start
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
	XOR_BIT(GPIOA->OBUF, BIT9);                    // Toggle IOA.8
	if((TM_INT->INTSTS & TIMER_INTSTS_TM0_CAPINT_FLAG) != 0)
	{
		TimerCaptureValue = TM0->COUNT;
		SET_BIT(TM0->CTRL, TIMER_CTRL_TM_RELOAD);	                              //	Timer Counter Reload
    TM_INT->INTSTS = TIMER_INTSTS_TM0_CAPINT_FLAG;
	}
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
	TM_INT->INTSTS = TIMER_INTSTS_TM1_INT_FLAG | TIMER_INTSTS_TM1_CAPINT_FLAG;    //	Clear Timer1 intterupt flag
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
	TM_INT->INTSTS = TIMER_INTSTS_TM2_INT_FLAG | TIMER_INTSTS_TM2_CAPINT_FLAG;    //	Clear Timer2 intterupt flag
}


