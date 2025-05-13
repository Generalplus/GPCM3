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
	 * CCPx Capture init
	 */
	CCP0_Capture_Initial(IoSel_IOA13_16, CAP_TRG_RISING);
	CCP1_Capture_Initial(IoSel_IOA26_29, CAP_TRG_RISING);

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
      case 0x01:                                                                              // IOA0+VDD:
        CCP0_Timer_Reload(CCP0_CAPTMR_PLOAD);    // Timer counter reset
        break;

		  case 0x02:                                                                              // IOA1+VDD:
        CCP0_Close();
        CCP1_Close();
        break;

		  case 0x04:                                                                              // IOA2+VDD:
        CCP0_CAPx_Ctrl(CCP_CAPCTRL_CAP0_ENABLE | CCP_CAPCTRL_CAP1_ENABLE);    // Enable CCP0 Pin.0 & 1 and disable Pin.2 & 3
        CCP1_CAPx_Ctrl(CCP_CAPCTRL_CAP2_ENABLE | CCP_CAPCTRL_CAP3_ENABLE);    // Enable CCP1 Pin.2 & 3 and disable Pin.0 & 1
        break;

		  case 0x08:                                                                              // IOA3+VDD:
        break;

		  case 0x10:                                                                              // IOA4+VDD:

        break;

      case 0x20:                                                                              // IOA5+VDD:
        break;

			case 0x40:                                                                              // IOA6+VDD:
			  break;

      case 0x80:                                                                              // IOA7+VDD:
        break;
		}
	}
}

/*---------------------------------------------------------------------------------------
 * IRQ Handler
 *---------------------------------------------------------------------------------------*/
/**
 * @brief
 *   CCP0 IRQ Handler
 * @param
 *   None.
 * @return
 *   None.
 */
void CCP0_IRQHandler(void)
{
	uint32_t CCPTimer_CurValue;

	if(READ_BIT(CCP0->INTSTS, CCP_INTEN_TMR_INT_FLAG))          // CCP0_Timer INT
	{
    XOR_BIT(GPIOA->OBUF, BIT8);                               // Toggle IOA.8
		CLEAR_FLAG(CCP0->INTSTS, CCP_INTEN_TMR_INTF_MSK, CCP_INTEN_TMR_INT_FLAG);
	}

	if(READ_BIT(CCP0->INTSTS, CCP_INTSTS_CAP0_INT_FLAG))        // CCP0_CAP0 INT
	{
    XOR_BIT(GPIOA->OBUF, BIT9);                               // Toggle IOA.9
		CCPTimer_CurValue = READ_REG(CCP0->CCPR0);
    CCP0_Timer_Reload(CCP0_CAPTMR_PLOAD);                                      // Reset Timer
		CLEAR_FLAG(CCP0->INTSTS, CCP_INTSTS_CAP0_INTF_MSK, CCP_INTSTS_CAP0_INT_FLAG);
	}

	if(READ_BIT(CCP0->INTSTS, CCP_INTSTS_CAP1_INT_FLAG))        // CCP0_CAP1 INT
	{
    XOR_BIT(GPIOA->OBUF, BIT10);                               // Toggle IOA.10
		CCPTimer_CurValue = READ_REG(CCP0->CCPR1);

		CLEAR_FLAG(CCP0->INTSTS, CCP_INTSTS_CAP1_INTF_MSK, CCP_INTSTS_CAP1_INT_FLAG);
	}

	if(READ_BIT(CCP0->INTSTS, CCP_INTSTS_CAP2_INT_FLAG))        // CCP0_CAP2 INT
	{
    XOR_BIT(GPIOA->OBUF, BIT11);                               // Toggle IOA.11
		CCPTimer_CurValue = READ_REG(CCP0->CCPR2);

		CLEAR_FLAG(CCP0->INTSTS, CCP_INTSTS_CAP2_INTF_MSK, CCP_INTSTS_CAP2_INT_FLAG);
	}

	if(READ_BIT(CCP0->INTSTS, CCP_INTSTS_CAP3_INT_FLAG))        // CCP0_CAP3 INT
	{
    XOR_BIT(GPIOA->OBUF, BIT12);                               // Toggle IOA.12
		CCPTimer_CurValue = READ_REG(CCP0->CCPR3);

		CLEAR_FLAG(CCP0->INTSTS, CCP_INTSTS_CAP3_INTF_MSK, CCP_INTSTS_CAP3_INT_FLAG);
	}

}

/**
 * @brief
 *   CCP1 IRQ Handler
 * @param
 *   None.
 * @return
 *   None.
 */
void CCP1_IRQHandler(void)
{
	uint32_t CCPTimer_CurValue;

	if(READ_BIT(CCP1->INTSTS, CCP_INTEN_TMR_INT_FLAG))          // CCP1_Timer INT
	{
		CLEAR_FLAG(CCP1->INTSTS, CCP_INTEN_TMR_INTF_MSK, CCP_INTEN_TMR_INT_FLAG);
	}

	if(READ_BIT(CCP1->INTSTS, CCP_INTSTS_CAP0_INT_FLAG))        // CCP1_CAP0 INT
	{
		CCPTimer_CurValue = READ_REG(CCP1->CCPR0);
    CCP1_Timer_Reload(CCP1_CAPTMR_PLOAD);                                      // Reset Timer
		CLEAR_FLAG(CCP1->INTSTS, CCP_INTSTS_CAP0_INTF_MSK, CCP_INTSTS_CAP0_INT_FLAG);
	}

	if(READ_BIT(CCP1->INTSTS, CCP_INTSTS_CAP1_INT_FLAG))        // CCP1_CAP1 INT
	{
		CCPTimer_CurValue = READ_REG(CCP1->CCPR1);

		CLEAR_FLAG(CCP1->INTSTS, CCP_INTSTS_CAP1_INTF_MSK, CCP_INTSTS_CAP1_INT_FLAG);
	}

	if(READ_BIT(CCP1->INTSTS, CCP_INTSTS_CAP2_INT_FLAG))        // CCP1_CAP2 INT
	{
		CCPTimer_CurValue = READ_REG(CCP1->CCPR2);

		CLEAR_FLAG(CCP1->INTSTS, CCP_INTSTS_CAP2_INTF_MSK, CCP_INTSTS_CAP2_INT_FLAG);
	}

	if(READ_BIT(CCP1->INTSTS, CCP_INTSTS_CAP3_INT_FLAG))        // CCP1_CAP3 INT
	{
		CCPTimer_CurValue = READ_REG(CCP1->CCPR3);

		CLEAR_FLAG(CCP1->INTSTS, CCP_INTSTS_CAP3_INTF_MSK, CCP_INTSTS_CAP3_INT_FLAG);
	}

}
