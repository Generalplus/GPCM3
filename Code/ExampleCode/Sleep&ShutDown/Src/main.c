/************************************************************************************
 *  File:     main.c
 *  Purpose:  GPCM3 example code for sleep & shut down mode.
 *
 *  Date:     13 January 2023
 *  Info:     IOA.0 Key:    Shut down mode & key wake up.
 *            IOA.1 Key:    Shut down mode & Iosc16KHz wake up.
 *            IOA.2 Key:    Deep sleep mode & key wake up.
 *            IOA.3 Key:    Sleep mode & key wake up.
 *            IOA.4 Key:    Halt mode &Timebase 32K @Iosc clk src.
 *            IOA.5 Key:    Halt mode &Timebase 32K @X'TAL clk src.
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
// void ShowMainTestList(void);

// extern void DmaIsr_Dma0Trigger(void);
// extern void DmaIsr_DMA0(void);

#if defined(__GNUC__)
extern unsigned int __boot_table_start; //in GPCM3Test.ld
__attribute__ ((aligned (4))) KEYSCAN_WORKING_RAM KeyScanWorkingRam;
#endif

extern uint8_t  __user_spifc_load_addr;
extern uint8_t  __user_spifc_start;
extern uint32_t __user_spifc_size;


/*---------------------------------------------------------------------------------------
 * Gobal Variable Declaration Area
 *---------------------------------------------------------------------------------------*/
  uint32_t ScanedKey;
  uint16_t icount;


/*---------------------------------------------------------------------------------------
 * Subroutine Area
 *---------------------------------------------------------------------------------------*/
  uint32_t SWTrim_CheckBusy(void)
  {
    return (CLOCK->SWTRIM & CLOCK_SWTRIM_COUNTER_RDY);
  }

  void GPIO_SLEEP_Initial()
  {
    NVIC_EnableIRQ(KEYCHG_IRQn);                        // Enable Key Changed interrupt

    WRITE_REG(IOFUNC->WAKEEN,0xFF0000FF );
    SET_BIT(IOFUNC->WAKEEN2,GPIOFUNC_WAKEEN2_INT_ENABLE);   //GPIO[31:24] & [7:0] KEY_WAKE_INT_ENABLE
  }

/*---------------------------------------------------------------------------------------
 * Main Function
 *---------------------------------------------------------------------------------------*/
int main()
{
  // GPIO initialize
  GPIO_SetMode(GPIOA, 0xFFFFFFFF, GPIO_MODE_INPUT);     //IOA[31:0] Input pull-low, bypass IOA[14:13] UART & IOA.8 Output
  MODIFY_REG(GPIOA->OBUF, 0xFFFFFFFF, 0);

  GPIO_SetMode(GPIOB, 0xFFFFFFFF, GPIO_MODE_INPUT);     //IOB Input pull-low
  MODIFY_REG(GPIOB->OBUF, 0xFFFFFFFF, 0);

  GPIO_SetMode(GPIOC, 0xFFFFFFFF, GPIO_MODE_INPUT);     //IOC Input pull-low
  MODIFY_REG(GPIOC->OBUF, 0xFFFFFFFF, 0);

  GPIO_SetMode(GPIOD, 0xFFFFFFFF, GPIO_MODE_INPUT);     //IOD Input pull-low
  MODIFY_REG(GPIOD->OBUF, 0xFFFFFFFF, 0);

  MODIFY_REG(SMU->SYSLOCK, SMU_SYSLOCK_UNLOCK_KEY_MSK, SMU_SYSLOCK_UNLOCK_KEY1);
  MODIFY_REG(SMU->SYSLOCK, SMU_SYSLOCK_UNLOCK_KEY_MSK, SMU_SYSLOCK_UNLOCK_KEY2);	  // SMU nulock
  MODIFY_REG(ACU->REG33_CTRL, ACU_REG33_CTRL_SLEEP_EN_MSK, ACU_REG33_CTRL_SLEEP_EN_ENABLE);
  MODIFY_REG(SMU->SYSLOCK, SMU_SYSLOCK_UNLOCK_KEY_MSK, 0);	                        // SMU locked

  while(1)
  {
    WDT_Clear();
    KeyScan_ServiceLoop();

    ScanedKey = KeyScan_GetCh();
	  switch(ScanedKey)
		{
			case 0x01:  // IOA.0: Shut down mode & Key wake up
        MODIFY_REG(GPIOA->IE, 0xFF0000FF, (GPIOA_IE_IO1_ENABLE | GPIOA_IE_IO2_ENABLE | GPIOA_IE_IO3_ENABLE
                                          | GPIOA_IE_IO4_ENABLE | GPIOA_IE_IO5_ENABLE | GPIOA_IE_IO6_ENABLE | GPIOA_IE_IO7_ENABLE
                                          | GPIOA_IE_IO24_ENABLE| GPIOA_IE_IO25_ENABLE| GPIOA_IE_IO26_ENABLE| GPIOA_IE_IO27_ENABLE
                                          | GPIOA_IE_IO28_ENABLE| GPIOA_IE_IO29_ENABLE| GPIOA_IE_IO30_ENABLE| GPIOA_IE_IO31_ENABLE));
        GPIO_SLEEP_Initial();
        READ_REG(GPIOA->IDATA);	                // IOA status latched.
        MODIFY_REG(SMU->SHUTDOWN_ON,0xFF,0x55); // Shut down mode enabled.
        SCB->SCR = (1UL << 2);
        __WFI();
				break;

			case 0x02:  // IOA.1: Shut down mode & Iosc16KHz wake up
			  GPIO_SetMode(GPIOA, 0x00000200, GPIO_MODE_OUTPUT);  // IOA.9 Output
        MODIFY_REG(GPIOA->OBUF, 0x00000200, 0);

        SET_BIT(SMU->SHUTDOWN_CTRL, SMU_SHUTDOWN_CTRL_WK_SRC_I16K);
        SET_BIT(GPIOA->OBUF, BIT9);                         // IOA.9 = Hi
        READ_REG(GPIOA->IDATA);	                            // IOA status latched.
        MODIFY_REG(SMU->SHUTDOWN_ON,0xFF,0x55);             // Shut down mode enabled 0x50000058 = 0x55
        SCB->SCR = (1UL << 2);                              // Entry the Deep Sleep Mode
        __WFI();
				break;

			case 0x04:  // IOA.2: Deep sleep mode
        GPIO_SLEEP_Initial();
        WRITE_REG(IOFUNC->WAKEEN,0xFFFFFFFF );
        SET_BIT(IOFUNC->WAKEEN2,GPIOFUNC_WAKEEN2_INT_ENABLE);   //GPIO_KEY_WAKE_INT_ENABLE

        MODIFY_REG(ACU->OPT_KEYCODE, ACU_OPT_UNLOCK_KEY_MSK, ACU_OPT_UNLOCK_KEY_KEY1);
        MODIFY_REG(ACU->OPT_KEYCODE, ACU_OPT_UNLOCK_KEY_MSK, ACU_OPT_UNLOCK_KEY_KEY2);	          // Option nulock
        MODIFY_REG(SMU->SYSLOCK, SMU_SYSLOCK_UNLOCK_KEY_MSK, SMU_SYSLOCK_UNLOCK_KEY1);
        MODIFY_REG(SMU->SYSLOCK, SMU_SYSLOCK_UNLOCK_KEY_MSK, SMU_SYSLOCK_UNLOCK_KEY2);	          // SMU nulock
        CLEAR_BIT(ACU->X32K_CTRL, ACU_X32K_CTRL_32K_SLP_ENABLE);                                  // XTAL32 Sleep disabled.
        MODIFY_REG(ACU->REG12_CTRL, ACU_REG12_CTRL_SLEEP_EN_MSK, ACU_REG12_CTRL_SLEEP_EN_ENABLE); // V12 Sleep enabled.
        MODIFY_REG(ACU->OPT_KEYCODE, ACU_OPT_UNLOCK_KEY_MSK, 0);	                                // ACU nulock
        MODIFY_REG(SMU->SYSLOCK, SMU_SYSLOCK_UNLOCK_KEY_MSK, 0);	                                // SMU nulock
        while(1)
        {
          WDT_Clear();
          READ_REG(GPIOA->IDATA);	                      //IO status latched.
          READ_REG(GPIOB->IDATA);	                      //IO status latched.
          READ_REG(GPIOC->IDATA);	                      //IO status latched.
          READ_REG(GPIOD->IDATA);	                      //IO status latched.
          SCB->SCR = (1UL << 2); // Deep Sleep Mode
          __WFI();
          icount = 0;
          while(icount<10000)
          {
            icount++;
          }
        }
				break;

			case 0x08:  // IOA.3: Sleep mode
        GPIO_SLEEP_Initial();
        WRITE_REG(IOFUNC->WAKEEN,0xFFFFFFFF );
        SET_BIT(IOFUNC->WAKEEN2,GPIOFUNC_WAKEEN2_INT_ENABLE);   //GPIO_KEY_WAKE_INT_ENABLE

        MODIFY_REG(ACU->OPT_KEYCODE, ACU_OPT_UNLOCK_KEY_MSK, ACU_OPT_UNLOCK_KEY_KEY1);
        MODIFY_REG(ACU->OPT_KEYCODE, ACU_OPT_UNLOCK_KEY_MSK, ACU_OPT_UNLOCK_KEY_KEY2);	          // Option nulock
        MODIFY_REG(SMU->SYSLOCK, SMU_SYSLOCK_UNLOCK_KEY_MSK, SMU_SYSLOCK_UNLOCK_KEY1);
        MODIFY_REG(SMU->SYSLOCK, SMU_SYSLOCK_UNLOCK_KEY_MSK, SMU_SYSLOCK_UNLOCK_KEY2);	          // SMU nulock
        CLEAR_BIT(ACU->X32K_CTRL, ACU_X32K_CTRL_32K_SLP_ENABLE);                                  // XTAL32 Sleep disabled.
        MODIFY_REG(ACU->REG12_CTRL, ACU_REG12_CTRL_SLEEP_EN_MSK, ACU_REG12_CTRL_SLEEP_EN_ENABLE); // V12 Sleep enabled.
        MODIFY_REG(ACU->OPT_KEYCODE, ACU_OPT_UNLOCK_KEY_MSK, 0);	                                // ACU nulock
        MODIFY_REG(SMU->SYSLOCK, SMU_SYSLOCK_UNLOCK_KEY_MSK, 0);	                                // SMU nulock
        while(1)
        {
          WDT_Clear();
          READ_REG(GPIOA->IDATA);	                      //IO status latched.
          READ_REG(GPIOB->IDATA);	                      //IO status latched.
          READ_REG(GPIOC->IDATA);	                      //IO status latched.
          READ_REG(GPIOD->IDATA);	                      //IO status latched.
          // SCB->SCR = (1UL << 2); // Deep Sleep Mode
          __WFI();
          icount = 0;
          while(icount<10000)
          {
            icount++;
          }
        }
				break;

			case 0x10:  // IOA.4: Halt mode @Iosc 32KHz
        TIMEBASE_Open(TB_CLK_SEL_IOSC32K);
        TIMEBASE_EnableInt(TB_INT_0D125HZ);
        NVIC_EnableIRQ(TIMEBASE_IRQn);
				while(1)
				{
          WDT_Clear();
					SCB->SCR = (1UL << 2); // Deep Sleep Mode
	        __WFI();
				}
				break;

			case 0x20:  // IOA.5: Halt mode @XTAL 32KHz
        TIMEBASE_Open(TB_CLK_SEL_XTAL32K);
        TIMEBASE_EnableInt(TB_INT_2HZ);
        NVIC_EnableIRQ(TIMEBASE_IRQn);
				while(1)
				{
          WDT_Clear();
					SCB->SCR = (1UL << 2); // Deep Sleep Mode
	        __WFI();
				}
				break;

			case 0x40:  // IOA.6:

				break;

			case 0x80:  // IOA.7:

				break;
    }
  }
  return 0;
}

/*---------------------------------------------------------------------------------------
 * IRQ Handler
 *---------------------------------------------------------------------------------------*/

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


/*
 * @brief
 *  				Cortex-M0 Processor Interrupt & Exception Handlers
 * @param
 *          None.
 * @return
 *          None.
 */
void KEYCHG_IRQHandler(void)
{

	IOFUNC->STS = GPIOFUNC_STS_INTF_FLAG;
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
