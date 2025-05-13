/************************************************************************************
 *  File:     main.c
 *  Purpose:  GPCM3/GPFM1 SAR-ADC Regular manual mode demo
 *            Replace with your code.
 *  Date:     04 March 2024
 *  Info:     1.LineInPad: IOA.27 & IOA.31 (Detect Voltage Range: 0 ~ 3.3V)
 *            2.SAR-ADC data will be displayed on the screen via UART[14:13]
 *
 *
 ************************************************************************************/

/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include <stdio.h>
#include "Serial.h"
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

uint16_t SAR_ADC_DATA_IOA27[16];
uint16_t SAR_ADC_DATA_IOA31[16];
uint8_t  IOA27_DataCount = 0;
uint8_t  IOA31_DataCount = 0;
uint32_t AdcCovChTempIdx = 0;
uint32_t iCount;


/*---------------------------------------------------------------------------------------
 * Subroutine Area
 *---------------------------------------------------------------------------------------*/
  uint32_t SWTrim_CheckBusy(void)
  {
    return (CLOCK->SWTRIM & CLOCK_SWTRIM_COUNTER_RDY);
  }


/**
 * @brief
 *   Initialize SAR-ADC regular manual mode
 *    - Input from SAR-ADC ch0(IOA.27) & ch7(IOA.31)
 *    - Output the convered data to UART.
 * @param
 *   None.
 * @return
 *   None.
 */
void SarAdc_RegularModeLoopInitial()
{
	uint8_t AdcChSampleTimeTable[] = {
	                                  SAR_ADC_SMPTIME_32ADCCLK,        // Sar-ADC Channel 0 sample time
	                                  SAR_ADC_SMPTIME_32ADCCLK,        // Sar-ADC Channel 1 sample time
	                                 };
  uint8_t RegModeSequenceTable[] = {3,7};                            // Ch sequence: ch3(IOA[27]) -> ch7(IOA[31]) -> (loop)
	SAR_ADC_Open();
	SAR_ADC_SetAdcMultiChSampleTime(AdcChSampleTimeTable, RegModeSequenceTable, sizeof(RegModeSequenceTable));
  SAR_ADC_RegMode_Init(RegModeSequenceTable, SAR_ADC_REG_CH_NUM_2, SAR_ADC_GAP_SEL_MANUAL, SAR_ADC_LOOP_DISABLE);
	SAR_ADC_LineInPadEnable(LINE_PAD_IOA27 | LINE_PAD_IOA31);
	WRITE_REG(SAR_ADC->STS , SAR_ADC_STS_REG_MODE_INT_FLAG);           // Clear regular mode interrupt flag
	// SAR_ADC_RegMode_EnableInt();                                    // SAR-ADC regular mode interrupt enabled
	// NVIC_EnableIRQ(SAR_ADC_IRQn);
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
   * IOA[15:8] output low
	 */
  GPIO_SetMode(GPIOA, 0x0000FF00, GPIO_MODE_OUTPUT);
  MODIFY_REG(GPIOA->OBUF, 0x0000FF00, 0);

  /*
   * UART initial
	 */
	UART_Init(UART0, UART0_IOA13_14, 115200);             // UART IO: IOA[14:13]

  printf("Reset Flag = %x \r\n", READ_REG(RCU->STS));
	WRITE_REG(RCU->STS,0x0000FFFF);                       // Clear Reset status


  // SAR-ADC initialize for Regulator Manual Mode
  SarAdc_RegularModeLoopInitial();                 // SAR-ADC Input pins: IOA.27 & IOA.31
  SAR_ADC_RegMode_Start();

  // Timer 0 initialize
  TIMER_Open(TM0);
  TIMER_SetFreq(TM0, 625);          // 625Hz = 1.6ms
  TIMER_EnableInt(TM0);
  NVIC_EnableIRQ(TIMER0_IRQn);

  TIMEBASE_Open(TB_CLK_SEL_IOSC32K);
	TIMEBASE_EnableInt(TB_INT_2HZ);
	NVIC_EnableIRQ(TIMEBASE_IRQn);

  while(1)
  {
    WDT_Clear();
		KeyScan_ServiceLoop();

		ScanedKey = KeyScan_GetCh();
	  switch(ScanedKey)
		{
		  case 0x01:
        MODIFY_REG(SAR_ADC->CTRL, SAR_ADC_CTRL_SFT_STR_MSK, SAR_ADC_CTRL_SFT_START);
        WRITE_REG(SAR_ADC->REG_DATA,0x1);               // SAR-ADC triggered in manual mode
        break;

      case 0x02:

        break;

      default:
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
 *   Sar-ADC IRQ handler
 * @param
 *   None.
 * @return
 *   None.
 */
void SAR_ADC_IRQHandler(void)
{
	if((SAR_ADC->STS & SAR_ADC_STS_REG_MODE_INT_FLAG) != 0)
	{
		SAR_ADC->STS = SAR_ADC_STS_REG_MODE_INT_FLAG;                    // Clear regular mode interrupt flag
	}

	if((SAR_ADC->STS & SAR_ADC_STS_INJ0_INT_FLAG) != 0)
	{
		SAR_ADC->STS = SAR_ADC_STS_INJ0_INT_FLAG;                        // Clear injection0 interrupt flag
	}

}

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
		switch(SAR_ADC->REG_COV)
		{
			case 0x3:             // IOA.27
        SAR_ADC_DATA_IOA27[IOA27_DataCount] = SAR_ADC->REG_DATA;
        if(IOA27_DataCount < 16)
        {
          IOA27_DataCount++;
        }
        else
        {
          IOA27_DataCount = 0;
        }
				break;

			case 0x7:             // IOA.31
				SAR_ADC_DATA_IOA31[IOA31_DataCount] = SAR_ADC->REG_DATA;
        if(IOA31_DataCount < 16)
        {
          IOA31_DataCount++;
        }
        else
        {
          IOA31_DataCount = 0;
        }
				break;
		}

  // SAR-ADC Re-triggered in manual mode
  MODIFY_REG(SAR_ADC->CTRL, SAR_ADC_CTRL_SFT_STR_MSK, SAR_ADC_CTRL_SFT_START);
  WRITE_REG(SAR_ADC->REG_DATA,0x1);

	TM_INT->INTSTS = TIMER_INTSTS_TM0_INT_FLAG;    //	Clear Timer0 intterupt flag
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
  uint8_t ADC_Count = 0;
  uint32_t ADC_IOA27Sum = 0;
  uint32_t ADC_IOA31Sum = 0;


	if((TIMEBASE->STS & TIMEBASE_STS_2HZ_INTF_FLAG) != 0)           // TB_2Hz
	{
    for(ADC_Count= 0; ADC_Count<16; ADC_Count++)
    {
      ADC_IOA27Sum += SAR_ADC_DATA_IOA27[ADC_Count];
      ADC_IOA31Sum += SAR_ADC_DATA_IOA31[ADC_Count];
    }
    ADC_IOA27Sum = ADC_IOA27Sum >> 4;           // ADC_IOA27Sum/ 16
    ADC_IOA31Sum = ADC_IOA31Sum >> 4;           // ADC_IOA27Sum/ 16

    printf("SAR-ADC IOA.27 = %x \r\n", ADC_IOA27Sum);
    printf("SAR-ADC IOA.31 = %x \r\n", ADC_IOA31Sum);
		TIMEBASE->STS = TIMEBASE_STS_2HZ_INTF_FLAG;                   // Clear TB INT flag
	}
}


