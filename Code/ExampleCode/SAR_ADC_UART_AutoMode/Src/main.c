/************************************************************************************
 *  File:     main.c
 *  Purpose:  GPCM3/GPFM1 SAR-ADC Regular auto mode demo
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

uint8_t AdcCovChTemp[256];                                           // Temp buffer for recording ADC regular mode sequence
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
 *   Initialize SAR-ADC regular mode
 *    - Input from SAR-ADC ch6 (line-in from IOA[23])
 *    - When ADC conversion is done, IOA[8] is toggled.
 *    - Output the convered data to IOB[6:0].
 * @param
 *   None.
 * @return
 *   None.
 */
void SarAdc_RegularModeInitial()
{
	uint8_t AdcChSeq[] = {6};                                          // sequence: always ch6

	SAR_ADC_Open();
	SAR_ADC_SetAdcChSampleTime(6, SAR_ADC_SMPTIME_32ADCCLK);	         // Set SAR-ADC ch6 sample time = 32 ADC clock
	SAR_ADC_RegMode_Init(AdcChSeq, SAR_ADC_REG_CH_NUM_1, SAR_ADC_GAP_SEL_TM0, SAR_ADC_LOOP_DISABLE);
	SAR_ADC_LineInPadEnable(LINE_PAD_IOA27);
	WRITE_REG(SAR_ADC->STS , SAR_ADC_STS_REG_MODE_INT_FLAG);           // Clear regular mode interrupt flag
	SAR_ADC_RegMode_EnableInt();
	NVIC_EnableIRQ(SAR_ADC_IRQn);
}

/**
 * @brief
 *   Initialize SAR-ADC regular loop mode
 *    - Input from SAR-ADC ch0(IOA.27) & ch7(IOA.31)
 *    - When ADC conversion is done, IOA[8] is toggled.
 *    - Output the convered data to IOA[15:8].
 * @param
 *   None.
 * @return
 *   None.
 */
void SarAdc_RegularModeLoopInitial()
{
	uint8_t AdcChSampleTimeTable[] = {
	                                  SAR_ADC_SMPTIME_32ADCCLK,        // Sar-ADC Channel 3 sample time
	                                  SAR_ADC_SMPTIME_32ADCCLK,        // Sar-ADC Channel 7 sample time
	                                 };
  uint8_t RegModeSequenceTable[] = {3,7};                            // Ch sequence: ch3(IOA[27]) -> ch7(IOA[31]) -> (loop)
	SAR_ADC_Open();
	SAR_ADC_SetAdcMultiChSampleTime(AdcChSampleTimeTable, RegModeSequenceTable, sizeof(RegModeSequenceTable));
  SAR_ADC_RegMode_Init(RegModeSequenceTable, SAR_ADC_REG_CH_NUM_2, SAR_ADC_GAP_SEL_TM0, SAR_ADC_LOOP_ENABLE);
	SAR_ADC_LineInPadEnable(LINE_PAD_IOA27 | LINE_PAD_IOA31);
	WRITE_REG(SAR_ADC->STS , SAR_ADC_STS_REG_MODE_INT_FLAG);           // Clear regular mode interrupt flag
	SAR_ADC_RegMode_EnableInt();                                       // SAR-ADC regular mode interrupt enabled
	NVIC_EnableIRQ(SAR_ADC_IRQn);
}

/**
 * @brief
 *   Initialize SAR-ADC injection mode
 *    - Input from SAR-ADC ch5
 *    - When ADC conversion is done, IOA[9] is toggled.
 *    - Output the convered data to IOB[6:0].
 * @param
 *   None.
 * @return
 *   None.
 */
void SarAdc_Injection_Initial()
{
  SAR_ADC_Open();
	SAR_ADC_SetAdcChSampleTime(5, SAR_ADC_SMPTIME_32ADCCLK);           // Set SAR-ADC ch5 sample time = 32 adc clock
	SAR_ADC_LineInPadEnable(LINE_PAD_IOA27);
	SAR_ADC_InjMode_Init(0, 5, SAR_ADC_INJ_TRG_SEL_TM0);               // Initialize SAR-ADC injection 0, ADC channel: ch5, trigger source: Timer0.
	WRITE_REG(SAR_ADC->STS , SAR_ADC_STS_INJ0_INT_FLAG);               // Clear injection0 interrupt flag
	SAR_ADC_InjMode_EnableInt(0);
	NVIC_EnableIRQ(SAR_ADC_IRQn);
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


  // SAR-ADC initialize
  TIMER_Close(TM0);
  SarAdc_RegularModeLoopInitial();                 // SAR-ADC Input pins: IOA.27 & IOA.31
  SAR_ADC_RegMode_Start();
  TIMER_Open(TM0);
  TIMER_SetFreq(TM0, 2000);

  TIMEBASE_Open(TB_CLK_SEL_IOSC32K);
	TIMEBASE_EnableInt(TB_INT_2HZ);
	NVIC_EnableIRQ(TIMEBASE_IRQn);


  while(1)
  {
    Input0x(&ScanedKey, 2);
	  switch(ScanedKey)
		{
		  case 0x00:
			  printf("Your input number is 0 \r\n");
			  ScanedKey = 0;
        break;

      case 0x01:
			  printf("Your input number is 1 \r\n");
			  ScanedKey = 0;
        break;

      default:
        printf("Your input number is %x \r\n",ScanedKey);
        ScanedKey = 0;
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
		GPIOA->OBUF ^= BIT8;
		AdcCovChTemp[AdcCovChTempIdx++] = SAR_ADC->REG_COV;
		if(AdcCovChTempIdx == sizeof(AdcCovChTemp))
		{
			AdcCovChTempIdx = 0;
		}

		SAR_ADC->STS = SAR_ADC_STS_REG_MODE_INT_FLAG;                    // Clear regular mode interrupt flag
	}

	if((SAR_ADC->STS & SAR_ADC_STS_INJ0_INT_FLAG) != 0)
	{
		GPIOA->OBUF ^= BIT9;
		GPIOA->OBUF = (SAR_ADC->INJ0_DATA);

		SAR_ADC->STS = SAR_ADC_STS_INJ0_INT_FLAG;                        // Clear injection0 interrupt flag
	}

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
		TIMEBASE->STS = TIMEBASE_STS_0D125HZ_INTF_FLAG;               // Clear TB INT flag
	}

	if((TIMEBASE->STS & TIMEBASE_STS_0D25HZ_INTF_FLAG) != 0)        // TB_0.25Hz
	{
		TIMEBASE->STS = TIMEBASE_STS_0D25HZ_INTF_FLAG;                // Clear TB INT flag
	}

	if((TIMEBASE->STS & TIMEBASE_STS_0D5HZ_INTF_FLAG) != 0)         // TB_0.5Hz
	{
		TIMEBASE->STS = TIMEBASE_STS_0D5HZ_INTF_FLAG;                 // Clear TB INT flag
	}

	if((TIMEBASE->STS & TIMEBASE_STS_1HZ_INTF_FLAG) != 0)           // TB_1Hz
	{
		TIMEBASE->STS = TIMEBASE_STS_1HZ_INTF_FLAG;                   // Clear TB INT flag
	}

	if((TIMEBASE->STS & TIMEBASE_STS_2HZ_INTF_FLAG) != 0)           // TB_2Hz
	{
    // GPIOA->OBUF = (SAR_ADC->REG_DATA);
    printf("SAR-ADC Data = %x \r\n", READ_REG(SAR_ADC->REG_DATA));
		TIMEBASE->STS = TIMEBASE_STS_2HZ_INTF_FLAG;                   // Clear TB INT flag
	}

	if((TIMEBASE->STS & TIMEBASE_STS_16HZ_INTF_FLAG) != 0)          // TB_16Hz
	{
		TIMEBASE->STS = TIMEBASE_STS_16HZ_INTF_FLAG;                  // Clear TB INT flag
	}

	if((TIMEBASE->STS & TIMEBASE_STS_64HZ_INTF_FLAG) != 0)          // TB_64Hz
	{
		TIMEBASE->STS = TIMEBASE_STS_64HZ_INTF_FLAG;                  // Clear TB INT flag
	}

	if((TIMEBASE->STS & TIMEBASE_STS_512HZ_INTF_FLAG) != 0)          // TB_512Hz
	{
		TIMEBASE->STS = TIMEBASE_STS_512HZ_INTF_FLAG;                  // Clear TB INT flag
	}

	if((TIMEBASE->STS & TIMEBASE_STS_2KHZ_INTF_FLAG) != 0)           // TB_2KHz
	{
		TIMEBASE->STS = TIMEBASE_STS_2KHZ_INTF_FLAG;                   // Clear TB INT flag
	}

	if((TIMEBASE->STS & TIMEBASE_STS_4KHZ_INTF_FLAG) != 0)          // TB_4KHz
	{
		TIMEBASE->STS = TIMEBASE_STS_4KHZ_INTF_FLAG;                   // Clear TB INT flag
	}
}


