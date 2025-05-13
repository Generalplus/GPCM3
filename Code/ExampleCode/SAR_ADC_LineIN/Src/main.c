/************************************************************************************
 *  File:     main.c
 *  Purpose:  GPCM3/GPFM1 Example Code
 *            Replace with your code.
 *  Date:     12 February 2025
 *  Info:     SAR-ADC Input pin: IOA.27
 *            Output: IOA[15:8]
 *  Channel Mapping CH0 = IOA.24, CH1 = IOA.25 , CH2 = IOA.26,..., CH7 = IOA.31
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

uint8_t AdcCovChTemp[256];                                           // Temp buffer for recording ADC regular mode sequence
uint32_t AdcCovChTempIdx = 0;
uint32_t iCount;
uint8_t RegScanFlag = 0;


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
 *    - Input from SAR-ADC ch7 (line-in from IOA[27])
 * @param
 *   None.
 * @return
 *   None.
 */
void SarAdc_RegularModeInitial()
{
	uint8_t AdcChSeq[] = {3};                                          // sequence: always ch6

	SAR_ADC_Open();
	SAR_ADC_SetAdcChSampleTime(3, SAR_ADC_SMPTIME_32ADCCLK);	         // Set SAR-ADC ch3 sample time = 32 ADC clock
	SAR_ADC_RegMode_Init(AdcChSeq, SAR_ADC_REG_CH_NUM_1, SAR_ADC_GAP_SEL_7ADCCLK, SAR_ADC_LOOP_DISABLE);


	SAR_ADC_LineInPadEnable(LINE_PAD_IOA27);
	WRITE_REG(SAR_ADC->STS , SAR_ADC_STS_REG_MODE_INT_FLAG);           // Clear regular mode interrupt flag
	SAR_ADC_RegMode_EnableInt();
	NVIC_EnableIRQ(SAR_ADC_IRQn);
}

/**
 * @brief
 *   Initialize SAR-ADC regular loop mode
 *    - Input from SAR-ADC ch0~9
 * @param
 *   None.
 * @return
 *   None.
 */
void SarAdc_RegularModeLoopInitial()
{
	uint8_t AdcChSampleTimeTable[] = {
                                    SAR_ADC_SMPTIME_32ADCCLK,        // Sar-ADC Channel 1 sample time
                                    SAR_ADC_SMPTIME_32ADCCLK,        // Sar-ADC Channel 3 sample time
                                    SAR_ADC_SMPTIME_32ADCCLK,        // Sar-ADC Channel 2 sample time
	                                  SAR_ADC_SMPTIME_32ADCCLK,        // Sar-ADC Channel 0 sample time
	                                  SAR_ADC_SMPTIME_32ADCCLK,        // Sar-ADC Channel 4 sample time
	                                  SAR_ADC_SMPTIME_32ADCCLK,        // Sar-ADC Channel 5 sample time
	                                  SAR_ADC_SMPTIME_32ADCCLK,        // Sar-ADC Channel 8 sample time
	                                  SAR_ADC_SMPTIME_32ADCCLK         // Sar-ADC Channel 9 sample time
	                                 };
  uint8_t RegModeSequenceTable[] = {0, 1, 2, 3, 4, 5, 6, 7};         // Channel sequence of the regular loop mode: ch0(IOA[24]) -> ch1(IOA[25]) -> ch2(IOA[26]) -> ch3(IOA[27]) ->
	SAR_ADC_Open();
	SAR_ADC_SetAdcMultiChSampleTime(AdcChSampleTimeTable, RegModeSequenceTable, sizeof(RegModeSequenceTable));
 	SAR_ADC_RegMode_Init(RegModeSequenceTable, SAR_ADC_REG_CH_NUM_8, SAR_ADC_GAP_SEL_7ADCCLK, SAR_ADC_LOOP_ENABLE);
	SAR_ADC_LineInPadEnable(LINE_PAD_IOA25 | LINE_PAD_IOA26 | LINE_PAD_IOA27);
	WRITE_REG(SAR_ADC->STS , SAR_ADC_STS_REG_MODE_INT_FLAG);           // Clear regular mode interrupt flag
	SAR_ADC_RegMode_EnableInt();
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
	SAR_ADC_LineInPadEnable(LINE_PAD_IOA29);
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
   * SAR-ADC Regulator Mode initialize
	 */
  SarAdc_RegularModeInitial();
  SAR_ADC_RegMode_Start();

  while(1)
  {
    WDT_Clear();
		KeyScan_ServiceLoop();

		if(RegScanFlag ==1)
		{
      RegScanFlag = 0;
      SAR_ADC_RegMode_Start();
		}

		ScanedKey = KeyScan_GetCh();
		switch(ScanedKey)
		{
		  case 0x01:                                                     // IOA24+VDD: Regular mode
        SarAdc_RegularModeInitial();
			  SAR_ADC_RegMode_Start();
        break;

		  case 0x02:                                                     // IOA25+VDD:	Regular loop mode
			  SarAdc_RegularModeLoopInitial();
				SAR_ADC_RegMode_Start();
        break;

		  case 0x04:                                                     // IOA26+VDD: Stop regular loop mode
				TIMER_Close(TM0);
        SAR_ADC_RegMode_Stop();
			  AdcCovChTempIdx = 0;
        break;

			case 0x08:                                                     // IOA27+VDD: Injection 0
				TIMER_Close(TM0);
				SAR_ADC_RegMode_Stop();
        SarAdc_Injection_Initial();
        TIMER_Open(TM0);
	      TIMER_SetFreq(TM0, 2000);
				break;

		  case 0x10:                                                     // IOA28+VDD:

        break;

      case 0x20:                                                     // IOA29+VDD: Disable TB all interrupt.

        break;

			case 0x40:                                                     // IOA30+VDD: Reset TB count.

			  break;

      case 0x80:                                                     // IOA31+VDD: Close TB.

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
		AdcCovChTemp[AdcCovChTempIdx++] = SAR_ADC->REG_COV;
		if(AdcCovChTempIdx == sizeof(AdcCovChTemp))
		{
			AdcCovChTempIdx = 0;
		}

		switch(SAR_ADC->REG_COV)
		{
			case 0x0:
				GPIOA->OBUF = (SAR_ADC->REG_DATA);
				break;

			case 0x1:
				GPIOA->OBUF = (SAR_ADC->REG_DATA);
				break;

			case 0x2:
				GPIOA->OBUF = (SAR_ADC->REG_DATA);
				break;

			case 0x3:
				GPIOA->OBUF = (SAR_ADC->REG_DATA);
				RegScanFlag = 1;
				break;

			case 0x4:
				GPIOA->OBUF = (SAR_ADC->REG_DATA);
				break;

			case 0x5:
				GPIOA->OBUF = (SAR_ADC->REG_DATA);
				break;

			case 0x6:
				GPIOA->OBUF = (SAR_ADC->REG_DATA);
				break;

			case 0x7:
				GPIOA->OBUF = (SAR_ADC->REG_DATA);
				break;

			case 0x8:
				GPIOA->OBUF = (SAR_ADC->REG_DATA);
				break;

			case 0x9:
				GPIOA->OBUF = (SAR_ADC->REG_DATA);
				break;
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



