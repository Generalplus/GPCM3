/************************************************************************************
 *  File:     main.c
 *  Purpose:  GPCM3/GPFM1 Sample Code
 *            AUDN: DAC_OUT & AUDP: configured as Output pin (Control by IOA.25 Key)
 *  Date:     07 AUG 2023
 *  Info:
 *
 *
 ************************************************************************************/

/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include <stdio.h>
#include "GPCM3_FM1.h"
#include "SACM_DVR1801_User.h"
#include "APP_SwVolumeControl.h"
#include "KeyScan.h"


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#if defined(__CC_ARM)
#define Aligned4B __align(4)
__align(4) SACM_DVR1801_WORKING_RAM Dvr1801WorkingRam;
__align(4) SACM_DVR1801_TEMP_RAM Dvr1801TempBuffer;
__align(4) SACM_DVR1801_PCM_BUFFER Dvr1801PcmBuffer;
__align(4) KEYSCAN_WORKING_RAM KeyScanWorkingRam;
#elif defined(__GNUC__)
extern unsigned int __boot_table_start; //in GPCM2_CM3.ld
#define Aligned4B __attribute__ ((aligned (4)))
__attribute__ ((aligned (4))) SACM_DVR1801_WORKING_RAM Dvr1801WorkingRam;
__attribute__ ((aligned (4))) SACM_DVR1801_TEMP_RAM Dvr1801TempBuffer;
__attribute__ ((aligned (4))) SACM_DVR1801_PCM_BUFFER Dvr1801PcmBuffer;
__attribute__ ((aligned (4))) KEYSCAN_WORKING_RAM KeyScanWorkingRam;
#endif

uint32_t ScanedKey;
uint16_t A1801Num = 0;
int16_t A1801PlayIdx = 1;
int8_t PlayCon = 0;
int16_t SwVolGain = 9;
uint8_t IO_TestFlag = 0;
uint8_t AudOutType = AUD_OUT_DAC;     //1:PWM 0:DAC Out
uint32_t IODataTemp;


extern uint8_t __user_spifc_load_addr;
extern uint8_t __user_spifc_start;
extern uint32_t __user_spifc_size;


/*---------------------------------------------------------------------------------------
 * Subroutine Area
 *---------------------------------------------------------------------------------------*/
  uint32_t SWTrim_CheckBusy(void)
  {
    return (CLOCK->SWTRIM & CLOCK_SWTRIM_COUNTER_RDY);
  }

  void MoveSpifcRamCode(void);


/*---------------------------------------------------------------------------------------
 * Main Function
 *---------------------------------------------------------------------------------------*/
int main()
{
	WRITE_REG(RCU->STS,0x0000FFFF);                           // Clear Reset status
	MoveSpifcRamCode();

  // GPIO initialize
  GPIO_SetMode(GPIOA, 0xFFFFFEFF, GPIO_MODE_INPUT);         //IOA[31:0] Input pull-low
  GPIO_SetMode(GPIOA, 0x00FF007F, GPIO_MODE_OUTPUT);        //IOA[6:0] & IOA[23:16] Output, Lo.
  GPIO_SetMode(GPIOA, 0x00000180, GPIO_MODE_FLOATING);      //IOA[8:7] X32K pins.
  MODIFY_REG(GPIOA->OBUF, 0xFFFFFFFF, 0);
  GPIO_SetMode(GPIOC, 0x00000003, GPIO_MODE_INPUT);         //IOC Input pull-low
  MODIFY_REG(GPIOC->OBUF, 0x00000003, 0);
  GPIO_SetMode(GPIOD, 0x00000003, GPIO_MODE_OUTPUT);        //IOD Output, Lo
  MODIFY_REG(GPIOD->OBUF, 0x00000003, 0);

	A1801Num = DVR1801_User_GetA1801Num();
	SACM_DVR1801_Initial(&Dvr1801WorkingRam, &Dvr1801TempBuffer, &Dvr1801PcmBuffer);

	APP_SwVolCtrl_Init();                                    // Software volume control init.
	APP_SwVolCtrl_SetVolGain(0, SwVolGain);
	KeyScan_Initial(&KeyScanWorkingRam);                     // key scan init.

	DAC-> AUDPWM_CTRL |= (AUDPWM_CTRL_AUDPWM_ENABLE | AUDPWM_CTRL_AUDPWM_IP_ENABLE | AUDPWM_CTRL_DAC_CH0_ENABLE);

  SACM_DVR1801_Play(DVR1801_User_GetA1801StartAddr(A1801PlayIdx), DVR1801_DAC_CH0 , DVR1801_AUTO_RAMP_UP + DVR1801_AUTO_RAMP_DOWN);

  while(1)
	{
		WDT_Clear();
		SACM_DVR1801_ServiceLoop();
		KeyScan_ServiceLoop();

		ScanedKey = KeyScan_GetCh();
	  switch(ScanedKey)
		{
			case 0x01:                                           // IOA24+VDD: Recording
				PlayCon = 0;
				SACM_DVR1801_Stop();
			  SACM_DVR1801_Initial(&Dvr1801WorkingRam, &Dvr1801TempBuffer, &Dvr1801PcmBuffer);
				SPIFC_BlockErase(0x00200000);
			  SPIFC_BlockErase(0x00210000);
			  SACM_DVR1801_Rec((int16_t *) 0x00200000, DVR1801_RECORD_BITRATE_16000);
			  break;

		  case 0x02:                                           // IOA25+VDD: AUDP_Output Hi/Lo
				PlayCon = 0;
				SACM_DVR1801_Stop();
        break;

			case 0x04:                                           // IOA26+VDD: Play recording voice
				PlayCon = 0;
				SACM_DVR1801_Stop();
			  SACM_DVR1801_Initial(&Dvr1801WorkingRam, &Dvr1801TempBuffer, &Dvr1801PcmBuffer);
			  SACM_DVR1801_Play((int16_t *)0x04200000, DVR1801_DAC_CH0, DVR1801_AUTO_RAMP_UP + DVR1801_AUTO_RAMP_DOWN);
				break;

			case 0x08:                                           // IOA27+VDD:
        if(IO_TestFlag == 0)
        {
          DAC-> AUDPWM_CTRL |= AUDPWM_CTRL_AUDP_STATE_HIGH;     // AUDP_Out Hi
          IO_TestFlag = 1;
        }
        else
        {
          DAC-> AUDPWM_CTRL &= ~AUDPWM_CTRL_AUDP_STATE_HIGH;     // AUDP_Out Lo
           IO_TestFlag = 0;
        }
				break;

			case 0x10:                                           // IOA28+VDD: DAC Out
			  SACM_DVR1801_Stop();
			  AudOutType = AUD_OUT_DAC;          //0:DAC Out
			  SACM_DVR1801_Initial(&Dvr1801WorkingRam, &Dvr1801TempBuffer, &Dvr1801PcmBuffer);
			  SACM_DVR1801_Play(DVR1801_User_GetA1801StartAddr(A1801PlayIdx++), DVR1801_DAC_CH0, DVR1801_AUTO_RAMP_UP + DVR1801_AUTO_RAMP_DOWN);
			  break;

			case 0x20:	                                         // IOA29+VDD: PWM Out
				PlayCon = 0;
				SACM_DVR1801_Stop();
			  AudOutType = AUD_OUT_PWM;          //1:PWM Out
			  SACM_DVR1801_Initial(&Dvr1801WorkingRam, &Dvr1801TempBuffer, &Dvr1801PcmBuffer);
				SACM_DVR1801_Play(DVR1801_User_GetA1801StartAddr(A1801PlayIdx), DVR1801_DAC_CH0 , DVR1801_AUTO_RAMP_UP + DVR1801_AUTO_RAMP_DOWN);
			  A1801PlayIdx = ++A1801PlayIdx > A1801Num ? 1 : A1801PlayIdx;
				break;

			case 0x40:                                           // IOA30+VDD:

				break;

      case 0x80:                                           // IOA31+VDD:
				break;
	  }

		if((PlayCon != 0) && (SACM_DVR1801_Check_Con() == 0))
		{
			SACM_DVR1801_Play_Con(DVR1801_User_GetA1801StartAddr(A1801PlayIdx++), DVR1801_DAC_CH0, DVR1801_AUTO_RAMP_UP + DVR1801_AUTO_RAMP_DOWN);
			A1801PlayIdx = A1801PlayIdx > A1801Num ? 1 : A1801PlayIdx;
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
		XOR_BIT(GPIOA->OBUF, BIT0);                                   // Toggle IOA[0]
		TIMEBASE->STS = TIMEBASE_STS_0D125HZ_INTF_FLAG;               // Clear TB INT flag
	}

	if((TIMEBASE->STS & TIMEBASE_STS_0D25HZ_INTF_FLAG) != 0)        // TB_0.25Hz
	{
		XOR_BIT(GPIOA->OBUF, BIT1);                                   // Toggle IOA[1]
		TIMEBASE->STS = TIMEBASE_STS_0D25HZ_INTF_FLAG;                // Clear TB INT flag
	}

	if((TIMEBASE->STS & TIMEBASE_STS_0D5HZ_INTF_FLAG) != 0)         // TB_0.5Hz
	{
		XOR_BIT(GPIOA->OBUF, BIT2);                                  // Toggle IOA[2]
		TIMEBASE->STS = TIMEBASE_STS_0D5HZ_INTF_FLAG;                 // Clear TB INT flag
	}

	if((TIMEBASE->STS & TIMEBASE_STS_1HZ_INTF_FLAG) != 0)           // TB_1Hz
	{
		XOR_BIT(GPIOA->OBUF, BIT0);                                  // Toggle IOA[1:0]
		XOR_BIT(GPIOA->OBUF, BIT1);
		TIMEBASE->STS = TIMEBASE_STS_1HZ_INTF_FLAG;                   // Clear TB INT flag
	}

	if((TIMEBASE->STS & TIMEBASE_STS_2HZ_INTF_FLAG) != 0)           // TB_2Hz
	{
		XOR_BIT(GPIOA->OBUF, BIT2);                                  // Toggle IOA[3:2]
		XOR_BIT(GPIOA->OBUF, BIT3);
		TIMEBASE->STS = TIMEBASE_STS_2HZ_INTF_FLAG;                   // Clear TB INT flag
	}

	if((TIMEBASE->STS & TIMEBASE_STS_16HZ_INTF_FLAG) != 0)          // TB_16Hz
	{
		XOR_BIT(GPIOA->OBUF, BIT4);                                  // Toggle IOA[6:4]
		XOR_BIT(GPIOA->OBUF, BIT5);
		XOR_BIT(GPIOA->OBUF, BIT6);
		TIMEBASE->STS = TIMEBASE_STS_16HZ_INTF_FLAG;                  // Clear TB INT flag
	}

	if((TIMEBASE->STS & TIMEBASE_STS_64HZ_INTF_FLAG) != 0)          // TB_64Hz
	{
		XOR_BIT(GPIOA->OBUF, BIT3);                                  // Toggle IOA[3]
		TIMEBASE->STS = TIMEBASE_STS_64HZ_INTF_FLAG;                  // Clear TB INT flag
	}

	if((TIMEBASE->STS & TIMEBASE_STS_512HZ_INTF_FLAG) != 0)          // TB_512Hz
	{
		XOR_BIT(GPIOA->OBUF, BIT5);                                   // Toggle IOA[5]
		TIMEBASE->STS = TIMEBASE_STS_512HZ_INTF_FLAG;                  // Clear TB INT flag
	}

	if((TIMEBASE->STS & TIMEBASE_STS_2KHZ_INTF_FLAG) != 0)           // TB_2KHz
	{
		XOR_BIT(GPIOA->OBUF, BIT4);                                   // Toggle IOA[4]
		TIMEBASE->STS = TIMEBASE_STS_2KHZ_INTF_FLAG;                   // Clear TB INT flag
	}

	if((TIMEBASE->STS & TIMEBASE_STS_4KHZ_INTF_FLAG) != 0)          // TB_4KHz
	{
		XOR_BIT(GPIOA->OBUF, BIT3);                                   // Toggle IOA[3]
		TIMEBASE->STS = TIMEBASE_STS_4KHZ_INTF_FLAG;                   // Clear TB INT flag
	}
}

