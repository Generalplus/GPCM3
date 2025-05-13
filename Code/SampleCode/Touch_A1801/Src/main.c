/************************************************************************************
 *  File:     main.c
 *  Purpose:  GPCM3/GPFM1 Sample Code
 *            Replace with your code.
 *  Date:     31 January 2023
 *  Info:
 *
 *
 ************************************************************************************/

/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include <stdio.h>
#include "GPCM3_FM1.h"
#include "SACM_A1801_User.h"
#include "Touch_Sensor.h"
#include "Touch_User.h"
#include "TouchProbe_Driver.h"
#include "APP_SwVolumeControl.h"
#include "KeyScan.h"


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#if defined(__CC_ARM)
#define Aligned4B __align(4)
__align(4) SACM_A1801_WORKING_RAM A1801WorkingRam;
__align(4) SACM_A1801_TEMP_RAM A1801TempBuffer;
__align(4) SACM_A1801_PCM_BUFFER A1801PcmBuffer;
__align(4) KEYSCAN_WORKING_RAM KeyScanWorkingRam;
#elif defined(__GNUC__)
extern unsigned int __boot_table_start; //in GPCM2_CM3.ld
#define Aligned4B __attribute__ ((aligned (4)))
__attribute__ ((aligned (4))) SACM_A1801_WORKING_RAM A1801WorkingRam;
__attribute__ ((aligned (4))) SACM_A1801_TEMP_RAM A1801TempBuffer;
__attribute__ ((aligned (4))) SACM_A1801_PCM_BUFFER A1801PcmBuffer;
__attribute__ ((aligned (4))) KEYSCAN_WORKING_RAM KeyScanWorkingRam;
#endif

uint32_t ScanedKey;
uint16_t SpeechNum = 0;
int16_t A1801PlayIdx = 3;
int8_t PlayCon = 0;
int8_t SwVolGain = 6;
uint32_t CTSResult=0;
uint32_t CTSResultPre=0;

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


/*---------------------------------------------------------------------------------------
 * Main Function
 *---------------------------------------------------------------------------------------*/
int main()
{
	WRITE_REG(RCU->STS,0x0000FFFF);                             // Clear Reset status

  // GPIO initialize
  GPIO_SetMode(GPIOA, 0xFFFFFFFF, GPIO_MODE_INPUT);         //IOA[31:0] Input pull-low, bypass IOA[14:13] UART & IOA.8 Output
  MODIFY_REG(GPIOA->OBUF, 0xFFFFFFFF, 0);
  GPIO_SetMode(GPIOB, 0xFFFFFFFF, GPIO_MODE_INPUT);         //IOB Input pull-low
  MODIFY_REG(GPIOB->OBUF, 0xFFFFFFFF, 0);
  GPIO_SetMode(GPIOC, 0xFFFFFFFF, GPIO_MODE_INPUT);         //IOC Input pull-low
  MODIFY_REG(GPIOC->OBUF, 0xFFFFFFFF, 0);
  GPIO_SetMode(GPIOD, 0xFFFFFFFF, GPIO_MODE_INPUT);         //IOD Input pull-low
  MODIFY_REG(GPIOD->OBUF, 0xFFFFFFFF, 0);

	SpeechNum = A1801_User_GetA1801Num();
	SACM_A1801_Initial(&A1801WorkingRam, &A1801TempBuffer, &A1801PcmBuffer);

	APP_SwVolCtrl_Init();                                    // Software volume control init.
	APP_SwVolCtrl_SetVolGain(0, SwVolGain);
	KeyScan_Initial(&KeyScanWorkingRam);                     // key scan init.

	CTS_Initial();
	CTS_FilterSetting(3);
	CTS_Scan();

	// TP_Initial();
	// TP_Start();
  // TIMEBASE_Open(TB_CLK_SEL_IOSC32K);

  // SACM_A1801_Play(A1801_User_GetA1801StartAddr(A1801PlayIdx), A1801_DAC_CH0 , A1801_AUTO_RAMP_UP + A1801_AUTO_RAMP_DOWN);

  while(1)
	{
		WDT_Clear();
		SACM_A1801_ServiceLoop();
		CTS_ServiceLoop();
		CTSResult = (uint32_t )CTS_GetResult();
		ScanedKey = CTSResult & ~CTSResultPre;
		CTSResultPre = CTSResult;
		// KeyScan_ServiceLoop();

		if(SACM_A1801_CheckCpuOverload() != 0)
		{
			SACM_A1801_ClearCpuOverload();
		}

		// ScanedKey = KeyScan_GetCh();
		if(ScanedKey < 0x100)
		{
      switch(ScanedKey)
      {
        case 0x01:                                     // IOA0+VDD: Stop
          PlayCon = 0;
          SACM_A1801_Stop();
          break;

        case 0x02:                                     // IOA1+VDD: Play next song
          PlayCon = 0;
          SACM_A1801_Stop();
          A1801PlayIdx = ++A1801PlayIdx > SpeechNum ? 1 : A1801PlayIdx;
          SACM_A1801_Play(A1801_User_GetA1801StartAddr(A1801PlayIdx), A1801_DAC_CH0 , A1801_AUTO_RAMP_UP + A1801_AUTO_RAMP_DOWN);
          break;

        case 0x04:	                                   // IOA2+VDD: Play prev song
          PlayCon = 0;
          SACM_A1801_Stop();
          A1801PlayIdx = --A1801PlayIdx < 1 ? SpeechNum : A1801PlayIdx;
          SACM_A1801_Play(A1801_User_GetA1801StartAddr(A1801PlayIdx), A1801_DAC_CH1, A1801_AUTO_RAMP_UP + A1801_AUTO_RAMP_DOWN);
          break;

        case 0x08:                                    // IOA3+VDD: Play concatenate
          PlayCon = 1;
          SACM_A1801_Stop();
          A1801PlayIdx = 1;
          SACM_A1801_Play(A1801_User_GetA1801StartAddr(A1801PlayIdx++), A1801_DAC_CH0, A1801_AUTO_RAMP_UP + A1801_AUTO_RAMP_DOWN);
          break;

        case 0x10:                                    // IOA4+VDD:	Pause/Resume
          if((SACM_A1801_GetStatus() & A1801_PAUSE_FLAG) == 0)
          {
            SACM_A1801_Pause();
          }
          else
          {
            SACM_A1801_Resume();
          }
          break;

        case 0x20:                                    // IOA5+VDD:
          SwVolGain = ++SwVolGain >= MAX_VOL_GAIN ? MAX_VOL_GAIN : SwVolGain;
          APP_SwVolCtrl_SetVolGain(0, SwVolGain);
          break;

        case 0x40:                                    // IOA6+VDD:
          SwVolGain = --SwVolGain <= MIN_VOL_GAIN ? MIN_VOL_GAIN : SwVolGain;
          APP_SwVolCtrl_SetVolGain(0, SwVolGain);
          break;

        case 0x80:                                    // IOA7:
          break;
      }
		}

		else if(ScanedKey < 0x10000)
		{
      ScanedKey = ScanedKey >> 8;
      switch(ScanedKey)
      {
        case 0x01:                                     // IOA8: Stop
          PlayCon = 0;
          SACM_A1801_Stop();
          break;

        case 0x02:                                     // IOA9: Play next song
          PlayCon = 0;
          SACM_A1801_Stop();
          A1801PlayIdx = ++A1801PlayIdx > SpeechNum ? 1 : A1801PlayIdx;
          SACM_A1801_Play(A1801_User_GetA1801StartAddr(A1801PlayIdx), A1801_DAC_CH0 , A1801_AUTO_RAMP_UP + A1801_AUTO_RAMP_DOWN);
          break;

        case 0x04:	                                   // IOA10: Play prev song
          PlayCon = 0;
          SACM_A1801_Stop();
          A1801PlayIdx = --A1801PlayIdx < 1 ? SpeechNum : A1801PlayIdx;
          SACM_A1801_Play(A1801_User_GetA1801StartAddr(A1801PlayIdx), A1801_DAC_CH1, A1801_AUTO_RAMP_UP + A1801_AUTO_RAMP_DOWN);
          break;

        case 0x08:                                    // IOA11: Play concatenate
          PlayCon = 1;
          SACM_A1801_Stop();
          A1801PlayIdx = 1;
          SACM_A1801_Play(A1801_User_GetA1801StartAddr(A1801PlayIdx++), A1801_DAC_CH0, A1801_AUTO_RAMP_UP + A1801_AUTO_RAMP_DOWN);
          break;

        case 0x10:                                    // IOA12:	Pause/Resume
          if((SACM_A1801_GetStatus() & A1801_PAUSE_FLAG) == 0)
          {
            SACM_A1801_Pause();
          }
          else
          {
            SACM_A1801_Resume();
          }
          break;

        case 0x20:                                    // IOA13:
          SwVolGain = ++SwVolGain >= MAX_VOL_GAIN ? MAX_VOL_GAIN : SwVolGain;
          APP_SwVolCtrl_SetVolGain(0, SwVolGain);
          break;

        case 0x40:                                    // IOA14:
          SwVolGain = --SwVolGain <= MIN_VOL_GAIN ? MIN_VOL_GAIN : SwVolGain;
          APP_SwVolCtrl_SetVolGain(0, SwVolGain);
          break;

        case 0x80:                                    // IOA15:
          break;
      }
		}

		else if(ScanedKey < 0x1000000)
		{
      ScanedKey = ScanedKey >> 16;
      switch(ScanedKey)
      {
        case 0x01:                                     // IOA16: Stop
          PlayCon = 0;
          SACM_A1801_Stop();
          break;

        case 0x02:                                     // IOA17: Play next song
          PlayCon = 0;
          SACM_A1801_Stop();
          A1801PlayIdx = ++A1801PlayIdx > SpeechNum ? 1 : A1801PlayIdx;
          SACM_A1801_Play(A1801_User_GetA1801StartAddr(A1801PlayIdx), A1801_DAC_CH0 , A1801_AUTO_RAMP_UP + A1801_AUTO_RAMP_DOWN);
          break;

        case 0x04:	                                   // IOA18: Play prev song
          PlayCon = 0;
          SACM_A1801_Stop();
          A1801PlayIdx = --A1801PlayIdx < 1 ? SpeechNum : A1801PlayIdx;
          SACM_A1801_Play(A1801_User_GetA1801StartAddr(A1801PlayIdx), A1801_DAC_CH1, A1801_AUTO_RAMP_UP + A1801_AUTO_RAMP_DOWN);
          break;

        case 0x08:                                    // IOA19: Play concatenate
          PlayCon = 1;
          SACM_A1801_Stop();
          A1801PlayIdx = 1;
          SACM_A1801_Play(A1801_User_GetA1801StartAddr(A1801PlayIdx++), A1801_DAC_CH0, A1801_AUTO_RAMP_UP + A1801_AUTO_RAMP_DOWN);
          break;

        case 0x10:                                    // IOA20:	Pause/Resume
          if((SACM_A1801_GetStatus() & A1801_PAUSE_FLAG) == 0)
          {
            SACM_A1801_Pause();
          }
          else
          {
            SACM_A1801_Resume();
          }
          break;

        case 0x20:                                    // IOA21:
          SwVolGain = ++SwVolGain >= MAX_VOL_GAIN ? MAX_VOL_GAIN : SwVolGain;
          APP_SwVolCtrl_SetVolGain(0, SwVolGain);
          break;

        case 0x40:                                    // IOA22:
          SwVolGain = --SwVolGain <= MIN_VOL_GAIN ? MIN_VOL_GAIN : SwVolGain;
          APP_SwVolCtrl_SetVolGain(0, SwVolGain);
          break;

        case 0x80:                                    // IOA23:
          break;
      }
		}

		else
		{
      ScanedKey = ScanedKey >> 24;
      switch(ScanedKey)
      {
        case 0x01:                                     // IOA24: Stop
          PlayCon = 0;
          SACM_A1801_Stop();
          break;

        case 0x02:                                     // IOA25: Play next song
          PlayCon = 0;
          SACM_A1801_Stop();
          A1801PlayIdx = ++A1801PlayIdx > SpeechNum ? 1 : A1801PlayIdx;
          SACM_A1801_Play(A1801_User_GetA1801StartAddr(A1801PlayIdx), A1801_DAC_CH0 , A1801_AUTO_RAMP_UP + A1801_AUTO_RAMP_DOWN);
          break;

        case 0x04:	                                   // IOA26: Play prev song
          PlayCon = 0;
          SACM_A1801_Stop();
          A1801PlayIdx = --A1801PlayIdx < 1 ? SpeechNum : A1801PlayIdx;
          SACM_A1801_Play(A1801_User_GetA1801StartAddr(A1801PlayIdx), A1801_DAC_CH1, A1801_AUTO_RAMP_UP + A1801_AUTO_RAMP_DOWN);
          break;

        case 0x08:                                    // IOA27: Play concatenate
          PlayCon = 1;
          SACM_A1801_Stop();
          A1801PlayIdx = 1;
          SACM_A1801_Play(A1801_User_GetA1801StartAddr(A1801PlayIdx++), A1801_DAC_CH0, A1801_AUTO_RAMP_UP + A1801_AUTO_RAMP_DOWN);
          break;

        case 0x10:                                    // IOA28:	Pause/Resume
          if((SACM_A1801_GetStatus() & A1801_PAUSE_FLAG) == 0)
          {
            SACM_A1801_Pause();
          }
          else
          {
            SACM_A1801_Resume();
          }
          break;

        case 0x20:                                    // IOA29:
          SwVolGain = ++SwVolGain >= MAX_VOL_GAIN ? MAX_VOL_GAIN : SwVolGain;
          APP_SwVolCtrl_SetVolGain(0, SwVolGain);
          break;

        case 0x40:                                    // IOA30:
          SwVolGain = --SwVolGain <= MIN_VOL_GAIN ? MIN_VOL_GAIN : SwVolGain;
          APP_SwVolCtrl_SetVolGain(0, SwVolGain);
          break;

        case 0x80:                                    // IOA31:
          break;
      }
		}

		if((PlayCon != 0) && (SACM_A1801_Check_Con() == 0))
		{
			SACM_A1801_Play_Con(A1801_User_GetA1801StartAddr(A1801PlayIdx++), A1801_DAC_CH0, A1801_AUTO_RAMP_UP + A1801_AUTO_RAMP_DOWN);
			A1801PlayIdx = A1801PlayIdx > SpeechNum ? 1 : A1801PlayIdx;
		}
	}


  return 0;
}

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



