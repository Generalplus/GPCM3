/************************************************************************************
 *  File:     main.c
 *  Purpose:  GPCM3/GPFM1 Sample Code
 *            Replace with your code.
 *  Date:     28 April 2025
 *  Info:
 *
 *
 ************************************************************************************/

/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include <stdio.h>
#include "GPCM3_FM1.h"
#include "SACM_DVRIMA_User.h"
#include "APP_SwVolumeControl.h"
#include "KeyScan.h"


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#if defined(__CC_ARM)
#define Aligned4B __align(4)
#elif defined(__GNUC__)
extern unsigned int __boot_table_start; //in GPCM2_CM3.ld
#define Aligned4B __attribute__ ((aligned (4)))
__attribute__ ((aligned (4))) SACM_DVRIMA_WORKING_RAM DVRIMAWorkingRam;
__attribute__ ((aligned (4))) SACM_DVRIMA_TEMP_RAM DVRIMATempBuffer;
__attribute__ ((aligned (4))) SACM_DVRIMA_PCM_BUFFER DVRIMAPcmBuffer;
__attribute__ ((aligned (4))) KEYSCAN_WORKING_RAM KeyScanWorkingRam;
#endif

uint32_t ScanedKey;
uint16_t SpeechNum = 0;
int16_t DVRIMAPlayIdx = 1;
int8_t PlayCon = 0;
int8_t SwVolGain = 7;

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
  GPIO_SetMode(GPIOC, 0xFFFFFFFF, GPIO_MODE_INPUT);         //IOC Input pull-low
  MODIFY_REG(GPIOC->OBUF, 0xFFFFFFFF, 0);
  GPIO_SetMode(GPIOD, 0xFFFFFFFF, GPIO_MODE_INPUT);         //IOD Input pull-low
  MODIFY_REG(GPIOD->OBUF, 0xFFFFFFFF, 0);

	SpeechNum = DVRIMA_User_GetDVRIMANum();
	SACM_DVRIMA_Initial(&DVRIMAWorkingRam, &DVRIMATempBuffer, &DVRIMAPcmBuffer);

	APP_SwVolCtrl_Init();                                    // Software volume control init.
	APP_SwVolCtrl_SetVolGain(0, SwVolGain);
	KeyScan_Initial(&KeyScanWorkingRam);                     // key scan init.

  SACM_DVRIMA_Play(DVRIMA_User_GetDVRIMAStartAddr(DVRIMAPlayIdx), DVRIMA_DAC_CH0 , DVRIMA_AUTO_RAMP_UP + DVRIMA_AUTO_RAMP_DOWN);
  // SACM_DVRIMA_Play(DVRIMA_User_GetDVRIMAStartAddr(DVRIMAPlayIdx), DVRIMA_DAC_CH1 , DVRIMA_AUTO_RAMP_UP + DVRIMA_AUTO_RAMP_DOWN);

  while(1)
	{
		WDT_Clear();
		SACM_DVRIMA_ServiceLoop();
		KeyScan_ServiceLoop();

		if(SACM_DVRIMA_CheckCpuOverload() != 0)
		{
			SACM_DVRIMA_ClearCpuOverload();
		}

		ScanedKey = KeyScan_GetCh();
	  switch(ScanedKey)
		{
		  case 0x01:                                           // IOA24+VDD: Stop
				PlayCon = 0;
				SACM_DVRIMA_Stop();
        break;

			case 0x02:                                           // IOA25+VDD: Play next song
				PlayCon = 0;
				SACM_DVRIMA_Stop();
			  DVRIMAPlayIdx = ++DVRIMAPlayIdx > SpeechNum ? 1 : DVRIMAPlayIdx;
				SACM_DVRIMA_Play(DVRIMA_User_GetDVRIMAStartAddr(DVRIMAPlayIdx), DVRIMA_DAC_CH0 , DVRIMA_AUTO_RAMP_UP + DVRIMA_AUTO_RAMP_DOWN);
				break;

			case 0x04:	                                         // IOA26+VDD: Play prev song
				PlayCon = 0;
				SACM_DVRIMA_Stop();
			  DVRIMAPlayIdx = --DVRIMAPlayIdx < 1 ? SpeechNum : DVRIMAPlayIdx;
				SACM_DVRIMA_Play(DVRIMA_User_GetDVRIMAStartAddr(DVRIMAPlayIdx), DVRIMA_DAC_CH1, DVRIMA_AUTO_RAMP_UP + DVRIMA_AUTO_RAMP_DOWN);
				break;

			case 0x08:                                           // IOA27+VDD: Play concatenate
				PlayCon = 1;
			  SACM_DVRIMA_Stop();
			  DVRIMAPlayIdx = 1;
			  SACM_DVRIMA_Play(DVRIMA_User_GetDVRIMAStartAddr(DVRIMAPlayIdx++), DVRIMA_DAC_CH0, DVRIMA_AUTO_RAMP_UP + DVRIMA_AUTO_RAMP_DOWN);
			  break;

			case 0x10:                                           // IOA28+VDD:	Pause/Resume
				if((SACM_DVRIMA_GetStatus() & DVRIMA_PAUSE_FLAG) == 0)
				{
				  SACM_DVRIMA_Pause();
				}
				else
				{
				  SACM_DVRIMA_Resume();
				}
				break;

      case 0x20:                                           // IOA29+VDD: Volume Up
				SwVolGain = ++SwVolGain >= MAX_VOL_GAIN ? MAX_VOL_GAIN : SwVolGain;
			  APP_SwVolCtrl_SetVolGain(0, SwVolGain);
				break;

			case 0x40:                                           // IOA30+VDD: Volume Dn
			  SwVolGain = --SwVolGain <= MIN_VOL_GAIN ? MIN_VOL_GAIN : SwVolGain;
			  APP_SwVolCtrl_SetVolGain(0, SwVolGain);
			  break;

			case 0x80:                                           // IOA31+VDD:
				break;
	  }

		if((PlayCon != 0) && (SACM_DVRIMA_Check_Con() == 0))
		{
			SACM_DVRIMA_Play_Con(DVRIMA_User_GetDVRIMAStartAddr(DVRIMAPlayIdx++), DVRIMA_DAC_CH0, DVRIMA_AUTO_RAMP_UP + DVRIMA_AUTO_RAMP_DOWN);
			DVRIMAPlayIdx = DVRIMAPlayIdx > SpeechNum ? 1 : DVRIMAPlayIdx;
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

