/************************************************************************************
 *  File:     main.c
 *  Purpose:  GPCM3/GPFM1 Sample Code
 *            Replace with your code.
 *  Date:     01 June 2023
 *  Info:
 *
 *
 ************************************************************************************/

/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include <stdio.h>
#include "GPCM3_FM1.h"
#include "SACM_MS02PN_User.h"
#include "SACM_MS02PN_API.h"
#include "MS02PNDec.h"
#include "APP_SwVolumeControl.h"
#include "KeyScan.h"


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#if defined(__CC_ARM)
#define Aligned4B __align(4)
__align(4) SACM_MS02PN_WORKING_RAM MS02PNWorkingRam;
__align(4) uint8_t MS02PNKernelRam[MS02PN_DEC_Memory];
__align(4) KEYSCAN_WORKING_RAM KeyScanWorkingRam;
#elif defined(__GNUC__)
extern unsigned int __boot_table_start; //in GPCM2_CM3.ld
#define Aligned4B __attribute__ ((aligned (4)))
__attribute__ ((aligned (4))) SACM_MS02PN_WORKING_RAM MS02PNWorkingRam;
__attribute__ ((aligned (4))) uint8_t MS02PNKernelRam[MS02PN_DEC_Memory];
__attribute__ ((aligned (4))) KEYSCAN_WORKING_RAM KeyScanWorkingRam;
#endif

uint16_t MidiPNChNum = MS02PN_CH_Num;
int16_t MS02PcmBuffer[2*MS02_OUT_BUFFER_SIZE];

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
	uint32_t ScanedKey;

	int16_t MS02PNLibIdx =1;
	int8_t PNSwVolGain = 9;
	int16_t AudPwmGain = 0X1f;
	int16_t MidiGain = 0x40;
	uint8_t AudioOutType = AUD_OUT_PWM;

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

 	/*
	 * MS02PN init
	 */
  SACM_MS02PN_Initial(&MS02PNWorkingRam, MS02PcmBuffer, MS02PNKernelRam, GetMS02PNLibStartAddr(MS02PNLibIdx), AudioOutType, MS02_CH0, MS02_AUTO_RAMP_UP + MS02_AUTO_RAMP_DOWN, MS02_DAC_20K);

	APP_SwVolCtrl_Init();
  APP_SwVolCtrl_SetVolGain(1, PNSwVolGain);         // For MS02 PN
	// SACM_MS02PN_ChangeInstru(4,3);
	// SACM_MS02PN_ResetReleaseStep(5);           // Reset Ch.5 release step
  // SACM_MS02PN_NoteOff(5);
  // SACM_MS02PN_VibrationRate(32767);
  // SACM_MS02PN_VibrationEnable();
  // SACM_MS02PN_VibrationDisable();
	DAC_SetAudPwmGain(AudPwmGain);

	KeyScan_Initial(&KeyScanWorkingRam);                     // key scan init.

  while(1)
	{
		WDT_Clear();
		SACM_MS02PN_ServiceLoop();
		KeyScan_ServiceLoop();

		ScanedKey = KeyScan_GetCh();
	  switch(ScanedKey)
		{
		  case 0x01:                                 // IOA0+VDD: Play Note
		    SACM_MS02PN_PlayNote(5, 0x32, MidiGain, 0xFF);
		    SACM_MS02PN_PlayNote(4, 0x33, MidiGain, 0xFF);
        break;

			case 0x02:                                 // IOA1+VDD: Play Drum
        SACM_MS02PN_PlayDrum(3, 0x02, MidiGain, 0xFF);
        SACM_MS02PN_PlayDrum(2, 0x02, MidiGain, 0xFF);
				break;

			case 0x04:	                               // IOA2+VDD: Change the volume of channel 5
        SACM_MS02PN_PlayNote(5, 0x32, MidiGain, 0xFF);
        SACM_MS02PN_ChangeVelocity(5,127);
				break;

			case 0x08:                                 // IOA3+VDD: Stop
        SACM_MS02PN_Stop();
			  break;

			case 0x10:                                 // IOA4+VDD:	Set release step of ADSR
        SACM_MS02PN_PlayNote(5, 0x32, MidiGain, 0xFF);
        SACM_MS02PN_SetReleaseStep(5,-1);
			/*
				AudPwmGain+=6;
				if(AudPwmGain >= 63)
				{
				  AudPwmGain = 62;
				}
				DAC_SetAudPwmGain(AudPwmGain);
      */
				break;

			case 0x20:                                 // IOA4+VDD: Set release step of ADSR
      SACM_MS02PN_PlayNote(5, 0x32, MidiGain, 0xFF);
			SACM_MS02PN_SetReleaseStep(5,-32767);
			/*
				AudPwmGain-=6;
				if(AudPwmGain <= 0)
				{
				  AudPwmGain = 0;
				}
				DAC_SetAudPwmGain(AudPwmGain);
      */
				break;

			case 0x40:                                 // IOA6+VDD: MS02PN S/W volume Up
				PNSwVolGain = ++PNSwVolGain > MAX_VOL_GAIN ? 15 : PNSwVolGain;
			  APP_SwVolCtrl_SetVolGain(1, PNSwVolGain);
			  break;

			case 0x80:                                 // IOA7+VDD: MS02PN S/W volume Dn
			  PNSwVolGain = --PNSwVolGain < 2 ? 1 : PNSwVolGain;
			  APP_SwVolCtrl_SetVolGain(1, PNSwVolGain);
				break;
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

