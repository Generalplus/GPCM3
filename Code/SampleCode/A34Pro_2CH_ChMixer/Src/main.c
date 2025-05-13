/************************************************************************************
 *  File:     main.c
 *  Purpose:  GPCM3/GPFM1 Sample Code
 *            Replace with your code.
 *  Date:     06 June 2023
 *  Info:
 *
 *
 ************************************************************************************/

/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include <stdio.h>
#include "GPCM3_FM1.h"
#include "SACM_A3400Pro_User.h"
#include "SACM_A3400Pro_CH2_User.h"
#include "SACM_MIXER_Ch0_User.h"
#include "APP_SwVolumeControl.h"
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

Aligned4B SACM_A3400PRO_WORKING_RAM			A3400ProWorkingRam;
Aligned4B SACM_A3400PRO_WORKING_RAM			A3400ProCh2WorkingRam;

#if (MIXER_CH0_BITS == MIXER_CH0_MIC_ON)
	Aligned4B	int16_t MicCh0PcmBuffer[(2 * MIXER_CH0_FRAME_SIZE) + (2 * MIXER_CH0_FRAME_SIZE)];
#else
	Aligned4B	int16_t MicCh0PcmBuffer[(2 * MIXER_CH0_FRAME_SIZE)];        // Ch0 Mixer A/B Buffer.
#endif
Aligned4B	SACM_MIXER_CH0_WORKING_RAM	MixerCh0WorkingRam;

KEYSCAN_WORKING_RAM 										KeyScanWorkingRam;

uint32_t ScanedKey;
uint16_t A3400ProNum = 0;
int16_t A3400ProPlayIdx = 1;
int16_t A3400ProCh2PlayIdx = 2;
int8_t PlayCon = 0;
int8_t PauseFlag = 0;
int8_t A34Ch0SwVolGain = 7;
int8_t A34Ch1SwVolGain = 7;

uint8_t AudioOutType = AUD_OUT_PWM;
// int8_t ModeCtrl = MIC_EN;       // Mic ADC enabled.
int8_t ModeCtrl = 0;            // Mic ADC disabled.

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

  SACM_MIXER_Ch0_Initial(&MixerCh0WorkingRam, MicCh0PcmBuffer, MIXER_CH0_FRAME_SIZE, MIXER_CH0_BITS);
	SACM_MIXER_Ch0_Play(MIXER_CH0_DAC_CH0, MIXER_CH0_AUTO_RAMP_UP + MIXER_CH0_AUTO_RAMP_DOWN);

	/*
	 * A3400Pro init
	 */
	A3400ProNum = GetA3400ProNum();
	SACM_A3400Pro_Initial(&A3400ProWorkingRam, &MicCh0PcmBuffer[0], A3400PRO_FRAME_SIZE);
	SACM_A3400Pro_CH2_Initial(&A3400ProCh2WorkingRam, &MicCh0PcmBuffer[0], A3400PRO_FRAME_SIZE);

	APP_SwVolCtrl_Init();                                    // Software volume control init.
	APP_SwVolCtrl_SetVolGain(0, A34Ch0SwVolGain);
	APP_SwVolCtrl_SetVolGain(1, A34Ch1SwVolGain);
	KeyScan_Initial(&KeyScanWorkingRam);                     // key scan init.

  SACM_A3400Pro_Play(GetA3400ProStartAddr(A3400ProPlayIdx), A3400PRO_DAC_CH0 , A3400PRO_AUTO_RAMP_UP + A3400PRO_AUTO_RAMP_DOWN);
  SACM_A3400Pro_CH2_Play(GetA3400ProCh2StartAddr(A3400ProCh2PlayIdx), A3400PRO_DAC_CH0, A3400PRO_AUTO_RAMP_UP + A3400PRO_AUTO_RAMP_DOWN);

  while(1)
	{
    SACM_MIXER_Ch0_ServiceLoop();
		KeyScan_ServiceLoop();
		WDT_Clear();

		if(SACM_MIXER_Ch0_CheckCpuOverload() != 0)
		{
			SACM_MIXER_Ch0_ClearCpuOverload();
		}

		ScanedKey = KeyScan_GetCh();
	  switch(ScanedKey)
		{
		  case 0x01:                                 						// IOA0+VDD: Stop
				PlayCon = 0;
				SACM_A3400Pro_Stop();
				SACM_A3400Pro_CH2_Stop();
        break;

			case 0x02:                                 						// IOA1+VDD: A3400Pro play next song
				PlayCon = 0;
				SACM_A3400Pro_Stop();
			  A3400ProPlayIdx = ++A3400ProPlayIdx > A3400ProNum ? 1 : A3400ProPlayIdx;
				SACM_A3400Pro_Play(GetA3400ProStartAddr(A3400ProPlayIdx), A3400PRO_DAC_CH0 , A3400PRO_AUTO_RAMP_UP + A3400PRO_AUTO_RAMP_DOWN);
				break;

			case 0x04:                                 						// IOA2+VDD: A3400Pro_CH2 play next song
				PlayCon = 0;
				SACM_A3400Pro_CH2_Stop();
			  A3400ProCh2PlayIdx = ++A3400ProCh2PlayIdx > A3400ProNum ? 1 : A3400ProCh2PlayIdx;
				SACM_A3400Pro_CH2_Play(GetA3400ProCh2StartAddr(A3400ProCh2PlayIdx), A3400PRO_DAC_CH1, A3400PRO_AUTO_RAMP_UP + A3400PRO_AUTO_RAMP_DOWN);
				break;

			case 0x08:                                 						// IOA3+VDD: A3400Pro Play concatenate
				PlayCon = 1;
			  SACM_A3400Pro_Stop();
			  A3400ProPlayIdx = 1;
			  SACM_A3400Pro_Play(GetA3400ProStartAddr(A3400ProPlayIdx++), A3400PRO_DAC_CH0, A3400PRO_AUTO_RAMP_UP + A3400PRO_AUTO_RAMP_DOWN);
			  break;

			case 0x10:                                 						// IOA4+VDD:	Pause/Resume
				if(PauseFlag == 0)
				{
					PauseFlag = 1;
				  SACM_A3400Pro_Pause();
				  SACM_A3400Pro_CH2_Pause();
				}
				else
				{
					PauseFlag = 0;
				  SACM_A3400Pro_Resume();
				  SACM_A3400Pro_CH2_Resume();
				}
				break;

      case 0x20:                                 						// IOA5+VDD:	Volume Up
				A34Ch0SwVolGain = ++A34Ch0SwVolGain >= MAX_VOL_GAIN ? MAX_VOL_GAIN : A34Ch0SwVolGain;
				A34Ch1SwVolGain = ++A34Ch1SwVolGain >= MAX_VOL_GAIN ? MAX_VOL_GAIN : A34Ch1SwVolGain;
			  APP_SwVolCtrl_SetVolGain(0, A34Ch0SwVolGain);            	// Set A3400Pro CH1 software volume gain
			  APP_SwVolCtrl_SetVolGain(1, A34Ch1SwVolGain);            	// Set A3400Pro CH2 software volume gain
				break;

			case 0x40:                                 						// IOA6+VDD:	Volume Dn
        A34Ch0SwVolGain = --A34Ch0SwVolGain <= MIN_VOL_GAIN ? MIN_VOL_GAIN : A34Ch0SwVolGain;
			  A34Ch1SwVolGain = --A34Ch1SwVolGain <= MIN_VOL_GAIN ? MIN_VOL_GAIN : A34Ch1SwVolGain;
			  APP_SwVolCtrl_SetVolGain(0, A34Ch0SwVolGain);            	// Set A3400Pro CH1 software volume gain
			  APP_SwVolCtrl_SetVolGain(1, A34Ch1SwVolGain);            	// Set A3400Pro CH2 software volume gain
			  break;

			case 0x80:                                 						// IOA7+VDD:
				break;
	  }
		if((PlayCon != 0) && (SACM_A3400Pro_Check_Con() == 0))
		{
			A3400ProPlayIdx = ++A3400ProPlayIdx > A3400ProNum ? 1 : A3400ProPlayIdx;
			SACM_A3400Pro_Play_Con(GetA3400ProStartAddr(A3400ProPlayIdx), A3400PRO_DAC_CH0, A3400PRO_AUTO_RAMP_UP + A3400PRO_AUTO_RAMP_DOWN);
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

