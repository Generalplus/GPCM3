/************************************************************************************
 *  File:     main.c
 *  Purpose:  GPCM3/GPFM1 Sample Code
 *            Replace with your code.
 *  Date:     28 December 2022
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
#include "SACM_A1801_CH2_User.h"
#include "SACM_MIXER_Ch1_User.h"
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
__attribute__ ((aligned (4))) SACM_A1801_WORKING_RAM A1801Ch2WorkingRam;
__attribute__ ((aligned (4))) SACM_A1801_TEMP_RAM A1801TempBuffer;
__attribute__ ((aligned (4))) SACM_A1801_PCM_BUFFER A1801PcmBuffer;

__attribute__ ((aligned (4))) SACM_MIXER_CH1_WORKING_RAM	MixerCh1WorkingRam;
__attribute__ ((aligned (4))) int8_t MicCh1PcmBuffer[(2 * 2 * MIXER_CH1_FRAME_SIZE)];
__attribute__ ((aligned (4))) KEYSCAN_WORKING_RAM KeyScanWorkingRam;
#endif

uint32_t ScanedKey;
uint8_t AudioOutType = AUD_OUT_PWM;
uint16_t A1801Num = 0;
int16_t A1801PlayIdx = 1;
int16_t A1801Ch2PlayIdx = 2;

int8_t PauseFlag = 0;
int8_t PlayCon = 0;
int8_t SwVolGain = 5;
int8_t SwVolGainCh2 = 5;

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

	SACM_MIXER_Ch1_Initial(&MixerCh1WorkingRam, MicCh1PcmBuffer, MIXER_CH1_FRAME_SIZE, MIXER_CH1_BITS);
	SACM_MIXER_Ch1_Play(MIXER_CH1_DAC_CH1 , MIXER_CH1_AUTO_RAMP_UP + MIXER_CH1_AUTO_RAMP_DOWN);

	A1801Num = A1801_User_GetA1801Num();
  SACM_A1801_Initial(&A1801WorkingRam, &A1801TempBuffer, &A1801PcmBuffer);
	SACM_A1801_CH2_Initial(&A1801Ch2WorkingRam, &A1801TempBuffer, &A1801PcmBuffer);

	APP_SwVolCtrl_Init();                                    // Software volume control init.
	APP_SwVolCtrl_SetVolGain(0, SwVolGain);                  // Set A1801 CH1 software volume gain
	APP_SwVolCtrl_SetVolGain(1, SwVolGainCh2);               // Set A1801 CH2 software volume gain

	KeyScan_Initial(&KeyScanWorkingRam);                     // key scan init.

  SACM_A1801_Play(A1801_User_GetA1801StartAddr(A1801PlayIdx), A1801_DAC_CH1 , A1801_AUTO_RAMP_UP + A1801_AUTO_RAMP_DOWN);
	SACM_A1801_CH2_Play(A1801_User_GetA1801StartAddr(A1801Ch2PlayIdx), A1801_DAC_CH1, A1801_AUTO_RAMP_UP + A1801_AUTO_RAMP_DOWN);

  while(1)
	{
		WDT_Clear();
		SACM_MIXER_Ch1_ServiceLoop();
		KeyScan_ServiceLoop();

		if(SACM_MIXER_Ch1_CheckCpuOverload() != 0)
		{
			SACM_MIXER_Ch1_ClearCpuOverload();
		}

		ScanedKey = KeyScan_GetCh();
	  switch(ScanedKey)
		{
		  case 0x01:                                           // IOA24+VDD: Stop all
			  SACM_A1801_Stop();
			  SACM_A1801_CH2_Stop();
        break;

			case 0x02:                                           // IOA25+VDD: Play A1801
				SACM_A1801_Stop();
			  A1801PlayIdx = ++A1801PlayIdx > A1801Num ? 1 : A1801PlayIdx;
        SACM_A1801_Play(A1801_User_GetA1801StartAddr(A1801PlayIdx), A1801_DAC_CH1 , A1801_AUTO_RAMP_UP + A1801_AUTO_RAMP_DOWN);
				break;

			case 0x04:                                           // IOA26+VDD: Play A1801 Ch2
        SACM_A1801_CH2_Stop();
        A1801Ch2PlayIdx = ++A1801Ch2PlayIdx > A1801Num ? 1 : A1801Ch2PlayIdx;
			  SACM_A1801_CH2_Play(A1801_User_GetA1801StartAddr(A1801Ch2PlayIdx), A1801_DAC_CH1, A1801_AUTO_RAMP_UP + A1801_AUTO_RAMP_DOWN);
				break;

			case 0x08:                                           // IOA27+VDD: A1801 2-Ch Pause/Resume
				if(PauseFlag == 0)
				{
					PauseFlag = 1;
					SACM_MIXER_Ch1_Pause();
				}
				else
				{
					PauseFlag = 0;
					SACM_MIXER_Ch1_Resume();
				}
				break;


      case 0x10:                                           // IOA28+VDD:

				break;

			case 0x20:                                           // IOA29+VDD:

			  break;

      case 0x40:                                           // IOA30+VDD: Volume Up
				SwVolGain = ++SwVolGain >= MAX_VOL_GAIN ? MAX_VOL_GAIN : SwVolGain;
			  APP_SwVolCtrl_SetVolGain(0, SwVolGain);               // A1801

				SwVolGainCh2 = ++SwVolGainCh2 >= MAX_VOL_GAIN ? MAX_VOL_GAIN : SwVolGainCh2;
			  APP_SwVolCtrl_SetVolGain(1, SwVolGainCh2);            // A1801 Ch2
				break;

			case 0x80:                                           // IOA31+VDD: Volume Dn
			  SwVolGain = --SwVolGain <= MIN_VOL_GAIN ? MIN_VOL_GAIN : SwVolGain;
			  APP_SwVolCtrl_SetVolGain(0, SwVolGain);               // A1801

			  SwVolGainCh2 = --SwVolGainCh2 <= MIN_VOL_GAIN ? MIN_VOL_GAIN : SwVolGainCh2;
			  APP_SwVolCtrl_SetVolGain(1, SwVolGainCh2);            // A1801 CH2
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

