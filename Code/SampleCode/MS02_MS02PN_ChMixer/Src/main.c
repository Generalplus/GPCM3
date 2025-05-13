/************************************************************************************
 *  File:     main.c
 *  Purpose:  GPCM3 MS02 + MS02PN demo code
 *            Replace with your code.
 *  Date:     13 December 2023
 *  Info:
 *
 *
 ************************************************************************************/

/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include <stdio.h>
#include "GPCM3_FM1.h"
#include "SACM_MIXER_Ch1_User.h"
#include "SACM_MS02_User.h"
#include "SACM_MS02PN_User.h"
#include "MS02dec.h"
#include "MS02PNDec.h"
#include "APP_SwVolumeControl.h"
#include "KeyScan.h"


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#if defined(__CC_ARM)
#define Aligned4B __align(4)
__align(4) SACM_MS02_WORKING_RAM MS02WorkingRam;
__align(4) uint8_t MS02KernelRam[MS02_DEC_Memory];
__align(4) SACM_MS02PN_WORKING_RAM MS02PNWorkingRam;
__align(4) uint8_t MS02PNKernelRam[MS02PN_DEC_Memory];
__align(4) KEYSCAN_WORKING_RAM KeyScanWorkingRam;
#elif defined(__GNUC__)
extern unsigned int __boot_table_start; //in GPCM2_CM3.ld
#define Aligned4B __attribute__ ((aligned (4)))
__attribute__ ((aligned (4))) SACM_MS02_WORKING_RAM MS02WorkingRam;
__attribute__ ((aligned (4))) uint8_t MS02KernelRam[MS02_DEC_Memory];
__attribute__ ((aligned (4))) SACM_MS02PN_WORKING_RAM MS02PNWorkingRam;
__attribute__ ((aligned (4))) uint8_t MS02PNKernelRam[MS02PN_DEC_Memory];
__attribute__ ((aligned (4))) KEYSCAN_WORKING_RAM KeyScanWorkingRam;
#endif

Aligned4B	int16_t MicCh1PcmBuffer[(2 * MIXER_CH1_FRAME_SIZE)];        // Ch1 Mixer A/B Buffer.
Aligned4B	SACM_MIXER_CH1_WORKING_RAM	MixerCh1WorkingRam;

int16_t MS02InBuffer[MS02_IN_BUFFER_SIZE];
uint16_t MidiChNum = MS02_CH_Num;
uint16_t MidiPNChNum = MS02PN_CH_Num;

uint8_t AudioOutType = AUD_OUT_PWM;

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
	int8_t PauseFlag = 0;

	// For MS02
	uint16_t MidiNum = 0;
	int16_t MidiIdx = 2;
	int16_t MS02LibIdx =1;
	uint8_t OKONFlag = 0;

	// For MS02PN
  int16_t MS02PNLibIdx =1;
  int16_t MidiGain = 0x40;

  // GPIO initialize
  GPIO_SetMode(GPIOA, 0xFFFFFFFF, GPIO_MODE_INPUT);         //IOA[31:0] Input pull-low, bypass IOA[14:13] UART & IOA.8 Output
  MODIFY_REG(GPIOA->OBUF, 0xFFFFFFFF, 0);
  GPIO_SetMode(GPIOB, 0xFFFFFFFF, GPIO_MODE_INPUT);         //IOB Input pull-low
  MODIFY_REG(GPIOB->OBUF, 0xFFFFFFFF, 0);
  GPIO_SetMode(GPIOC, 0xFFFFFFFF, GPIO_MODE_INPUT);         //IOC Input pull-low
  MODIFY_REG(GPIOC->OBUF, 0xFFFFFFFF, 0);
  GPIO_SetMode(GPIOD, 0xFFFFFFFF, GPIO_MODE_INPUT);         //IOD Input pull-low
  MODIFY_REG(GPIOD->OBUF, 0xFFFFFFFF, 0);

  // SW Volume Ctrl
	int8_t SwVolGain = 3;
	int8_t PNSwVolGain = 13;
	APP_SwVolCtrl_Init();                             // Software volume control init.
	APP_SwVolCtrl_SetVolGain(0, SwVolGain);           // For MS02
	APP_SwVolCtrl_SetVolGain(1, PNSwVolGain);         // For MS02 PN

	WRITE_REG(RCU->STS,0x0000FFFF);                   // Clear Reset status

	// Ch1 Mixer: For MS02 & MS02PN(SR:20K)
	SACM_MIXER_Ch1_Initial(&MixerCh1WorkingRam, MicCh1PcmBuffer, MIXER_CH1_FRAME_SIZE, MIXER_CH1_BITS);
	SACM_MIXER_Ch1_Play(MIXER_CH1_DAC_CH1 , MIXER_CH1_AUTO_RAMP_UP + MIXER_CH1_AUTO_RAMP_DOWN);

	// MS02 initialize
	MidiNum = GetMidiNum();
  SACM_MS02_Initial(&MS02WorkingRam, MS02InBuffer, MicCh1PcmBuffer, MS02KernelRam, MS02_IN_BUFFER_SIZE, MS02_OUT_BUFFER_SIZE, MS02_DEC_Memory, AUD_OUT_PWM, MS02_CH1, MS02_Timer_20K);
	SACM_MS02_SetChannelStatus(0xFFFF);             //Set MIDI Ch 0~15 1:ON/0:OFF
  SACM_MS02_SetDrumStatus(0xFFFF);                //Set drum track 0~15 1:ON/0:OFF
  SACM_MS02_SetKeyChStatus(0xFFFF);               //Set keyshift ch 0~15 1:ON/0:OFF
  SACM_MS02_KeyShift(0);                          //Set initial value of the keyshift
	SACM_MS02_Play(GetMS02LibStartAddr(MS02LibIdx), GetMidiStartAddr(MidiIdx), MS02_AUTO_RAMP_UP + MS02_AUTO_RAMP_DOWN, MS02_DAC_20K);

  SACM_MS02PN_Initial(&MS02PNWorkingRam, MicCh1PcmBuffer, MS02PNKernelRam, GetMS02PNLibStartAddr(MS02PNLibIdx), AudioOutType, MS02_CH1, MS02_AUTO_RAMP_UP + MS02_AUTO_RAMP_DOWN, MS02_DAC_20K);

	KeyScan_Initial(&KeyScanWorkingRam);                     // key scan init.

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
		  case 0x01:                                           // IOA0+VDD: Play Note
		    SACM_MS02PN_PlayNote(5, 0x32, MidiGain, 0xFF);
        break;

			case 0x02:                                           // IOA1+VDD: Play Drum
        SACM_MS02PN_PlayDrum(3, 0x02, MidiGain, 0xFF);
				break;

			case 0x04:                                           // IOA2+VDD: Play next midi
        SACM_MS02_Stop();
				MidiIdx = ++MidiIdx > MidiNum ? 2 : MidiIdx;
				SACM_MS02_Play(GetMS02LibStartAddr(MS02LibIdx), GetMidiStartAddr(MidiIdx), MS02_AUTO_RAMP_UP + MS02_AUTO_RAMP_DOWN, MS02_DAC_20K);
        break;

			case 0x08:                                           // IOA3+VDD: Play current midi
        SACM_MS02_Stop();
				SACM_MS02_Play(GetMS02LibStartAddr(MS02LibIdx), GetMidiStartAddr(MidiIdx), MS02_AUTO_RAMP_UP + MS02_AUTO_RAMP_DOWN, MS02_DAC_20K);
        break;

			case 0x10:                                           // IOA4+VDD:	 MS02 stop
        SACM_MS02_Stop();
        // SACM_MS02PN_Stop();
        // SACM_MIXER_Ch1_Stop();
				break;

			case 0x20:                                 // IOA4+VDD:	Audio PWM gain Dn

				break;

			case 0x40:                                 // IOA6+VDD: MS02 S/W volume Up
				SwVolGain = ++SwVolGain >= MAX_VOL_GAIN ? MAX_VOL_GAIN : SwVolGain;
			  APP_SwVolCtrl_SetVolGain(0, SwVolGain);
			  break;

			case 0x80:                                 // IOA7+VDD: MS02 S/W volume Dn
			  SwVolGain = --SwVolGain <= MIN_VOL_GAIN ? MIN_VOL_GAIN : SwVolGain;
			  APP_SwVolCtrl_SetVolGain(0, SwVolGain);
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

