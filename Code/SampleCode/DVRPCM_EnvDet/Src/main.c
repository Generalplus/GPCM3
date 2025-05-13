/************************************************************************************
 *  File:     main.c
 *  Purpose:  GPCM3/GPFM1 Sample Code
 *            Replace with your code.
 *  Date:     28 June 2023
 *  Info:
 *            IOA.15 for Envelope detect LED: 1: Start Recording 0: Stop Recording
 *
 *
 ************************************************************************************/

/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include <stdio.h>
#include "GPCM3_FM1.h"
#include "SACM_DVRPCM_User.h"
#include "APP_SwVolumeControl.h"
#include "KeyScan.h"

#define START_ADDRESS   0x00200000


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#if defined(__CC_ARM)
#define Aligned4B __align(4)
#elif defined(__GNUC__)
extern unsigned int __boot_table_start; //in GPCM2_CM3.ld
#define Aligned4B __attribute__ ((aligned (4)))
#endif

Aligned4B SACM_DVRPCM_WORKING_RAM     DvrPcmWorkingRam;
Aligned4B int16_t                     DvrDataBuffer[2 * DVRPCM_FRAME_SIZE];
Aligned4B KEYSCAN_WORKING_RAM         KeyScanWorkingRam;

uint32_t ScanedKey;
uint16_t PcmNum = 0;
int16_t PcmPlayIdx = 1;
int8_t PlayCon = 0;
int8_t PauseFlag = 0;
int16_t SwVolGain = 9;

extern uint8_t __user_spifc_load_addr;
extern uint8_t __user_spifc_start;
extern uint32_t __user_spifc_size;
extern uint8_t R_ReleaseFlag;

void MoveSpifcRamCode(void);

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
	MoveSpifcRamCode();

  // GPIO initialize
  GPIO_SetMode(GPIOA, 0xFFFFFFFF, GPIO_MODE_INPUT);         //IOA[31:0] Input pull-low, bypass IOA[14:13] UART & IOA.8 Output
  MODIFY_REG(GPIOA->OBUF, 0xFFFFFFFF, 0);
  GPIO_SetMode(GPIOC, 0xFFFFFFFF, GPIO_MODE_INPUT);         //IOC Input pull-low
  MODIFY_REG(GPIOC->OBUF, 0xFFFFFFFF, 0);
  GPIO_SetMode(GPIOD, 0xFFFFFFFF, GPIO_MODE_INPUT);         //IOD Input pull-low
  MODIFY_REG(GPIOD->OBUF, 0xFFFFFFFF, 0);
  GPIO_SetMode(GPIOA, 0x00008000, GPIO_MODE_OUTPUT);        // IOA.15 for Envelope detect LED
  GPIOA_OBIT->OBIT15 = 0x00000000;                          // IOA.15 = Output Lo.

	APP_SwVolCtrl_Init();                                     // Software volume control init.
	APP_SwVolCtrl_SetVolGain(0, SwVolGain);
	KeyScan_Initial(&KeyScanWorkingRam);                      // key scan init.

	PcmNum = GetPcmNum();
	SACM_DVRPCM_Initial(&DvrPcmWorkingRam, &DvrDataBuffer[0], DVRPCM_FRAME_SIZE);
  SACM_DVRPCM_Play(GetPcmStartAddr(PcmPlayIdx), DVRPCM_DAC_CH0 , DVRPCM_AUTO_RAMP_UP + DVRPCM_AUTO_RAMP_DOWN);

  while(1)
	{
		WDT_Clear();
		SACM_DVRPCM_ServiceLoop();
    if(R_ReleaseFlag == 1)                      // For Envelope Release Detect
    {
      GPIOA_OBIT->OBIT15 = 0x00000000;          // IOA.15 = 0
      SACM_DVRPCM_Stop();
      R_ReleaseFlag = 0;
    }

		KeyScan_ServiceLoop();

		ScanedKey = KeyScan_GetCh();
	  switch(ScanedKey)
		{
			case 0x01:                                 // IOA0+VDD: Record & Envelope detect
				PlayCon = 0;
				SACM_DVRPCM_Stop();

				SACM_DVRPCM_SetRecLength(0x20000);
				SPIFC_BlockErase(START_ADDRESS);
			  SPIFC_BlockErase(START_ADDRESS+0x10000);
			  SACM_DVRPCM_Rec((int16_t *) START_ADDRESS);
			  break;

			case 0x02:                                 // IOA1+VDD: Play recording
				PlayCon = 0;
				SACM_DVRPCM_Stop();
			  SACM_DVRPCM_Play((int16_t *)(0x04000000+START_ADDRESS), DVRPCM_DAC_CH0, DVRPCM_AUTO_RAMP_UP + DVRPCM_AUTO_RAMP_DOWN);
			  break;

			case 0x04:                                 // IOA2+VDD: Play next song
				PlayCon = 0;
				SACM_DVRPCM_Stop();
				SACM_DVRPCM_Play(GetPcmStartAddr(PcmPlayIdx), DVRPCM_DAC_CH0 , DVRPCM_AUTO_RAMP_UP + DVRPCM_AUTO_RAMP_DOWN);
			  PcmPlayIdx = ++PcmPlayIdx > PcmNum ? 1 : PcmPlayIdx;
				break;

			case 0x08:                                 // IOA3+VDD: Stop
				PlayCon = 0;
				SACM_DVRPCM_Stop();
				break;


			case 0x10:                                 // IOA4+VDD: Pause/Resume
				if((SACM_DVRPCM_GetStatus() & DVRPCM_PAUSE_FLAG) == 0)
				{
				  SACM_DVRPCM_Pause();
				}
				else
				{
				  SACM_DVRPCM_Resume();
				}
				break;

      case 0x20:                                           // IOA5+VDD: Volume Up
				SwVolGain = ++SwVolGain >= MAX_VOL_GAIN ? MAX_VOL_GAIN : SwVolGain;
			  APP_SwVolCtrl_SetVolGain(0, SwVolGain);
				break;

			case 0x40:                                           // IOA6+VDD: Volume Dn
			  SwVolGain = --SwVolGain <= MIN_VOL_GAIN ? MIN_VOL_GAIN : SwVolGain;
			  APP_SwVolCtrl_SetVolGain(0, SwVolGain);
			  break;

			case 0x80:                                           // IOA7+VDD:
				break;
	  }

		if((PlayCon != 0) && (SACM_DVRPCM_Check_Con() == 0))
		{
			SACM_DVRPCM_Play_Con(GetPcmStartAddr(PcmPlayIdx++), DVRPCM_DAC_CH0, DVRPCM_AUTO_RAMP_UP + DVRPCM_AUTO_RAMP_DOWN);
			PcmPlayIdx = PcmPlayIdx > PcmNum ? 1 : PcmPlayIdx;
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


