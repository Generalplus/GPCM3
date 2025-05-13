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
#include "SACM_A3400Pro_User.h"
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
Aligned4B int16_t  											A3400ProBuffer[2 * A3400PRO_FRAME_SIZE];
KEYSCAN_WORKING_RAM 										KeyScanWorkingRam;

uint32_t ScanedKey;
uint16_t A3400ProNum = 0;
int16_t A3400ProPlayIdx = 1;
int8_t PlayCon = 0;
int8_t PauseFlag = 0;
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

	/*
	 * A3400Pro init
	 */
	A3400ProNum = GetA3400ProNum();
	SACM_A3400Pro_Initial(&A3400ProWorkingRam, &A3400ProBuffer[0], A3400PRO_FRAME_SIZE);

	APP_SwVolCtrl_Init();                                    // Software volume control init.
	APP_SwVolCtrl_SetVolGain(0, SwVolGain);
	KeyScan_Initial(&KeyScanWorkingRam);                     // key scan init.

  SACM_A3400Pro_Play(GetA3400ProStartAddr(A3400ProPlayIdx), A3400PRO_DAC_CH0 , A3400PRO_AUTO_RAMP_UP + A3400PRO_AUTO_RAMP_DOWN);

  while(1)
	{
 		SACM_A3400Pro_ServiceLoop();
		KeyScan_ServiceLoop();
		WDT_Clear();

		ScanedKey = KeyScan_GetCh();
	  switch(ScanedKey)
		{
		  case 0x01:                                 						// IOA0+VDD: Stop
				PlayCon = 0;
				SACM_A3400Pro_Stop();
        break;

			case 0x02:                                 						// IOA1+VDD: A3400Pro play next song
				PlayCon = 0;
				SACM_A3400Pro_Stop();
			  A3400ProPlayIdx = ++A3400ProPlayIdx > A3400ProNum ? 1 : A3400ProPlayIdx;
				SACM_A3400Pro_Play(GetA3400ProStartAddr(A3400ProPlayIdx), A3400PRO_DAC_CH0 , A3400PRO_AUTO_RAMP_UP + A3400PRO_AUTO_RAMP_DOWN);
				break;

			case 0x04:                                 						// IOA2+VDD: A3400Pro play prev song
				PlayCon = 0;
				SACM_A3400Pro_Stop();
			  A3400ProPlayIdx = --A3400ProPlayIdx < 1 ? A3400ProNum : A3400ProPlayIdx;
				SACM_A3400Pro_Play(GetA3400ProStartAddr(A3400ProPlayIdx), A3400PRO_DAC_CH0 , A3400PRO_AUTO_RAMP_UP + A3400PRO_AUTO_RAMP_DOWN);
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
				}
				else
				{
					PauseFlag = 0;
				  SACM_A3400Pro_Resume();
				}
				break;

      case 0x20:
				SwVolGain = ++SwVolGain >= MAX_VOL_GAIN ? MAX_VOL_GAIN : SwVolGain;
			  APP_SwVolCtrl_SetVolGain(0, SwVolGain);
				break;

			case 0x40:
			  SwVolGain = --SwVolGain <= MIN_VOL_GAIN ? MIN_VOL_GAIN : SwVolGain;
			  APP_SwVolCtrl_SetVolGain(0, SwVolGain);
			  break;

			case 0x80:
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

