/************************************************************************************
 *  File:     main.c
 *  Purpose:  GPCM3/GPFM1 Sample Code
 *            Replace with your code.
 *  Date:     31 July 2024
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
__attribute__ ((aligned (4))) SACM_A1801_PCM_BUFFER A1801Ch2PcmBuffer;
__attribute__ ((aligned (4))) KEYSCAN_WORKING_RAM KeyScanWorkingRam;
#endif

uint32_t ScanedKey;
uint8_t AudioOutType = AUD_OUT_DAC;
uint16_t A1801Num = 0;
int16_t A1801PlayIdx = 1;
int16_t A1801Ch2PlayIdx = 2;
int8_t PauseFlag = 0;
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

	A1801Num = A1801_User_GetA1801Num();
	SACM_A1801_Initial(&A1801WorkingRam, &A1801TempBuffer, &A1801PcmBuffer);
	SACM_A1801_CH2_Initial(&A1801Ch2WorkingRam, &A1801TempBuffer, &A1801Ch2PcmBuffer);

	APP_SwVolCtrl_Init();                                    // Software volume control init.
	APP_SwVolCtrl_SetVolGain(0, SwVolGain);                  // Set A1801 CH1 software volume gain
  APP_SwVolCtrl_SetVolGain(1, SwVolGain);                  // Set A1801 CH2 software volume gain
	KeyScan_Initial(&KeyScanWorkingRam);                     // key scan init.

  SACM_A1801_Play(A1801_User_GetA1801StartAddr(A1801PlayIdx), A1801_DAC_CH0 , A1801_AUTO_RAMP_UP + A1801_AUTO_RAMP_DOWN);
  SACM_A1801_CH2_Play(A1801_User_GetA1801StartAddr(A1801Ch2PlayIdx), A1801_DAC_CH1, A1801_AUTO_RAMP_UP + A1801_AUTO_RAMP_DOWN);
  // SACM_A1801_Play(A1801_User_GetA1801StartAddr(A1801PlayIdx), A1801_DAC_CH1 , A1801_AUTO_RAMP_UP + A1801_AUTO_RAMP_DOWN);

  while(1)
	{
		WDT_Clear();
		// SACM_A1801_ServiceLoop();
		A1801_User_A1801WithGpEventServerLoop();
    A1801_CH2_User_A1801Ch2WithGpEvent_ServerLoop();
		KeyScan_ServiceLoop();

		if(SACM_A1801_CheckCpuOverload() != 0)
		{
			SACM_A1801_ClearCpuOverload();
		}
		if(SACM_A1801_CH2_CheckCpuOverload() != 0)
		{
			SACM_A1801_CH2_ClearCpuOverload();
		}

		ScanedKey = KeyScan_GetCh();
	  switch(ScanedKey)
		{
		  case 0x01:                                           // IOA0+VDD: Stop
				PlayCon = 0;
				SACM_A1801_Stop();
				SACM_A1801_CH2_Stop();
        break;

			case 0x02:                                           // IOA1+VDD: Play next song
				PlayCon = 0;
				SACM_A1801_Stop();
				SACM_A1801_CH2_Stop();
        SACM_A1801_Play(A1801_User_GetA1801StartAddr(A1801PlayIdx), A1801_DAC_CH0 , A1801_AUTO_RAMP_UP + A1801_AUTO_RAMP_DOWN);
        SACM_A1801_CH2_Play(A1801_User_GetA1801StartAddr(A1801Ch2PlayIdx), A1801_DAC_CH1, A1801_AUTO_RAMP_UP + A1801_AUTO_RAMP_DOWN);
				break;

			case 0x04:	                                         // IOA2+VDD: A1801_CH2 play next song
				SACM_A1801_Stop();
				SACM_A1801_CH2_Stop();
        SACM_A1801_Play(A1801_User_GetA1801StartAddr(A1801PlayIdx+2), A1801_DAC_CH0 , A1801_AUTO_RAMP_UP + A1801_AUTO_RAMP_DOWN);
        SACM_A1801_CH2_Play(A1801_User_GetA1801StartAddr(A1801Ch2PlayIdx+2), A1801_DAC_CH1, A1801_AUTO_RAMP_UP + A1801_AUTO_RAMP_DOWN);
				break;

			case 0x08:                                           // IOA3+VDD: Play concatenate
				PlayCon = 1;
			  SACM_A1801_Stop();
			  A1801PlayIdx = 1;
			  SACM_A1801_Play(A1801_User_GetA1801StartAddr(A1801PlayIdx++), A1801_DAC_CH0, A1801_AUTO_RAMP_UP + A1801_AUTO_RAMP_DOWN);
			  break;

			case 0x10:                                           // IOA4+VDD:	Pause/Resume
				if(PauseFlag == 0)
				{
					PauseFlag = 1;
				  SACM_A1801_Pause();
					SACM_A1801_CH2_Pause();
				}
				else
				{
					PauseFlag = 0;
				  SACM_A1801_Resume();
					SACM_A1801_CH2_Resume();
				}
				break;

      case 0x20:                                           // IOA5+VDD:
				SwVolGain = ++SwVolGain >= MAX_VOL_GAIN ? MAX_VOL_GAIN : SwVolGain;
			  APP_SwVolCtrl_SetVolGain(0, SwVolGain);
			  APP_SwVolCtrl_SetVolGain(1, SwVolGain);            // Set A1801 CH2 software volume gain
				break;

			case 0x40:
			  SwVolGain = --SwVolGain <= MIN_VOL_GAIN ? MIN_VOL_GAIN : SwVolGain;
			  APP_SwVolCtrl_SetVolGain(0, SwVolGain);
			  APP_SwVolCtrl_SetVolGain(1, SwVolGain);            // Set A1801 CH2 software volume gain
			  break;

			case 0x80:
				break;
	  }

		if((PlayCon != 0) && (SACM_A1801_Check_Con() == 0))
		{
			SACM_A1801_Play_Con(A1801_User_GetA1801StartAddr(A1801PlayIdx++), A1801_DAC_CH0, A1801_AUTO_RAMP_UP + A1801_AUTO_RAMP_DOWN);
			A1801PlayIdx = A1801PlayIdx > A1801Num ? 1 : A1801PlayIdx;
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

