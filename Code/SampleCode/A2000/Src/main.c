/************************************************************************************
 *  File:     A2000 algorithm
 *  Purpose:  GPCM3/GPFM1 Sample Code
 *            Replace with your code.
 *  Date:     28 December 2022
 *  Info:
 *            IOA.[27:24] 32K SR & 32Kbps
 *            IOA.[31:28] 20K SR & 48Kbps
 *
 *
 ************************************************************************************/

/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include <stdio.h>
#include "GPCM3_FM1.h"
#include "SACM_A2000_User.h"
#include "APP_SwVolumeControl.h"
#include "KeyScan.h"


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#if defined(__CC_ARM)
#define Aligned4B __align(4)
__align(4) SACM_A2000_WORKING_RAM A2000WorkingRam;
__align(4) SACM_A2000_TEMP_RAM A2000TempBuffer;
__align(4) SACM_A2000_PCM_BUFFER A2000PcmBuffer;
__align(4) KEYSCAN_WORKING_RAM KeyScanWorkingRam;
#elif defined(__GNUC__)
extern unsigned int __boot_table_start; //in GPCM2_CM3.ld
#define Aligned4B __attribute__ ((aligned (4)))
__attribute__ ((aligned (4))) SACM_A2000_WORKING_RAM A2000WorkingRam;
__attribute__ ((aligned (4))) SACM_A2000_TEMP_RAM A2000TempBuffer;
__attribute__ ((aligned (4))) SACM_A2000_PCM_BUFFER A2000PcmBuffer;

__attribute__ ((aligned (4))) KEYSCAN_WORKING_RAM KeyScanWorkingRam;
#endif

uint32_t ScanedKey;

uint16_t A20ChNum = 0;
int16_t A2000PlayIdx = 1;
int8_t PlayCon = 0;

int8_t A20SwVolGain = 8;           // for A2000

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

	// WDT_Close();

  // GPIO initialize
  GPIO_SetMode(GPIOA, 0xFFFFFFFF, GPIO_MODE_INPUT);         //IOA[31:0] Input pull-low, bypass IOA[14:13] UART & IOA.8 Output
  MODIFY_REG(GPIOA->OBUF, 0xFFFFFFFF, 0);
  GPIO_SetMode(GPIOC, 0xFFFFFFFF, GPIO_MODE_INPUT);         //IOC Input pull-low
  MODIFY_REG(GPIOC->OBUF, 0xFFFFFFFF, 0);
  GPIO_SetMode(GPIOD, 0xFFFFFFFF, GPIO_MODE_INPUT);         //IOD Input pull-low
  MODIFY_REG(GPIOD->OBUF, 0xFFFFFFFF, 0);

  GPIO_SetMode(GPIOA, 0x00000100, GPIO_MODE_OUTPUT);         //IOA[8]  IOA.8 Output
  //MODIFY_REG(GPIOA->OBUF, 0x00000100, 0x00000100);
  MODIFY_REG(GPIOA->OBUF, 0x00000100, 0x00000000);

  // A2000 initialize
	A20ChNum = A2000_User_GetA2000Num();
	// A20ChNum = 24;
	SACM_A2000_Initial(&A2000WorkingRam, &A2000TempBuffer, &A2000PcmBuffer);

	APP_SwVolCtrl_Init();                                    // Software volume control init.
	APP_SwVolCtrl_SetVolGain(0, A20SwVolGain);
	KeyScan_Initial(&KeyScanWorkingRam);                     // key scan init.

	// SACM_A2000_Play(A2000_User_GetA2000StartAddr(A2000PlayIdx), A2000_DAC_CH0 , A2000_AUTO_RAMP_UP + A2000_AUTO_RAMP_DOWN);
	SACM_A2000_Play(A2000_User_GetA2000StartAddr(A2000PlayIdx), A2000_DAC_CH1 , A2000_AUTO_RAMP_UP + A2000_AUTO_RAMP_DOWN);

  while(1)
	{
		WDT_Clear();
		// SACM_A2000_ServiceLoop();
		A2000_User_A2000WithGpEventServerLoop();
		KeyScan_ServiceLoop();

		if(SACM_A2000_CheckCpuOverload() != 0)
		{
			SACM_A2000_ClearCpuOverload();
		}

		ScanedKey = KeyScan_GetCh();
	  switch(ScanedKey)
		{
			case 0x01:	                                         // IOA24+VDD: Stop
        PlayCon = 0;
        SACM_A2000_Stop();
				break;

			case 0x02:	                                         // IOA25+VDD:  Play next song
				PlayCon = 0;
				SACM_A2000_Stop();
			  A2000PlayIdx = ++A2000PlayIdx > A20ChNum ? 1 : A2000PlayIdx;
				SACM_A2000_Play(A2000_User_GetA2000StartAddr(A2000PlayIdx), A2000_DAC_CH0 , A2000_AUTO_RAMP_UP + A2000_AUTO_RAMP_DOWN);
				break;

      case 0x04:                                           // IOA26+VDD:  Play prev song
				PlayCon = 0;
				SACM_A2000_Stop();
			  A2000PlayIdx = --A2000PlayIdx < 1 ? A20ChNum : A2000PlayIdx;
				SACM_A2000_Play(A2000_User_GetA2000StartAddr(A2000PlayIdx), A2000_DAC_CH0 , A2000_AUTO_RAMP_UP + A2000_AUTO_RAMP_DOWN);
        break;

		  case 0x08:                                           // IOA27+VDD:  Play concatenate
		    PlayCon = 1;
        SACM_A2000_Stop();
        SACM_A2000_Play(A2000_User_GetA2000StartAddr(A2000PlayIdx), A2000_DAC_CH0 , A2000_AUTO_RAMP_UP + A2000_AUTO_RAMP_DOWN);
        break;

			case 0x10:                                           // IOA28+VDD: Play A2000
				if((SACM_A2000_GetStatus() & A2000_PAUSE_FLAG) == 0)
				{
				  SACM_A2000_Pause();
				}
				else
				{
				  SACM_A2000_Resume();
				}
			  break;

			case 0x20:                                           // IOA29+VDD: Play A1801
			  break;

      case 0x40:                                           // IOA30+VDD: Volume Up
				A20SwVolGain = ++A20SwVolGain >= MAX_VOL_GAIN ? MAX_VOL_GAIN : A20SwVolGain;
			  APP_SwVolCtrl_SetVolGain(0, A20SwVolGain);
				break;

			case 0x80:                                           // IOA31+VDD: Volume Dn
			  A20SwVolGain = --A20SwVolGain <= MIN_VOL_GAIN ? MIN_VOL_GAIN : A20SwVolGain;
			  APP_SwVolCtrl_SetVolGain(0, A20SwVolGain);
			  break;
	  }

		if((PlayCon != 0) && (SACM_A2000_Check_Con() == 0))
		{
			SACM_A2000_Play_Con(A2000_User_GetA2000StartAddr(A2000PlayIdx++), A2000_DAC_CH0, A2000_AUTO_RAMP_UP + A2000_AUTO_RAMP_DOWN);
			A2000PlayIdx = A2000PlayIdx > A20ChNum ? 1 : A2000PlayIdx;
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

