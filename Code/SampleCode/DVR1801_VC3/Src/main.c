/************************************************************************************
 *  File:     main.c
 *  Purpose:  GPCM3/GPFM1 Sample Code
 *            Replace with your code.
 *  Date:     23 December 2024
 *  Info:
 *
 *
 ************************************************************************************/

/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include <stdio.h>
#include "GPCM3_FM1.h"
#include "SACM_DVR1801_User.h"
#include "SACM_VC3_User.h"
#include "APP_SwVolumeControl.h"
#include "KeyScan.h"


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#if defined(__CC_ARM)
#define Aligned4B __align(4)
__align(4) SACM_DVR1801_WORKING_RAM Dvr1801WorkingRam;
__align(4) SACM_DVR1801_TEMP_RAM Dvr1801TempBuffer;
__align(4) SACM_DVR1801_PCM_BUFFER Dvr1801PcmBuffer;
__align(4) KEYSCAN_WORKING_RAM KeyScanWorkingRam;
#elif defined(__GNUC__)
extern unsigned int __boot_table_start; //in GPCM2_CM3.ld
#define Aligned4B __attribute__ ((aligned (4)))
__attribute__ ((aligned (4))) SACM_DVR1801_WORKING_RAM Dvr1801WorkingRam;
__attribute__ ((aligned (4))) SACM_DVR1801_TEMP_RAM Dvr1801TempBuffer;
__attribute__ ((aligned (4))) SACM_DVR1801_PCM_BUFFER Dvr1801PcmBuffer;

__attribute__ ((aligned (4))) SACM_VC3_API_WORKING_RAM Vc3ApiWorkingRam;
__attribute__ ((aligned (4))) SACM_VC3_KERNEL_RAM Vc3KernelWorkingRam;
__attribute__ ((aligned (4))) SACM_VC3_VM_KERNEL_RAM Vc3VmKernelWorkingRam;

__attribute__ ((aligned (4))) KEYSCAN_WORKING_RAM KeyScanWorkingRam;
#endif

uint32_t ScanedKey;
uint16_t A1801Num = 0;
int16_t A1801PlayIdx = 1;
uint8_t AudOutType = AUD_OUT_PWM;     //1:PWM 0:DAC Out
int8_t PlayCon = 0;
int16_t SwVolGain = 9;

int8_t Vc3Mode = VC3_SHIFT_PITCH_MODE;
int8_t Vc3ShiftPitch = 8;
int8_t Vc3ConstPitch = 0;
int8_t Vc3EchoGain = 6;
int8_t Vc3PlayEn = 1;
int16_t Vc3PcmBuffer[2*2*VC3_FRAME_SIZE];
short ModeCtrl = 0;

extern uint8_t __user_spifc_load_addr;
extern uint8_t __user_spifc_start;
extern uint32_t __user_spifc_size;

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
  GPIO_SetMode(GPIOB, 0xFFFFFFFF, GPIO_MODE_INPUT);         //IOB Input pull-low
  MODIFY_REG(GPIOB->OBUF, 0xFFFFFFFF, 0);
  GPIO_SetMode(GPIOC, 0xFFFFFFFF, GPIO_MODE_INPUT);         //IOC Input pull-low
  MODIFY_REG(GPIOC->OBUF, 0xFFFFFFFF, 0);
  GPIO_SetMode(GPIOD, 0xFFFFFFFF, GPIO_MODE_INPUT);         //IOD Input pull-low
  MODIFY_REG(GPIOD->OBUF, 0xFFFFFFFF, 0);

	A1801Num = DVR1801_User_GetA1801Num();
	SACM_DVR1801_Initial(&Dvr1801WorkingRam, &Dvr1801TempBuffer, &Dvr1801PcmBuffer);

  SACM_VC3_Initial(&Vc3ApiWorkingRam,Vc3PcmBuffer);
	SACM_VC3_Mode(Vc3Mode, &Vc3KernelWorkingRam);
	SACM_VC3_ShiftPitchMode_SetPitch(Vc3ShiftPitch);

	APP_SwVolCtrl_Init();                                    // Software volume control init.
	APP_SwVolCtrl_SetVolGain(0, SwVolGain);
	KeyScan_Initial(&KeyScanWorkingRam);                     // key scan init.

  SACM_DVR1801_Play(DVR1801_User_GetA1801StartAddr(A1801PlayIdx), DVR1801_DAC_CH0 , DVR1801_AUTO_RAMP_UP + DVR1801_AUTO_RAMP_DOWN);

  while(1)
	{
		WDT_Clear();
		SACM_DVR1801_ServiceLoop();
		KeyScan_ServiceLoop();

		ScanedKey = KeyScan_GetCh();
	  switch(ScanedKey)
		{
		  case 0x01:                                           // IOA24+VDD: Stop
				PlayCon = 0;
				SACM_DVR1801_Stop();
        break;

			case 0x02:                                           // IOA25+VDD: Play next song
				PlayCon = 0;
				SACM_DVR1801_Stop();
				SACM_DVR1801_Play(DVR1801_User_GetA1801StartAddr(A1801PlayIdx), DVR1801_DAC_CH0 , DVR1801_AUTO_RAMP_UP + DVR1801_AUTO_RAMP_DOWN);
			  A1801PlayIdx = ++A1801PlayIdx > A1801Num ? 1 : A1801PlayIdx;
				break;

			case 0x04:	                                         // IOA26+VDD: Play prev song
				PlayCon = 0;
				SACM_DVR1801_Stop();
				SACM_DVR1801_Play(DVR1801_User_GetA1801StartAddr(A1801PlayIdx), DVR1801_DAC_CH1, DVR1801_AUTO_RAMP_UP + DVR1801_AUTO_RAMP_DOWN);
			  A1801PlayIdx = --A1801PlayIdx < 1 ? A1801Num : A1801PlayIdx;
				break;

			case 0x08:                                           // IOA27+VDD: Pause/Resume
				if((SACM_DVR1801_GetStatus() & DVR1801_PAUSE_FLAG) == 0)
				{
				  SACM_DVR1801_Pause();
				}
				else
				{
				  SACM_DVR1801_Resume();
				}
			  break;

			case 0x10:                                           // IOA28+VDD:	Mode change
        if(ModeCtrl == 0)
        {
          ModeCtrl = 1;     // RTVC mode
        }
        else
        {
          ModeCtrl = 0;     // DVR1801 mode
        }
				break;

      case 0x20:                                           // IOA29+VDD: Record
 				PlayCon = 0;
				SACM_DVR1801_Stop();
			  SACM_DVR1801_Initial(&Dvr1801WorkingRam, &Dvr1801TempBuffer, &Dvr1801PcmBuffer);
				SPIFC_BlockErase(0x00200000);
			  SPIFC_BlockErase(0x00210000);
			  SACM_DVR1801_Rec((int16_t *) 0x00200000, DVR1801_RECORD_BITRATE_16000);
				break;

			case 0x40:                                           // IOA30+VDD: Play Recording voice
				PlayCon = 0;
				SACM_DVR1801_Stop();
			  SACM_DVR1801_Initial(&Dvr1801WorkingRam, &Dvr1801TempBuffer, &Dvr1801PcmBuffer);
			  SACM_DVR1801_Play((int16_t *)0x04200000, DVR1801_DAC_CH0, DVR1801_AUTO_RAMP_UP + DVR1801_AUTO_RAMP_DOWN);
			  break;

			case 0x80:                                           // IOA31+VDD: Volume control
				SwVolGain = ++SwVolGain > MAX_VOL_GAIN ? 1 : SwVolGain;
			  APP_SwVolCtrl_SetVolGain(0, SwVolGain);

				break;
	  }

		if((PlayCon != 0) && (SACM_DVR1801_Check_Con() == 0))
		{
			SACM_DVR1801_Play_Con(DVR1801_User_GetA1801StartAddr(A1801PlayIdx++), DVR1801_DAC_CH0, DVR1801_AUTO_RAMP_UP + DVR1801_AUTO_RAMP_DOWN);
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


