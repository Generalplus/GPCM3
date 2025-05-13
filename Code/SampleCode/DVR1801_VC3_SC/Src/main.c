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
#include "SACM_VC3_User.h"
#include "SACM_DVR1801_User.h"
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
__align(4) SACM_VC3_KERNEL_RAM Vc3KernelWorkingRam;
__align(4) SACM_VC3_KERNEL_RAM Vc3KernelWorkingRam2;
__align(4) SACM_VC3_VM_KERNEL_RAM Vc3VmKernelWorkingRam;
__align(4) KEYSCAN_WORKING_RAM KeyScanWorkingRam;
#elif defined(__GNUC__)
extern unsigned int __boot_table_start; //in GPCM2_CM3.ld
#define Aligned4B __attribute__ ((aligned (4)))
__attribute__ ((aligned (4))) SACM_DVR1801_WORKING_RAM gDvr1801WorkingRam;
__attribute__ ((aligned (4))) SACM_DVR1801_TEMP_RAM gDvr1801TempBuffer;
__attribute__ ((aligned (4))) SACM_DVR1801_PCM_BUFFER Dvr1801PcmBuffer;
__attribute__ ((aligned (4))) SACM_VC3_KERNEL_RAM Vc3KernelWorkingRam;
//__attribute__ ((aligned (4))) SACM_VC3_KERNEL_RAM Vc3KernelWorkingRam2;
//__attribute__ ((aligned (4))) SACM_VC3_KERNEL_RAM Vc3VmKernelWorkingRam;
__attribute__ ((aligned (4))) SACM_VC3_API_WORKING_RAM Vc3ApiWorkingRam;
// __attribute__ ((aligned (4))) KEYSCAN_WORKING_RAM KeyScanWorkingRam;
#endif

KEYSCAN_WORKING_RAM KeyScanWorkingRam;
uint32_t ScanedKey;
uint16_t A1801Num = 0;
int16_t A1801PlayIdx = 1;
uint8_t AudOutType = AUD_OUT_PWM;     //1:PWM 0:DAC Out
int8_t PlayCon = 0;

int8_t Vc3Mode = VC3_SHIFT_PITCH_MODE;
int8_t Vc3ChangeSpeed = -6;
int8_t Vc3ShiftPitch = 10;
uint8_t Vc3EffectEn = 0;

int16_t Dvr1801SwVolGain = 6;
int16_t Vc3SwVolGain = 9;

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

	A1801Num = GetA1801Num();
	SACM_DVR1801_Initial(&gDvr1801WorkingRam, &gDvr1801TempBuffer, &Dvr1801PcmBuffer);

	/*
	 * Initialize VC3: speed control + robot + echo
	 */
  SACM_VC3_Initial(&Vc3ApiWorkingRam, &Dvr1801PcmBuffer);
	SACM_VC3_Mode(VC3_CHANGE_SPEED_MODE, &Vc3KernelWorkingRam);
	SACM_VC3_ChangeSpeedMode_SetSpeed(Vc3ChangeSpeed);

  // SACM_VC3_Mode(VC3_SHIFT_PITCH_MODE, &Vc3KernelWorkingRam2);
	// SACM_VC3_ShiftPitchMode_SetPitch(Vc3ShiftPitch);

	// SACM_VC3_Mode(VC3_VM_ROBOT_EFFECT1_MODE, &Vc3VmKernelWorkingRam);
	// SACM_VC3_Mode(VC3_ECHO_MODE, &Vc3KernelWorkingRam2);
  // SACM_VC3_EchoMode_SetGain(4);


	APP_SwVolCtrl_Init();                                    // Software volume control init.
	APP_SwVolCtrl_SetVolGain(0, Dvr1801SwVolGain);
	APP_SwVolCtrl_SetVolGain(1, Vc3SwVolGain);

	KeyScan_Initial(&KeyScanWorkingRam);                     // key scan init.

  SACM_DVR1801_Play(GetA1801StartAddr(A1801PlayIdx++), DVR1801_DAC_CH0, DVR1801_AUTO_RAMP_UP + DVR1801_AUTO_RAMP_DOWN);

  while(1)
	{
		WDT_Clear();
		SACM_DVR1801_ServiceLoop();
		KeyScan_ServiceLoop();

		ScanedKey = KeyScan_GetCh();
	  switch(ScanedKey)
		{
			case 0x01:                                           // IOA0+VDD: VC3 effect enable/disable
				if(Vc3EffectEn == 0)
				{
					Vc3EffectEn = 1;
					A1801_Vc3EffectEnable();
				}
				else
				{
					Vc3EffectEn = 0;
					A1801_Vc3EffectDisable();
				}
			  break;

			case 0x02:                                           // IOA1+VDD: Speed control
				Vc3ChangeSpeed = ++Vc3ChangeSpeed > MAX_VC3_SPEED_CTRL_LIMIT ? MIN_VC3_SPEED_CTRL_LIMIT : Vc3ChangeSpeed;
			  SACM_VC3_ChangeSpeedMode_SetSpeed(Vc3ChangeSpeed);
				break;

			case 0x04:                                           // IOA2+VDD: Record
				PlayCon = 0;
				SACM_DVR1801_Stop();
			  //SPIFC_EnhanModeReset();
			  SACM_DVR1801_Initial(&gDvr1801WorkingRam, &gDvr1801TempBuffer, &Dvr1801PcmBuffer);
				SPIFC_BlockErase(0x00200000);
			  SPIFC_BlockErase(0x00210000);
			  SACM_DVR1801_Rec((int16_t *) 0x00200000, DVR1801_RECORD_BITRATE_16000);
			  break;

			case 0x08:                                           // IOA3+VDD: Play recorded voice
				PlayCon = 0;
				SACM_DVR1801_Stop();
			  SACM_DVR1801_Initial(&gDvr1801WorkingRam, &gDvr1801TempBuffer, &Dvr1801PcmBuffer);
			  SACM_DVR1801_Play((int16_t *)0x04200000, DVR1801_DAC_CH0, DVR1801_AUTO_RAMP_UP + DVR1801_AUTO_RAMP_DOWN);
				break;

		  case 0x10:                                           // IOA4+VDD: Stop
				PlayCon = 0;
				SACM_DVR1801_Stop();
        break;

			case 0x20:                                           // IOA5+VDD: Play next song
			  SACM_DVR1801_Stop();
			  // PlayCon = 1;
        A1801PlayIdx = ++A1801PlayIdx > A1801Num ? 1 : A1801PlayIdx;
			  SACM_DVR1801_Play(GetA1801StartAddr(A1801PlayIdx++), DVR1801_DAC_CH0, DVR1801_AUTO_RAMP_UP + DVR1801_AUTO_RAMP_DOWN);
			  break;

      case 0x40:                                           // IOA6+VDD: VC3 Volume up
				Vc3SwVolGain = ++Vc3SwVolGain >= MAX_VOL_GAIN ? MAX_VOL_GAIN : Vc3SwVolGain;
			  APP_SwVolCtrl_SetVolGain(1, Vc3SwVolGain);
				break;

			case 0x80:                                           // IOA7+VDD: VC3 Volume down
			  Vc3SwVolGain = --Vc3SwVolGain <= MIN_VOL_GAIN ? MIN_VOL_GAIN : Vc3SwVolGain;
			  APP_SwVolCtrl_SetVolGain(1, Vc3SwVolGain);
			  break;
	  }
		if((PlayCon != 0) && (SACM_DVR1801_Check_Con() == 0))
		{
			SACM_DVR1801_Play_Con(GetA1801StartAddr(A1801PlayIdx++), DVR1801_DAC_CH0, DVR1801_AUTO_RAMP_UP + DVR1801_AUTO_RAMP_DOWN);
			if(A1801PlayIdx > A1801Num)
		  {
				PlayCon = 0;
			}
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


