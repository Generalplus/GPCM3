/************************************************************************************
 *  File:     main.c
 *  Purpose:  GPCM3 Function Test Code
 *            Replace with your code.
 *  Date:     28 July 2022
 *  Info:     If __NO_SYSTEM_INIT is defined in the Build options,
 *            the startup code will not branch to SystemInit()
 *            and the function can be removed
 ************************************************************************************/

/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include <stdio.h>
#include "GPCM3_FM1.h"
#include "SACM_VC3_User.h"
#include "APP_SwVolumeControl.h"
#include "KeyScan.h"
#include "glibc_wrapper.h"

/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#if defined(__CC_ARM)
#define Aligned4B __align(4)
#elif defined(__GNUC__)
extern unsigned int __boot_table_start; //in GPCM2_CM3.ld
#define Aligned4B __attribute__ ((aligned (4)))
#endif

Aligned4B SACM_VC3_API_WORKING_RAM Vc3ApiWorkingRam;
Aligned4B SACM_VC3_PCM_BUFFER Vc3PcmBuffer;
Aligned4B SACM_VC3_KERNEL_RAM Vc3KernelWorkingRam;
Aligned4B SACM_VC3_KERNEL_RAM Vc3KernelWorkingRam2;
Aligned4B SACM_VC3_VM_KERNEL_RAM Vc3VmKernelWorkingRam;

Aligned4B KEYSCAN_WORKING_RAM KeyScanWorkingRam;

int8_t Vc3Mode = VC3_SHIFT_PITCH_MODE;
int8_t Vc3ShiftPitch = 12;
int8_t Vc3ConstPitch = 0;
int8_t Vc3EchoGain = 6;
int8_t Vc3PlayEn = 1;
uint32_t ScanedKey;
int8_t SwVolGain = 5;

extern uint32_t R_AEC_WorkFlag;

//-------------
extern uint8_t __user_spifc_load_addr;
extern uint8_t __user_spifc_start;
extern uint32_t __user_spifc_size;
//-------------

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

	GPIO_SetMode(GPIOA, 0xFFFFFF00, GPIO_MODE_OUTPUT);

	APP_SwVolCtrl_Init();                                    // Software volume control init.
	APP_SwVolCtrl_SetVolGain(0, SwVolGain);

	KeyScan_Initial(&KeyScanWorkingRam);                     // key scan init.

	SACM_VC3_Initial(&Vc3ApiWorkingRam, &Vc3PcmBuffer);
	SACM_VC3_Mode(Vc3Mode, &Vc3KernelWorkingRam);
	SACM_VC3_ShiftPitchMode_SetPitch(Vc3ShiftPitch);
	SACM_VC3_Play(VC3_DAC_CH0, VC3_AUTO_RAMP_UP + VC3_AUTO_RAMP_DOWN);

  while(1)
	{
		WDT_Clear();
		SACM_VC3_ServiceLoop();
		KeyScan_ServiceLoop();

		if(SACM_VC3_CheckCpuOverload() != 0)
		{
			SACM_VC3_ClearCpuOverload();
		}

	ScanedKey = KeyScan_GetCh();
	  switch(ScanedKey)
		{
			case 0x01:                                 // VC3 play/stop
				if(Vc3PlayEn == 0)
				{
					Vc3PlayEn = 1;
			    SACM_VC3_Play(VC3_DAC_CH0, VC3_AUTO_RAMP_UP + VC3_AUTO_RAMP_DOWN);
				}
				else
				{
					Vc3PlayEn = 0;
				  SACM_VC3_Stop();
				}
				break;

			case 0x02:                                 // VC3 effect parameter up
				switch(Vc3Mode)
				{
					case VC3_SHIFT_PITCH_MODE:
				    Vc3ShiftPitch++;
			      if(Vc3ShiftPitch >= MAX_VC3_SHIFT_PITCH_LIMIT)
				    {
					    Vc3ShiftPitch = MAX_VC3_SHIFT_PITCH_LIMIT;
				    }
            SACM_VC3_ShiftPitchMode_SetPitch(Vc3ShiftPitch);
						break;

					case VC3_CONST_PITCH_MODE:
				    Vc3ConstPitch++;
			      if(Vc3ConstPitch >= MAX_VC3_CONST_PITCH_LIMIT)
				    {
					    Vc3ConstPitch = MAX_VC3_CONST_PITCH_LIMIT;
				    }
						SACM_VC3_ConstPitchMode_SetPitch(Vc3ConstPitch);
						break;

					case VC3_ECHO_MODE:
				    Vc3EchoGain++;
			      if(Vc3EchoGain >= MAX_VC3_ECHO_GAIN_LIMIT)
				    {
					    Vc3EchoGain = MAX_VC3_ECHO_GAIN_LIMIT;
				    }
            SACM_VC3_EchoMode_SetGain(Vc3EchoGain);
						break;
				}
			  break;

			case 0x04:                                 // VC3 effect parameter down
				switch(Vc3Mode)
				{
					case VC3_SHIFT_PITCH_MODE:
				    Vc3ShiftPitch--;
			      if(Vc3ShiftPitch <= MIN_VC3_SHIFT_PITCH_LIMIT)
				    {
					    Vc3ShiftPitch = MIN_VC3_SHIFT_PITCH_LIMIT;
				    }
            SACM_VC3_ShiftPitchMode_SetPitch(Vc3ShiftPitch);
						break;

					case VC3_CONST_PITCH_MODE:
				    Vc3ConstPitch--;
			      if(Vc3ConstPitch <= MIN_VC3_CONST_PITCH_LIMIT)
				    {
					    Vc3ConstPitch = MIN_VC3_CONST_PITCH_LIMIT;
				    }
            SACM_VC3_ConstPitchMode_SetPitch(Vc3ConstPitch);
						break;

					case VC3_ECHO_MODE:
				    Vc3EchoGain--;
			      if(Vc3EchoGain <= MIN_VC3_ECHO_GAIN_LIMIT)
				    {
					    Vc3EchoGain = MIN_VC3_ECHO_GAIN_LIMIT;
				    }
            SACM_VC3_EchoMode_SetGain(Vc3EchoGain);
						break;
				}
				break;

		  case 0x08:                                 // change VC3 mode
				Vc3Mode++;
			  if(Vc3Mode > VC3_USER_DEFINE_MODE1)
				{
					Vc3Mode = VC3_SHIFT_PITCH_MODE;
				}

				switch(Vc3Mode)
				{
					case VC3_SHIFT_PITCH_MODE:
					case VC3_CONST_PITCH_MODE:
          case VC3_ECHO_MODE:
            SACM_VC3_Mode(Vc3Mode, &Vc3KernelWorkingRam);
				    SACM_VC3_ShiftPitchMode_SetPitch(Vc3ShiftPitch);
				    SACM_VC3_ConstPitchMode_SetPitch(Vc3ConstPitch);
				    SACM_VC3_EchoMode_SetGain(Vc3EchoGain);
						break;

					case VC3_VM_ROBOT_EFFECT1_MODE:
					case VC3_VM_ROBOT_EFFECT2_MODE:
					case VC3_VM_STRANGE_TONE_MODE:
					case VC3_VM_DJ_EFFECT_MODE:
					case VC3_VM_VIBRATION_MODE:
					case VC3_VM_JET_PLANE_MODE:
					case VC3_VM_DOUBLING_MODE:
					case VC3_VM_UNDERWATER_MODE:
            SACM_VC3_Mode(Vc3Mode, &Vc3VmKernelWorkingRam);
						break;

					default:
						SACM_VC3_Mode(VC3_USER_DEFINE_MODE1, NULL);
					  break;
				}
        break;

			case 0x10:                                 // "shift pitch + echo" mode
				SACM_VC3_Stop();
			  SACM_VC3_Mode(VC3_SHIFT_PITCH_MODE, &Vc3KernelWorkingRam);
			  SACM_VC3_ShiftPitchMode_SetPitch(10);
	      SACM_VC3_Mode(VC3_ECHO_MODE, &Vc3KernelWorkingRam2);
	      SACM_VC3_EchoMode_SetGain(8);
			  // SACM_VC3_Mode(VC3_USER_DEFINE_MODE2, NULL);
			  SACM_VC3_Play(VC3_DAC_CH0, VC3_AUTO_RAMP_UP + VC3_AUTO_RAMP_DOWN);
			  break;

			case 0x40:                                 // volume up
				//SwVolGain = ++SwVolGain >= MAX_VOL_GAIN ? MAX_VOL_GAIN : SwVolGain;
			  //APP_SwVolCtrl_SetVolGain(0, SwVolGain);
			  R_AEC_WorkFlag = 1;
				break;

      case 0x80:                                 // volume down
			  //SwVolGain = --SwVolGain <= MIN_VOL_GAIN ? MIN_VOL_GAIN : SwVolGain;
			  //APP_SwVolCtrl_SetVolGain(0, SwVolGain);
			  R_AEC_WorkFlag = 0;
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

