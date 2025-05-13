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
#include "APP_SwVolumeControl.h"
#include "KeyScan.h"


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#if defined(__CC_ARM)
__align(4) SACM_VC3_API_WORKING_RAM Vc3ApiWorkingRam;
__align(4) KEYSCAN_WORKING_RAM KeyScanWorkingRam;
#elif defined(__GNUC__)
extern unsigned int __boot_table_start; //in GPCM3Test.ld
  #define Aligned4B __attribute__ ((aligned (4)))
__attribute__ ((aligned (4))) SACM_VC3_API_WORKING_RAM Vc3ApiWorkingRam;
__attribute__ ((aligned (4))) KEYSCAN_WORKING_RAM KeyScanWorkingRam;
#endif

//============================== RAM Overlap allocate area ==============================
typedef union Lib_RAM_Allocate
{
  struct VC3_RAM      // For RTVC3
  {
    Aligned4B uint8_t Vc3KernelWorkingRam[sizeof(SACM_VC3_KERNEL_RAM)];
  }VC3_RAM;

  struct VC3_VM_RAM   // For RTVC3 VM
  {
    Aligned4B uint8_t Vc3VmKernelWorkingRam[sizeof(SACM_VC3_VM_KERNEL_RAM)];
  }VC3_VM_RAM;

}Lib_RAM_Allocate;

Aligned4B Lib_RAM_Allocate 						LibRAM;
SACM_VC3_KERNEL_RAM       *Vc3KernelWorkingRam  = (SACM_VC3_KERNEL_RAM*)LibRAM.VC3_RAM.Vc3KernelWorkingRam;
SACM_VC3_VM_KERNEL_RAM    *Vc3VmKernelWorkingRam= (SACM_VC3_VM_KERNEL_RAM*)LibRAM.VC3_VM_RAM.Vc3VmKernelWorkingRam;

// RTVC3 RAM initialize
int8_t Vc3Mode = VC3_SHIFT_PITCH_MODE;
int8_t Vc3ShiftPitch = 10;
int8_t Vc3ConstPitch = 0;
int8_t Vc3EchoGain = 6;
int8_t Vc3PlayEn = 1;
uint32_t ScanedKey;
int8_t SwVolGain = 7;
int16_t Vc3PcmBuffer[2*2*VC3_FRAME_SIZE];

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
  SystemInit();
	WRITE_REG(RCU->STS,0x0000FFFF);                             // Clear Reset status

  // GPIO initialize
  GPIO_SetMode(GPIOA, 0xFFFFFFFF, GPIO_MODE_INPUT);         //IOA[31:0] Input pull-low, bypass IOA[14:13] UART & IOA.8 Output
  MODIFY_REG(GPIOA->OBUF, 0xFFFFFFFF, 0);
  GPIO_SetMode(GPIOC, 0xFFFFFFFF, GPIO_MODE_INPUT);         //IOC Input pull-low
  MODIFY_REG(GPIOC->OBUF, 0xFFFFFFFF, 0);
  GPIO_SetMode(GPIOD, 0xFFFFFFFF, GPIO_MODE_INPUT);         //IOD Input pull-low
  MODIFY_REG(GPIOD->OBUF, 0xFFFFFFFF, 0);
  /*
	 * IOA[10:9] output low
	 */
  GPIO_SetMode(GPIOA, 0x00000600, GPIO_MODE_OUTPUT);
  MODIFY_REG(GPIOA->OBUF, 0x00000600, 0);

	APP_SwVolCtrl_Init();                                    // Software volume control init.
	APP_SwVolCtrl_SetVolGain(0, SwVolGain);
	KeyScan_Initial(&KeyScanWorkingRam);                     // key scan init.

	SACM_VC3_Initial(&Vc3ApiWorkingRam,Vc3PcmBuffer);
	SACM_VC3_Mode(Vc3Mode, Vc3KernelWorkingRam);
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
            SACM_VC3_Mode(Vc3Mode, Vc3KernelWorkingRam);
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
          SACM_VC3_Mode(Vc3Mode, Vc3VmKernelWorkingRam);
          break;

					default:
						SACM_VC3_Mode(VC3_USER_DEFINE_MODE1, NULL);
					  break;
				}
        break;

			case 0x10:                                 //
			  break;

			case 0x40:                                 // volume up
				SwVolGain = ++SwVolGain >= MAX_VOL_GAIN ? MAX_VOL_GAIN : SwVolGain;
			  APP_SwVolCtrl_SetVolGain(0, SwVolGain);
				break;

      case 0x80:                                 // volume down
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
