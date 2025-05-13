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
#include "KeyScan.h"
#include "MIDI.h"


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#if defined(__CC_ARM)
#define Aligned4B __align(4)
__align(4) KEYSCAN_WORKING_RAM KeyScanWorkingRam;
__align(4) MIDI_DRIVER_WORKING_RAM DvrMIDIWorkingRamPtr;
#elif defined(__GNUC__)
extern unsigned int __boot_table_start; //in GPCM2_CM3.ld
#define Aligned4B __attribute__ ((aligned (4)))
__attribute__ ((aligned (4))) KEYSCAN_WORKING_RAM KeyScanWorkingRam;
__attribute__ ((aligned (4))) MIDI_DRIVER_WORKING_RAM DvrMIDIWorkingRamPtr;
#endif

extern uint8_t __user_spifc_load_addr;
extern uint8_t __user_spifc_start;
extern uint32_t __user_spifc_size;

uint32_t ScanedKey;
uint32_t R_MIDI_Index = 0;

/*---------------------------------------------------------------------------------------
 * Main Function
 *---------------------------------------------------------------------------------------*/
int main()
{
	WRITE_REG(RCU->STS,0x0000FFFF);                             // Clear Reset status

  // GPIO initialize
  GPIO_SetMode(GPIOA, 0xFFFFFFFF, GPIO_MODE_INPUT);         //IOA[31:0] Input pull-low, bypass IOA[14:13] UART & IOA.8 Output
  MODIFY_REG(GPIOA->OBUF, 0xFFFFFFFF, 0);
  GPIO_SetMode(GPIOB, 0xFFFFFFFF, GPIO_MODE_INPUT);         //IOB Input pull-low
  MODIFY_REG(GPIOB->OBUF, 0xFFFFFFFF, 0);
  GPIO_SetMode(GPIOC, 0xFFFFFFFF, GPIO_MODE_INPUT);         //IOC Input pull-low
  MODIFY_REG(GPIOC->OBUF, 0xFFFFFFFF, 0);
  GPIO_SetMode(GPIOD, 0xFFFFFFFF, GPIO_MODE_INPUT);         //IOD Input pull-low
  MODIFY_REG(GPIOD->OBUF, 0xFFFFFFFF, 0);

  UART_Init(UART0, UART0_IOA13_14, 1500000);                // For GPCM3 => 115200*81920KHz/48000KHz for GPCM3
  printf(" Reset..\r\n");

	KeyScan_Initial(&KeyScanWorkingRam);                     // key scan init.
  SPU_Initial();
  MIDI_Initial(&DvrMIDIWorkingRamPtr);
  MIDI_Play(&DvrMIDIWorkingRamPtr, R_MIDI_Index);

  while(1)
	{
		WDT_Clear();
		MIDI_ServiceLoop();
		KeyScan_ServiceLoop();

		ScanedKey = KeyScan_GetCh();
	  switch(ScanedKey)
		{
		  case 0x01:                                           // IOA0+VDD: Stop
        R_MIDI_Index = 0;
        MIDI_Initial(&DvrMIDIWorkingRamPtr);
        MIDI_Play(&DvrMIDIWorkingRamPtr, R_MIDI_Index);
        break;

			case 0x02:
        R_MIDI_Index = 1;
        MIDI_Initial(&DvrMIDIWorkingRamPtr);
        MIDI_Play(&DvrMIDIWorkingRamPtr, R_MIDI_Index);
				break;

			case 0x04:
        R_MIDI_Index = 2;
        MIDI_Initial(&DvrMIDIWorkingRamPtr);
        MIDI_Play(&DvrMIDIWorkingRamPtr, R_MIDI_Index);
				break;

			case 0x08:
        R_MIDI_Index = 3;
        MIDI_Initial(&DvrMIDIWorkingRamPtr);
        MIDI_Play(&DvrMIDIWorkingRamPtr, R_MIDI_Index);
			  break;

			case 0x10:
        MIDI_Stop();
				break;

      case 0x20:
        MIDI_Stop();
				break;

			case 0x40:
        MIDI_Stop();
			  break;

			case 0x80:
        MIDI_Stop();
//        MIDI_PlaySingleNote(0, 1, 60, 100);
//        MIDI_PlaySingleDrum(1, 0, 100);
//        MIDI_OneKeyOneNote_PlayNote();
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

