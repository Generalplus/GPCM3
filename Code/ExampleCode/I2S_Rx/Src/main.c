/************************************************************************************
 *  File:     main.c
 *  Purpose:  GPCM3/GPFM1 Sample Code
 *            Replace with your code.
 *  Date:     19 November 2024
 *  Info:
 *            I2S Rx(default): MCLK:IOA[26]/ BCLK: IOA[27]/ SLR: IOA[28]/ DATA: IOA[29]
 *
 *
 ************************************************************************************/

/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include <stdio.h>
#include "GPCM3_FM1.h"
#include "SACM_A1801_User.h"
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
__attribute__ ((aligned (4))) SACM_A1801_TEMP_RAM A1801TempBuffer;
__attribute__ ((aligned (4))) SACM_A1801_PCM_BUFFER A1801PcmBuffer;
__attribute__ ((aligned (4))) KEYSCAN_WORKING_RAM KeyScanWorkingRam;
#endif



uint32_t ScanedKey;
uint16_t SpeechNum = 0;
int16_t A1801PlayIdx = 3;
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

    GPIO_SetMode(GPIOA, 0x00FF0000, GPIO_MODE_OUTPUT);

	SpeechNum = A1801_User_GetA1801Num();
//	SACM_A1801_Initial(&A1801WorkingRam, &A1801TempBuffer, &A1801PcmBuffer);
//	APP_SwVolCtrl_Init();                                    // Software volume control init.
//	APP_SwVolCtrl_SetVolGain(0, SwVolGain);
	KeyScan_Initial(&KeyScanWorkingRam);                     // key scan init.
// DAC init
	DAC_Open();
	DAC_Scale_Enable();
	DAC_CH0_Init(DAC_TRG_SEL_TM0);
	DAC_CH1_Init(DAC_TRG_SEL_TM0);
	SET_BIT(DAC->CTRL, DAC_CTRL_CH0_POSTWAVE_OUT_CH0 + DAC_CTRL_CH1_POSTWAVE_OUT_CH1);
// PWM init
	DAC_AudioPwm_Open();
	DAC_AudioPwm_IP_Enable();
	DAC_SetAudPwmGain(31);      // 1x Gain.

    I2S_RX_Initial();

    while(1)
	{
		WDT_Clear();
		KeyScan_ServiceLoop();
		I2S_RX_Service();

		ScanedKey = KeyScan_GetCh();
        switch(ScanedKey)
		{
		  case 0x01:                                           // IOA24+VDD: Stop
                break;

			case 0x02:                                           // IOA25+VDD: Play next song
				break;

			case 0x04:	                                         // IOA26+VDD: Play prev song
				break;

			case 0x08:                                           // IOA27+VDD: Play concatenate
			  break;

			case 0x10:                                           // IOA28+VDD:	Pause/Resume
				break;

            case 0x20:                                           // IOA29+VDD: Volume Up
				break;

			case 0x40:                                           // IOA30+VDD: Volume Dn
                break;

			case 0x80:                                           // IOA31+VDD:
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





