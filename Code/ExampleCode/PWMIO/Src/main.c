/************************************************************************************
 *  File:     main.c
 *  Purpose:  GPCM3/GPFM1 PWMIO(IOA[28:13]) example code
 *
 *  Date:     19 April 2023
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


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#if defined(__CC_ARM)
#define Aligned4B __align(4)
#elif defined(__GNUC__)
extern unsigned int __boot_table_start; //in GPCM3Test.ld
#define Aligned4B __attribute__ ((aligned (4)))
#endif

KEYSCAN_WORKING_RAM     KeyScanWorkingRam;
uint32_t ScanedKey;
uint8_t IoToggleEn = 0;


 /*---------------------------------------------------------------------------------------
 * Subroutine Area
 *---------------------------------------------------------------------------------------*/
/**
 * @brief
 *   Initialize PWMIO: IOA[12:5]
 * @param
 *   None.
 * @return
 *   None.
 */
void PWMIO_Init(void)
{
  SET_BIT(CLOCK->APBCKEN, CLOCK_APBCKEN_PWMIO_CLK_ENABLE);	         // PWMIO clock enable

	PWMIO->CTRL0= PWMIO_CTRL0_SYNC_ENABLE | PWMIO_CTRL_CLK_SEL_FCPU_DIV_16 |
                PWMIO_CTRL_PWM0_ENABLE | PWMIO_CTRL_PWM1_ENABLE | PWMIO_CTRL_PWM2_ENABLE | PWMIO_CTRL_PWM3_ENABLE |
                PWMIO_CTRL_PWM4_ENABLE | PWMIO_CTRL_PWM5_ENABLE | PWMIO_CTRL_PWM6_ENABLE | PWMIO_CTRL_PWM7_ENABLE |
                PWMIO_CTRL_PWM8_ENABLE | PWMIO_CTRL_PWM9_ENABLE | PWMIO_CTRL_PWM10_ENABLE | PWMIO_CTRL_PWM11_ENABLE |
                PWMIO_CTRL_PWM12_ENABLE | PWMIO_CTRL_PWM13_ENABLE | PWMIO_CTRL_PWM14_ENABLE | PWMIO_CTRL_PWM15_ENABLE;

	PWMIO->PERIOD_CTRL = PWMIO_CTRL_PERIOD0_256 | PWMIO_CTRL_PERIOD1_256 | PWMIO_CTRL_PERIOD2_256 | PWMIO_CTRL_PERIOD3_256 |
                       PWMIO_CTRL_PERIOD4_256 | PWMIO_CTRL_PERIOD5_256 | PWMIO_CTRL_PERIOD6_256 | PWMIO_CTRL_PERIOD7_256;

	PWMIO->PWMIO0_Duty = 0;
	PWMIO->PWMIO1_Duty = 5;
	PWMIO->PWMIO2_Duty = 10;
	PWMIO->PWMIO3_Duty = 20;
	PWMIO->PWMIO4_Duty = 40;
	PWMIO->PWMIO5_Duty = 60;
	PWMIO->PWMIO6_Duty = 80;
	PWMIO->PWMIO7_Duty = 100;
	PWMIO->PWMIO8_Duty = 120;
	PWMIO->PWMIO9_Duty = 140;
	PWMIO->PWMIO10_Duty = 160;
	PWMIO->PWMIO11_Duty = 180;
	PWMIO->PWMIO12_Duty = 200;
	PWMIO->PWMIO13_Duty = 220;
	PWMIO->PWMIO14_Duty = 240;
	PWMIO->PWMIO15_Duty = 255;
}

  uint32_t SWTrim_CheckBusy(void)
  {
    return (CLOCK->SWTRIM & CLOCK_SWTRIM_COUNTER_RDY);
  }

/*---------------------------------------------------------------------------------------
 * Main Function
 *---------------------------------------------------------------------------------------*/
int main()
{
  // GPIO Initialize
  GPIO_SetMode(GPIOA, 0xFFFFFFFF, GPIO_MODE_INPUT);     //IOA[31:0] Input pull-low, bypass IOA[14:13] UART & IOA.8 Output
  MODIFY_REG(GPIOA->OBUF, 0xFFFFFFFF, 0);

  GPIO_SetMode(GPIOB, 0xFFFFFFFF, GPIO_MODE_INPUT);     //IOB Input pull-low
  MODIFY_REG(GPIOB->OBUF, 0xFFFFFFFF, 0);

  GPIO_SetMode(GPIOC, 0xFFFFFFFF, GPIO_MODE_INPUT);     //IOC Input pull-low
  MODIFY_REG(GPIOC->OBUF, 0xFFFFFFFF, 0);

  GPIO_SetMode(GPIOD, 0xFFFFFFFF, GPIO_MODE_INPUT);     //IOD Input pull-low
  MODIFY_REG(GPIOD->OBUF, 0xFFFFFFFF, 0);
  KeyScan_Initial(&KeyScanWorkingRam);                  // Key scan init

	PWMIO_Init();

  while(1)
  {
    WDT_Clear();
		KeyScan_ServiceLoop();

		ScanedKey = KeyScan_GetCh();
	  switch(ScanedKey)
		{
		  case 0x01:  // IOA0+VDD: Mask PWMIO0
        PWMIO->CTRL1 |= (PWMIO_CTRL_PWM0_MSK_ON | PWMIO_CTRL_PWM1_MSK_ON | PWMIO_CTRL_PWM2_MSK_ON | PWMIO_CTRL_PWM3_MSK_ON |
                         PWMIO_CTRL_PWM4_MSK_ON | PWMIO_CTRL_PWM5_MSK_ON | PWMIO_CTRL_PWM6_MSK_ON | PWMIO_CTRL_PWM7_MSK_ON |
                         PWMIO_CTRL_PWM8_MSK_ON | PWMIO_CTRL_PWM9_MSK_ON | PWMIO_CTRL_PWM10_MSK_ON | PWMIO_CTRL_PWM11_MSK_ON |
                         PWMIO_CTRL_PWM12_MSK_ON | PWMIO_CTRL_PWM13_MSK_ON | PWMIO_CTRL_PWM14_MSK_ON | PWMIO_CTRL_PWM15_MSK_ON);
        break;

			case 0x02:  // IOA1+VDD: Unmask PWMIO0
				PWMIO->CTRL1 &= ~(PWMIO_CTRL_PWM0_MSK_ON | PWMIO_CTRL_PWM1_MSK_ON | PWMIO_CTRL_PWM2_MSK_ON | PWMIO_CTRL_PWM3_MSK_ON |
                         PWMIO_CTRL_PWM4_MSK_ON | PWMIO_CTRL_PWM5_MSK_ON | PWMIO_CTRL_PWM6_MSK_ON | PWMIO_CTRL_PWM7_MSK_ON |
                         PWMIO_CTRL_PWM8_MSK_ON | PWMIO_CTRL_PWM9_MSK_ON | PWMIO_CTRL_PWM10_MSK_ON | PWMIO_CTRL_PWM11_MSK_ON |
                         PWMIO_CTRL_PWM12_MSK_ON | PWMIO_CTRL_PWM13_MSK_ON | PWMIO_CTRL_PWM14_MSK_ON | PWMIO_CTRL_PWM15_MSK_ON);
				break;

			case 0x04:  // IOA2+VDD: Inverse PWMIO
				PWMIO->CTRL1 |= (PWMIO_CTRL_PWM0_INV_ON | PWMIO_CTRL_PWM1_INV_ON | PWMIO_CTRL_PWM2_INV_ON | PWMIO_CTRL_PWM3_INV_ON |
			                   PWMIO_CTRL_PWM4_INV_ON | PWMIO_CTRL_PWM5_INV_ON | PWMIO_CTRL_PWM6_INV_ON | PWMIO_CTRL_PWM7_INV_ON |
                         PWMIO_CTRL_PWM8_INV_ON | PWMIO_CTRL_PWM9_INV_ON | PWMIO_CTRL_PWM10_INV_ON | PWMIO_CTRL_PWM11_INV_ON |
			                   PWMIO_CTRL_PWM12_INV_ON | PWMIO_CTRL_PWM13_INV_ON | PWMIO_CTRL_PWM14_INV_ON | PWMIO_CTRL_PWM15_INV_ON);
				break;

			case 0x08: // IOA3+VDD: Non-Inverse PWMIO                                                     // IOA3+VDD: PWMIO non-inverse output.
				PWMIO->CTRL1 &= ~(PWMIO_CTRL_PWM0_INV_ON | PWMIO_CTRL_PWM1_INV_ON | PWMIO_CTRL_PWM2_INV_ON | PWMIO_CTRL_PWM3_INV_ON |
			                   PWMIO_CTRL_PWM4_INV_ON | PWMIO_CTRL_PWM5_INV_ON | PWMIO_CTRL_PWM6_INV_ON | PWMIO_CTRL_PWM7_INV_ON |
                         PWMIO_CTRL_PWM8_INV_ON | PWMIO_CTRL_PWM9_INV_ON | PWMIO_CTRL_PWM10_INV_ON | PWMIO_CTRL_PWM11_INV_ON |
			                   PWMIO_CTRL_PWM12_INV_ON | PWMIO_CTRL_PWM13_INV_ON | PWMIO_CTRL_PWM14_INV_ON | PWMIO_CTRL_PWM15_INV_ON);
				break;

      case 0x10: // IOA4+VDD: Toggle PWMIO ON/OFF
				if(IoToggleEn == 0)
				{
					IoToggleEn = 1;
					PWMIO->TOGGLE_CTRL = (PWMIO_CTRL_TOGGLE0_ENABLE | PWMIO_CTRL_TOGGLE1_ENABLE | PWMIO_CTRL_TOGGLE2_ENABLE | PWMIO_CTRL_TOGGLE3_ENABLE |
					                     PWMIO_CTRL_TOGGLE4_ENABLE | PWMIO_CTRL_TOGGLE5_ENABLE | PWMIO_CTRL_TOGGLE6_ENABLE | PWMIO_CTRL_TOGGLE7_ENABLE |
					                     PWMIO_CTRL_TOGGLE8_ENABLE | PWMIO_CTRL_TOGGLE9_ENABLE | PWMIO_CTRL_TOGGLE10_ENABLE | PWMIO_CTRL_TOGGLE11_ENABLE |
					                     PWMIO_CTRL_TOGGLE12_ENABLE | PWMIO_CTRL_TOGGLE13_ENABLE | PWMIO_CTRL_TOGGLE14_ENABLE | PWMIO_CTRL_TOGGLE15_ENABLE);
				}
				else
				{
					IoToggleEn = 0;
					PWMIO->TOGGLE_CTRL = 0;
				}
        break;

      case 0x20: // IOA5+VDD: Offset PWMIO ON
					PWMIO->OFFSET = (PWMIO_OFFSET_PWM0_1_DELAY1 | PWMIO_OFFSET_PWM2_3_NODELAY | PWMIO_OFFSET_PWM4_5_DELAY3 | PWMIO_OFFSET_PWM6_7_DELAY1 |
                           PWMIO_OFFSET_PWM8_9_DELAY3 | PWMIO_OFFSET_PWM10_11_NODELAY | PWMIO_OFFSET_PWM12_13_DELAY3 | PWMIO_OFFSET_PWM14_15_DELAY3);
       break;

      case 0x40: // IOA6+VDD: Offset PWMIO OFF
					PWMIO->OFFSET = 0;
       break;
		}

  }
  return 0;
}

/*---------------------------------------------------------------------------------------
 * IRQ Handler
 *---------------------------------------------------------------------------------------*/



