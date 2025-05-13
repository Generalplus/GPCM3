/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   PWMIO_GPCM3_FM1.c
 * @Version:
 *   V0.9.0
 * @Date:
 *   April 23, 2024
 * @Abstract: IOA[28:13]
 *					GPIOA[13]: PWMIO0
 *					GPIOA[14]: PWMIO1
 *					GPIOA[15]: PWMIO2
 *					GPIOA[16]: PWMIO3
 *					GPIOA[17]: PWMIO4
 *					GPIOA[18]: PWMIO5
 *					GPIOA[19]: PWMIO6
 *					GPIOA[20]: PWMIO7
 *					GPIOA[21]: PWMIO8
 *					GPIOA[22]: PWMIO9
 *					GPIOA[23]: PWMIO10
 *					GPIOA[24]: PWMIO11
 *					GPIOA[25]: PWMIO12
 *					GPIOA[26]: PWMIO13
 *					GPIOA[27]: PWMIO14
 *					GPIOA[28]: PWMIO15
 **************************************************************************************************/


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "GPCM3_FM1.h"

const uint32_t PWMIO_ENABLE[16] = {
	PWMIO_CTRL_PWM0_ENABLE,
	PWMIO_CTRL_PWM1_ENABLE,
	PWMIO_CTRL_PWM2_ENABLE,
	PWMIO_CTRL_PWM3_ENABLE,
	PWMIO_CTRL_PWM4_ENABLE,
	PWMIO_CTRL_PWM5_ENABLE,
	PWMIO_CTRL_PWM6_ENABLE,
	PWMIO_CTRL_PWM7_ENABLE,
	PWMIO_CTRL_PWM8_ENABLE,
	PWMIO_CTRL_PWM9_ENABLE,
	PWMIO_CTRL_PWM10_ENABLE,
	PWMIO_CTRL_PWM11_ENABLE,
	PWMIO_CTRL_PWM12_ENABLE,
	PWMIO_CTRL_PWM13_ENABLE,
	PWMIO_CTRL_PWM14_ENABLE,
	PWMIO_CTRL_PWM15_ENABLE
};

/*---------------------------------------------------------------------------------------
 * Subroutine Area
 *---------------------------------------------------------------------------------------*/

/*
 * @brief
 *				Initial PWMIO module
 * @param
 * 				None
 * @return
 * 				None
 */
void PWMIO_Open(void)
{
	SET_BIT(CLOCK->APBCKEN, CLOCK_APBCKEN_PWMIO_CLK_ENABLE);

	PWMIO->CTRL0  = PWMIO_CTRL0_SYNC_ENABLE | PWMIO_CTRL_CLK_SEL_FCPU_DIV_4096;

	PWMIO->PERIOD_CTRL  = PWMIO_CTRL_PERIOD0_256 | PWMIO_CTRL_PERIOD1_256 | PWMIO_CTRL_PERIOD2_256 | PWMIO_CTRL_PERIOD3_256;
	PWMIO->PERIOD_CTRL |= PWMIO_CTRL_PERIOD4_256 | PWMIO_CTRL_PERIOD5_256 | PWMIO_CTRL_PERIOD6_256 | PWMIO_CTRL_PERIOD7_256;

	PWMIO->PWMIO0_Duty = 0;
	PWMIO->PWMIO1_Duty = 0;
	PWMIO->PWMIO2_Duty = 0;
	PWMIO->PWMIO3_Duty = 0;
	PWMIO->PWMIO4_Duty = 0;
	PWMIO->PWMIO5_Duty = 0;
	PWMIO->PWMIO6_Duty = 0;
	PWMIO->PWMIO7_Duty = 0;


  SET_BIT(CLOCK->APBCKEN, CLOCK_APBCKEN_PWMIO_CLK_ENABLE);	         // PWMIO clock enable

	PWMIO->CTRL0= PWMIO_CTRL0_SYNC_ENABLE | PWMIO_CTRL_CLK_SEL_FCPU_DIV_4096 |
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

/**
 * @brief
 *
 * @param
 * 	 None.
 * @return
 *   None.
 */
void PWMIO_Close(void)
{
	PWMIO->CTRL0 = 0;
	PWMIO->CTRL1 = 0;

	CLEAR_BIT(CLOCK->APBCKEN, CLOCK_APBCKEN_PWMIO_CLK_ENABLE);
}

/**
 * @brief
 *
 * @param
 *   ClkIn [in]:
 *			PWMIO_CTRL_CLK_SEL_FCPU_DIV_4096
 *			PWMIO_CTRL_CLK_SEL_FCPU_DIV_1024
 *			PWMIO_CTRL_CLK_SEL_FCPU_DIV_256
 *			PWMIO_CTRL_CLK_SEL_FCPU_DIV_64
 *			PWMIO_CTRL_CLK_SEL_FCPU_DIV_32
 *			PWMIO_CTRL_CLK_SEL_FCPU_DIV_16
 *			PWMIO_CTRL_CLK_SEL_FCPU_DIV_8
 *			PWMIO_CTRL_CLK_SEL_CCP1
 *			PWMIO_CTRL_CLK_SEL_CCP0
 *			PWMIO_CTRL_CLK_SEL_TIMER2
 *			PWMIO_CTRL_CLK_SEL_TIMER1
 *			PWMIO_CTRL_CLK_SEL_TIMER0
 * @return
 *   None.
 */
void PWMIO_SetClkIn(uint32_t ClkIn)
{
  MODIFY_REG(PWMIO->CTRL0, PWMIO_CTRL_CLK_SEL_MSK, ClkIn);
}

/**
 * @brief
 *  Set GPIO operation mode
 * @param
 *  *gpio [in]: The base address of GPIO
 *    - GPIOA, GPIOB
 *   PinMask [in]:
 *   GpioMode [in]:
 *    - GPIO_MODE_INPUT, GPIO_MODE_FLOATING, GPIO_MODE_OPEN_DRAIN, GPIO_MODE_OUTPUT
 * @return
 *  None.
 */
void PWMIO_SetDuty(uint32_t NumPin, uint8_t Duty)
{
	SET_BIT(PWMIO->CTRL1, PWMIO_ENABLE[NumPin]);

	switch(NumPin)
	{
		case 0:
			PWMIO->PWMIO0_Duty = Duty;
			break;
		case 1:
			PWMIO->PWMIO1_Duty = Duty;
			break;
		case 2:
			PWMIO->PWMIO2_Duty = Duty;
			break;
		case 3:
			PWMIO->PWMIO3_Duty = Duty;
			break;
		case 4:
			PWMIO->PWMIO4_Duty = Duty;
			break;
		case 5:
			PWMIO->PWMIO5_Duty = Duty;
			break;
		case 6:
			PWMIO->PWMIO6_Duty = Duty;
			break;
		case 7:
			PWMIO->PWMIO7_Duty = Duty;
			break;
	}
}
