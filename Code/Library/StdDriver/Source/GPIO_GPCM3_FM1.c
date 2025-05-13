/**************************************************************************************************
 * Copyright(c) 2022 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   GPIO_GPCM3_FM1.c
 * @Version:
 *   V0.9.2
 * @Date:
 *   July 27, 2022
 * @Abstract:
 *
 **************************************************************************************************/


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "GPCM3_FM1.h"


/*---------------------------------------------------------------------------------------
 * Subroutine Area
 *---------------------------------------------------------------------------------------*/

/**
 * @brief
 *   Set GPIO operation mode
 * @param
 *  *GpioHandle [in]: The base address of GPIO
 *    - GPIOA, GPIOB, GPIOC, GPIOD
 *   PinMask [in]:
 *   GpioMode [in]:
 *    - GPIO_MODE_INPUT, GPIO_MODE_FLOATING, GPIO_MODE_OPEN_DRAIN, GPIO_MODE_OUTPUT
 * @return
 *   None.
 */
void GPIO_SetMode(GPIO_TYPE_DEF *GpioHandle, uint32_t PinMask, uint32_t GpioMode)
{
  uint32_t iCount;
	uint32_t IoNum;
	uint32_t IoCfg0;
	uint32_t IoCfg1;

	SET_BIT(CLOCK->AHBCKEN, CLOCK_AHBCKEN_GPIO_CLK_ENABLE);	           // GPIO clk enable

    if(GpioHandle == GPIOA)
    {
        IoNum = 32;
    }
    else if(GpioHandle == GPIOB)
    {
        IoNum = 6;
    }
    else if(GpioHandle == GPIOC)
    {
        IoNum = 3;
    }
    else if(GpioHandle == GPIOD)
    {
        IoNum = 2;
    }
    else
    {
        IoNum = 0;
    }

    IoCfg0 = GpioHandle->CFG0;
    IoCfg1 = GpioHandle->CFG1;
    for(iCount = 0; iCount < IoNum; iCount++)
    {
        if((PinMask & (1 << iCount)) != 0)
        {
            if(iCount < 16)
            {
                IoCfg0 &= ~(0x3 << (iCount << 1));
                IoCfg0 |= (GpioMode << (iCount << 1));
            }
            else
            {
                IoCfg1 &= ~(0x3 << ((iCount - 16) << 1));
                IoCfg1 |= (GpioMode << ((iCount - 16) << 1));
            }
        }
    }
	GpioHandle->CFG0 = IoCfg0;
	GpioHandle->CFG1 = IoCfg1;
}

/**
 * @brief
 *   Key change wake up IO selection
 * @param
 *   WakeupIoSel (Multiple selection)
 *    - KEYCHG_WAKEUP_EN_IOA0,  KEYCHG_WAKEUP_EN_IOA1,  KEYCHG_WAKEUP_EN_IOA2,  KEYCHG_WAKEUP_EN_IOA3,
 *      KEYCHG_WAKEUP_EN_IOA4,  KEYCHG_WAKEUP_EN_IOA5,  KEYCHG_WAKEUP_EN_IOA6,  KEYCHG_WAKEUP_EN_IOA7,
 *      KEYCHG_WAKEUP_EN_IOA8,  KEYCHG_WAKEUP_EN_IOA9,  KEYCHG_WAKEUP_EN_IOA10, KEYCHG_WAKEUP_EN_IOA11,
 *      KEYCHG_WAKEUP_EN_IOA12, KEYCHG_WAKEUP_EN_IOA13, KEYCHG_WAKEUP_EN_IOA14, KEYCHG_WAKEUP_EN_IOA15,
 *      KEYCHG_WAKEUP_EN_IOA16, KEYCHG_WAKEUP_EN_IOA17, KEYCHG_WAKEUP_EN_IOA18, KEYCHG_WAKEUP_EN_IOA19,
 *      KEYCHG_WAKEUP_EN_IOA20, KEYCHG_WAKEUP_EN_IOA21, KEYCHG_WAKEUP_EN_IOA22, KEYCHG_WAKEUP_EN_IOA23,
 *      KEYCHG_WAKEUP_EN_IOA24, KEYCHG_WAKEUP_EN_IOA25, KEYCHG_WAKEUP_EN_IOA26, KEYCHG_WAKEUP_EN_IOA27,
 *      KEYCHG_WAKEUP_EN_IOA28, KEYCHG_WAKEUP_EN_IOA29, KEYCHG_WAKEUP_EN_IOA30, KEYCHG_WAKEUP_EN_IOA31,
 *      KEYCHG_WAKEUP_EN_IOB0,  KEYCHG_WAKEUP_EN_IOB1,  KEYCHG_WAKEUP_EN_IOB2,
 *      KEYCHG_WAKEUP_EN_IOB3,  KEYCHG_WAKEUP_EN_IOB4,  KEYCHG_WAKEUP_EN_IOB5,
 *      KEYCHG_WAKEUP_EN_IOC0,  KEYCHG_WAKEUP_EN_IOC1,  KEYCHG_WAKEUP_EN_IOC2,
 *      KEYCHG_WAKEUP_EN_IOD0,  KEYCHG_WAKEUP_EN_IOD1,
 * @return
 *   None.
 */
void GPIO_KeyChangeWakeupIoSel(uint32_t WakeupIoSel)
{
    // WRITE_REG(IOFUNC->WAKEEN,(WakeupIoSel | GPIOFUNC_WAKEEN_INT_ENABLE));
    WRITE_REG(IOFUNC->WAKEEN2,(WakeupIoSel | GPIOFUNC_WAKEEN2_INT_ENABLE));
}

/**
 * @brief
 *   Key change wake up interrupt enable
 * @param
 *   None.
 * @return
 *   None.
 */
void GPIO_KeyChangeWakeupIntEnable()
{
	// SET_BIT(IOFUNC->WAKEEN, GPIOFUNC_WAKEEN_INT_ENABLE);
	SET_BIT(IOFUNC->WAKEEN2, GPIOFUNC_WAKEEN2_INT_ENABLE);
}

/**
 * @brief
 *   Key change wake up interrupt disable
 * @param
 *   None.
 * @return
 *   None.
 */
void GPIO_KeyChangeWakeupIntDisable()
{
	// CLEAR_BIT(IOFUNC->WAKEEN, GPIOFUNC_WAKEEN_INT_ENABLE);
	CLEAR_BIT(IOFUNC->WAKEEN2, GPIOFUNC_WAKEEN2_INT_ENABLE);
}

/**
 * @brief
 *   External interrupt init
 * @param
 *   ExtIntNum [in]:
 *    - EXT_INT0, EXT_INT1, EXT_INT2, EXT_INT3
 *   ExtIntTriggerMethod [in]:
 *    - EXT_INT_RISING_EDGE_TRIG, EXT_INT_FALLING_EDGE_TRIG,
 *      EXT_INT_HIGH_LVL_TRIG, EXT_INT_LOW_LVL_TRIG
 *   ExtIntIoSel [in]:
 *    - EXT_INT_SEL_IOA0,  EXT_INT_SEL_IOA1,  EXT_INT_SEL_IOA2,  EXT_INT_SEL_IOA3,
 *      EXT_INT_SEL_IOA4,  EXT_INT_SEL_IOA5,  EXT_INT_SEL_IOA6,  EXT_INT_SEL_IOA7,
 *      EXT_INT_SEL_IOA8,  EXT_INT_SEL_IOA9,  EXT_INT_SEL_IOA10, EXT_INT_SEL_IOA11,
 *      EXT_INT_SEL_IOA12, EXT_INT_SEL_IOA13, EXT_INT_SEL_IOA14, EXT_INT_SEL_IOA15,
 *      EXT_INT_SEL_IOA16, EXT_INT_SEL_IOA17, EXT_INT_SEL_IOA18, EXT_INT_SEL_IOA19,
 *      EXT_INT_SEL_IOA20, EXT_INT_SEL_IOA21, EXT_INT_SEL_IOA22, EXT_INT_SEL_IOA23,
 *      EXT_INT_SEL_IOA24, EXT_INT_SEL_IOB0,  EXT_INT_SEL_IOB1,  EXT_INT_SEL_IOB2,
 *      EXT_INT_SEL_IOB3,  EXT_INT_SEL_IOB4,  EXT_INT_SEL_IOB5
 * @return
 *   None.
 */
void GPIO_ExtInterrupt_Init(uint32_t ExtIntNum, uint32_t ExtIntTriggerMethod, uint32_t ExtIntIoSel)
{
	MODIFY_REG(IOFUNC->CTRL2, (GPIOFUNC_CTRL2_EXT0_PINSEL1_MSK << (ExtIntNum << 3)), (ExtIntIoSel << (ExtIntNum << 3)));

	if(ExtIntTriggerMethod < 2)
	{
		MODIFY_REG(ITU->EXTRHT, ((0x1 << ExtIntNum) << (16 * ExtIntTriggerMethod)), ((0x1 << ExtIntNum) << (16 * ExtIntTriggerMethod)));
	}
	else
	{
		MODIFY_REG(ITU->EXTFLT, ((0x1 << ExtIntNum) << (16 * (ExtIntTriggerMethod - 2))), ((0x1 << ExtIntNum) << (16 * (ExtIntTriggerMethod - 2))));
	}
}

/**
 * @brief
 *   External interrupt close
 * @param
 *   ExtIntNum [in]:
 *    - EXT_INT0, EXT_INT1, EXT_INT2, EXT_INT3
 * @return
 *   None.
 */
void GPIO_ExtInterrupt_Close(uint32_t ExtIntNum)
{
    GPIO_ExtInterrupt_Disable(ExtIntNum);
	MODIFY_REG(ITU->EXTRHT, ((0x01 << ExtIntNum) | ((0x01 << ExtIntNum) << 16)), 0);
	MODIFY_REG(ITU->EXTFLT, ((0x01 << ExtIntNum) | ((0x01 << ExtIntNum) << 16)), 0);
}

/**
 * @brief
 *   External interrupt enable
 * @param
 *   ExtIntNum [in]:
 *    - EXT_INT0, EXT_INT1, EXT_INT2, EXT_INT3
 * @return
 *   None.
 */
void GPIO_ExtInterrupt_Enable(uint32_t ExtIntNum)
{
	ITU->EXTIFLG = (0x01 << ExtIntNum);
	SET_BIT(ITU->EXTIEN, (0x01 << ExtIntNum));
}

/**
 * @brief
 *   External interrupt disable
 * @param
 *   ExtIntNum [in]:
 *    - EXT_INT0, EXT_INT1, EXT_INT2, EXT_INT3
 * @return
 *   None.
 */
void GPIO_ExtInterrupt_Disable(uint32_t ExtIntNum)
{
	CLEAR_BIT(ITU->EXTIEN, (0x01 << ExtIntNum));
}

/**
 * @brief
 *   GPIO IO First Enable
 * @param
 *  *GpioHandle [in]: The base address of GPIO
 *    - GPIOA, GPIOB
 *   PinMask [in]:
 * @return
 *   None.
 */
void GPIO_IOFirst_Enable(GPIO_TYPE_DEF *GpioHandle, uint32_t PinMask)
{
    SET_BIT(GpioHandle->FST, PinMask);
}

/**
 * @brief
 *   GPIO IO First Disable
 * @param
 *  *GpioHandle [in]: The base address of GPIO
 *    - GPIOA, GPIOB
 *   PinMask [in]:
 * @return
 *   None.
 */
void GPIO_IOFirst_Disable(GPIO_TYPE_DEF *GpioHandle, uint32_t PinMask)
{
	CLEAR_BIT(GpioHandle->FST, PinMask);
}
