/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   GPIO_GPCM3_FM1.h
 * @Version:
 *   V0.9.2
 * @Date:
 *   July 27, 2020
 * @Abstract:
 *
 **************************************************************************************************/
#ifndef _GPIO_GPCM3_FM1_H_
#define _GPIO_GPCM3_FM1_H_


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "GPCM3_FM1.h"


/*---------------------------------------------------------------------------------------
 * Definition Area
 *---------------------------------------------------------------------------------------*/

/*
 * Definition for the 2nd parameter of GPIO_SetMode(), GPIO_ExtInterrupt_Enable, GPIO_ExtInterrupt_Disable
 */
#define GPIO_MODE_INPUT              (0x0UL)               // Input Mode
#define GPIO_MODE_FLOATING           (0x1UL)               // Floating Mode
#define GPIO_MODE_OPEN_DRAIN         (0x2UL)               // Open-Drain Mode
#define GPIO_MODE_OUTPUT             (0x3UL)               // Output Mode

/*
 * Definition for the 1st parameter of GPIO_KeyChangeWakeupIoSel()
 */
#define KEYCHG_WAKEUP_EN_IOA0        (GPIOFUNC_WAKEEN_IOA0_ENABLE)
#define KEYCHG_WAKEUP_EN_IOA1        (GPIOFUNC_WAKEEN_IOA1_ENABLE)
#define KEYCHG_WAKEUP_EN_IOA2        (GPIOFUNC_WAKEEN_IOA2_ENABLE)
#define KEYCHG_WAKEUP_EN_IOA3        (GPIOFUNC_WAKEEN_IOA3_ENABLE)
#define KEYCHG_WAKEUP_EN_IOA4        (GPIOFUNC_WAKEEN_IOA4_ENABLE)
#define KEYCHG_WAKEUP_EN_IOA5        (GPIOFUNC_WAKEEN_IOA5_ENABLE)
#define KEYCHG_WAKEUP_EN_IOA6        (GPIOFUNC_WAKEEN_IOA6_ENABLE)
#define KEYCHG_WAKEUP_EN_IOA7        (GPIOFUNC_WAKEEN_IOA7_ENABLE)
#define KEYCHG_WAKEUP_EN_IOA8        (GPIOFUNC_WAKEEN_IOA8_ENABLE)
#define KEYCHG_WAKEUP_EN_IOA9        (GPIOFUNC_WAKEEN_IOA9_ENABLE)
#define KEYCHG_WAKEUP_EN_IOA10       (GPIOFUNC_WAKEEN_IOA10_ENABLE)
#define KEYCHG_WAKEUP_EN_IOA11       (GPIOFUNC_WAKEEN_IOA11_ENABLE)
#define KEYCHG_WAKEUP_EN_IOA12       (GPIOFUNC_WAKEEN_IOA12_ENABLE)
#define KEYCHG_WAKEUP_EN_IOA13       (GPIOFUNC_WAKEEN_IOA13_ENABLE)
#define KEYCHG_WAKEUP_EN_IOA14       (GPIOFUNC_WAKEEN_IOA14_ENABLE)
#define KEYCHG_WAKEUP_EN_IOA15       (GPIOFUNC_WAKEEN_IOA15_ENABLE)
#define KEYCHG_WAKEUP_EN_IOA16       (GPIOFUNC_WAKEEN_IOA16_ENABLE)
#define KEYCHG_WAKEUP_EN_IOA17       (GPIOFUNC_WAKEEN_IOA17_ENABLE)
#define KEYCHG_WAKEUP_EN_IOA18       (GPIOFUNC_WAKEEN_IOA18_ENABLE)
#define KEYCHG_WAKEUP_EN_IOA19       (GPIOFUNC_WAKEEN_IOA19_ENABLE)
#define KEYCHG_WAKEUP_EN_IOA20       (GPIOFUNC_WAKEEN_IOA20_ENABLE)
#define KEYCHG_WAKEUP_EN_IOA21       (GPIOFUNC_WAKEEN_IOA21_ENABLE)
#define KEYCHG_WAKEUP_EN_IOA22       (GPIOFUNC_WAKEEN_IOA22_ENABLE)
#define KEYCHG_WAKEUP_EN_IOA23       (GPIOFUNC_WAKEEN_IOA23_ENABLE)
#define KEYCHG_WAKEUP_EN_IOA24       (GPIOFUNC_WAKEEN_IOA24_ENABLE)
#define KEYCHG_WAKEUP_EN_IOB0        (GPIOFUNC_WAKEEN_IOB0_ENABLE)
#define KEYCHG_WAKEUP_EN_IOB1        (GPIOFUNC_WAKEEN_IOB1_ENABLE)
#define KEYCHG_WAKEUP_EN_IOB2        (GPIOFUNC_WAKEEN_IOB2_ENABLE)
#define KEYCHG_WAKEUP_EN_IOB3        (GPIOFUNC_WAKEEN_IOB3_ENABLE)
#define KEYCHG_WAKEUP_EN_IOB4        (GPIOFUNC_WAKEEN_IOB4_ENABLE)
#define KEYCHG_WAKEUP_EN_IOB5        (GPIOFUNC_WAKEEN_IOB5_ENABLE)

/*
 * Definition for the 1st parameter of GPIO_ExtInterrup_Init()
 */
#define EXT_INT0                     (0x0UL)
#define EXT_INT1                     (0x1UL)
#define EXT_INT2                     (0x2UL)
#define EXT_INT3                     (0x3UL)

/*
 * Definition for the 2nd parameter of GPIO_ExtInterrup_Init()
 */
#define EXT_INT_RISING_EDGE_TRIG     (0x0UL)
#define EXT_INT_FALLING_EDGE_TRIG    (0x2UL)
#define EXT_INT_HIGH_LVL_TRIG        (0x1UL)
#define EXT_INT_LOW_LVL_TRIG         (0x3UL)

/*
 * Definition for the 3rd parameter of GPIO_ExtInterrup_Init()
 */
#define EXT_INT_SEL_IOA0             (GPIOFUNC_CTRL2_EXT0_IOA0)
#define EXT_INT_SEL_IOA1             (GPIOFUNC_CTRL2_EXT0_IOA1)
#define EXT_INT_SEL_IOA2             (GPIOFUNC_CTRL2_EXT0_IOA2)
#define EXT_INT_SEL_IOA3             (GPIOFUNC_CTRL2_EXT0_IOA3)
#define EXT_INT_SEL_IOA4             (GPIOFUNC_CTRL2_EXT0_IOA4)
#define EXT_INT_SEL_IOA5             (GPIOFUNC_CTRL2_EXT0_IOA5)
#define EXT_INT_SEL_IOA6             (GPIOFUNC_CTRL2_EXT0_IOA6)
#define EXT_INT_SEL_IOA7             (GPIOFUNC_CTRL2_EXT0_IOA7)
#define EXT_INT_SEL_IOA8             (GPIOFUNC_CTRL2_EXT0_IOA8)
#define EXT_INT_SEL_IOA9             (GPIOFUNC_CTRL2_EXT0_IOA9)
#define EXT_INT_SEL_IOA10            (GPIOFUNC_CTRL2_EXT0_IOA10)
#define EXT_INT_SEL_IOA11            (GPIOFUNC_CTRL2_EXT0_IOA11)
#define EXT_INT_SEL_IOA12            (GPIOFUNC_CTRL2_EXT0_IOA12)
#define EXT_INT_SEL_IOA13            (GPIOFUNC_CTRL2_EXT0_IOA13)
#define EXT_INT_SEL_IOA14            (GPIOFUNC_CTRL2_EXT0_IOA14)
#define EXT_INT_SEL_IOA15            (GPIOFUNC_CTRL2_EXT0_IOA15)
#define EXT_INT_SEL_IOA16            (GPIOFUNC_CTRL2_EXT0_IOA16)
#define EXT_INT_SEL_IOA17            (GPIOFUNC_CTRL2_EXT0_IOA17)
#define EXT_INT_SEL_IOA18            (GPIOFUNC_CTRL2_EXT0_IOA18)
#define EXT_INT_SEL_IOA19            (GPIOFUNC_CTRL2_EXT0_IOA19)
#define EXT_INT_SEL_IOA20            (GPIOFUNC_CTRL2_EXT0_IOA20)
#define EXT_INT_SEL_IOA21            (GPIOFUNC_CTRL2_EXT0_IOA21)
#define EXT_INT_SEL_IOA22            (GPIOFUNC_CTRL2_EXT0_IOA22)
#define EXT_INT_SEL_IOA23            (GPIOFUNC_CTRL2_EXT0_IOA23)
#define EXT_INT_SEL_IOA24            (GPIOFUNC_CTRL2_EXT0_IOA24)
#define EXT_INT_SEL_IOB0             (GPIOFUNC_CTRL2_EXT0_IOB0)
#define EXT_INT_SEL_IOB1             (GPIOFUNC_CTRL2_EXT0_IOB1)
#define EXT_INT_SEL_IOB2             (GPIOFUNC_CTRL2_EXT0_IOB2)
#define EXT_INT_SEL_IOB3             (GPIOFUNC_CTRL2_EXT0_IOB3)
#define EXT_INT_SEL_IOB4             (GPIOFUNC_CTRL2_EXT0_IOB4)
#define EXT_INT_SEL_IOB5             (GPIOFUNC_CTRL2_EXT0_IOB5)


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C"{
#endif

void GPIO_SetMode(GPIO_TYPE_DEF *gpio, uint32_t PinMask, uint32_t GpioMode);

void GPIO_KeyChangeWakeupIoSel(uint32_t WakeupIoSel);
void GPIO_KeyChangeWakeupIntEnable(void);
void GPIO_KeyChangeWakeupIntDisable(void);

void GPIO_ExtInterrupt_Init(uint32_t ExtIntNum, uint32_t ExtIntTriggerMethod, uint32_t ExtIntIoSel);
void GPIO_ExtInterrupt_Close(uint32_t ExtIntNum);
void GPIO_ExtInterrupt_Enable(uint32_t ExtIntNum);
void GPIO_ExtInterrupt_Disable(uint32_t ExtIntNum);


#ifdef __cplusplus
}
#endif

#endif
