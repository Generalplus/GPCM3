/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   MAC_GPCM3_FM1.c
 * @Version:
 *   V0.9.1
 * @Date:
 *   April 29, 2020
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
 *   Initialize MAC
 * @param
 *   SatMode [in]: Saturation mode
 *    - MAC_SAT_MODE_8B, MAC_SAT_MODE_16B, MAC_SAT_MODE_32B
 *   OperationMode [in]: MAC calculate by signed or unsigned mode.
 *    - MAC_SU, MAC_SS
 *   ArrayXStep [in]:
 *    - MAC_ARRAY_X_STEP_PLUS_0,   MAC_ARRAY_X_STEP_PLUS_1,   MAC_ARRAY_X_STEP_PLUS_2,   MAC_ARRAY_X_STEP_PLUS_3,
 *      MAC_ARRAY_X_STEP_PLUS_4,   MAC_ARRAY_X_STEP_PLUS_5,   MAC_ARRAY_X_STEP_PLUS_6,   MAC_ARRAY_X_STEP_PLUS_7,
 *      MAC_ARRAY_X_STEP_PLUS_8,   MAC_ARRAY_X_STEP_PLUS_9,   MAC_ARRAY_X_STEP_PLUS_10,  MAC_ARRAY_X_STEP_PLUS_11,
 *      MAC_ARRAY_X_STEP_PLUS_12,  MAC_ARRAY_X_STEP_PLUS_13,  MAC_ARRAY_X_STEP_PLUS_14,  MAC_ARRAY_X_STEP_PLUS_15,
 *      MAC_ARRAY_X_STEP_MINUS_1,  MAC_ARRAY_X_STEP_MINUS_2,  MAC_ARRAY_X_STEP_MINUS_3,  MAC_ARRAY_X_STEP_MINUS_4,
 *      MAC_ARRAY_X_STEP_MINUS_5,  MAC_ARRAY_X_STEP_MINUS_6,  MAC_ARRAY_X_STEP_MINUS_7,  MAC_ARRAY_X_STEP_MINUS_8,
 *      MAC_ARRAY_X_STEP_MINUS_9,  MAC_ARRAY_X_STEP_MINUS_10, MAC_ARRAY_X_STEP_MINUS_11, MAC_ARRAY_X_STEP_MINUS_12,
 *      MAC_ARRAY_X_STEP_MINUS_13, MAC_ARRAY_X_STEP_MINUS_14, MAC_ARRAY_X_STEP_MINUS_15, MAC_ARRAY_X_STEP_MINUS_16,
 *   ArrayYStep [in]:
 *    - MAC_ARRAY_Y_STEP_PLUS_0,   MAC_ARRAY_Y_STEP_PLUS_1,   MAC_ARRAY_Y_STEP_PLUS_2,   MAC_ARRAY_Y_STEP_PLUS_3,
 *      MAC_ARRAY_Y_STEP_PLUS_4,   MAC_ARRAY_Y_STEP_PLUS_5,   MAC_ARRAY_Y_STEP_PLUS_6,   MAC_ARRAY_Y_STEP_PLUS_7,
 *      MAC_ARRAY_Y_STEP_PLUS_8,   MAC_ARRAY_Y_STEP_PLUS_9,   MAC_ARRAY_Y_STEP_PLUS_10,  MAC_ARRAY_Y_STEP_PLUS_11,
 *      MAC_ARRAY_Y_STEP_PLUS_12,  MAC_ARRAY_Y_STEP_PLUS_13,  MAC_ARRAY_X_STEP_PLUS_14,  MAC_ARRAY_Y_STEP_PLUS_15,
 *      MAC_ARRAY_Y_STEP_MINUS_1,  MAC_ARRAY_Y_STEP_MINUS_2,  MAC_ARRAY_Y_STEP_MINUS_3,  MAC_ARRAY_Y_STEP_MINUS_4,
 *      MAC_ARRAY_Y_STEP_MINUS_5,  MAC_ARRAY_Y_STEP_MINUS_6,  MAC_ARRAY_Y_STEP_MINUS_7,  MAC_ARRAY_Y_STEP_MINUS_8,
 *      MAC_ARRAY_Y_STEP_MINUS_9,  MAC_ARRAY_Y_STEP_MINUS_10, MAC_ARRAY_Y_STEP_MINUS_11, MAC_ARRAY_Y_STEP_MINUS_12,
 *      MAC_ARRAY_Y_STEP_MINUS_13, MAC_ARRAY_Y_STEP_MINUS_14, MAC_ARRAY_Y_STEP_MINUS_15, MAC_ARRAY_Y_STEP_MINUS_16,
 *   OutputShiftCtrl [in]: MAC output shift control. Can shift left side 1 or shift right side 0~16.
 *    - MAC_OUTPUT_SHIFT_RIGHT_0,  MAC_OUTPUT_SHIFT_RIGHT_1,  MAC_OUTPUT_SHIFT_RIGHT_2,  MAC_OUTPUT_SHIFT_RIGHT_3,
 *      MAC_OUTPUT_SHIFT_RIGHT_4,  MAC_OUTPUT_SHIFT_RIGHT_5,  MAC_OUTPUT_SHIFT_RIGHT_6,  MAC_OUTPUT_SHIFT_RIGHT_7,
 *      MAC_OUTPUT_SHIFT_RIGHT_8,  MAC_OUTPUT_SHIFT_RIGHT_9,  MAC_OUTPUT_SHIFT_RIGHT_10, MAC_OUTPUT_SHIFT_RIGHT_11,
 *      MAC_OUTPUT_SHIFT_RIGHT_12, MAC_OUTPUT_SHIFT_RIGHT_13, MAC_OUTPUT_SHIFT_RIGHT_14, MAC_OUTPUT_SHIFT_RIGHT_15,
 *      MAC_OUTPUT_SHIFT_RIGHT_16, MAC_OUTPUT_SHIFT_LEFT_1
 * @return
 *   None.
 */
void MAC_Init(uint32_t SatMode, uint32_t OperationMode, uint32_t ArrayXStep, uint32_t ArrayYStep, uint32_t OutputShiftCtrl)
{
  SET_BIT(CLOCK->AHBCKEN, CLOCK_AHBCKEN_MAC_CLK_ENABLE);             //	Enable MAC Clock
	MAC->CTRL = SatMode | OperationMode | OutputShiftCtrl;
	MAC->STEP_XY = ArrayXStep | ArrayYStep;
}

/**
 * @brief
 *   Close MAC
 * @param
 *   None.
 * @return
 *   None.
 */
void MAC_Close()
{
  CLEAR_BIT(CLOCK->AHBCKEN, CLOCK_AHBCKEN_MAC_CLK_ENABLE);	         //	Disable MAC Clock
}

/**
 * @brief
 *   Reset MAC
 * @param
 *   None.
 * @return
 *   None.
 */
void MAC_Reset()
{
	MAC->CTRL |= MAC_CTRL_RESET;
}

/**
 * @brief
 *   MAC controller start to do MAC calculation with length.
 * @param
 *   ArrayXAddr [in]: Array X Address
 *   ArrayYAddr [in]: Array Y Address
 *   MacLen [in]: MAC length
 * @return
 *   None.
 */
void MAC_Start(uint32_t ArrayXAddr, uint32_t ArrayYAddr, uint8_t MacLen)
{
	MAC->ADDR_X = ArrayXAddr;
	MAC->ADDR_Y = ArrayYAddr;
	MAC->LENGTH = MacLen - 1;
	MAC->CTRL |= MAC_CTRL_START;
}

/**
 * @brief
 *   Check MAC busy status
 * @param
 *   None.
 * @return
 *   0 -> MAC done.
 *   1 -> MAC busy.
 */
uint32_t MAC_CheckBusy()
{
	return (MAC->CTRL & MAC_CTRL_START);
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   MAC Output
 */
int32_t MAC_GetResult()
{
	return MAC->OUTPUT;
}
