/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   MAC_GPCM3_FM1.h
 * @Version:
 *   V0.9.2
 * @Date:
 *   November 28, 2022
 * @Abstract:
 *
 **************************************************************************************************/
#ifndef _MAC_GPCM3_FM1_H_
#define _MAC_GPCM3_FM1_H_


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "GPCM3_FM1.h"


/*---------------------------------------------------------------------------------------
 * Definition Area
 *---------------------------------------------------------------------------------------*/

/*
 * Definition for the 1st parameter of MAC_Init()
 */
#define MAC_SAT_MODE_8B                MAC_CTRL_SATURATION_MODE_SEL_8B
#define MAC_SAT_MODE_16B               MAC_CTRL_SATURATION_MODE_SEL_16B
#define MAC_SAT_MODE_32B               MAC_CTRL_SATURATION_MODE_SEL_32B

/*
 * Definition for the 2nd parameter of MAC_Init()
 */
#define MAC_SU                         MAC_CTRL_OPERATION_MODE_SEL_SU
#define MAC_SS                         MAC_CTRL_OPERATION_MODE_SEL_SS

/*
 * Definition for the 3rd parameter of MAC_Init()
 */
#define MAC_ARRAY_X_STEP_PLUS_0        MAC_STEP_XY_ADDRX_STEP_PLUS_0
#define MAC_ARRAY_X_STEP_PLUS_1        MAC_STEP_XY_ADDRX_STEP_PLUS_1
#define MAC_ARRAY_X_STEP_PLUS_2        MAC_STEP_XY_ADDRX_STEP_PLUS_2
#define MAC_ARRAY_X_STEP_PLUS_3        MAC_STEP_XY_ADDRX_STEP_PLUS_3
#define MAC_ARRAY_X_STEP_PLUS_4        MAC_STEP_XY_ADDRX_STEP_PLUS_4
#define MAC_ARRAY_X_STEP_PLUS_5        MAC_STEP_XY_ADDRX_STEP_PLUS_5
#define MAC_ARRAY_X_STEP_PLUS_6        MAC_STEP_XY_ADDRX_STEP_PLUS_6
#define MAC_ARRAY_X_STEP_PLUS_7        MAC_STEP_XY_ADDRX_STEP_PLUS_7
#define MAC_ARRAY_X_STEP_PLUS_8        MAC_STEP_XY_ADDRX_STEP_PLUS_8
#define MAC_ARRAY_X_STEP_PLUS_9        MAC_STEP_XY_ADDRX_STEP_PLUS_9
#define MAC_ARRAY_X_STEP_PLUS_10       MAC_STEP_XY_ADDRX_STEP_PLUS_10
#define MAC_ARRAY_X_STEP_PLUS_11       MAC_STEP_XY_ADDRX_STEP_PLUS_11
#define MAC_ARRAY_X_STEP_PLUS_12       MAC_STEP_XY_ADDRX_STEP_PLUS_12
#define MAC_ARRAY_X_STEP_PLUS_13       MAC_STEP_XY_ADDRX_STEP_PLUS_13
#define MAC_ARRAY_X_STEP_PLUS_14       MAC_STEP_XY_ADDRX_STEP_PLUS_14
#define MAC_ARRAY_X_STEP_PLUS_15       MAC_STEP_XY_ADDRX_STEP_PLUS_15
#define MAC_ARRAY_X_STEP_MINUS_1       MAC_STEP_XY_ADDRX_STEP_MINUS_1
#define MAC_ARRAY_X_STEP_MINUS_2       MAC_STEP_XY_ADDRX_STEP_MINUS_2
#define MAC_ARRAY_X_STEP_MINUS_3       MAC_STEP_XY_ADDRX_STEP_MINUS_3
#define MAC_ARRAY_X_STEP_MINUS_4       MAC_STEP_XY_ADDRX_STEP_MINUS_4
#define MAC_ARRAY_X_STEP_MINUS_5       MAC_STEP_XY_ADDRX_STEP_MINUS_5
#define MAC_ARRAY_X_STEP_MINUS_6       MAC_STEP_XY_ADDRX_STEP_MINUS_6
#define MAC_ARRAY_X_STEP_MINUS_7       MAC_STEP_XY_ADDRX_STEP_MINUS_7
#define MAC_ARRAY_X_STEP_MINUS_8       MAC_STEP_XY_ADDRX_STEP_MINUS_8
#define MAC_ARRAY_X_STEP_MINUS_9       MAC_STEP_XY_ADDRX_STEP_MINUS_9
#define MAC_ARRAY_X_STEP_MINUS_10      MAC_STEP_XY_ADDRX_STEP_MINUS_10
#define MAC_ARRAY_X_STEP_MINUS_11      MAC_STEP_XY_ADDRX_STEP_MINUS_11
#define MAC_ARRAY_X_STEP_MINUS_12      MAC_STEP_XY_ADDRX_STEP_MINUS_12
#define MAC_ARRAY_X_STEP_MINUS_13      MAC_STEP_XY_ADDRX_STEP_MINUS_13
#define MAC_ARRAY_X_STEP_MINUS_14      MAC_STEP_XY_ADDRX_STEP_MINUS_14
#define MAC_ARRAY_X_STEP_MINUS_15      MAC_STEP_XY_ADDRX_STEP_MINUS_15
#define MAC_ARRAY_X_STEP_MINUS_16      MAC_STEP_XY_ADDRX_STEP_MINUS_16

/*
 * Definition for the 4th parameter of MAC_Init()
 */
#define MAC_ARRAY_Y_STEP_PLUS_0        MAC_STEP_XY_ADDRY_STEP_PLUS_0
#define MAC_ARRAY_Y_STEP_PLUS_1        MAC_STEP_XY_ADDRY_STEP_PLUS_1
#define MAC_ARRAY_Y_STEP_PLUS_2        MAC_STEP_XY_ADDRY_STEP_PLUS_2
#define MAC_ARRAY_Y_STEP_PLUS_3        MAC_STEP_XY_ADDRY_STEP_PLUS_3
#define MAC_ARRAY_Y_STEP_PLUS_4        MAC_STEP_XY_ADDRY_STEP_PLUS_4
#define MAC_ARRAY_Y_STEP_PLUS_5        MAC_STEP_XY_ADDRY_STEP_PLUS_5
#define MAC_ARRAY_Y_STEP_PLUS_6        MAC_STEP_XY_ADDRY_STEP_PLUS_6
#define MAC_ARRAY_Y_STEP_PLUS_7        MAC_STEP_XY_ADDRY_STEP_PLUS_7
#define MAC_ARRAY_Y_STEP_PLUS_8        MAC_STEP_XY_ADDRY_STEP_PLUS_8
#define MAC_ARRAY_Y_STEP_PLUS_9        MAC_STEP_XY_ADDRY_STEP_PLUS_9
#define MAC_ARRAY_Y_STEP_PLUS_10       MAC_STEP_XY_ADDRY_STEP_PLUS_10
#define MAC_ARRAY_Y_STEP_PLUS_11       MAC_STEP_XY_ADDRY_STEP_PLUS_11
#define MAC_ARRAY_Y_STEP_PLUS_12       MAC_STEP_XY_ADDRY_STEP_PLUS_12
#define MAC_ARRAY_Y_STEP_PLUS_13       MAC_STEP_XY_ADDRY_STEP_PLUS_13
#define MAC_ARRAY_Y_STEP_PLUS_14       MAC_STEP_XY_ADDRY_STEP_PLUS_14
#define MAC_ARRAY_Y_STEP_PLUS_15       MAC_STEP_XY_ADDRY_STEP_PLUS_15
#define MAC_ARRAY_Y_STEP_MINUS_1       MAC_STEP_XY_ADDRY_STEP_MINUS_1
#define MAC_ARRAY_Y_STEP_MINUS_2       MAC_STEP_XY_ADDRY_STEP_MINUS_2
#define MAC_ARRAY_Y_STEP_MINUS_3       MAC_STEP_XY_ADDRY_STEP_MINUS_3
#define MAC_ARRAY_Y_STEP_MINUS_4       MAC_STEP_XY_ADDRY_STEP_MINUS_4
#define MAC_ARRAY_Y_STEP_MINUS_5       MAC_STEP_XY_ADDRY_STEP_MINUS_5
#define MAC_ARRAY_Y_STEP_MINUS_6       MAC_STEP_XY_ADDRY_STEP_MINUS_6
#define MAC_ARRAY_Y_STEP_MINUS_7       MAC_STEP_XY_ADDRY_STEP_MINUS_7
#define MAC_ARRAY_Y_STEP_MINUS_8       MAC_STEP_XY_ADDRY_STEP_MINUS_8
#define MAC_ARRAY_Y_STEP_MINUS_9       MAC_STEP_XY_ADDRY_STEP_MINUS_9
#define MAC_ARRAY_YXSTEP_MINUS_10      MAC_STEP_XY_ADDRY_STEP_MINUS_10
#define MAC_ARRAY_Y_STEP_MINUS_11      MAC_STEP_XY_ADDRY_STEP_MINUS_11
#define MAC_ARRAY_Y_STEP_MINUS_12      MAC_STEP_XY_ADDRY_STEP_MINUS_12
#define MAC_ARRAY_Y_STEP_MINUS_13      MAC_STEP_XY_ADDRY_STEP_MINUS_13
#define MAC_ARRAY_Y_STEP_MINUS_14      MAC_STEP_XY_ADDRY_STEP_MINUS_14
#define MAC_ARRAY_Y_STEP_MINUS_15      MAC_STEP_XY_ADDRY_STEP_MINUS_15
#define MAC_ARRAY_Y_STEP_MINUS_16      MAC_STEP_XY_ADDRY_STEP_MINUS_16

/*
 * Definition for the 5th parameter of MAC_Init()
 */
#define MAC_OUTPUT_SHIFT_LEFT_1        MAC_CTRL_OUTPUT_SHIFT_LEFT_1
#define MAC_OUTPUT_SHIFT_RIGHT_0       MAC_CTRL_OUTPUT_SHIFT_RIGHT_0
#define MAC_OUTPUT_SHIFT_RIGHT_1       MAC_CTRL_OUTPUT_SHIFT_RIGHT_1
#define MAC_OUTPUT_SHIFT_RIGHT_2       MAC_CTRL_OUTPUT_SHIFT_RIGHT_2
#define MAC_OUTPUT_SHIFT_RIGHT_3       MAC_CTRL_OUTPUT_SHIFT_RIGHT_3
#define MAC_OUTPUT_SHIFT_RIGHT_4       MAC_CTRL_OUTPUT_SHIFT_RIGHT_4
#define MAC_OUTPUT_SHIFT_RIGHT_5       MAC_CTRL_OUTPUT_SHIFT_RIGHT_5
#define MAC_OUTPUT_SHIFT_RIGHT_6       MAC_CTRL_OUTPUT_SHIFT_RIGHT_6
#define MAC_OUTPUT_SHIFT_RIGHT_7       MAC_CTRL_OUTPUT_SHIFT_RIGHT_7
#define MAC_OUTPUT_SHIFT_RIGHT_8       MAC_CTRL_OUTPUT_SHIFT_RIGHT_8
#define MAC_OUTPUT_SHIFT_RIGHT_9       MAC_CTRL_OUTPUT_SHIFT_RIGHT_9
#define MAC_OUTPUT_SHIFT_RIGHT_10      MAC_CTRL_OUTPUT_SHIFT_RIGHT_10
#define MAC_OUTPUT_SHIFT_RIGHT_11      MAC_CTRL_OUTPUT_SHIFT_RIGHT_11
#define MAC_OUTPUT_SHIFT_RIGHT_12      MAC_CTRL_OUTPUT_SHIFT_RIGHT_12
#define MAC_OUTPUT_SHIFT_RIGHT_13      MAC_CTRL_OUTPUT_SHIFT_RIGHT_13
#define MAC_OUTPUT_SHIFT_RIGHT_14      MAC_CTRL_OUTPUT_SHIFT_RIGHT_14
#define MAC_OUTPUT_SHIFT_RIGHT_15      MAC_CTRL_OUTPUT_SHIFT_RIGHT_15
#define MAC_OUTPUT_SHIFT_RIGHT_16      MAC_CTRL_OUTPUT_SHIFT_RIGHT_16


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C"{
#endif

void MAC_Init(uint32_t SatMode, uint32_t OperationMode, uint32_t ArrayXStep, uint32_t ArrayYStep, uint32_t OutputShiftCtrl);
void MAC_Close(void);
void MAC_Reset(void);
void MAC_Start(uint32_t ArrayXAddr, uint32_t ArrayYAddr, uint8_t MacLen);
uint32_t MAC_CheckBusy(void);
int32_t MAC_GetResult(void);


#ifdef __cplusplus
}
#endif

#endif
