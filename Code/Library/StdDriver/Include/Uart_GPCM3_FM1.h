/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File: Uart_GPCM3_FM1.h
 *
 * @Version:
 *   V0.9.1
 * @Date:
 *   May 18, 2023
 * @Abstract:
 *
 **************************************************************************************************/
#ifndef _UART_GPCM3_FM1_H_
#define _UART_GPCM3_FM1_H_


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include <stdint.h>
#include "GPCM3_FM1.h"

/*---------------------------------------------------------------------------------------
 * Definition Area
 *---------------------------------------------------------------------------------------*/

/*
 * Definition for the 2st parameter of UART_Init()
 */
#define UART0_IOA0_1                  GPIOFUNC_CTRL0_UART0_IOA0_1
#define UART0_IOA13_14                GPIOFUNC_CTRL0_UART0_IOA13_14
#define UART0_IOA30_31                GPIOFUNC_CTRL0_UART0_IOA30_31
#define UART0_IOC2_RST                GPIOFUNC_CTRL0_UART0_IOC2_RST

#define UART1_IOA5_6                  GPIOFUNC_CTRL0_UART1_IOA5_6
#define UART1_IOA20_21                GPIOFUNC_CTRL0_UART1_IOA20_21
#define UART1_IOA26_27                GPIOFUNC_CTRL0_UART1_IOA26_27

//#define UART_BAUD_RATE_ERR         0.045


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C"{
#endif

void UART_Init(UART_TYPE_DEF *UartHandle, uint32_t UartSelIo, uint32_t BaudRate);
void UART_Close(UART_TYPE_DEF *UartHandle);
void UART_TxInt_Enable(UART_TYPE_DEF *UartHandle);
void UART_TxInt_Disable(UART_TYPE_DEF *UartHandle);
void UART_RxInt_Enable(UART_TYPE_DEF *UartHandle);
void UART_RxInt_Disable(UART_TYPE_DEF *UartHandle);


#ifdef __cplusplus
}
#endif

#endif
