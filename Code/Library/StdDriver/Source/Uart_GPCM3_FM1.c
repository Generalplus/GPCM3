/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   Uart_GPCM3_FM1.c
 * @Version:
 *   V0.9.1
 * @Date:
 *   May 18, 2023
 * @Abstract:
 *
 **************************************************************************************************/


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "GPCM3_FM1.h"
#include <math.h>


/*---------------------------------------------------------------------------------------
 * Subroutine Area
 *---------------------------------------------------------------------------------------*/

/******************************************************************************
 * @brief
 *  UART Baudrate calculation:
 *   - Baud rate is 115200. PCLK is 80MHz. Baud rate divider (BR_DIV) is equal to 80M/(16*115200)=43.40278; therefore, BR_DIV is set 44.
 *   - Buad rate compensation cycle is selected 2 becasue 1/(44-43.40278)=1.67. Therefore, 2 is selected.
 *   - Because BR_DIV select number is less than the value that the user selects, the compensation should use "SUB" to compensate cumluative error.
 * @param
 *  *UartHandle [in]:
 *    - UART0, UART1
 *  UartSelIo [in]:
 *    - UART0_IOA0_1, UART0_IOA13_14, UART0_IOA30_31, UART0_IOC2_RST
 *    - UART1_IOA5_6, UART1_IOA20_21, UART1_IOA26_27
 *   BaudRate [in]: Baud rate
 * @return
 *   None.
 ******************************************************************************/
void UART_Init(UART_TYPE_DEF *UartHandle, uint32_t UartSelIo, uint32_t BaudRate)
{
	float UartBaudRateDiv;
	float UartBaudRateDivFloor;
	float UartBaudRateDivCeil;
	float CmpCycle;
	float CmpCycleFloor;
	float CmpCycleCeil;
	float UartErr1;
	float UartErr2;

	SET_BIT(CLOCK->AHBCKEN, CLOCK_AHBCKEN_GPIO_CLK_ENABLE);	           // GPIO clk enable
	if (UartHandle == UART0)
	{
		MODIFY_REG(IOFUNC->CTRL0, GPIOFUNC_CTRL0_UART0_IOSEL_MSK, UartSelIo);
		SET_BIT(CLOCK->APBCKEN, CLOCK_APBCKEN_UART0_CLK_ENABLE);	           //	APB UART0 Clock enable
	}
	else if (UartHandle == UART1)
	{
		MODIFY_REG(IOFUNC->CTRL0, GPIOFUNC_CTRL0_UART1_IOSEL_MSK, UartSelIo);
		SET_BIT(CLOCK->APBCKEN, CLOCK_APBCKEN_UART1_CLK_ENABLE);	           //	APB UART1 Clock enable
	}


    UartHandle->CTRL = (UART_CTRL_UART_ENABLE | UART_CTRL_UART_RX_ENABLE | UART_CTRL_UART_TX_ENABLE);                                                     // Initialize UART Controller
    CLEAR_FLAG(UartHandle->STS, (UART_STS_RX_IDLE_MSK | UART_STS_TX_DONE_MSK), (UART_STS_RX_IDLE_FLAG | UART_STS_TX_DONE_FLAG | UART_STS_RX_BRK_FLAG));	  // Reset UART status

    UartBaudRateDiv = ((float)(System_GetCoreClock() >> 4) / BaudRate);
    UartBaudRateDivCeil = ceil(UartBaudRateDiv);
    UartBaudRateDivFloor = floor(UartBaudRateDiv);

    UartErr1 = (UartBaudRateDivCeil - UartBaudRateDiv) / UartBaudRateDiv;
    UartErr2 = (UartBaudRateDiv - UartBaudRateDivFloor) / UartBaudRateDiv;
	//if((UartErr1 > UART_BAUD_RATE_ERR) && (UartErr2 > UART_BAUD_RATE_ERR))
  //{
    //return -1;
  //}

    if(UartErr1 < UartErr2)
    {
        CmpCycle = (1 / (UartBaudRateDivCeil - UartBaudRateDiv));
        CmpCycleCeil = ceil(CmpCycle);
        CmpCycleFloor = floor(CmpCycle);

        UartErr1 = (CmpCycleCeil - CmpCycle) / CmpCycle;
        UartErr2 = (CmpCycle - CmpCycleFloor) / CmpCycle;
        //if((UartErr1 > UART_BAUD_RATE_ERR) && (UartErr2 > UART_BAUD_RATE_ERR))
        //{
        // return -1;
        //}
        MODIFY_REG(UartHandle->BAUD_RATE, UART_BAUD_RATE_BR_DIV_MSK, (uint32_t)UartBaudRateDivCeil);
        SET_BIT(UartHandle->BAUD_RATE, UART_BAUD_RATE_BR_CMP_SEL_SUB);
    }
    else
    {
        CmpCycle = (1 / (UartBaudRateDiv - UartBaudRateDivFloor));
        CmpCycleCeil = ceil(CmpCycle);
        CmpCycleFloor = floor(CmpCycle);

        UartErr1 = (CmpCycleCeil - CmpCycle) / CmpCycle;
        UartErr2 = (CmpCycle - CmpCycleFloor) / CmpCycle;
        //if((UartErr1 > UART_BAUD_RATE_ERR) && (UartErr2 > UART_BAUD_RATE_ERR))
        //{
        //return -1;
        //}
        MODIFY_REG(UartHandle->BAUD_RATE, UART_BAUD_RATE_BR_DIV_MSK, (uint32_t)UartBaudRateDivFloor);
        SET_BIT(UartHandle->BAUD_RATE, UART_BAUD_RATE_BR_CMP_SEL_ADD);
    }

    if(UartErr1 < UartErr2)
    {
        MODIFY_REG(UartHandle->BAUD_RATE, UART_BAUD_RATE_BR_CMP_CYCLE_MSK, ((uint32_t)CmpCycleCeil << UART_BAUD_RATE_BR_CMP_CYCLE_POS));
    }
    else
    {
        MODIFY_REG(UartHandle->BAUD_RATE, UART_BAUD_RATE_BR_CMP_CYCLE_MSK, ((uint32_t)CmpCycleFloor << UART_BAUD_RATE_BR_CMP_CYCLE_POS));
    }

    //return 0;
}

/**
 * @brief
 *
 * @param
 *  *UartHandle [in]:
 *    - UART0, UART1
 * @return
 *   None.
 */
void UART_Close(UART_TYPE_DEF *UartHandle)
{
	UartHandle->CTRL = 0;

	if (UartHandle == UART0)
	{
        CLEAR_BIT(CLOCK->APBCKEN, CLOCK_APBCKEN_UART0_CLK_ENABLE);	         //	UART0 Clock disable
	}
	else if (UartHandle == UART1)
	{
		CLEAR_BIT(CLOCK->APBCKEN, CLOCK_APBCKEN_UART1_CLK_ENABLE);	         //	UART1 Clock disable
	}
}

/**
 * @brief
 *
 * @param
 *  *UartHandle [in]:
 *    - UART0, UART1
 * @return
 *   None.
 */
void UART_TxInt_Enable(UART_TYPE_DEF *UartHandle)
{
    UartHandle->STS = UART_STS_TX_DONE_FLAG;
	SET_BIT(UartHandle->CTRL, UART_CTRL_TX_INT_ENABLE);
}

/**
 * @brief
 *
 * @param
 *  *UartHandle [in]:
 *    - UART0, UART1
 * @return
 *   None.
 */
void UART_TxInt_Disable(UART_TYPE_DEF *UartHandle)
{
	CLEAR_BIT(UartHandle->CTRL, UART_CTRL_TX_INT_ENABLE);
	UartHandle->STS = UART_STS_TX_DONE_FLAG;
}

/**
 * @brief
 *
 * @param
 *  *UartHandle [in]:
 *    - UART0, UART1
 * @return
 *   None.
 */
void UART_RxInt_Enable(UART_TYPE_DEF *UartHandle)
{
  UartHandle->STS = UART_STS_RX_DAT_NEMP_FLAG;
  SET_BIT(UartHandle->CTRL, UART_CTRL_RX_INT_ENABLE);
}

/**
 * @brief
 *
 * @param
 *  *UartHandle [in]:
 *    - UART0, UART1
 * @return
 *   None.
 */
void UART_RxInt_Disable(UART_TYPE_DEF *UartHandle)
{
  CLEAR_BIT(UartHandle->CTRL, UART_CTRL_RX_INT_ENABLE);
  UartHandle->STS = UART_STS_RX_DAT_NEMP_FLAG;
}
