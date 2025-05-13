/************************************************************************************
 *  File:     main.c
 *  Purpose:  GPCM3/GPFM1 Example Code
 *            Replace with your code.
 *  Date:     21 March 2023
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

uint8_t str1[] = "Hello";
uint8_t str2[] = " Generalplus!";

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
    uint8_t temp;
    uint8_t *bufptr;

    WDT_Close();

    // KeyScan_Initial(&KeyScanWorkingRam);                  // Key scan init

    //Init UART1
    UART_Init(UART1, UART1_IOA5_6, 115200);

    //Init DMA0
    DMA_Init(DMA0, DMA_CTRL_DMA_REQSEL_UART1, DMA_SRC_DATA_8B, DMA_SRC_ADDR_INC, DMA_DST_DATA_8B, DMA_DST_ADDR_FIX);

    //Send str1
    DMA_Trigger(DMA0, (uint32_t)&str1[0], (uint32_t)&UART1->DATA, sizeof(str1)-1);
    while((DMA0->CTRL & DMA_CTRL_DMA_ENABLE) != 0);
    DMA_INT->INTSTS = DMA_INTSTS_DMA0_DONE_INT_FLAG;
    while((UART1->STS & UART_STS_TX_DONE_FLAG) == 0);
    UART1->STS = UART_STS_TX_DONE_FLAG;

    //Send str2
    DMA_Trigger(DMA0, (uint32_t)&str2[0], (uint32_t)&UART1->DATA, sizeof(str2)-1);
    while((DMA0->CTRL & DMA_CTRL_DMA_ENABLE) != 0);
    DMA_INT->INTSTS = DMA_INTSTS_DMA0_DONE_INT_FLAG;
    while((UART1->STS & UART_STS_TX_DONE_FLAG) == 0);
    UART1->STS = UART_STS_TX_DONE_FLAG;

    while(1)
    {
        WDT_Clear();
    }
  return 0;

}


