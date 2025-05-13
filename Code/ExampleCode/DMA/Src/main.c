/************************************************************************************
 *  File:     main.c
 *  Purpose:  GPCM3/GPFM1 Example Code
 *            Replace with your code.
 *  Date:     16 May 2023
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


/*---------------------------------------------------------------------------------------
 * Variable Declaration Area
 *---------------------------------------------------------------------------------------*/
  uint32_t DMASrcBuf[1000];
  uint32_t DMADstBuf[1000];
  uint16_t DMASrcBuf_16[100];
  uint16_t DMADstBuf_16[100];
  uint8_t DMASrcBuf_8[100];
  uint8_t DMADstBuf_8[100];

  uint32_t DMA0_INT_Count;
  uint8_t DMA0_Flag = 0;

  uint16_t icount;
  uint32_t BufCount = 0;
  uint32_t DMADataTemp = 0;

KEYSCAN_WORKING_RAM     KeyScanWorkingRam;
uint32_t ScanedKey;


/*---------------------------------------------------------------------------------------
 * Subroutine Area
 *---------------------------------------------------------------------------------------*/
  uint32_t SWTrim_CheckBusy(void)
  {
    return (CLOCK->SWTRIM & CLOCK_SWTRIM_COUNTER_RDY);
  }

  /**
   * @brief
   *
   * @param
   *   None.
   * @return
   *   None.
   */
  void DmaIsr_Dma0Trigger()
  {
    DMA0_INT_Count += 1;

    if (DMA0_Flag == 0)
    {
      DMA0_Flag = 1;        // For A/B Buffer test
    }
    else
    {
      DMA0_Flag = 0;
    }
    XOR_BIT(GPIOA->OBUF, BIT9);						  // Toggle IOA[9]
  }

/*
 * @brief
 *
 * @param
 *          None.
 * @return
 *          None.
 */
void DMA0_Initial(void)
{
  DMA_Init(DMA0, DMA_REQSEL_MEM, DMA_SRC_DATA_32B, DMA_SRC_ADDR_INC, DMA_DST_DATA_32B, DMA_DST_ADDR_INC);
	DMA_InstallIsrService(DMA0, DmaIsr_Dma0Trigger);
	DMA_EnableInt(DMA0);
	NVIC_EnableIRQ(DMA0_IRQn);
}


/*---------------------------------------------------------------------------------------
 * Main Function
 *---------------------------------------------------------------------------------------*/
int main()
{
  // GPIO Initialize
  GPIO_SetMode(GPIOA, 0xFFFFFFFF, GPIO_MODE_INPUT);     //IOA[31:0] Input pull-low, bypass IOA[14:13] UART & IOA.8 Output
  MODIFY_REG(GPIOA->OBUF, 0xFFFFFFFF, 0);
  GPIO_SetMode(GPIOC, 0xFFFFFFFF, GPIO_MODE_INPUT);     //IOC Input pull-low
  MODIFY_REG(GPIOC->OBUF, 0xFFFFFFFF, 0);
  GPIO_SetMode(GPIOD, 0xFFFFFFFF, GPIO_MODE_INPUT);     //IOD Input pull-low
  MODIFY_REG(GPIOD->OBUF, 0xFFFFFFFF, 0);

  /*
	 * IOA[12:8] output low
	 */
  GPIO_SetMode(GPIOA, 0x00001F00, GPIO_MODE_OUTPUT);
  MODIFY_REG(GPIOA->OBUF, 0x00001F00, 0);

  // KeyScan_Initial(&KeyScanWorkingRam);                  // Key scan init

/* ===================================== DMA Memory to Memory test ===================================== */
  for(icount=0; icount<100; icount++)
  {
    DMASrcBuf_8[icount] = icount+1;
    DMASrcBuf_16[icount] = icount+1;
  }
  for(icount=0; icount<500; icount++)
  {
    DMASrcBuf[icount] = icount+1;
  }
  for(icount=0; icount<150; icount++)
  {
    DMADstBuf[icount] = 0;
  }

  DMA0_Initial();

  while(1)
  {
    while(DMA_CheckBusy(DMA0) != 0);
    if((GPIOA->IDATA & BIT0)!= 0)         // IOA.0 = 1, return
    {
      return;
    }
    XOR_BIT(GPIOA->OBUF, BIT8);						  // Toggle IOA[8]
    DMA_Trigger(DMA0, (uint32_t)&DMASrcBuf, (uint32_t)&DMADstBuf, 100);          // DMA0 first trigger
    WDT_Clear();
  }

  return 0;
}

/*---------------------------------------------------------------------------------------
 * IRQ Handler
 *---------------------------------------------------------------------------------------*/



