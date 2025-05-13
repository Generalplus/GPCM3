/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   DMA_GPCM3_FM1.c
 * @Version:
 *   V0.9.6
 * @Date:
 *   May 16, 2023
 * @Abstract:
 *    2022.05.19 Add the DMA_Stop() function.
 *    2023.05.16 Update the DMA_CTRL_DMA_REQSEL_UART1 of DMA_Trigger() function.
 *
 **************************************************************************************************/


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "GPCM3_FM1.h"


/*---------------------------------------------------------------------------------------
 * DMA Module Vairable Declaration Area
 *---------------------------------------------------------------------------------------*/
void (*mDmaCh0IsrServiceFunPtr)(void);
void (*mDmaCh1IsrServiceFunPtr)(void);
void (*mDmaCh2IsrServiceFunPtr)(void);
void (*mDmaCh3IsrServiceFunPtr)(void);
void (*mDmaCh4IsrServiceFunPtr)(void);


/*---------------------------------------------------------------------------------------
 * Subroutine Area
 *---------------------------------------------------------------------------------------*/

/**
 * @brief
 *  Initialize DMA
 * @param
 *  *DmaHandler [in]: The base address of DMAx module
 *    - DMA0, DMA1, DMA2, DMA3, DMA4
 *   DmaReq [in]:
 *    - DMA_REQSEL_ADC, DMA_REQSEL_DAC_CH0, DMA_REQSEL_DAC_CH1, DMA_REQSEL_SPI0, DMA_REQSEL_SPI1
 *      DMA_REQSEL_I2S, DMA_REQSEL_I2C, DMA_REQSEL_DSADC, DMA_REQSEL_UART, DMA_REQSEL_MEM
 *   SrcDataWidth [in]:
 *    - DMA_SRC_DATA_8B, DMA_SRC_DATA_16B, DMA_SRC_DATA_32B
 *   SrcAddrAttrib [in]:
 *    - DMA_SRC_ADDR_INC, DMA_SRC_ADDR_FIX
 *   DstDataWidth [in]:
 *    - DMA_DST_DATA_8B, DMA_DST_DATA_16B, DMA_DST_DATA_32B
 *   DstAddrAttrib [in]:
 *    - DMA_DST_ADDR_INC, DMA_DST_ADDR_FIX
 * @return
 *   None.
 */
void DMA_Init(DMA_TYPE_DEF *DmaHandle, uint32_t DmaReq, uint32_t SrcDataWidth, uint32_t SrcAddrAttrib, uint32_t DstDataWidth, uint32_t DstAddrAttrib)
{
	SET_BIT(CLOCK->AHBCKEN, CLOCK_AHBCKEN_DMA_CLK_ENABLE);             // DMA clock enable

	if(DmaHandle == DMA0)
	{
		DMA_INT->INTSTS = DMA_INTSTS_DMA0_DONE_INT_FLAG;
	}
	else if(DmaHandle == DMA1)
	{
		DMA_INT->INTSTS = DMA_INTSTS_DMA1_DONE_INT_FLAG;
	}
  else if(DmaHandle == DMA2)
	{
		DMA_INT->INTSTS = DMA_INTSTS_DMA2_DONE_INT_FLAG;
	}
  else if(DmaHandle == DMA3)
	{
		DMA_INT->INTSTS = DMA_INTSTS_DMA3_DONE_INT_FLAG;
	}
  else if(DmaHandle == DMA4)
	{
		DMA_INT->INTSTS = DMA_INTSTS_DMA4_DONE_INT_FLAG;
	}

	if(DmaReq == DMA_CTRL_DMA_REQSEL_MEM)
	{
		DmaHandle->CTRL = DmaReq | SrcDataWidth | DstDataWidth | SrcAddrAttrib | DstAddrAttrib | DMA_CTRL_DMA_BURST_ENABLE;
	}
	else
	{
    DmaHandle->CTRL = DmaReq | SrcDataWidth | DstDataWidth | SrcAddrAttrib | DstAddrAttrib;
	}
}

/**
 * @brief
 *
 * @param
 *  *DmaHandler [in]: The base address of DMAx module
 *    - DMA0, DMA1, DMA2, DMA3, DMA4
 * @return
 *   None.
 */
void DMA_Stop(DMA_TYPE_DEF *DmaHandle)
{
	if(DmaHandle == DMA0)
	{
		SET_BIT(DMA0->CTRL, DMA_CTRL_DMA_STOP);
	}
	else if(DmaHandle == DMA1)
	{
		SET_BIT(DMA1->CTRL, DMA_CTRL_DMA_STOP);
	}
  else if(DmaHandle == DMA2)
	{
		SET_BIT(DMA2->CTRL, DMA_CTRL_DMA_STOP);
	}
  else if(DmaHandle == DMA3)
	{
		SET_BIT(DMA3->CTRL, DMA_CTRL_DMA_STOP);
	}
  else if(DmaHandle == DMA4)
	{
		SET_BIT(DMA4->CTRL, DMA_CTRL_DMA_STOP);
	}



}


/**
 * @brief
 *
 * @param
 *  *DmaHandler [in]: The base address of DMAx module
 *    - DMA0, DMA1, DMA2, DMA3, DMA4
 * @return
 *   None.
 */
void DMA_Close(DMA_TYPE_DEF *DmaHandle)
{
  DMA_DisableInt(DmaHandle);
	DmaHandle->CTRL = 0;
  DMA_UnInstallIsrService(DmaHandle);

	if((DMA0->CTRL == 0) && (DMA1->CTRL == 0) && (DMA2->CTRL == 0)  && (DMA3->CTRL == 0) && (DMA4->CTRL == 0))
	{
		CLEAR_BIT(CLOCK->AHBCKEN, CLOCK_AHBCKEN_DMA_CLK_ENABLE);         // DMA clock disable
	}
}

/**
 * @brief
 *
 * @param
 *   *DmaHandler [in]: The base address of DMAx module
 *     - DMA0, DMA1, DMA2, DMA3, DMA4
 * @return
 *   None.
 */
void DMA_EnableInt(DMA_TYPE_DEF *DmaHandle)
{
	DmaHandle->CTRL |= DMA_CTRL_DMA_DONE_INT_ENABLE;
}

/**
 * @brief
 *
 * @param
 *  *DmaHandler [in]:
 *    - DMA0, DMA1, DMA2, DMA3, DMA4
 * @return
 *   None.
 */
void DMA_DisableInt(DMA_TYPE_DEF *DmaHandle)
{
	DmaHandle->CTRL &= ~DMA_CTRL_DMA_DONE_INT_ENABLE;
}

/**
 * @brief
 *
 * @param
 *  *DmaHandler [in]: The base address of DMAx module
 *    - DMA0, DMA1, DMA2
 *   SrcAddr [in]: Source address
 *   DstAddr [in]: Destination address
 *   TransCount [in]: Transfer count
 * @return
 *   None.
 */
void DMA_Trigger(DMA_TYPE_DEF *DmaHandle, uint32_t SrcAddr, uint32_t DstAddr, uint32_t TransCount)
{
  DmaHandle->CTRL &= ~DMA_CTRL_DMA_ENABLE;
	DmaHandle->SRCADR = SrcAddr;
	DmaHandle->DSTADR = DstAddr;
	DmaHandle->DTN = TransCount;
	DmaHandle->CTRL |= DMA_CTRL_DMA_ENABLE;

	switch(DmaHandle->CTRL & DMA_CTRL_DMA_REQSEL_MSK)
	{
    case DMA_CTRL_DMA_REQSEL_SARADC:
			SAR_ADC->GCTRL |= SAR_ADC_GCTRL_DMA_ENABLE;
      break;

		case DMA_CTRL_DMA_REQSEL_DAC_CH0:
			DAC->CTRL |= DAC_CTRL_DAC_CH0_DMA_ENABLE;
			break;

		case DMA_CTRL_DMA_REQSEL_DAC_CH1:
			DAC->CTRL |= DAC_CTRL_DAC_CH1_DMA_ENABLE;
			break;

    case DMA_CTRL_DMA_REQSEL_SPI0:
			if(DmaHandle->DSTADR == (uint32_t)&SPI0->TX_DATA)
			{
				SPI0->CTRL &= ~SPI_CTRL_SPI_ENABLE;
				SPI0->CTRL |= SPI_CTRL_TX_DMA_ENABLE;
				SPI0->CTRL |= SPI_CTRL_SPI_ENABLE;
			}
			else if(DmaHandle->SRCADR == (uint32_t)&SPI0->RX_DATA)
			{
				SPI0->CTRL |= SPI_CTRL_RX_DMA_ENABLE | SPI_CTRL_MASTER_RX_TRIG_ENABLE;
			}
      break;

		case DMA_CTRL_DMA_REQSEL_SPI1:
			if(DmaHandle->DSTADR == (uint32_t)&SPI1->TX_DATA)
			{
				SPI1->CTRL &= ~SPI_CTRL_SPI_ENABLE;
				SPI1->CTRL |= SPI_CTRL_TX_DMA_ENABLE;
				SPI1->CTRL |= SPI_CTRL_SPI_ENABLE;
			}
			else if(DmaHandle->SRCADR == (uint32_t)&SPI1->RX_DATA)
			{
				SPI1->CTRL |= SPI_CTRL_RX_DMA_ENABLE | SPI_CTRL_MASTER_RX_TRIG_ENABLE;
			}
		  break;

		case DMA_CTRL_DMA_REQSEL_UART0:
			if(DmaHandle->DSTADR == (uint32_t)&UART0->DATA)
			{
				UART0->CTRL |= UART_CTRL_TX_DMA_ENABLE;
			}
			else
			{
				UART0->CTRL |= UART_CTRL_RX_DMA_ENABLE;
			}
		  break;

		case DMA_CTRL_DMA_REQSEL_UART1:
			if(DmaHandle->DSTADR == (uint32_t)&UART1->DATA)
			{
				UART1->CTRL |= UART_CTRL_TX_DMA_ENABLE;
			}
			else
			{
				UART1->CTRL |= UART_CTRL_RX_DMA_ENABLE;
			}
		  break;

		case DMA_CTRL_DMA_REQSEL_I2C:
			if(DmaHandle->DSTADR == (uint32_t)&I2C->DATA)
			{
				I2C->CTRL |= I2C_CTRL_TX_DMA_ENABLE;
			}
			else
			{
				I2C->CTRL |= I2C_CTRL_RX_DMA_ENABLE;
			}
		  break;

		case DMA_CTRL_DMA_REQSEL_MEM:
			DmaHandle->CTRL |= DMA_CTRL_DMA_START;
		  break;

    case DMA_CTRL_DMA_REQSEL_I2S:
		case DMA_CTRL_DMA_REQSEL_DSADC:
		  break;
	}
}

/**
 * @brief
 *   Check DMA busy status
 * @param
 *   None.
 * @return
 *   0 -> DMA done.
 *   1 -> DMA busy.
 */
uint32_t DMA_CheckBusy(DMA_TYPE_DEF *DmaHandle)
{
  return (DmaHandle->CTRL & DMA_CTRL_DMA_BUSY_FLAG);
  // return (DmaHandle->CTRL & DMA_CTRL_DMA_ENABLE);
}

/**
 * @brief
 *
 * @param
 *  *DmaHandler [in]: The base address of DMAx module
 *    - DMA0, DMA1, DMA2, DMA3, DMA4
 *  *DmaFun [in]: DMA ISR service function address
 * @return
 *   None.
 */
void DMA_InstallIsrService(DMA_TYPE_DEF *DmaHandle, void(*DmaIsrServiceFunc)(void))
{
	if(DmaHandle == DMA0)
	{
		mDmaCh0IsrServiceFunPtr = DmaIsrServiceFunc;
	}
	else if(DmaHandle == DMA1)
	{
		mDmaCh1IsrServiceFunPtr = DmaIsrServiceFunc;
	}
  else if(DmaHandle == DMA2)
	{
		mDmaCh2IsrServiceFunPtr = DmaIsrServiceFunc;
	}
  else if(DmaHandle == DMA3)
	{
		mDmaCh3IsrServiceFunPtr = DmaIsrServiceFunc;
	}
  else if(DmaHandle == DMA4)
	{
		mDmaCh4IsrServiceFunPtr = DmaIsrServiceFunc;
	}
}

/**
 * @brief
 *
 * @param
 *  *DmaHandler [in]: The base address of DMAx module
 *    - DMA0, DMA1, DMA2, DMA3, DMA4
 * @return
 *   None.
 */
void DMA_UnInstallIsrService(DMA_TYPE_DEF *DmaHandle)
{
	if(DmaHandle == DMA0)
	{
		mDmaCh0IsrServiceFunPtr = NULL;
	}
	else if(DmaHandle == DMA1)
	{
		mDmaCh1IsrServiceFunPtr = NULL;
	}
  else if(DmaHandle == DMA2)
	{
		mDmaCh2IsrServiceFunPtr = NULL;
	}
  else if(DmaHandle == DMA3)
	{
		mDmaCh3IsrServiceFunPtr = NULL;
	}
  else if(DmaHandle == DMA4)
	{
		mDmaCh4IsrServiceFunPtr = NULL;
	}
}


/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void DMA0_IRQHandler()
{
  if(mDmaCh0IsrServiceFunPtr == NULL)
  {
    switch((DMA0->CTRL & DMA_CTRL_DMA_REQSEL_MSK))
    {
      case DMA_CTRL_DMA_REQSEL_SARADC:
        break;

      case DMA_CTRL_DMA_REQSEL_DAC_CH0:
        break;

      case DMA_CTRL_DMA_REQSEL_DAC_CH1:
        break;

      case DMA_CTRL_DMA_REQSEL_SPI0:
        break;

      case DMA_CTRL_DMA_REQSEL_SPI1:
        break;

      case DMA_CTRL_DMA_REQSEL_UART0:
        break;

      case DMA_CTRL_DMA_REQSEL_UART1:
        break;

      case DMA_CTRL_DMA_REQSEL_I2S:
        break;

      case DMA_CTRL_DMA_REQSEL_I2C:
        break;

      case DMA_CTRL_DMA_REQSEL_DSADC:
        break;

      case DMA_CTRL_DMA_REQSEL_MEM:
        break;
    }
  }
  else
  {
    mDmaCh0IsrServiceFunPtr();
  }

  DMA_INT->INTSTS = DMA_INTSTS_DMA0_DONE_INT_FLAG;
}


/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void DMA1_IRQHandler()
{
  if(mDmaCh1IsrServiceFunPtr == NULL)
  {
    switch((DMA1->CTRL & DMA_CTRL_DMA_REQSEL_MSK))
    {
      case DMA_CTRL_DMA_REQSEL_SARADC:
        break;

      case DMA_CTRL_DMA_REQSEL_DAC_CH0:
        break;

      case DMA_CTRL_DMA_REQSEL_DAC_CH1:
        break;

      case DMA_CTRL_DMA_REQSEL_SPI0:
        break;

      case DMA_CTRL_DMA_REQSEL_SPI1:
        break;

      case DMA_CTRL_DMA_REQSEL_UART0:
        break;

      case DMA_CTRL_DMA_REQSEL_UART1:
        break;

      case DMA_CTRL_DMA_REQSEL_I2S:
        break;

      case DMA_CTRL_DMA_REQSEL_I2C:
        break;

      case DMA_CTRL_DMA_REQSEL_DSADC:
        break;

      case DMA_CTRL_DMA_REQSEL_MEM:
        break;
    }
  }
  else
  {
    mDmaCh1IsrServiceFunPtr();
  }

  DMA_INT->INTSTS = DMA_INTSTS_DMA1_DONE_INT_FLAG;
}


/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void DMA2_IRQHandler()
{
  if(mDmaCh2IsrServiceFunPtr == NULL)
  {
    switch((DMA2->CTRL & DMA_CTRL_DMA_REQSEL_MSK))
    {
      case DMA_CTRL_DMA_REQSEL_SARADC:
        break;

      case DMA_CTRL_DMA_REQSEL_DAC_CH0:
        break;

      case DMA_CTRL_DMA_REQSEL_DAC_CH1:
        break;

      case DMA_CTRL_DMA_REQSEL_SPI0:
        break;

      case DMA_CTRL_DMA_REQSEL_SPI1:
        break;

      case DMA_CTRL_DMA_REQSEL_UART0:
        break;

      case DMA_CTRL_DMA_REQSEL_UART1:
        break;

      case DMA_CTRL_DMA_REQSEL_I2S:
        break;

      case DMA_CTRL_DMA_REQSEL_I2C:
        break;

      case DMA_CTRL_DMA_REQSEL_DSADC:
        break;

      case DMA_CTRL_DMA_REQSEL_MEM:
        break;
    }
  }
  else
  {
    mDmaCh2IsrServiceFunPtr();
  }

  DMA_INT->INTSTS = DMA_INTSTS_DMA2_DONE_INT_FLAG;
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void DMA3_IRQHandler()
{
  if(mDmaCh3IsrServiceFunPtr == NULL)
  {
    switch((DMA3->CTRL & DMA_CTRL_DMA_REQSEL_MSK))
    {
      case DMA_CTRL_DMA_REQSEL_SARADC:
        break;

      case DMA_CTRL_DMA_REQSEL_DAC_CH0:
        break;

      case DMA_CTRL_DMA_REQSEL_DAC_CH1:
        break;

      case DMA_CTRL_DMA_REQSEL_SPI0:
        break;

      case DMA_CTRL_DMA_REQSEL_SPI1:
        break;

      case DMA_CTRL_DMA_REQSEL_UART0:
        break;

      case DMA_CTRL_DMA_REQSEL_UART1:
        break;

      case DMA_CTRL_DMA_REQSEL_I2S:
        break;

      case DMA_CTRL_DMA_REQSEL_I2C:
        break;

      case DMA_CTRL_DMA_REQSEL_DSADC:
        break;

      case DMA_CTRL_DMA_REQSEL_MEM:
        break;
    }
  }
  else
  {
    mDmaCh3IsrServiceFunPtr();
  }

  DMA_INT->INTSTS = DMA_INTSTS_DMA3_DONE_INT_FLAG;
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void DMA4_IRQHandler()
{
  if(mDmaCh4IsrServiceFunPtr == NULL)
  {
    switch((DMA4->CTRL & DMA_CTRL_DMA_REQSEL_MSK))
    {
      case DMA_CTRL_DMA_REQSEL_SARADC:
        break;

      case DMA_CTRL_DMA_REQSEL_DAC_CH0:
        break;

      case DMA_CTRL_DMA_REQSEL_DAC_CH1:
        break;

      case DMA_CTRL_DMA_REQSEL_SPI0:
        break;

      case DMA_CTRL_DMA_REQSEL_SPI1:
        break;

      case DMA_CTRL_DMA_REQSEL_UART0:
        break;

      case DMA_CTRL_DMA_REQSEL_UART1:
        break;

      case DMA_CTRL_DMA_REQSEL_I2S:
        break;

      case DMA_CTRL_DMA_REQSEL_I2C:
        break;

      case DMA_CTRL_DMA_REQSEL_DSADC:
        break;

      case DMA_CTRL_DMA_REQSEL_MEM:
        break;
    }
  }
  else
  {
    mDmaCh4IsrServiceFunPtr();
  }

  DMA_INT->INTSTS = DMA_INTSTS_DMA4_DONE_INT_FLAG;
}
