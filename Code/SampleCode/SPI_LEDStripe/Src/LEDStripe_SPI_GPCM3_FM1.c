/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   LEDStripe_SPIDriver_GPCM3_FM1.c
 * @Version:
 *   V0.9.0
 * @Date:
 *   June 12, 2023
 * @Abstract:
 *
 **************************************************************************************************/


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "GPCM3_FM1.h"
#include "LEDStripe_SPI_GPCM3_FM1.h"


/*---------------------------------------------------------------------------------------
 * Module Vairable Declaration Area
 *---------------------------------------------------------------------------------------*/
DMA_TYPE_DEF *mLedStripDmaHandle;
LEDSTRIPE_SPI_WORKING_RAM LedStripWorkingRam;

#define C_LED_MAXRAWDATANUM     27
uint8_t  t_LEDSTRIP_RAWDATA[C_LED_MAXRAWDATANUM]=
{
  0xE0, 0x00, 0x00,    //LED1 GRB
  0x00, 0xE0, 0x00,    //LED2 GRB
  0x00, 0x00, 0xE0,    //LED3 GRB
  0x00, 0xE0, 0x00,    //LED1 GRB
  0x00, 0x00, 0xE0,    //LED2 GRB
  0xE0, 0x00, 0x00,    //LED3 GRB
  0x00, 0x00, 0xE0,    //LED1 GRB
  0xE0, 0x00, 0x00,    //LED2 GRB
  0x00, 0xE0, 0x00,    //LED3 GRB
};
uint8_t *t_LEDSTRIP_RAWDATAptr = &t_LEDSTRIP_RAWDATA[0];
uint16_t LEDRawDataCnt = 0;


/*---------------------------------------------------------------------------------------
 * Subroutine Area
 *---------------------------------------------------------------------------------------*/
/**
 * @brief
 *   Working RAM initialization, open SPI0, SPI1 & 512Hz RTC.
 * @param
 *   None.
 * @return
 *   None.
 */
 void LEDStripe_SPI_Initial(void)
{
	mLedStripDmaHandle = DMA3;

	/*
   * SPI1 initial
	 * GPIOA[25:24]: configured as MOSI pins
	 */
  GPIOA->FST |= (BIT22 | BIT23);                                                                   // IOA[23:22] configured as GPIO pins.
	SET_BIT(CLOCK->APBCKEN, CLOCK_APBCKEN_SPI1_CLK_ENABLE);                                          // APB SPI1 Clock enable
	// MODIFY_REG(IOFUNC->CTRL0, GPIOFUNC_CTRL0_SPI1_HWCS_EN_MSK, GPIOFUNC_CTRL0_SPI1_HWCS_DISABLE);    // SPI1 S/W CS
	MODIFY_REG(IOFUNC->CTRL0, GPIOFUNC_CTRL0_SPI1_IOSEL_MSK, GPIOFUNC_CTRL0_SPI1_IOA22_25_MOSI);
  SPI1->CTRL = SPI_CTRL_MASTER_MODE | SPI_CTRL_MAUT_RW_EN_ENABLE | SPI_CTRL_CLK_PHA_NORMAL | SPI_CTRL_CLK_POL0 | SPI_CTRL_SPICLK_CONTIUNE_ENABLE;
  SPI1->CLK_DIV = C_SPI1_CLK_DIV << SPI_CLK_DIV_PCLK_DIV_POS;                                      // Kim: 0x113BL for SK6812D 2.25us

	GPIO_SetMode(GPIOA, (BIT24 | BIT25), GPIO_MODE_OUTPUT);
	GPIOA->OBUF &= ~( BIT24 | BIT25);

	LedStripWorkingRam.R_LED_PlayFlag &= ~(SPI1_PLAY_FLAG + SPI1_UPDATE_FLAG);
	LedStripWorkingRam.R_SPI1_ColorDataG = 0;
	LedStripWorkingRam.R_SPI1_ColorDataR = 0;
	LedStripWorkingRam.R_SPI1_ColorDataB = 0;
	LedStripWorkingRam.R_SPI1_ColorEffect = 0;
	LedStripWorkingRam.R_512Hz_Count = 0;
  LEDStripe_SPI1_ClearAll();

  TIMEBASE_Reset();
  TIMEBASE_Open(TB_CLK_SEL_IOSC32K);
	TIMEBASE_EnableInt(TB_INT_512HZ);
	NVIC_EnableIRQ(TIMEBASE_IRQn);

  /*
	 * GPIOA[26](CLK): output low
	 * GPIOA[27](MOSI): output low
	 * GPIOA[29](MISO): input pull low
	 */
/*
	GPIO_SetMode(GPIOA, BIT29, GPIO_MODE_INPUT);
	GPIO_SetMode(GPIOA, (BIT26 | BIT27), GPIO_MODE_OUTPUT);
	GPIOA->OBUF &= ~(BIT26 | BIT27 | BIT29);

	SET_BIT(CLOCK->APBCKEN, CLOCK_APBCKEN_SPI0_CLK_ENABLE);                                          // APB SPI0 Clock enable
	MODIFY_REG(IOFUNC->CTRL0, GPIOFUNC_CTRL0_SPI0_HWCS_EN_MSK, GPIOFUNC_CTRL0_SPI0_HWCS_DISABLE);    // SPI0 S/W CS
	MODIFY_REG(IOFUNC->CTRL0, GPIOFUNC_CTRL0_SPI0_IOSEL_MSK, GPIOFUNC_CTRL0_SPI0_IOA26_29);
	mLedStripDmaHandle = DMA3;

    SPI0->CTRL = SPI_CTRL_MASTER_MODE | SPI_CTRL_MAUT_RW_EN_ENABLE | SPI_CTRL_CLK_PHA_NORMAL | SPI_CTRL_CLK_POL0 | SPI_CTRL_SPICLK_CONTIUNE_ENABLE;
    // SPI0->CLK_DIV = 0x10BUL << SPI_CLK_DIV_PCLK_DIV_POS;
    SPI0->CLK_DIV = 0x113UL << SPI_CLK_DIV_PCLK_DIV_POS;                                             // Kim: 0x113BL for SK6812D 2.25us
	LedStripWorkingRam.R_LED_PlayFlag &= ~(SPI0_PLAY_FLAG + SPI0_UPDATE_FLAG);
	LedStripWorkingRam.R_SPI0_ColorDataG = 0;
	LedStripWorkingRam.R_SPI0_ColorDataR = 0;
	LedStripWorkingRam.R_SPI0_ColorDataB = 0;
	LedStripWorkingRam.R_SPI0_ColorEffect = 0;
	LEDStripe_SPI0_ClearAll();
*/
}


/**
 * @brief
 *   Reset LED Stripe that hook on SPI0
 * @param
 *   None.
 * @return
 *   None.
 */
void LEDStripe_SPI0_ClearAll(void)
{
  uint16_t i;

  for(i=0;i<C_SPI0_BUFFER_SIZE;i++)
  {
    LedStripWorkingRam.R_SPI0_DataBuffer[i] = C_LED_LOGICAL_ZERO_HN + C_LED_LOGICAL_ZERO_LN;
  }

  DMA_Init(mLedStripDmaHandle, DMA_REQSEL_SPI0, DMA_SRC_DATA_8B, DMA_SRC_ADDR_INC, DMA_DST_DATA_8B, DMA_DST_ADDR_FIX);
  DMA_Trigger(mLedStripDmaHandle, (uint32_t)LedStripWorkingRam.R_SPI0_DataBuffer, (uint32_t)&SPI0->TX_DATA, C_SPI0_BUFFER_SIZE);
  SPI0->CTRL |= SPI_CTRL_SPI_ENABLE;
  while(DMA_CheckBusy(DMA3) != 0);
  while((READ_BIT(SPI0->STS, SPI_CTRL_TX_DONE_FLAG)) == 0);    // Wait until SPI transmission is completed.
  SPI0->STS |= SPI_CTRL_TX_DONE_FLAG;
  SPI0->CTRL &= ~SPI_CTRL_SPI_ENABLE;
}


/**
 * @brief
 *   Reset LED Stripe that hook on SPI1
 * @param
 *   None.
 * @return
 *   None.
 */
void LEDStripe_SPI1_ClearAll(void)
{
  uint16_t i;

  for(i=0;i<C_SPI1_BUFFER_SIZE;i++)
  {
    LedStripWorkingRam.R_SPI1_DataBuffer[i] = C_LED_LOGICAL_ZERO_HN + C_LED_LOGICAL_ZERO_LN;
  }

  GPIOA->FST &= ~BIT24;                 // LED stripe output from IOA.24
  GPIOA->FST |= BIT25;                  // Mask IOA.25
  DMA_Init(mLedStripDmaHandle, DMA_REQSEL_SPI1, DMA_SRC_DATA_8B, DMA_SRC_ADDR_INC, DMA_DST_DATA_8B, DMA_DST_ADDR_FIX);
  DMA_Trigger(mLedStripDmaHandle, (uint32_t)LedStripWorkingRam.R_SPI1_DataBuffer, (uint32_t)&SPI1->TX_DATA, C_SPI1_BUFFER_SIZE);
	SPI1->CTRL |= SPI_CTRL_SPI_ENABLE;
  while(DMA_CheckBusy(DMA3) != 0);
  while((READ_BIT(SPI1->STS, SPI_CTRL_TX_DONE_FLAG)) == 0);    // Wait until SPI transmission is completed.
  SPI1->STS |= SPI_CTRL_TX_DONE_FLAG;

  GPIOA->FST &= ~BIT25;                 // LED stripe output from IOA.25
  GPIOA->FST |= BIT24;                  // Mask IOA.24
  DMA_Init(mLedStripDmaHandle, DMA_REQSEL_SPI1, DMA_SRC_DATA_8B, DMA_SRC_ADDR_INC, DMA_DST_DATA_8B, DMA_DST_ADDR_FIX);
  DMA_Trigger(mLedStripDmaHandle, (uint32_t)LedStripWorkingRam.R_SPI1_DataBuffer, (uint32_t)&SPI1->TX_DATA, C_SPI1_BUFFER_SIZE);
	SPI1->CTRL |= SPI_CTRL_SPI_ENABLE;
  while(DMA_CheckBusy(DMA3) != 0);
  while((READ_BIT(SPI1->STS, SPI_CTRL_TX_DONE_FLAG)) == 0);    // Wait until SPI transmission is completed.
  SPI1->STS |= SPI_CTRL_TX_DONE_FLAG;

  SPI1->CTRL &= ~SPI_CTRL_SPI_ENABLE;
}


/**
 * @brief
 *   DMA interrupt service routine for SPI0.
 * @param
 *   None.
 * @return
 *   None.
 */
void LEDStripe_SPI0_DmaIsr(void)
{
  while((READ_BIT(SPI0->STS, SPI_CTRL_TX_DONE_FLAG)) == 0);    // Wait until SPI transmission is completed.
  SPI0->STS |= SPI_CTRL_TX_DONE_FLAG;
  DMA_Close(mLedStripDmaHandle);
  SPI0->CTRL &= ~SPI_CTRL_SPI_ENABLE;
  LedStripWorkingRam.R_LED_PlayFlag |= SPI0_UPDATE_FLAG;
  GPIOA->FST |= BIT27;
}


/**
 * @brief
 *   DMA interrupt service routine for SPI1.
 * @param
 *   None.
 * @return
 *   None.
 */
void LEDStripe_SPI1_DmaIsr(void)
{
  while((READ_BIT(SPI1->STS, SPI_CTRL_TX_DONE_FLAG)) == 0);    // Wait until SPI transmission is completed.
  SPI1->STS |= SPI_CTRL_TX_DONE_FLAG;
  DMA_Close(mLedStripDmaHandle);
  SPI1->CTRL &= ~SPI_CTRL_SPI_ENABLE;
  LedStripWorkingRam.R_LED_PlayFlag |= SPI1_UPDATE_FLAG;
}


/**
 * @brief
 *   LED stripe frame servie.
 * @param
 *   None.
 * @return
 *   None.
 */
void LEDStripe_SPI_FrameServiceIsr(void)
{
  LedStripWorkingRam.R_512Hz_Count += 1;
  if(LedStripWorkingRam.R_512Hz_Count >= 0x10)
  {
    LedStripWorkingRam.R_512Hz_Count = 0;

    if((LedStripWorkingRam.R_LED_PlayFlag & SPI1_PLAY_FLAG) == SPI1_PLAY_FLAG)
    {
      GPIOA->FST &= ~BIT25;                 // LED stripe output from IOA.25
      GPIOA->FST |= BIT24;                  // Mask IOA.24
      DMA_Init(mLedStripDmaHandle, DMA_REQSEL_SPI1, DMA_SRC_DATA_8B, DMA_SRC_ADDR_INC, DMA_DST_DATA_8B, DMA_DST_ADDR_FIX);
      DMA_InstallIsrService(mLedStripDmaHandle, LEDStripe_SPI1_DmaIsr);
      DMA_EnableInt(mLedStripDmaHandle);
      NVIC_EnableIRQ(DMA3_IRQn);
      DMA_Trigger(mLedStripDmaHandle, (uint32_t)LedStripWorkingRam.R_SPI1_DataBuffer, (uint32_t)&SPI1->TX_DATA, C_SPI1_BUFFER_SIZE);
      SPI1->CTRL |= SPI_CTRL_SPI_ENABLE;
    }
  }
  if(LedStripWorkingRam.R_512Hz_Count == 0x08)
  {
    if((LedStripWorkingRam.R_LED_PlayFlag & SPI1_PLAY_FLAG) == SPI1_PLAY_FLAG)
    {
      GPIOA->FST &= ~BIT24;                 // LED stripe output from IOA.24
      GPIOA->FST |= BIT25;                  // Mask IOA.25
      DMA_Init(mLedStripDmaHandle, DMA_REQSEL_SPI1, DMA_SRC_DATA_8B, DMA_SRC_ADDR_INC, DMA_DST_DATA_8B, DMA_DST_ADDR_FIX);
      DMA_InstallIsrService(mLedStripDmaHandle, LEDStripe_SPI1_DmaIsr);
      DMA_EnableInt(mLedStripDmaHandle);
      NVIC_EnableIRQ(DMA3_IRQn);
      DMA_Trigger(mLedStripDmaHandle, (uint32_t)LedStripWorkingRam.R_SPI1_DataBuffer, (uint32_t)&SPI1->TX_DATA, C_SPI1_BUFFER_SIZE);
      SPI1->CTRL |= SPI_CTRL_SPI_ENABLE;
    }
  }
    /*
    if((LedStripWorkingRam.R_LED_PlayFlag & SPI0_PLAY_FLAG) == SPI0_PLAY_FLAG)
    {
      DMA_Init(mLedStripDmaHandle, DMA_REQSEL_SPI0, DMA_SRC_DATA_8B, DMA_SRC_ADDR_INC, DMA_DST_DATA_8B, DMA_DST_ADDR_FIX);
      DMA_InstallIsrService(mLedStripDmaHandle, LEDStripe_SPI0_DmaIsr);
      DMA_EnableInt(mLedStripDmaHandle);
      NVIC_EnableIRQ(DMA3_IRQn);
      GPIOA->FST &= ~BIT27;
      DMA_Trigger(mLedStripDmaHandle, (uint32_t)LedStripWorkingRam.R_SPI0_DataBuffer, (uint32_t)&SPI0->TX_DATA, C_SPI0_BUFFER_SIZE);
      SPI0->CTRL |= SPI_CTRL_SPI_ENABLE;
    }
    */
}


/**
 * @brief
 *   LED Stripe mainloop service.
 * @param
 *   None.
 * @return
 *   None.
 */
void LEDStripe_SPI_MainLoopService(void)
{
  uint8_t LedCnt;
  uint16_t BufIdx;

  if(DMA_CheckBusy(mLedStripDmaHandle) != 0)
  {
    return;
  }

  if((LedStripWorkingRam.R_LED_PlayFlag & SPI1_UPDATE_FLAG) != 0)
  {
    for(LedCnt=0,BufIdx=0;LedCnt<C_SPI1_LED_Number;LedCnt++,BufIdx+=12,t_LEDSTRIP_RAWDATAptr+=3)
    {
      if(LEDRawDataCnt >= C_LED_MAXRAWDATANUM)
      {
        // LEDRawDataCnt = 0;
        LEDRawDataCnt = 3;
        t_LEDSTRIP_RAWDATAptr = &t_LEDSTRIP_RAWDATA[0];
      }
      else
      {
        LEDRawDataCnt += 3;
      }
      LEDStripe_ReloadBuf(t_LEDSTRIP_RAWDATAptr,&LedStripWorkingRam.R_SPI1_DataBuffer[BufIdx]);
    }
    LedStripWorkingRam.R_LED_PlayFlag &= ~SPI1_UPDATE_FLAG;
  }
  /*
  if((LedStripWorkingRam.R_LED_PlayFlag & SPI0_UPDATE_FLAG) != 0)
  {
    //LedStripWorkingRam.R_SPI0_ColorEffect += 4;

    // User define effect ++
    //LedStripWorkingRam.R_SPI0_ColorDataG = LedStripWorkingRam.R_SPI0_ColorEffect;
    //LedStripWorkingRam.R_SPI0_ColorDataR = 0;
    //LedStripWorkingRam.R_SPI0_ColorDataB = 0;
    // User define effect --

    for(LedCnt=0,BufIdx=0;LedCnt<C_SPI0_LED_Number;LedCnt++,BufIdx+=12)
    {
      LEDStripe_ReloadBuf(&LedStripWorkingRam.R_SPI0_ColorDataG,&LedStripWorkingRam.R_SPI0_DataBuffer[BufIdx]);
    }
    LedStripWorkingRam.R_LED_PlayFlag &= ~SPI0_UPDATE_FLAG;
  }
  */
}


/**
 * @brief
 *   Translate RGB data to SPI data then write into buffer
 * @param
 *   *pRGBData [in]: RGB raw data address
 *   *pDstBuf [in]: destinatin buffer address
 * @return
 *   None.
 */
void LEDStripe_ReloadBuf(uint8_t* pRGBData, uint8_t* pDstBuf)
{
  uint8_t BitMask,RGBCnt,BufIdx;

  for(RGBCnt=0,BufIdx=0;RGBCnt<3;RGBCnt++)
  {
    for(BitMask=0x80;BitMask!=0;BitMask>>=1)
    {
      if((BitMask & 0x55) == 0)
      {
        if((pRGBData[RGBCnt] & BitMask) != 0)
        {
          pDstBuf[BufIdx] = C_LED_LOGICAL_ONE_HN;
        }
        else
        {
          pDstBuf[BufIdx] = C_LED_LOGICAL_ZERO_HN;
        }
      }
      else
      {
        if((pRGBData[RGBCnt] & BitMask) != 0)
        {
          pDstBuf[BufIdx] |= C_LED_LOGICAL_ONE_LN;
        }
        else
        {
          pDstBuf[BufIdx] |= C_LED_LOGICAL_ZERO_LN;
        }
        BufIdx += 1;
      }
    }
  }
}


/**
 * @brief
 *   SPI0 start
 * @param
 *   None.
 * @return
 *   None.
 */
void LEDStripe_SPI0_Start(void)
{
  LedStripWorkingRam.R_LED_PlayFlag |= SPI0_PLAY_FLAG + SPI0_UPDATE_FLAG;
}


/**
 * @brief
 *   SPI1 start
 * @param
 *   None.
 * @return
 *   None.
 */
void LEDStripe_SPI1_Start(void)
{
  LedStripWorkingRam.R_LED_PlayFlag |= SPI1_PLAY_FLAG + SPI1_UPDATE_FLAG;
}

/**
 * @brief
 *   SPI0 stop
 * @param
 *   None.
 * @return
 *   None.
 */
void LEDStripe_SPI0_Stop(void)
{
  LedStripWorkingRam.R_LED_PlayFlag &= ~(SPI0_PLAY_FLAG + SPI0_UPDATE_FLAG);
}


/**
 * @brief
 *   SPI1 stop
 * @param
 *   None.
 * @return
 *   None.
 */
void LEDStripe_SPI1_Stop(void)
{
  LedStripWorkingRam.R_LED_PlayFlag &= ~(SPI1_PLAY_FLAG + SPI1_UPDATE_FLAG);
}



