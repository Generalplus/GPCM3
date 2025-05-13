/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   MMC_SD_User.c
 * @Version:
 *   V1.0.0
 * @Date:
 *   March 9, 2021
 * @Abstract:
 *
 **************************************************************************************************/

 /*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "MMC_SD_User.h"
#include "GPCM3_FM1.h"
#include "SPI_Flash_GPCM3_FM1.h"

#define	UseDMA										 0x01		//01:DMA  00:Normal
//#define	DMA_TX_CtrIO							 BIT15	//for DMA Ctrl   normal:FL   Ctrl:PH

/*---------------------------------------------------------------------------------------
 * Gobal Variable Declaration Area
 *---------------------------------------------------------------------------------------*/
SPI_TYPE_DEF 	*mSDHandle;
__IO uint32_t *mSDCsPin;
extern SPI_TYPE_DEF 	*mSpiHandle;
/*---------------------------------------------------------------------------------------
 * User Subroutine Area
 *---------------------------------------------------------------------------------------*/
/**
 * @brief
 * SPI Initial
 * @param
 *  None.
 * @return
 *  None
 */
void SD_SPI_Init(void)
{

	#if SPI_OUTPUT_SEL == SPI0_IOA3_6
	/*
	 * GPIOA[3](CLK): output low
	 * GPIOA[4](MOSI): output low
	 * GPIOA[5](CS): output high
	 * GPIOA[6](MISO): input floating
	 */
	GPIO_SetMode(GPIOA, BIT6, GPIO_MODE_FLOATING);
	GPIO_SetMode(GPIOA, (BIT3 | BIT4 | BIT5), GPIO_MODE_OUTPUT);
	GPIOA->OBUF &= ~(BIT3 | BIT4);
	GPIOA->OBUF |= BIT5;

	SET_BIT(CLOCK->APBCKEN, CLOCK_APBCKEN_SPI0_CLK_ENABLE);                                          // APB SPI0 Clock enable
	MODIFY_REG(IOFUNC->CTRL0, GPIOFUNC_CTRL0_SPI0_HWCS_EN_MSK, GPIOFUNC_CTRL0_SPI0_HWCS_DISABLE);    // SPI0 S/W CS
    MODIFY_REG(IOFUNC->CTRL0, GPIOFUNC_CTRL0_SPI0_IOSEL_MSK, GPIOFUNC_CTRL0_SPI0_IOA3_6);
	mSDHandle = SPI0;
	mSDCsPin = &GPIOA_OBIT->OBIT05;
	#endif

	#if SPI_OUTPUT_SEL == SPI0_IOA26_29
	/*
	 * GPIOA[26](CLK): output low
	 * GPIOA[27](MOSI) : output low
	 * GPIOA[28](CS) : output high
	 * GPIOA[29](MISO) : input floating
	 */
	GPIO_SetMode(GPIOA, BIT29, GPIO_MODE_FLOATING);
	GPIO_SetMode(GPIOA, (BIT26 | BIT27 | BIT28), GPIO_MODE_OUTPUT);
	GPIOA->OBUF &= ~(BIT26 | BIT27);
	GPIOA->OBUF |= BIT28;

	SET_BIT(CLOCK->APBCKEN, CLOCK_APBCKEN_SPI0_CLK_ENABLE);                                          // APB SPI0 Clock enable
	MODIFY_REG(IOFUNC->CTRL0, GPIOFUNC_CTRL0_SPI0_HWCS_EN_MSK, GPIOFUNC_CTRL0_SPI0_HWCS_DISABLE);    // SPI0 S/W CS
	MODIFY_REG(IOFUNC->CTRL0, GPIOFUNC_CTRL0_SPI0_IOSEL_MSK, GPIOFUNC_CTRL0_SPI0_IOA26_29);
    mSDHandle = SPI0;
	mSDCsPin = &GPIOA_OBIT->OBIT28;
	#endif

	#if SPI_OUTPUT_SEL == SPI0_IOB1_4
	/*
	 * GPIOB[1](CLK): output low
	 * GPIOB[2](MOSI) : output low
	 * GPIOB[3](CS) : output high
	 * GPIOB[4](MISO) : input floating
	 */
	GPIO_SetMode(GPIOB, BIT4, GPIO_MODE_FLOATING);
	GPIO_SetMode(GPIOB, (BIT1 | BIT2 | BIT3), GPIO_MODE_OUTPUT);
	GPIOB->OBUF &= ~(BIT1 | BIT2);
	GPIOB->OBUF |= BIT3;

	SET_BIT(CLOCK->APBCKEN, CLOCK_APBCKEN_SPI0_CLK_ENABLE);                                          // APB SPI0 Clock enable
	MODIFY_REG(IOFUNC->CTRL0, GPIOFUNC_CTRL0_SPI0_HWCS_EN_MSK, GPIOFUNC_CTRL0_SPI0_HWCS_DISABLE);    // SPI0 S/W CS
	MODIFY_REG(IOFUNC->CTRL0, GPIOFUNC_CTRL0_SPI0_IOSEL_MSK, GPIOFUNC_CTRL0_SPI0_IOB1_4);
    mSDHandle = SPI0;
	mSDCsPin = &GPIOB_OBIT->OBIT03;
	#endif

	#if SPI_OUTPUT_SEL == SPI1_IOA9_12
	/*
	 * GPIOA[9](CLK): output low
	 * GPIOA[10](MOSI) : output low
	 * GPIOA[11](CS) : output high
	 * GPIOA[12](MISO) : input floating
	 */
	GPIO_SetMode(GPIOA, BIT12, GPIO_MODE_FLOATING);
	GPIO_SetMode(GPIOA, (BIT9 | BIT10 | BIT11), GPIO_MODE_OUTPUT);
	GPIOA->OBUF &= ~(BIT9 | BIT10);
	GPIOA->OBUF |= BIT11;

	SET_BIT(CLOCK->APBCKEN, CLOCK_APBCKEN_SPI1_CLK_ENABLE);                                          // APB SPI1 Clock enable
	MODIFY_REG(IOFUNC->CTRL0, GPIOFUNC_CTRL0_SPI1_HWCS_EN_MSK, GPIOFUNC_CTRL0_SPI1_HWCS_DISABLE);    // SPI1 S/W CS
	MODIFY_REG(IOFUNC->CTRL0, GPIOFUNC_CTRL0_SPI1_IOSEL_MSK, GPIOFUNC_CTRL0_SPI1_IOA9_12);
    mSDHandle = SPI1;
	mSDCsPin = &GPIOA_OBIT->OBIT11;
	#endif

	#if SPI_OUTPUT_SEL == SPI1_IOA22_25
	/*
	 * GPIOA[22](CLK): output low
	 * GPIOA[23](MOSI) : output low
	 * GPIOA[24](CS) : output high
	 * GPIOA[25](MISO) : input floating
	 */
	GPIO_SetMode(GPIOA, BIT25, GPIO_MODE_FLOATING);
	GPIO_SetMode(GPIOA, (BIT22 | BIT23 | BIT24), GPIO_MODE_OUTPUT);
	GPIOA->OBUF &= ~(BIT22 | BIT23);
	GPIOA->OBUF |= BIT24;

	SET_BIT(CLOCK->APBCKEN, CLOCK_APBCKEN_SPI1_CLK_ENABLE);                                          // APB SPI1 Clock enable
	MODIFY_REG(IOFUNC->CTRL0, GPIOFUNC_CTRL0_SPI1_HWCS_EN_MSK, GPIOFUNC_CTRL0_SPI1_HWCS_DISABLE);    // SPI1 S/W CS
	MODIFY_REG(IOFUNC->CTRL0, GPIOFUNC_CTRL0_SPI1_IOSEL_MSK, GPIOFUNC_CTRL0_SPI1_IOA22_25);
    mSDHandle = SPI1;
	mSDCsPin = &GPIOA_OBIT->OBIT24;
	#endif

	mSDHandle->CTRL = SPI_CTRL_MASTER_MODE | SPI_CTRL_SPI_ENABLE | SPI_CTRL_MAUT_RW_EN_ENABLE;
	mSDHandle->CLK_DIV = SPI_CLK_DIV_PCLK_DIV_8;
	*mSDCsPin = 0xFFFFFFFF;                                                                          // CS high

	#if SPI_FLASH_4BYTE_ADDR_EN
	SPI_Flash_EN4B();
	#endif
	#if UseDMA
	//GPIO_SetMode(GPIOA, DMA_TX_CtrIO, GPIO_MODE_FLOATING);		//CtrlIO  Floating
	SetSD_SPIMode(1);		//0 normal  1:use DMA
	#endif
}

/**
 * @brief
 *  write data and return read data
 * @param
 *  data [in]: the data to be written
 * @return
 *  read data
 */
uint8_t SD_SPI_ReadWriteByte(uint8_t data)
{
	mSDHandle->TX_DATA = data;
//  mSDHandle->CTRL |= SPI_CTRL_MASTER_RX_TRIG_ENABLE;
	while((READ_BIT(mSDHandle->STS, SPI_CTRL_TX_DONE_FLAG)) == 0);    // Wait until SPI transmission is completed.
	mSDHandle->STS |= SPI_CTRL_TX_DONE_FLAG;                          // Clear transmission complete flag
	return mSDHandle->RX_DATA;
}
void SPI_DMARead(uint32_t DstAddr, uint16_t TransCount)
{
	#if UseDMA

    mSDHandle->CTRL |= SPI_CTRL_MOSI_DASEL_SRC_TX;
    DMA_Init(DMA2, DMA_REQSEL_SPI1, DMA_SRC_DATA_8B, DMA_SRC_ADDR_FIX, DMA_DST_DATA_8B, DMA_DST_ADDR_INC);
	DMA_Trigger(DMA2, (uint32_t)&mSDHandle->RX_DATA, DstAddr, TransCount);
//	mSDHandle->CTRL |= SPI_CTRL_MASTER_RX_TRIG_ENABLE;
	while(DMA_CheckBusy(DMA2) != 0);
	DMA_Close(DMA2);

	#endif


//    mSpiHandle->CTRL |= SPI_CTRL_MOSI_DASEL_SRC_TX;
//    DMA_Init(DMA2, DMA_REQSEL_SPI1, DMA_SRC_DATA_8B, DMA_SRC_ADDR_FIX, DMA_DST_DATA_8B, DMA_DST_ADDR_INC);
//    DMA_Trigger(DMA2, (uint32_t)&mSpiHandle->RX_DATA, DstAddr, TransCount);
//
////	DMA_Trigger(DMA2, (uint32_t)&mSpiHandle->RX_DATA, DstAddr, TransCount);
////	mSpiHandle->CTRL |= SPI_CTRL_MASTER_RX_TRIG_ENABLE;
//	while(DMA_CheckBusy(DMA2) != 0);
//	DMA_Close(DMA2);
}

void SPI_DMAWrite(uint32_t SrcAddr, uint16_t TransCount)
{
	#if UseDMA
    DMA_Init(DMA2, DMA_REQSEL_SPI1, DMA_SRC_DATA_8B, DMA_SRC_ADDR_INC, DMA_DST_DATA_8B, DMA_SRC_ADDR_FIX);
	DMA_Trigger(DMA2, SrcAddr, (uint32_t)&mSDHandle->TX_DATA, TransCount);

//	mSDHandle->CTRL &= ~SPI_CTRL_SPI_ENABLE;
//	mSDHandle->CTRL |= SPI_CTRL_SPI_ENABLE; // Need a SPI Enable Rising Edge

	while(DMA_CheckBusy(DMA2) != 0);
	DMA_Close(DMA2);
//	mSDHandle->CTRL &= ~SPI_CTRL_SPI_ENABLE;
//	mSDHandle->CTRL |= SPI_CTRL_SPI_ENABLE; // Need a SPI Enable Rising Edge
	#endif
}
/**
 * @brief
 *  SD low speed mode
 * @param
 *  None.
 * @return
 *  None
 */
void SD_SPI_SpeedLow(void)
{
	//MODIFY_REG(mSDHandle->CTRL, SPI_CLK_DIV_PCLK_DIV_MSK, SPI_CLKSEL_PCLK_DIV1024);
	MODIFY_REG(mSDHandle->CLK_DIV, SPI_CLK_DIV_PCLK_DIV_MSK, SPI_CLKSEL_PCLK_DIV512);
}

/**
 * @brief
 * SD high speed mode
 * @param
 *  None.
 * @return
 *  None
 */
void SD_SPI_SpeedHigh(void)
{
	//MODIFY_REG(mSDHandle->CTRL, SPI_CLK_DIV_PCLK_DIV_MSK, SPI_CLKSEL_PCLK_DIV6);
	//MODIFY_REG(mSDHandle->CLK_DIV, SPI_CLK_DIV_PCLK_DIV_MSK, SPI_CLKSEL_PCLK_DIV8);
MODIFY_REG(mSDHandle->CLK_DIV, SPI_CLK_DIV_PCLK_DIV_MSK, SPI_CLKSEL_PCLK_DIV8);
}

/**
 * @brief
 *  SD CS Pin Output High
 * @param
 *  None.
 * @return
 *  None
 */
void SD_CSPin_High(void)
{
	*mSDCsPin = 0xFFFFFFFF;
}

/**
 * @brief
 *  SD CS Pin Output Low
 * @param
 *  None.
 * @return
 *  None
 */
void SD_CSPin_Low(void)
{
	*mSDCsPin = 0x000000;
}





























