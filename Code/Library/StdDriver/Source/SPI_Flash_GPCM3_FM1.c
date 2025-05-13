/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   SPI_Flash_GPCM3_FM1.c
 * @Version:
 *   V0.9.0
 * @Date:
 *   September 2, 2022
 * @Abstract:
 *
 **************************************************************************************************/


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "GPCM3_FM1.h"


/*---------------------------------------------------------------------------------------
 * Module Vairable Declaration Area
 *---------------------------------------------------------------------------------------*/
SPI_TYPE_DEF 	*mSpiHandle;
__IO uint32_t *mSpiCsPin;


/*---------------------------------------------------------------------------------------
 * Subroutine Area
 *---------------------------------------------------------------------------------------*/

/**
 * @brief
 *   Open SPI
 * @param
 *   None.
 * @return
 *   None.
 */
void SPI_Flash_Open()
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
	// MODIFY_REG(IOFUNC->CTRL0, GPIOFUNC_CTRL0_SPI0_IOSEL_MSK, GPIOFUNC_CTRL0_SPI0_IOA3_6);
	mSpiHandle = SPI0;
	mSpiCsPin = &GPIOA_OBIT->OBIT05;
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
  mSpiHandle = SPI0;
	mSpiCsPin = &GPIOA_OBIT->OBIT28;
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
  mSpiHandle = SPI0;
	mSpiCsPin = &GPIOB_OBIT->OBIT03;
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
  mSpiHandle = SPI1;
	mSpiCsPin = &GPIOA_OBIT->OBIT11;
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
  mSpiHandle = SPI1;
	mSpiCsPin = &GPIOA_OBIT->OBIT24;
	#endif

	mSpiHandle->CTRL = SPI_CTRL_MASTER_MODE | SPI_CTRL_SPI_ENABLE | SPI_CTRL_MAUT_RW_EN_ENABLE;
	mSpiHandle->CLK_DIV = SPI_CLK_DIV_PCLK_DIV_16;
	*mSpiCsPin = 0xFFFFFFFF;                                                                          // CS high

	#if SPI_FLASH_4BYTE_ADDR_EN
	SPI_Flash_EN4B();
	#endif
}

/**
 * @brief
 *   Close SPI
 * @param
 *   None.
 * @return
 *   None.
 */
void SPI_Flash_Close()
{
  if(mSpiHandle != NULL)
	{
		mSpiHandle->CTRL = 0;
	}

	#if (SPI_OUTPUT_SEL == SPI0_IOA3_6) || (SPI_OUTPUT_SEL == SPI0_IOA26_29) || (SPI_OUTPUT_SEL == SPI0_IOB1_4)
	CLEAR_BIT(CLOCK->APBCKEN, CLOCK_APBCKEN_SPI0_CLK_ENABLE);          // APB SPI0 clock disable
	#endif

	#if (SPI_OUTPUT_SEL == SPI1_IOA9_12) || (SPI_OUTPUT_SEL == SPI1_IOA22_25)
	CLEAR_BIT(CLOCK->APBCKEN, CLOCK_APBCKEN_SPI1_CLK_ENABLE);          // APB SPI1 clock disable
	#endif
}

/**
 * @brief
 *
 * @param
 *   SpiClkDiv:
 *    - SPI_CLKSEL_PCLK_DIV2,  SPI_CLKSEL_PCLK_DIV4,  SPI_CLKSEL_PCLK_DIV6,   SPI_CLKSEL_PCLK_DIV8,   SPI_CLKSEL_PCLK_DIV10,  SPI_CLKSEL_PCLK_DIV16
 *      SPI_CLKSEL_PCLK_DIV32, SPI_CLKSEL_PCLK_DIV64, SPI_CLKSEL_PCLK_DIV128, SPI_CLKSEL_PCLK_DIV256, SPI_CLKSEL_PCLK_DIV512, SPI_CLKSEL_PCLK_DIV1024
 * @return
 *   None
 */
void SPI_Flash_SetClkDiv(uint32_t SpiClkDiv)
{
  MODIFY_REG(mSpiHandle->CLK_DIV, SPI_CLK_DIV_PCLK_DIV_MSK, SpiClkDiv);
}

/**
 * @brief
 *   Write a byte
 * @param
 *   Data [in]: 1 byte data
 * @return
 *   None.
 */
void SPI_Write(uint8_t Data)
{
	mSpiHandle->TX_DATA = Data;
	while((READ_BIT(mSpiHandle->STS, SPI_CTRL_TX_DONE_FLAG)) == 0);    // Wait until SPI transmission is completed.
	mSpiHandle->STS |= SPI_CTRL_TX_DONE_FLAG;                          // Clear transmission complete flag
}

/**
 * @brief
 *   Read a byte
 * @param
 *   None
 * @return
 *   1 byte data.
 */
uint8_t SPI_Read()
{
	mSpiHandle->CTRL |= SPI_CTRL_MASTER_RX_TRIG_ENABLE;
	while((mSpiHandle->STS&SPI_CTRL_TX_DONE_FLAG) == 0);               // Wait until SPI transmission is completed.
	mSpiHandle->STS |= SPI_CTRL_TX_DONE_FLAG;                          // Clear transmission complete flag

	return mSpiHandle->RX_DATA;
}

/**
 * @brief
 *   Enable flash to be written or erased
 * @param
 *   None.
 * @return
 *   None.
 */
void Flash_Write_Enable()
{
	*mSpiCsPin = 0;                                                    // CS low
	SPI_Write(SPI_FLASH_WREN);
	*mSpiCsPin = 0xFFFFFFFF;                                           // CS high
}

/**
 * @brief
 *   Disable flash to be written or erased
 * @param
 *   None.
 * @return
 *   None.
 */

void Flash_Write_Disable()
{
	*mSpiCsPin = 0;                                                    // CS low
	SPI_Write(SPI_FLASH_WRDI);
	*mSpiCsPin = 0xFFFFFFFF;                                           // CS high
}

/**
 * @brief
 *   Write N bytes data to flash
 * @param
 *  *pSrcBuf [in]: Source buffer address
 *   DstSpiAddr [in]: SPI flash address
 *   NumBytes [in]: Data length (Byte)
 * @return
 *   None
 */
void SPI_Flash_WritePage(uint8_t* pSrcBuf, uint32_t DstSpiAddr , uint32_t NumBytes)
{
	uint32_t index;

	Flash_Write_Enable();
	*mSpiCsPin = 0;                                                    // CS low
	SPI_Write(SPI_FLASH_PP);

	#if SPI_FLASH_4BYTE_ADDR_EN
	SPI_Write((WriteAddr & 0xFF000000) >> 24);                         // for 4-bytes address
	#endif

	SPI_Write((DstSpiAddr & 0xFF0000) >> 16);                          // addressH
	SPI_Write((DstSpiAddr & 0xFF00) >> 8);                             // addressM
	SPI_Write(DstSpiAddr & 0xFF);                                      // addressL
	for(index = 0; index < NumBytes; index++)
	{
		SPI_Write(*pSrcBuf++);
	}

	*mSpiCsPin = 0xFFFFFFFF;                                           // CS high
	while((SPI_Flash_ReadSR() & SPI_FLASH_SR_WIP_BIT) != 0);
}

/**
 * @brief
 *   Enable 4 bytes mode
 * @param
 *   None
 * @return
 *   None
 */
void SPI_Flash_EN4B()
{
	*mSpiCsPin = 0;                                                    // CS low
	SPI_Write(SPI_FLASH_EN4B);
	*mSpiCsPin = 0xFFFFFFFF;                                           // CS high
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   SPI flash ID
 */
uint32_t SPI_Flash_ReadID()
{
	uint32_t SpiId;

	*mSpiCsPin = 0;                                                    // CS low
	SPI_Write(SPI_FLASH_RDID);
	SpiId = SPI_Read();
	SpiId |= (SPI_Read() << 8);
	SpiId |= (SPI_Read() << 16);
	*mSpiCsPin = 0xFFFFFFFF;                                           // CS high

	return SpiId;
}

/**
 * @brief
 *   Read SPI Flash status register
 * @param
 *   None.
 * @return
 *   SPI Status Register
 */
uint8_t SPI_Flash_ReadSR()
{
	uint8_t	SpiStatusRegister = 0;

	*mSpiCsPin = 0;                                                    // CS low
	SPI_Write(SPI_FLASH_RDSR);
	SpiStatusRegister = SPI_Read();
	*mSpiCsPin = 0xFFFFFFFF;                                           // CS high

	return SpiStatusRegister;
}

/**
 * @brief
 *   Read configuration Register
 * @param
 *   None.
 * @return
 *   SPI configuration register
 */
uint8_t SPI_Flash_ReadCR()
{
	uint8_t	SpiStatusRegister = 0;

	*mSpiCsPin = 0;                                                    // CS low
	SPI_Write(SPI_FLASH_RDCR);
	SpiStatusRegister = SPI_Read();
	*mSpiCsPin = 0xFFFFFFFF;                                           // CS high

	return SpiStatusRegister;
}

 /**
 * @brief
 *   Erase one sector of flash
 * @param
 *   SpiAddr [in]: SPI flash address
 * @return
 *   None.
 */
void SPI_Flash_Sector_Erase(uint32_t SpiAddr)
{
	Flash_Write_Enable();
	*mSpiCsPin = 0;                                                    // CS low
	SPI_Write(SPI_FLASH_SE);

	#if SPI_FLASH_4BYTE_ADDR_EN
	SPI_Write((SectorAddr & 0xFF000000) >> 24);                        // for 4-bytes address
	#endif

	SPI_Write((SpiAddr & 0xFF0000) >> 16);                             // address high
	SPI_Write((SpiAddr & 0xFF00) >> 8);                                // address middle
	SPI_Write(SpiAddr & 0xFF);                                         // address low
	*mSpiCsPin = 0xFFFFFFFF;                                           // CS high
	while((SPI_Flash_ReadSR() & SPI_FLASH_SR_WIP_BIT) != 0);
}

/**
 * @brief
 *   Erase one block of flash
 * @param
 *   SpiAddr [in]: SPI flash address
 * @return
 *   None.
 */
void SPI_Flash_Block_Erase(uint32_t SpiAddr)
{
	Flash_Write_Enable();
	*mSpiCsPin = 0;                                                    // CS low
	SPI_Write(SPI_FLASH_BE);

	#if SPI_FLASH_4BYTE_ADDR_EN
	SPI_Write((BlockAddr & 0xFF000000) >> 24);                         // for 4-bytes address
	#endif

	SPI_Write((SpiAddr & 0xFF0000) >> 16);                             // address high
	SPI_Write((SpiAddr & 0xFF00) >> 8);                                // address middle
	SPI_Write(SpiAddr & 0xFF);                                         // address low
	*mSpiCsPin = 0xFFFFFFFF;                                           // CS high
	while((SPI_Flash_ReadSR() & SPI_FLASH_SR_WIP_BIT) != 0);
}

/**
 * @brief
 *   Erase hole chip of flash
 * @param
 *   None.
 * @return
 *   None.
 */
void SPI_Flash_Chip_Erase()
{
	Flash_Write_Enable();
	*mSpiCsPin = 0;                                                    // CS low
	SPI_Write(SPI_FLASH_CE);
	*mSpiCsPin = 0xFFFFFFFF;                                           // CS high
	while((SPI_Flash_ReadSR() & SPI_FLASH_SR_WIP_BIT) != 0);
}

/**
 * @brief
 *   Read a byte data from flash
 * @param
 *   SpiAddr [in]: SPI flash address
 * @return
 *   1 byte SPI flash data
 */
uint8_t SPI_Flash_ReadAByte(uint32_t SpiAddr)
{
	uint8_t SpiData;

	*mSpiCsPin = 0;                                                    // CS low
	SPI_Write(SPI_FLASH_READ);

	#if SPI_FLASH_4BYTE_ADDR_EN
	SPI_Write((Bytes_Address & 0xFF000000) >> 24);                     // for 4-bytes address
	#endif

	SPI_Write((SpiAddr & 0xFF0000) >> 16);                             // addressH
	SPI_Write((SpiAddr & 0xFF00) >> 8);                                // addressM
	SPI_Write(SpiAddr & 0xFF);                                         // addressL
	SpiData = SPI_Read();
	*mSpiCsPin = 0xFFFFFFFF;                                           // CS high

	return SpiData;
}

/**
 * @brief
 *   Read bytes data from flash
 * @param
 *   SpiAddr [in]: SPI flash address
 *  *pDstBuf [out]: Destination buffer address
 *   NumByteToRead [in]: Data length (Byte)
 * @return
 *   None
 */
void SPI_Flash_ReadNBytes(uint32_t SpiAddr, uint8_t *pDstBuf, uint32_t NumByteToRead)
{
	uint32_t index;

	*mSpiCsPin = 0;                                                    // CS low
	SPI_Write(SPI_FLASH_READ);

	#if SPI_FLASH_4BYTE_ADDR_EN
	SPI_Write((ReadAddr & 0xFF000000) >> 24);                          // for 4-bytes address
	#endif

	SPI_Write((SpiAddr & 0xFF0000) >> 16);                             // addressH
	SPI_Write((SpiAddr & 0xFF00) >> 8);                                // addressM
	SPI_Write(SpiAddr & 0xFF);                                         // addressL
	for(index = 0; index < NumByteToRead; index++)
	{
		*pDstBuf++ = SPI_Read();
	}
	*mSpiCsPin = 0xFFFFFFFF;                                           // CS high
}

/**
 * @brief
 *   Write a byte data to flash
 * @param
 *   SpiAddr [in]: SPI flash address
 *  *pSrcBuf [in]: Source buffer address
 * @return
 *   None.
 */
void SPI_Flash_SendAByte(uint32_t SpiAddr, uint8_t pSrcBuf)
{
	Flash_Write_Enable();
	*mSpiCsPin = 0;                                                    // CS low
	SPI_Write(SPI_FLASH_PP);

	#if SPI_FLASH_4BYTE_ADDR_EN
	SPI_Write((Bytes_Address & 0xFF000000) >> 24);                     // for 4-bytes address
	#endif

	SPI_Write((SpiAddr & 0xFF0000) >> 16);                             // addressH
	SPI_Write((SpiAddr & 0xFF00) >> 8);                                // addressM
	SPI_Write(SpiAddr & 0xFF);                                         // addressL
	SPI_Write(pSrcBuf);
	*mSpiCsPin = 0xFFFFFFFF;                                           // CS high
	while((SPI_Flash_ReadSR() & SPI_FLASH_SR_WIP_BIT) != 0);
}

/**
 * @brief
 *   Write N bytes data to flash
 * @param
 *   SpiAddr [in]: SPI flash address
 *  *pSrcBuf [in]: Source buffer address
 *   NumByteToWrite [in]: Data length (Byte)
 * @return
 *   None
 */
void SPI_Flash_SendNBytes(uint32_t SpiAddr, uint8_t* pSrcBuf, uint32_t NumByteToWrite)
{
	uint32_t PageEmptyBytes;

	while(1)
	{
		PageEmptyBytes = 0x100 - (SpiAddr & 0x00FF);
		if(NumByteToWrite <= PageEmptyBytes)
		{
			SPI_Flash_WritePage(pSrcBuf, SpiAddr, NumByteToWrite);
			break;
		}
		else
		{
			SPI_Flash_WritePage(pSrcBuf, SpiAddr, PageEmptyBytes);
			pSrcBuf += PageEmptyBytes;
			SpiAddr += PageEmptyBytes;
			NumByteToWrite -= PageEmptyBytes;
		}
	}
}
