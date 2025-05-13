/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   SPI_Flash_GPCM3_FM1.h
 * @Version:
 *   V0.9.0
 * @Date:
 *   September 2, 2022
 * @Abstract:
 *
 **************************************************************************************************/
#ifndef _SPI_FLASH_GPCM3_FM1_H_
#define _SPI_FLASH_GPCM3_FM1_H_


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include <stdint.h>


/*---------------------------------------------------------------------------------------
 * User Setting Area
 *---------------------------------------------------------------------------------------*/

/*
 *  User Setting:
 *   - SPI_OUTPUT_SEL: Select SPI0_IOA[12:9]/SPI0_IOB[4:1]/SPI1_IOA[3:0]/SPI1_IOA[21:18]
 *   - SPI_FLASH_4BYTE_ADDR_EN: Select SPI Flash 3byte/4byte address mode
 */
//#define  SD_SPI0_IOA9_12                    (1)
//#define  SD_SPI0_IOB1_4                     (2)
//#define  SD_SPI1_IOA0_3                     (3)
//#define  SD_SPI1_IOA18_21                   (4)
//#define  SD_SPI_OUTPUT_SEL                  SD_SPI1_IOA18_21             // User Setting

#define  SPI0_IOA3_6                      (1)
#define  SPI0_IOA26_29                    (2)
#define  SPI0_IOB1_4                      (3)
#define  SPI1_IOA9_12                     (4)
#define  SPI1_IOA22_25                    (5)
#define  SPI_OUTPUT_SEL                   SPI1_IOA9_12             // User Setting
#define  SPI_FLASH_4BYTE_ADDR_EN          DISABLE                  // User Setting


/*---------------------------------------------------------------------------------------
 * Definition Area
 *---------------------------------------------------------------------------------------*/

/*
 *  SPI Flash Command Set Definition
 */
#define SPI_FLASH_CE                     (0xC7)
#define SPI_FLASH_READ                   (0x03)
#define SPI_FLASH_BE                     (0xD8)
#define SPI_FLASH_SE                     (0x20)
#define SPI_FLASH_PP                     (0x02)
#define SPI_FLASH_RDSR                   (0x05)
#define SPI_FLASH_WREN                   (0x06)
#define SPI_FLASH_WRDI                   (0x04)
#define SPI_FLASH_RDID                   (0x9F)
#define SPI_FLASH_EN4B                   (0xB7)
#define SPI_FLASH_EX4B                   (0xE9)
#define SPI_FLASH_RDCR                   (0x15)

/*
 * SPI Flash Status Register Definition
 */
#define SPI_FLASH_SR_WIP_BIT             BIT0
#define SPI_FLASH_SR_WEL_BIT             BIT1
#define SPI_FLASH_SR_BP0_BIT             BIT2
#define SPI_FLASH_SR_BP1_BIT             BIT3
#define SPI_FLASH_SR_BP2_BIT             BIT4

/*
 * Definition for the 1st parameter of SPI_Flash_SetClkDiv()
 */
#define SPI_CLKSEL_PCLK_DIV2             SPI_CLK_DIV_PCLK_DIV_2
#define SPI_CLKSEL_PCLK_DIV4             SPI_CLK_DIV_PCLK_DIV_4
#define SPI_CLKSEL_PCLK_DIV6             SPI_CLK_DIV_PCLK_DIV_6
#define SPI_CLKSEL_PCLK_DIV8             SPI_CLK_DIV_PCLK_DIV_8
#define SPI_CLKSEL_PCLK_DIV10            SPI_CLK_DIV_PCLK_DIV_10
#define SPI_CLKSEL_PCLK_DIV16            SPI_CLK_DIV_PCLK_DIV_16
#define SPI_CLKSEL_PCLK_DIV32            SPI_CLK_DIV_PCLK_DIV_32
#define SPI_CLKSEL_PCLK_DIV64            SPI_CLK_DIV_PCLK_DIV_64
#define SPI_CLKSEL_PCLK_DIV128           SPI_CLK_DIV_PCLK_DIV_128
#define SPI_CLKSEL_PCLK_DIV256           SPI_CLK_DIV_PCLK_DIV_256
#define SPI_CLKSEL_PCLK_DIV512           SPI_CLK_DIV_PCLK_DIV_512
#define SPI_CLKSEL_PCLK_DIV1024          SPI_CLK_DIV_PCLK_DIV_1024


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C"{
#endif

void SPI_Flash_Open(void);
void SPI_Flash_Close(void);
void SPI_Flash_SetClkDiv(uint32_t SpiClkDiv);
void SPI_Write(uint8_t Data);
void Flash_Write_Enable(void);
void SPI_Flash_EN4B(void);
uint32_t SPI_Flash_ReadID(void);
uint8_t SPI_Flash_ReadSR(void);
uint8_t SPI_Flash_ReadCR(void);
void SPI_Flash_Sector_Erase(uint32_t SpiAddr);
void SPI_Flash_Block_Erase(uint32_t SpiAddr);
void SPI_Flash_Chip_Erase(void);
uint8_t SPI_Flash_ReadAByte(uint32_t SpiAddr);
void SPI_Flash_ReadNBytes(uint32_t SpiAddr, uint8_t *pDstBuf, uint32_t NumByteToRead);
void SPI_Flash_SendAByte(uint32_t SpiAddr, uint8_t pSrcBuf);
void SPI_Flash_SendNBytes(uint32_t SpiAddr, uint8_t* pSrcBuf, uint32_t NumByteToWrite);


#ifdef __cplusplus
}
#endif

#endif
