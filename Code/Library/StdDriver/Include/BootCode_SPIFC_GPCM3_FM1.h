/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   BOOTCODE_SPIFC_GPCM3_FM1.h
 * @Version:
 *   V1.0.2
 * @Date:
 *   Nov. 3, 2020
 * @Abstract:
 *
 **************************************************************************************************/
#ifndef _BOOTCODE_SPIFC_GPCM3_FM1_H_
#define _BOOTCODE_SPIFC_GPCM3_FM1_H_


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>


/*---------------------------------------------------------------------------------------
 * User Definition Area
 *---------------------------------------------------------------------------------------*/
#define    SPIFC_CHECK_SUM_START_ADDRESS    (SPIFC_EXT_MEM_START_ADDRESS)
#define    SPIFC_CHECK_SUM_LENGTH           (64)               // unit: word
#define    SPIFC_CHECK_SUM_VALUE            (0x5A810A3C)       // User must define SPIFC_CHECK_SUM_VALUE according to real SPI flash data.


/*---------------------------------------------------------------------------------------
 * Definition Area
 *---------------------------------------------------------------------------------------*/
#define    SPIFC_EXT_MEM_START_ADDRESS      (0x04000000)

#define    SPIFC_CALIBRATION_PASS           (0)
#define    SPIFC_CALIBRATION_FAIL           (0xFFFFFFFF)

/*
 *  SPI Flash Command Set Definition
 */
#define    SPIFC_FLASH_CE		                (0xC7)
#define    SPIFC_FLASH_READ                 (0x03)
#define    SPIFC_FLASH_READ4B               (0x13)
#define    SPIFC_FLASH_2READ                (0xBB)
#define    SPIFC_FLASH_2READ4B              (0xBC)
#define    SPIFC_FLASH_4READ                (0xEB)
#define    SPIFC_FLASH_4READ4B              (0xEC)
#define    SPIFC_FLASH_BE                   (0xD8)
#define    SPIFC_FLASH_SE                   (0x20)
#define    SPIFC_FLASH_PP                   (0x02)
#define    SPIFC_FLASH_RDSR                 (0x05)
#define    SPIFC_FLASH_WRSR                 (0x01)
#define    SPIFC_FLASH_WREN                 (0x06)
#define    SPIFC_FLASH_WRDI                 (0x04)
#define    SPIFC_FLASH_RDID                 (0x9F)
#define    SPIFC_FLASH_REMS                 (0x90)
#define    SPIFC_FLASH_DP                   (0xB9)
#define    SPIFC_FLASH_RDP                  (0xAB)
#define    SPIFC_FLASH_EN4B                 (0xB7)
#define    SPIFC_FLASH_EX4B                 (0xE9)
#define    SPIFC_FLASH_RDCR                 (0x15)

/*
 * SPI Flash Status Register Definition
 */
#define    SPIFC_SR_WIP_BIT                 BIT0
#define    SPIFC_SR_WEL_BIT                 BIT1
#define    SPIFC_SR_BP0_BIT                 BIT2
#define    SPIFC_SR_BP1_BIT                 BIT3
#define    SPIFC_SR_BP2_BIT                 BIT4

/*
 * Definition for the 1st parameter of SPIFC_SetClkDiv()
 */
#define    SPIFC_CLKSEL_HCLK_DIV1           SPIFC_CTRL2_CLK_SEL_HCLK_DIV_1
#define    SPIFC_CLKSEL_HCLK_DIV2           SPIFC_CTRL2_CLK_SEL_HCLK_DIV_2
#define    SPIFC_CLKSEL_HCLK_DIV3           SPIFC_CTRL2_CLK_SEL_HCLK_DIV_3
#define    SPIFC_CLKSEL_HCLK_DIV4           SPIFC_CTRL2_CLK_SEL_HCLK_DIV_4

/*
 * Definition for the 1st parameter of SPIFC_AutoMode()
 */
#define    SPIFC_1IO_MODE                   (1)
#define    SPIFC_2IO_MODE                   (2)
#define    SPIFC_4IO_MODE                   (3)
#define    SPIFC_4IO_ENHANCE_MODE           (4)
#define    SPIFC_2IO_ENHANCE_MODE           (5)

#if defined(__CC_ARM)
#define DISABLE_OPT
#elif  defined(__GNUC__)
#define DISABLE_OPT __attribute__((optimize("O0")))
#endif


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C"{
#endif

int32_t BootCode_SPIFC_TimingFineTune(void);
void BootCode_SPIFC_Open(void);
void BootCode_SPIFC_Close(void);
void BootCode_SPIFC_AutoMode(uint32_t MultiIoMode);
void BootCode_SPIFC_EnhanModeReset(void);
void BootCode_SPIFC_SetClkDiv(uint32_t SpiClkDiv);

void BootCode_SPIFC_EX4B(void);
void BootCode_SPIFC_EN4B(void);
uint32_t BootCode_SPIFC_ReadID(void);
void BootCode_SPIFC_WriteEnable(void);
void BootCode_SPIFC_WriteDisable(void);
uint16_t BootCode_SPIFC_ReadSR(void);
void BootCode_SPIFC_WriteSR(uint8_t SpiStatusRegister);
uint8_t BootCode_SPIFC_ReadCR(void);
void BootCode_SPIFC_SectorErase(uint32_t SpiAddr);
void BootCode_SPIFC_BlockErase(uint32_t SpiAddr);
void BootCode_SPIFC_ReadNBytes(uint32_t ReadAddr, uint8_t *pDstBuf, uint32_t NumBytes);
void BootCode_SPIFC_WriteNBytes(uint32_t SpiAddr, uint8_t *pSrcBuf, uint32_t NumBytes);

void BootCode_SPIFC_Cmd(uint8_t SpiCmd);
void BootCode_SPIFC_CmdAddr(uint8_t SpiCmd, uint32_t SpiAddr);
void BootCode_SPIFC_CmdRx(uint8_t SpiCmd, uint8_t *pDstBuf, uint32_t NumBytes);
void SPIFC_CmdAddrRx(uint8_t SpiCmd, uint32_t SpiAddr, uint8_t *pDstBuf, uint32_t NumBytes);
void BootCode_SPIFC_CmdTx(uint8_t SpiCmd, uint8_t *pSrcBuf, uint32_t NumBytes);
void BootCode_SPIFC_CmdAddrTx(uint8_t SpiCmd, uint32_t SpiAddr, uint8_t *pSrcBuf, uint32_t NumBytes);
void BootCode_SPIFC_CmdAddrRx(uint8_t SpiCmd, uint32_t SpiAddr, uint8_t *pDstBuf, uint32_t NumBytes);


#ifdef __cplusplus
}
#endif

#endif
