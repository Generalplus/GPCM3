/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   SPIFC_GPCM3_FM1.h
 * @Version:
 *   V1.0.2
 * @Date:
 *   Nov. 3, 2020
 * @Abstract:
 *
 **************************************************************************************************/
#ifndef _SPIFC_GPCM3_FM1_H_
#define _SPIFC_GPCM3_FM1_H_


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C"{
#endif


void SPIFC_SetClkDiv(uint32_t SpiClkDiv);
void SPIFC_EX4B(void);
void SPIFC_EN4B(void);
uint32_t SPIFC_ReadID(void);
void SPIFC_WriteEnable(void);
void SPIFC_WriteDisable(void);
uint16_t SPIFC_ReadSR(void);
void SPIFC_WriteSR(uint8_t SpiStatusRegister);
uint8_t SPIFC_ReadCR(void);
void SPIFC_SectorErase(uint32_t SpiAddr);
void SPIFC_BlockErase(uint32_t SpiAddr);
void SPIFC_ReadNBytes(uint32_t ReadAddr, uint8_t *pDstBuf, uint32_t NumBytes);
void SPIFC_WriteNBytes(uint32_t SpiAddr, uint8_t *pSrcBuf, uint32_t NumBytes);
void SPIFC_WriteNWords(uint32_t SpiAddr, uint8_t *pSrcBuf, uint32_t NumBytes);
void SPIFC_Cmd(uint8_t SpiCmd);
void SPIFC_CmdAddr(uint8_t SpiCmd, uint32_t SpiAddr);
void SPIFC_CmdRx(uint8_t SpiCmd, uint8_t *pDstBuf, uint32_t NumBytes);
void SPIFC_CmdAddrRx(uint8_t SpiCmd, uint32_t SpiAddr, uint8_t *pDstBuf, uint32_t NumBytes);
void SPIFC_CmdTx(uint8_t SpiCmd, uint8_t *pSrcBuf, uint32_t NumBytes);
void SPIFC_CmdAddrTx(uint8_t SpiCmd, uint32_t SpiAddr, uint8_t *pSrcBuf, uint32_t NumBytes);


#ifdef __cplusplus
}
#endif

#endif
