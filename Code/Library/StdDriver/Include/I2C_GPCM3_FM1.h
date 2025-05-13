/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   I2C_GPCM3_FM1.h
 * @Version:
 *   V0.9.1
 * @Date:
 *   July 30, 2021
 * @Abstract:
 *
 **************************************************************************************************/
#ifndef _I2C_GPCM3_FM1_H_
#define _I2C_GPCM3_FM1_H_


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "GPCM3_FM1.h"


/*---------------------------------------------------------------------------------------
 * Definition Area
 *---------------------------------------------------------------------------------------*/
/*
 * Definition for the 1st parameter of I2C_Master_Init()
 */
#define I2C_Master                     I2C_CTRL_MODE_SEL_MASTER
#define I2C_Slave                      I2C_CTRL_MODE_SEL_SLAVE

/*
 * Definition for the 2nd parameter of I2C_Master_Init()
 */
#define I2C_IOA0_1                     GPIOFUNC_CTRL0_I2C_IOA0_1
#define I2C_IOA15_16                   GPIOFUNC_CTRL0_I2C_IOA15_16
#define I2C_IOA20_21                   GPIOFUNC_CTRL0_I2C_IOA20_21


/*
 * Definition for the 1st parameter of I2C_SetClkDiv()
 */
#define I2C_CLK_SEL_HCLK_DIV_16         I2C_CTRL_CLK_SEL_HCLK_DIV_16
#define I2C_CLK_SEL_HCLK_DIV_32         I2C_CTRL_CLK_SEL_HCLK_DIV_32
#define I2C_CLK_SEL_HCLK_DIV_64         I2C_CTRL_CLK_SEL_HCLK_DIV_64
#define I2C_CLK_SEL_HCLK_DIV_128        I2C_CTRL_CLK_SEL_HCLK_DIV_128
#define I2C_CLK_SEL_HCLK_DIV_256        I2C_CTRL_CLK_SEL_HCLK_DIV_256
#define I2C_CLK_SEL_HCLK_DIV_768        I2C_CTRL_CLK_SEL_HCLK_DIV_768
#define I2C_CLK_SEL_HCLK_DIV_1024       I2C_CTRL_CLK_SEL_HCLK_DIV_1024

/*
 * Definition for the 2nd parameter of I2C_Master_StartAndAddr()
 */
#define I2C_WRITE                       I2C_RW_SEL_WRITE
#define I2C_READ                        I2C_RW_SEL_READ


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C"{
#endif

void I2C_Init(uint32_t I2CModeSel, uint32_t I2CSelIo);
void I2C_Close(void);
void I2C_SetClkDiv(uint32_t I2cClkDiv);
void I2C_SetSlaveAddr(uint8_t Addr);
uint32_t I2C_CheckBusy(void);
uint8_t I2C_Master_Start(uint8_t RW_bit);
void I2C_Master_Stop(void);
void I2C_Master_SendByte(uint8_t DataByte);
void I2C_Master_ReadByteTrigger(void);
uint8_t I2C_Master_ReadByteAck(void);
uint8_t I2C_Master_ReadByteNack(void);
uint8_t I2C_Slave_SendByte(uint8_t DataByte);
uint8_t I2C_Slave_ReadByte(void);

void I2C_Master_WriteNBytes(uint8_t *pSrcBuf, uint32_t DataLen);
void I2C_Master_ReadNBytes(uint8_t *pDstBuf, uint32_t DataLen);
void I2C_Master_WriteNBytes_DMA(DMA_TYPE_DEF *DmaHandler, uint8_t *pSrcBuf, uint32_t DataLen);
void I2C_Master_ReadNBytes_DMA(DMA_TYPE_DEF *DmaHandler, uint8_t *pDstBuf, uint32_t DataLen);

void I2C_Slave_WriteNBytes(uint8_t *pSrcBuf, uint32_t DataLen);
void I2C_Slave_ReadNBytes(uint8_t *pDstBuf, uint32_t DataLen);
void I2C_Slave_WriteNBytes_DMA(DMA_TYPE_DEF *mDMAHandler, uint8_t *pSrcBuf, uint32_t DataLen);
void I2C_Slave_ReadNBytes_DMA(DMA_TYPE_DEF *mDMAHandler, uint8_t *pSrcBuf, uint32_t DataLen);


#ifdef __cplusplus
}
#endif

#endif
