/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   DMA_GPCM3_FM1.h
 * @Version:
 *   V0.9.3.1
 * @Date:
 *   April 28, 2022
 * @Abstract:
 *  2022.04.28 Add the define of DMA_REQSEL_CCP
 *
 **************************************************************************************************/
#ifndef _DMA_GPCM3_FM1_H_
#define _DMA_GPCM3_FM1_H_


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "GPCM3_FM1.h"


/*---------------------------------------------------------------------------------------
 * Definition Area
 *---------------------------------------------------------------------------------------*/

/*
 * Definition for the 2nd parameter of DMA_Init()
 */
#define DMA_REQSEL_ADC               DMA_CTRL_DMA_REQSEL_SARADC
#define DMA_REQSEL_CCP               DMA_CTRL_DMA_REQSEL_CCP
#define DMA_REQSEL_DAC_CH0           DMA_CTRL_DMA_REQSEL_DAC_CH0
#define DMA_REQSEL_DAC_CH1           DMA_CTRL_DMA_REQSEL_DAC_CH1
#define DMA_REQSEL_SPI0              DMA_CTRL_DMA_REQSEL_SPI0
#define DMA_REQSEL_UART              DMA_CTRL_DMA_REQSEL_UART
#define DMA_REQSEL_I2S               DMA_CTRL_DMA_REQSEL_I2S
#define DMA_REQSEL_I2C               DMA_CTRL_DMA_REQSEL_I2C
#define DMA_REQSEL_DSADC             DMA_CTRL_DMA_REQSEL_DSADC
#define DMA_REQSEL_SPI1              DMA_CTRL_DMA_REQSEL_SPI1
#define DMA_REQSEL_MEM               DMA_CTRL_DMA_REQSEL_MEM

/*
 * Definition for the 3rd parameter of DMA_Init()
 */
#define DMA_SRC_DATA_8B              DMA_CTRL_DMA_SRCSIZE_8B
#define DMA_SRC_DATA_16B             DMA_CTRL_DMA_SRCSIZE_16B
#define DMA_SRC_DATA_32B             DMA_CTRL_DMA_SRCSIZE_32B

/*
 * Definition for the 4th parameter of DMA_Init()
 */
#define DMA_SRC_ADDR_INC             DMA_CTRL_DMA_SRCINC_ENABLE
#define DMA_SRC_ADDR_FIX             DMA_CTRL_DMA_SRCINC_DISABLE

/*
 * Definition for the 5th parameter of DMA_Init()
 */
#define DMA_DST_DATA_8B              DMA_CTRL_DMA_DSTSIZE_8B
#define DMA_DST_DATA_16B             DMA_CTRL_DMA_DSTSIZE_16B
#define DMA_DST_DATA_32B             DMA_CTRL_DMA_DSTSIZE_32B

/*
 * Definition for the 6th parameter of DMA_Init()
 */
#define DMA_DST_ADDR_INC             DMA_CTRL_DMA_DSTINC_ENABLE
#define DMA_DST_ADDR_FIX             DMA_CTRL_DMA_DSTINC_DISABLE


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C"{
#endif

void DMA_Init(DMA_TYPE_DEF *DmaHandle, uint32_t DmaChannel, uint32_t SrcDataWidth, uint32_t SrcAddrAttrib, uint32_t DstDataWidth, uint32_t DstAddrAttrib);
void DMA_Close(DMA_TYPE_DEF *DmaHandle);
void DMA_EnableInt(DMA_TYPE_DEF *DmaHandle);
void DMA_DisableInt(DMA_TYPE_DEF *DmaHandle);
void DMA_Trigger(DMA_TYPE_DEF *DmaHandle, uint32_t SrcAddr, uint32_t DstAddr, uint32_t TransCount);
uint32_t DMA_CheckBusy(DMA_TYPE_DEF *DmaHandle);
void DMA_InstallIsrService(DMA_TYPE_DEF *DmaHandle, void(*DmaIsrServiceFunc)(void));
void DMA_UnInstallIsrService(DMA_TYPE_DEF *DmaHandle);


#ifdef __cplusplus
}
#endif

#endif
