/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   LEDStripe_SPIDriver_GPCM1Fx.h
 * @Version:
 *   V0.0.9
 * @Date:
 *   June 17, 2021
 * @Abstract:
 *
 **************************************************************************************************/
#ifndef _LEDSTRIPE_SPI_DRIVER_GPCM1FXH_H_
#define _LEDSTRIPE_SPI_DRIVER_GPCM1FXH_H_

/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include <stdint.h>


/*---------------------------------------------------------------------------------------
 * User Definition Area
 *---------------------------------------------------------------------------------------*/
#define    C_SPI0_LED_Number        (1)
//#define    C_SPI1_LED_Number        (3)
#define    C_SPI1_LED_Number        (7)


/*---------------------------------------------------------------------------------------
 * Definition Area
 *---------------------------------------------------------------------------------------*/

// For SK6812D LED stripe ．0・ = 1000b = 0x8, ．1・ = 1110b = 0xE
#define    C_SPI1_CLK_DIV           (0x23UL)
//#define    C_SPI1_CLK_DIV           (0x113UL)    // 0x113BL for SK6812D 2.25us

// For SK6812D LED stripe ．0・ = 1000b = 0x8, ．1・ = 1110b = 0xE
#define    C_LED_LOGICAL_ONE_HN     (0xE0)
#define    C_LED_LOGICAL_ZERO_HN    (0x80)
#define    C_LED_LOGICAL_ONE_LN     (0x0E)
#define    C_LED_LOGICAL_ZERO_LN    (0x08)

#define    C_SPI0_BUFFER_SIZE       ((C_SPI0_LED_Number * 24) / 2)
#define    C_SPI1_BUFFER_SIZE       ((C_SPI1_LED_Number * 24) / 2)

#define    SPI0_PLAY_FLAG           (0x01)
#define    SPI0_UPDATE_FLAG         (0x02)
#define    SPI1_PLAY_FLAG           (0x04)
#define    SPI1_UPDATE_FLAG         (0x08)


/*---------------------------------------------------------------------------------------
 * Structure Definition Area
 *---------------------------------------------------------------------------------------*/
typedef struct _LEDSTRIPE_SPI_WORKING_RAM
{
  uint8_t R_LED_PlayFlag;
  uint8_t R_512Hz_Count;
  uint8_t R_SPI0_ColorEffect;
  uint8_t R_SPI0_ColorDataG;
  uint8_t R_SPI0_ColorDataR;
  uint8_t R_SPI0_ColorDataB;
  uint8_t R_SPI0_DataBuffer[C_SPI0_BUFFER_SIZE];
  uint8_t R_SPI1_ColorEffect;
  uint8_t R_SPI1_ColorDataG;
  uint8_t R_SPI1_ColorDataR;
  uint8_t R_SPI1_ColorDataB;
  uint8_t R_SPI1_DataBuffer[C_SPI1_BUFFER_SIZE];
} LEDSTRIPE_SPI_WORKING_RAM;


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C"{
#endif

void LEDStripe_SPI_Initial(void);
void LEDStripe_SPI0_ClearAll(void);
void LEDStripe_SPI1_ClearAll(void);
void LEDStripe_SPI0_DmaIsr(void);
void LEDStripe_SPI1_DmaIsr(void);
void LEDStripe_SPI_FrameServiceIsr(void);
void LEDStripe_ReloadBuf(uint8_t* pRGBData, uint8_t* pDstBuf);
void LEDStripe_SPI_MainLoopService(void);
void LEDStripe_SPI0_Start(void);
void LEDStripe_SPI0_Stop(void);
void LEDStripe_SPI1_Start(void);
void LEDStripe_SPI1_Stop(void);

#ifdef __cplusplus
}
#endif

#endif
