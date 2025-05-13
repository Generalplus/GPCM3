#ifndef _MMC_SD_USER_H
#define _MMC_SD_USER_H
/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "stdint.h"

void SD_SPI_Init(void);
uint8_t SD_SPI_ReadWriteByte(uint8_t data);
void SD_SPI_SpeedLow(void);
void SD_SPI_SpeedHigh(void);
void SD_CSPin_High(void);
void SD_CSPin_Low(void);
void SPI_DMARead(uint32_t DstAddr, uint16_t TransCount);
void SPI_DMAWrite(uint32_t SrcAddr, uint16_t TransCount);
#endif
