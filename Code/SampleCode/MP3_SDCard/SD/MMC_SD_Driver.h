#ifndef _MMC_SD_DRIVER_H_
#define _MMC_SD_DRIVER_H_
/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "stdint.h"	 
//////////////////////////////////////////////////////////////////////////////////	 
//********************************************************************************
//////////////////////////////////////////////////////////////////////////////////  
// SD TYPE 
#define SD_TYPE_ERR     0X00
#define SD_TYPE_MMC     0X01
#define SD_TYPE_V1      0X02
#define SD_TYPE_V2      0X04
#define SD_TYPE_V2HC    0X06	   
 	   
#define CMD0    0       //Reset
#define CMD1    1
#define CMD8    8       //CMD8 £¬SEND_IF_COND
#define CMD9    9       //CMD9 £¬Read CSD 
#define CMD10   10      //CMD10£¬Read CID 
#define CMD12   12      //CMD12£¬Stop data transmission
#define CMD16   16      //CMD16£¬Set sectorSize to return 0x00
#define CMD17   17      //CMD17£¬Read sector
#define CMD18   18      //CMD18£¬Read Multi sector
#define CMD23   23      //CMD23£¬Erases N blocks before writing N sector
#define CMD24   24      //CMD24£¬Write sector
#define CMD25   25      //CMD25£¬Write Multi sector
#define CMD41   41      //CMD41£¬It should return 0x00
#define CMD55   55      //CMD55£¬It should return 0x01
#define CMD58   58      //CMD58£¬Read OCR
#define CMD59   59      //CMD59£¬Enable/Disable CRC£¬It should return 0x00


#define MSD_DATA_OK                0x05
#define MSD_DATA_CRC_ERROR         0x0B
#define MSD_DATA_WRITE_ERROR       0x0D
#define MSD_DATA_OTHER_ERROR       0xFF

#define MSD_RESPONSE_NO_ERROR      0x00
#define MSD_IN_IDLE_STATE          0x01
#define MSD_ERASE_RESET            0x02
#define MSD_ILLEGAL_COMMAND        0x04
#define MSD_COM_CRC_ERROR          0x08
#define MSD_ERASE_SEQUENCE_ERROR   0x10
#define MSD_ADDRESS_ERROR          0x20
#define MSD_PARAMETER_ERROR        0x40
#define MSD_RESPONSE_FAILURE       0xFF

uint8_t SD_Initialize(void);
uint8_t SD_WaitReady(void);
void SD_DisSelect(void);
uint8_t SD_Select(void);
uint8_t SD_GetResponse(uint8_t Response);
uint8_t SD_RecvData(uint8_t*buf,uint16_t len);
uint8_t SD_SendBlock(uint8_t*buf,uint8_t cmd);
uint8_t SD_SendCmd(uint8_t cmd, uint32_t arg, uint8_t crc);
uint8_t SD_GetCID(uint8_t *cid_data);
uint8_t SD_GetCSD(uint8_t *csd_data);
uint8_t SD_GetSectorCount(void);
uint8_t SD_ReadDisk(uint8_t*buf,uint32_t sector,uint8_t cnt);
uint8_t SD_WriteDisk(uint8_t*buf,uint32_t sector,uint8_t cnt);

#endif




