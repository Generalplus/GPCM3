/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   MMC_SD_User.c
 * @Version:
 *   V1.0.1
 * @Date:
 *   March 9, 2021
 * @Abstract:
 *
 **************************************************************************************************/

 /*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "MMC_SD_Driver.h"
#include "MMC_SD_USER.h"
/*---------------------------------------------------------------------------------------
 * Gobal Variable Declaration Area
 *---------------------------------------------------------------------------------------*/
uint8_t  SD_Type=0;
uint8_t  SD_SPIMode=0;
/*---------------------------------------------------------------------------------------
 * User Subroutine Area
 *---------------------------------------------------------------------------------------*/
/**
 * @brief
 *  SD Initialize.
 * @param
 *  None.
 * @return
 *  0:success.
 *	other:error.
 */
uint8_t SD_Initialize(void)
{
	uint8_t r1;
	uint16_t retry;
	uint8_t buf[4];
	uint16_t i;

	SD_SPI_Init();
 	SD_SPI_SpeedLow();
	for(i=0;i<10;i++)SD_SPI_ReadWriteByte(0XFF);//send 74 pulses
	retry=20;
	do
	{
		r1=SD_SendCmd(CMD0,0,0x95);      // IDLE Model
	}while((r1!=0X01) && retry--);
 	SD_Type=0;//no Card
	if(r1==0X01)
	{
		if(SD_SendCmd(CMD8,0x1AA,0x87)==1)// SD V2.0
		{
			for(i=0;i<4;i++)buf[i]=SD_SPI_ReadWriteByte(0XFF);	//Get trailing return value of R7 resp
			if(buf[2]==0X01&&buf[3]==0XAA)  // The card is supported 2.7~3.6V
			{
				retry=0XFFFE;
				do
				{
					SD_SendCmd(CMD55,0,0X01);	 													// Send CMD55
					r1=SD_SendCmd(CMD41,0x40000000,0X01); 							// Send CMD41
				}while(r1&&retry--);
				if(retry&&SD_SendCmd(CMD58,0,0X01)==0)								// Check SD2.0
				{
					for(i=0;i<4;i++)buf[i]=SD_SPI_ReadWriteByte(0XFF);  // Get OCR
					if(buf[0]&0x40)SD_Type=SD_TYPE_V2HC;   							// Check CCS
					else SD_Type=SD_TYPE_V2;
				}
			}
		}else//SD V1.x/ MMC	V3
		{
			SD_SendCmd(CMD55,0,0X01);					//Send CMD55
			r1=SD_SendCmd(CMD41,0,0X01);			//Send CMD41
			if(r1<=1)
			{
				SD_Type=SD_TYPE_V1;
				retry=0XFFFE;
				do 															// Waiting to exit idle
				{
					SD_SendCmd(CMD55,0,0X01);			// Send CMD55
					r1=SD_SendCmd(CMD41,0,0X01);	// Send CMD41
				}while(r1&&retry--);
			}else
			{
				SD_Type=SD_TYPE_MMC;						// MMC V3
				retry=0XFFFE;
				do 															// Waiting to exit idle
				{
					r1=SD_SendCmd(CMD1,0,0X01);		// Send CMD1
				}while(r1&&retry--);
			}
			if(retry==0||SD_SendCmd(CMD16,512,0X01)!=0)SD_Type=SD_TYPE_ERR;	// Err Card
		}
	}
	SD_DisSelect();
	SD_SPI_SpeedHigh();
	if(SD_Type)return 0;
	else if(r1)return r1;
	return 0xaa;
}
void SetSD_SPIMode(uint8_t data)
{
		SD_SPIMode = data;
}
/**
 * @brief
 *  SD WaitReady.
 * @param
 *  None.
 * @return
*   0:success.
		other:error.
 */
uint8_t SD_WaitReady(void)
{
	uint32_t t=0;
	do
	{
		if(SD_SPI_ReadWriteByte(0XFF)==0XFF)return 0;	// OK
		t++;
	}while(t<0XFFFFFF);
	return 1;
}
/**
 * @brief
 *  SD DisSelect
 * @param
 *  None.
 * @return
 *  None
 */
void SD_DisSelect(void)
{
	SD_CSPin_High();
	SD_SPI_ReadWriteByte(0xff);		 // Provides an additional 8 clocks
}
/**
 * @brief
 *  SD Select
 * @param
 *  None.
 * @return
*   0:success
		1:error
 */
uint8_t SD_Select(void)
{
	SD_CSPin_Low();
	if(SD_WaitReady()==0)return 0;	// Wait Pass
	SD_DisSelect();
	return 1;												// Wait Error
}
/**
 * @brief
 *   SD Get Response
 * @param
*   Response [in]: The response value to get
 * @return
*   0:success
		other:error
 */
uint8_t SD_GetResponse(uint8_t Response)
{
	uint16_t Count=0xFFF;		// Wait Number

	while ((SD_SPI_ReadWriteByte(0XFF)!=Response)&&Count)Count--;
	if (Count==0)return MSD_RESPONSE_FAILURE;
	else return MSD_RESPONSE_NO_ERROR;
}

/**
 * @brief
 *  read a block(512 bytes) data from SD card.
 * @param
 *  buf[out]: Destination buffer address pointer.
 *	len[in]: length of data to read.
 * @return
 *	0: Success.
 *  other: error.
 */
uint8_t SD_RecvData(uint8_t*buf,uint16_t len)
{
	if(SD_GetResponse(0xFE))return 1;//Wait for the SD card to send back the 0xFE
		if(SD_SPIMode)
		{
			    SPI_DMARead((uint32_t)buf,len);
		}
		else
		{
                while(len--)//Start receiving data
                {
                    *buf=SD_SPI_ReadWriteByte(0xFF);
                    buf++;
                }
         }
    //Here is two CRC£¨dummy CRC£©
    SD_SPI_ReadWriteByte(0xFF);
    SD_SPI_ReadWriteByte(0xFF);
    return 0;
}

/**
 * @brief
 *  write a block(512 bytes) data to SD card.
 * @param
 *  buf[in]: Source buffer address pointer.
 *	cmd[in]: command.
 * @return
 *	0: Success.
 *  other: error.
 */
uint8_t SD_SendBlock(uint8_t*buf,uint8_t cmd)
{
	uint16_t t;

	if(SD_WaitReady())return 1;
	SD_SPI_ReadWriteByte(cmd);
	if(cmd!=0XFD)
	{
			if(SD_SPIMode)		//DMA Mode
		{
			SPI_DMAWrite((uint32_t)buf,512);
		}
		else
		{
		   for(t=0;t<512;t++)SD_SPI_ReadWriteByte(buf[t]);
		}
	  SD_SPI_ReadWriteByte(0xFF);
	  SD_SPI_ReadWriteByte(0xFF);
		t=SD_SPI_ReadWriteByte(0xFF);
		if((t&0x1F)!=0x05)return 2;
	}
    return 0;
}

/**
 * @brief
 *  Send a command to the SD card.
 * @param
 *  cmd[in]: The address pointer to hold CID(at least 16 bytes).
 *	arg[in]: command parameter.
 *	crc[in]: crc check value.
 * @return
 *	r1: SD card response.
 */
uint8_t SD_SendCmd(uint8_t cmd, uint32_t arg, uint8_t crc)
{
  uint8_t r1,Retry =0;

	SD_DisSelect();
	if(SD_Select())return 0XFF;
	// Send
    SD_SPI_ReadWriteByte(cmd | 0x40);
    SD_SPI_ReadWriteByte(arg >> 24);
    SD_SPI_ReadWriteByte(arg >> 16);
    SD_SPI_ReadWriteByte(arg >> 8);
    SD_SPI_ReadWriteByte(arg);
    SD_SPI_ReadWriteByte(crc);
	if(cmd==CMD12)SD_SPI_ReadWriteByte(0xff); //Skip a stuff byte when stop reading

    // Wait for a response, or time out to exit
	Retry=0X1F;
	do
	{
		r1=SD_SPI_ReadWriteByte(0xFF);
	}while((r1&0X80) && Retry--);
    return r1;
}

/**
 * @brief
 *  Get the CID information of the SD card(Include manufacturer information).
 * @param
*   cid_data[out]: The address pointer to hold CID(at least 16 bytes).
 * @return
 *	0: success.
 *	1: error.
 */
uint8_t SD_GetCID(uint8_t *cid_data)
{
    uint8_t r1;

    r1=SD_SendCmd(CMD10,0,0x01); // Send CMD10 for Read CID
    if(r1==0x00)
	{
		r1=SD_RecvData(cid_data,16); // Receive 16 bytes of data
    }
	SD_DisSelect();
	if(r1)return 1;
	else return 0;
}

/**
 * @brief
 *  Get the CSD information of the SD card(Includes capacity and speed information).
 * @param
*   csd_data[out]: The address pointer to hold CID(at least 16 bytes).
 * @return
 *	0: success.
 *	1: error.
 */
uint8_t SD_GetCSD(uint8_t *csd_data)
{
    uint8_t r1;
    r1=SD_SendCmd(CMD9,0,0x01);			// Send CMD9
    if(r1==0)
	{
    	r1=SD_RecvData(csd_data, 16);
    }
	SD_DisSelect();
	if(r1)return 1;
	else return 0;
}

/**
 * @brief
 *  Gets the total number of sectors of the SD card.
 * @param
 *  None.
 * @return
 *	Capacity: the number of sectors.
 */
uint8_t SD_GetSectorCount(void)
{
    uint8_t csd[16];
    uint32_t Capacity;
    uint8_t n;
	uint16_t csize;

    if(SD_GetCSD(csd)!=0) return 0;
    // If SDHC card, calculate as follows
    if((csd[0]&0xC0)==0x40)	 					       // V2.00 Card
    {
		csize = csd[9] + ((uint16_t)csd[8] << 8) + 1;
		Capacity = (uint32_t)csize << 10;        // get the number of sectors
    }else//V1.XX Card
    {
		n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
		csize = (csd[8] >> 6) + ((uint16_t)csd[7] << 2) + ((uint16_t)(csd[6] & 3) << 10) + 1;
		Capacity= (uint32_t)csize << (n - 9);   // get the number of sectors
    }
    return Capacity;
}

/**
 * @brief
 *  Read disk.
 * @param
 *  buf [out]: Destination buffer address pointer.
 *	sector [in] : start address.
 *	cnt [in] : Number of sectors to read.
 * @return
 *	r1:0:success,other:error.
 */
uint8_t SD_ReadDisk(uint8_t*buf,uint32_t sector,uint8_t cnt)
{
	uint8_t r1;
	if(SD_Type!=SD_TYPE_V2HC)sector <<= 9;
	if(cnt==1)
	{
		r1=SD_SendCmd(CMD17,sector,0X01);
		if(r1==0)
		{
			r1=SD_RecvData(buf,512);
		}
	}else
	{
		r1=SD_SendCmd(CMD18,sector,0X01);
		do
		{
			r1=SD_RecvData(buf,512);
			buf+=512;
		}while(--cnt && r1==0);
		SD_SendCmd(CMD12,0,0X01);
	}
	SD_DisSelect();
	return r1;
}

/**
 * @brief
 *  Write disk.
 * @param
*   buf [in]: Source buffer address pointer.
		sector [in] : start address.
		cnt [in] : Number of sectors to write.
 * @return
		r1
*   0:success.
		other:error.
 */
uint8_t SD_WriteDisk(uint8_t*buf,uint32_t sector,uint8_t cnt)
{
	uint8_t r1;
	if(SD_Type!=SD_TYPE_V2HC)sector *= 512;
	if(cnt==1)
	{
		r1=SD_SendCmd(CMD24,sector,0X01);
		if(r1==0)
		{
			r1=SD_SendBlock(buf,0xFE);
		}
	}else
	{
		if(SD_Type!=SD_TYPE_MMC)
		{
			SD_SendCmd(CMD55,0,0X01);
			SD_SendCmd(CMD23,cnt,0X01);
		}
 		r1=SD_SendCmd(CMD25,sector,0X01);
		if(r1==0)
		{
			do
			{
				r1=SD_SendBlock(buf,0xFC);
				buf+=512;
			}while(--cnt && r1==0);
			r1=SD_SendBlock(0,0xFD);
		}
	}
	SD_DisSelect();
	return r1;
}
