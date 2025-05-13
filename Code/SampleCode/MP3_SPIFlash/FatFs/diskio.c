/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2019        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/

#include "ff.h"			/* Obtains integer types */
#include "diskio.h"		/* Declarations of disk functions */
// #include "mmc_sd_Driver.h"
/* Definitions of physical drive number for each drive */
//#define DEV_RAM		0	/* Example: Map Ramdisk to physical drive 0 */
//#define DEV_MMC		1	/* Example: Map MMC/SD card to physical drive 1 */
//#define DEV_USB		2	/* Example: Map USB MSD to physical drive 2 */

#define SD_CARD	 0  //
/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
	BYTE pdrv		/* Physical drive nmuber to identify the drive */
)
{
//	DSTATUS stat;
//	int result;

//	switch (pdrv) {
//	case DEV_RAM :
//		result = RAM_disk_status();

//		// translate the reslut code here

//		return stat;

//	case DEV_MMC :
//		result = MMC_disk_status();

//		// translate the reslut code here

//		return stat;

//	case DEV_USB :
//		result = USB_disk_status();

//		// translate the reslut code here

//		return stat;
//	}
//	return STA_NOINIT;
	return RES_OK;
}



/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
	BYTE pdrv				/* Physical drive nmuber to identify the drive */
)
{
//	DSTATUS stat;
//	int result;
	BYTE res=0;
	switch (pdrv) {
//	case DEV_RAM :
//		result = RAM_disk_initialize();

//		// translate the reslut code here

//		return stat;

//	case DEV_MMC :
//		result = MMC_disk_initialize();

//		// translate the reslut code here

//		return stat;

//	case DEV_USB :
//		result = USB_disk_initialize();

//		// translate the reslut code here
		case SD_CARD:
			res=SD_Initialize();
  			break;
		default:
			res=1;
	}
if(res)return  STA_NOINIT;
	else return 0;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber to identify the drive */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Start sector in LBA */
	UINT count		/* Number of sectors to read */
)
{
	BYTE res=0;
    if (!count)return RES_PARERR;
	switch(pdrv)
	{
		case SD_CARD:
		/*
			res=SD_ReadDisk(buff,sector,count);
			while(res)
			{
				SD_Initialize();
				res=SD_ReadDisk(buff,sector,count);
			}
		*/
			break;

		default:
			res=1;
		}
    if(res==0x00)return RES_OK;
    else return RES_ERROR;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if FF_FS_READONLY == 0

DRESULT disk_write (
	BYTE pdrv,			/* Physical drive nmuber to identify the drive */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Start sector in LBA */
	UINT count			/* Number of sectors to write */
)
{
	BYTE res=0;
    if (!count)return RES_PARERR;
	switch(pdrv)
	{
		case SD_CARD:
		/*
			res=SD_WriteDisk((BYTE*)buff,sector,count);
			while(res)
			{
				SD_Initialize();
				res=SD_WriteDisk((BYTE*)buff,sector,count);
			}
		*/
			break;
		default:
			res=1;
	}
    if(res == 0x00)return RES_OK;
    else return RES_ERROR;
}

#endif


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
DRESULT res;
	if(pdrv==SD_CARD)//SD¿¨
	{
	    switch(cmd)
	    {
		    case CTRL_SYNC:
				res = RES_OK;
		        break;
		    case GET_SECTOR_SIZE:
				*(DWORD*)buff = 512;
		        res = RES_OK;
		        break;
		    case GET_BLOCK_SIZE:
//				*(WORD*)buff = SDCardInfo.CardBlockSize;
		        res = RES_OK;
		        break;
		    case GET_SECTOR_COUNT:
//		        *(DWORD*)buff = SDCardInfo.CardCapacity/512;
		        res = RES_OK;
		        break;
		    default:
		        res = RES_PARERR;
		        break;
	    }
	}else res=RES_ERROR;
    return res;
}
//User defined function to give a current time to fatfs module      */
//31-25: Year(0-127 org.1980), 24-21: Month(1-12), 20-16: Day(1-31) */
//15-11: Hour(0-23), 10-5: Minute(0-59), 4-0: Second(0-29 *2) */
DWORD get_fattime (void)
{
	return 0;
}

