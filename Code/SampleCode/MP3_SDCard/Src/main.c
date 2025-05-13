/************************************************************************************
 *  File:     main.c
 *  Purpose:  GPCM3/GPFM1 MP3 + FAT SD card Sample Code
 *            Replace with your code.
 *  Date:     09 May 2023
 *  Info:
 *
 *
 ************************************************************************************/

/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include <stdio.h>
#include "glibc_wrapper.h"
#include "GPCM3_FM1.h"
#include "MP3_User.h"
#include "KeyScan.h"
#include "MMC_SD_Driver.h"
#include "MMC_SD_User.h"
#include "ff.h"
#include "sys.h"
void SD_PlayMP3(uint8_t SDReadInx);
void GetMP3Data(uint8_t *DstBufAddr, uint8_t *SrcDataAddr, uint16_t DataLen);
uint8_t SD_MP3Num(char *filname,uint8_t SDReadInx);
uint8_t SaveFileData(char *TXT_FileName,uint16_t flen);
/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#if defined(__CC_ARM)
#define Aligned4B __align(4)
__align(4) MP3_LIB_WORKING_RAM Mp3LibWorkingRam;
__align(4) KEYSCAN_WORKING_RAM KeyScanWorkingRam;
#elif defined(__GNUC__)
extern unsigned int __boot_table_start; //in GPCM2_CM3.ld
#define Aligned4B __attribute__ ((aligned (4)))
__attribute__ ((aligned (4))) MP3_LIB_WORKING_RAM Mp3LibWorkingRam;
__attribute__ ((aligned (4))) KEYSCAN_WORKING_RAM KeyScanWorkingRam;
#endif

uint8_t Mp3Num;
int32_t ScanedKey;
int16_t AudPwmGain = 31;
int16_t Mp3PlayIdx = 1;
int8_t PlayCon = 0;

extern uint8_t __user_spifc_load_addr;
extern uint8_t __user_spifc_start;
extern uint32_t __user_spifc_size;


/*---------------------------------------------------------------------------------------
 * Subroutine Area
 *---------------------------------------------------------------------------------------*/
  uint32_t SWTrim_CheckBusy(void)
  {
    return (CLOCK->SWTRIM & CLOCK_SWTRIM_COUNTER_RDY);
  }

/*---------------------------------------------------------------------------------------
 * Main Function
 *---------------------------------------------------------------------------------------*/

 uint32_t sd_size;
FATFS fatfs;
uint8_t res;
uint8_t PlaySD;
uint8_t PlayIndex;
FIL file;
uint32_t Fi_size;
//uint16_t SDDataInx;
UINT bww;
uint8_t SpeechNum;
int main()
{
	WRITE_REG(RCU->STS,0x0000FFFF);                             // Clear Reset status

	// GPIO initialize
  GPIO_SetMode(GPIOA, 0xFFFFFFFF, GPIO_MODE_INPUT);         //IOA[31:0] Input pull-low, bypass IOA[14:13] UART & IOA.8 Output
  MODIFY_REG(GPIOA->OBUF, 0xFFFFFFFF, 0);
  GPIO_SetMode(GPIOB, 0xFFFFFFFF, GPIO_MODE_INPUT);         //IOB Input pull-low
  MODIFY_REG(GPIOB->OBUF, 0xFFFFFFFF, 0);
  GPIO_SetMode(GPIOC, 0xFFFFFFFF, GPIO_MODE_INPUT);         //IOC Input pull-low
  MODIFY_REG(GPIOC->OBUF, 0xFFFFFFFF, 0);
  GPIO_SetMode(GPIOD, 0xFFFFFFFF, GPIO_MODE_INPUT);         //IOD Input pull-low
  MODIFY_REG(GPIOD->OBUF, 0xFFFFFFFF, 0);

	KeyScan_Initial(&KeyScanWorkingRam);                     // key scan init.

  Mp3Num = GetMp3Num();
	Mp3Num -= 1;	                                           // The last MP3 is a pseudo file. The (n)th MP3 address - the (n-1)th MP3 address is the data length of the (n-1)th MP3.
	MP3_Initial(&Mp3LibWorkingRam);
   // MP3_Play(GetMp3StartAddr(Mp3PlayIdx), GetMp3Length(Mp3PlayIdx), MP3_DAC_CH0 , MP3_AUTO_RAMP_UP + MP3_AUTO_RAMP_DOWN);
	while(SD_Initialize()!=0)//检测不到SD卡
	{
		WDT_Clear();
	}
		sd_size=SD_GetSectorCount();//得到扇区数
	res = f_mount(&fatfs,"0:",1);
     SaveFileData("0:/SYS_DATA.bin",512);
	//获取音频数
		for(SpeechNum = 1;;SpeechNum++)
		{
			if(SD_MP3Num("0:/MP3/MUSIC000.mp3",SpeechNum))break;//打开失败就跳出，打开成功就继续循环。
		}
     SpeechNum=SpeechNum-1;

	PlayIndex = 1;
	SD_PlayMP3(PlayIndex);

  while(1)
	{
		WDT_Clear();
		MP3_ServiceLoop();
		KeyScan_ServiceLoop();

		if(MP3_CheckCpuOverload() != 0)
		{
			MP3_ClearCpuOverload();
		}

		ScanedKey = KeyScan_GetCh();
		switch (ScanedKey)
		{
			case 0x01:                                           // IOA0+VDD: Stop
				PlayCon = 0;
				MP3_Stop();
				break;

			case 0x02:                                           // IOA1+VDD: replay
        PlayIndex = ++PlayIndex > SpeechNum ? 1 : PlayIndex;
        SD_PlayMP3(PlayIndex);
				break;

			case 0x04:                                           // IOA2+VDD: Play next song
				PlaySD = 0;
				PlayCon = 0;
				MP3_Stop();
			    Mp3PlayIdx = ++Mp3PlayIdx > Mp3Num ? 1 : Mp3PlayIdx;
				MP3_Play(GetMp3StartAddr(Mp3PlayIdx), GetMp3Length(Mp3PlayIdx), MP3_DAC_CH1, MP3_AUTO_RAMP_UP + MP3_AUTO_RAMP_DOWN);
			   DAC_SetAudPwmGain(AudPwmGain);
				break;

			case 0x08:                                           // IOA3+VDD: Play concatenate
			PlaySD = 0;
				PlayCon = 1;
			  MP3_Stop();
			  Mp3PlayIdx = 1;
			  MP3_Play(GetMp3StartAddr(Mp3PlayIdx), GetMp3Length(Mp3PlayIdx), MP3_DAC_CH0, MP3_AUTO_RAMP_UP + MP3_AUTO_RAMP_DOWN);
			  DAC_SetAudPwmGain(AudPwmGain);
			  break;

			case 0x10:                                           // IOA4+VDD:	Pause/Resume
				if((MP3_GetStatus() & MP3_PAUSE_FLAG) == 0)
				{
				  MP3_Pause();
				}
				else
				{
				  MP3_Resume();
				}
				break;

			case 0x40:
				AudPwmGain++;
			  if(AudPwmGain >= 63)
				{
				  AudPwmGain = 62;
				}
			  DAC_SetAudPwmGain(AudPwmGain);
				break;

			case 0x80:
				AudPwmGain--;
			  if(AudPwmGain <= 1)
				{
				  AudPwmGain = 1;
				}
			  DAC_SetAudPwmGain(AudPwmGain);
				break;

			default:
				break;
		}

		if((PlayCon != 0) && (MP3_Check_Con() == 0))
		{
      Mp3PlayIdx = ++Mp3PlayIdx > Mp3Num ? 1 : Mp3PlayIdx;
			MP3_Play_Con(GetMp3StartAddr(Mp3PlayIdx), GetMp3Length(Mp3PlayIdx), MP3_DAC_CH0, MP3_AUTO_RAMP_UP + MP3_AUTO_RAMP_DOWN);
		}
	}
  return 0;
}


void MoveSpifcRamCode(void)
{
    uint8_t *pCode_LDA = &__user_spifc_load_addr;
    uint8_t *pCode_VMA = &__user_spifc_start;
    uint32_t l_CodeSize = (uint32_t)&__user_spifc_size;
    uint32_t iCount;

    for(iCount=0;iCount<l_CodeSize;iCount++)
    {
      pCode_VMA[iCount] = pCode_LDA[iCount];
    }

    return;
}
void SD_PlayMP3(uint8_t SDReadInx)
{
		char SDFile[20] = "0:/MP3/MUSIC000.MP3";
	//		SDFile[15] = 0x30+SDReadInx/100;
//		SDFile[16] = 0x30+SDReadInx/10;
//		SDFile[17] = 0x30+SDReadInx%10;
		SDFile[13] = 0x30+SDReadInx/10;
		SDFile[14] = 0x30+SDReadInx%10;
		PlaySD = 1;
	res = f_open(&file,SDFile,FA_READ);
	MP3_Stop();
	while(MP3_GetStatus())
	{
		WDT_Clear();
		MP3_ServiceLoop();
	}
	if(res == 0)
	{
		Fi_size=f_size(&file);
		MP3_Play(GetMp3StartAddr(1), GetMp3Length(1), MP3_DAC_CH0 , MP3_AUTO_RAMP_UP + MP3_AUTO_RAMP_DOWN);
//		SDDataInx = 0x0000;
	}
}

void GetMP3Data(uint8_t *DstBufAddr, uint8_t *SrcDataAddr, uint16_t DataLen)
{
	uint32_t	R_Addr;
	R_Addr =(uint32_t) SrcDataAddr;
	if(R_Addr!=file.fptr) f_lseek(&file,R_Addr);							//偏移到文件头.
	f_read (&file, DstBufAddr,DataLen,&bww);

}

uint8_t SD_MP3Num(char *filname,uint8_t SDReadInx)
{
	      int i;
		char SDFile[20];
		for(i=0;i<19;i++)
			{
				SDFile[i] = filname[i];
			}
		SDFile[19] = ' ';		//文件名结束
//		SDFile[15] = 0x30+SDReadInx/100;
		SDFile[13] = 0x30+SDReadInx/10;
		SDFile[14] = 0x30+SDReadInx%10;
		res = f_open(&file,SDFile,FA_READ);
			return res;
}

uint8_t SaveFileData(char *TXT_FileName,uint16_t flen)
{
u8 buf[512];
int i;
    for(i=0;i<512;i++)
    {
        buf[i] = i;
    }
	res = f_open(&file,TXT_FileName,FA_CREATE_ALWAYS|FA_WRITE);
	if(res == 0)
	{
		res = f_write(&file,buf,flen,&bww);
		f_close(&file);
	}
	return res;
}
