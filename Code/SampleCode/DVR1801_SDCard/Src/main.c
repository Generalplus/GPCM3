/************************************************************************************
 *  File:     main.c
 *  Purpose:  GPCM3/GPFM1 Sample Code
 *            Replace with your code.
 *  Date:     28 December 2022
 *  Info:
 *
 *
 ************************************************************************************/

/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include <stdio.h>
#include "GPCM3_FM1.h"
#include "SACM_DVR1801_User.h"
#include "APP_SwVolumeControl.h"
#include "KeyScan.h"

#include "MMC_SD_Driver.h"
#include "MMC_SD_User.h"
#include "sys.h"
#include "ff.h"

void ReadA18SDLoop(void);
void SD_PlayA18(char *,u8);
void SD_WriteA18(char *,u8);
void WriteA18SDEnd(u8 *);

#define MaxSpeechNum      4

/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#if defined(__CC_ARM)
#define Aligned4B __align(4)
__align(4) SACM_DVR1801_WORKING_RAM Dvr1801WorkingRam;
__align(4) SACM_DVR1801_TEMP_RAM Dvr1801TempBuffer;
__align(4) SACM_DVR1801_PCM_BUFFER Dvr1801PcmBuffer;
__align(4) KEYSCAN_WORKING_RAM KeyScanWorkingRam;
#elif defined(__GNUC__)
extern unsigned int __boot_table_start; //in GPCM2_CM3.ld
#define Aligned4B __attribute__ ((aligned (4)))
__attribute__ ((aligned (4))) SACM_DVR1801_WORKING_RAM Dvr1801WorkingRam;
__attribute__ ((aligned (4))) SACM_DVR1801_TEMP_RAM Dvr1801TempBuffer;
__attribute__ ((aligned (4))) SACM_DVR1801_PCM_BUFFER Dvr1801PcmBuffer;
__attribute__ ((aligned (4))) KEYSCAN_WORKING_RAM KeyScanWorkingRam;
#endif

uint32_t ScanedKey;
uint16_t A1801Num = 0;
int16_t A1801PlayIdx = 1;
uint8_t AudOutType = AUD_OUT_PWM;     //1:PWM 0:DAC Out
int8_t PlayCon = 0;
int16_t SwVolGain = 9;

// For SD Card
u32 sd_size=0;
u8 res = 0;
uint16_t PlaySD;
UINT bww;
	FIL file;
	DIR dir;
	FATFS fatfs;
u8 SD_RECWNum = 0;
u8 SD_RECRNum = 0;
u8 SD_MusicNum = 0;
uint16_t SDbuf[256];
uint16_t SDDataInx;

extern uint8_t __user_spifc_load_addr;
extern uint8_t __user_spifc_start;
extern uint32_t __user_spifc_size;

void MoveSpifcRamCode(void);

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
int main()
{
	WRITE_REG(RCU->STS,0x0000FFFF);                             // Clear Reset status
	MoveSpifcRamCode();

  // GPIO initialize
  GPIO_SetMode(GPIOA, 0xFFFFFFFF, GPIO_MODE_INPUT);         //IOA[31:0] Input pull-low, bypass IOA[14:13] UART & IOA.8 Output
  MODIFY_REG(GPIOA->OBUF, 0xFFFFFFFF, 0);
  GPIO_SetMode(GPIOB, 0xFFFFFFFF, GPIO_MODE_INPUT);         //IOB Input pull-low
  MODIFY_REG(GPIOB->OBUF, 0xFFFFFFFF, 0);
  GPIO_SetMode(GPIOC, 0xFFFFFFFF, GPIO_MODE_INPUT);         //IOC Input pull-low
  MODIFY_REG(GPIOC->OBUF, 0xFFFFFFFF, 0);
  GPIO_SetMode(GPIOD, 0xFFFFFFFF, GPIO_MODE_INPUT);         //IOD Input pull-low
  MODIFY_REG(GPIOD->OBUF, 0xFFFFFFFF, 0);

	A1801Num = DVR1801_User_GetA1801Num();
	SACM_DVR1801_Initial(&Dvr1801WorkingRam, &Dvr1801TempBuffer, &Dvr1801PcmBuffer);

	APP_SwVolCtrl_Init();                                    // Software volume control init.
	APP_SwVolCtrl_SetVolGain(0, SwVolGain);
	KeyScan_Initial(&KeyScanWorkingRam);                     // key scan init.


  while(SD_Initialize()!=0)
	{
		WDT_Clear();
	}
	sd_size=SD_GetSectorCount();
	res = f_mount(&fatfs,"0:",1);

		SD_RECWNum = 0;
		SD_MusicNum = 0;
		SD_RECRNum = 0;
		SD_PlayA18("0:/MUSIC0/MUSIC000.A18",SD_MusicNum++);

  while(1)
	{
		WDT_Clear();
		SACM_DVR1801_ServiceLoop();
		KeyScan_ServiceLoop();

		ScanedKey = KeyScan_GetCh();
	  switch(ScanedKey)
		{
		  case 0x01:                                           // IOA24+VDD: Stop
				PlayCon = 0;
				SACM_DVR1801_Stop();
        break;

			case 0x02:                                           // IOA25+VDD: Play next song
				PlayCon = 0;
				SACM_DVR1801_Stop();
				if((SACM_DVR1801_GetStatus() != 0))
				{
					WDT_Clear();
					SACM_DVR1801_ServiceLoop();
				}
				if(SD_MusicNum > MaxSpeechNum)
				{
          SD_MusicNum = 0;
				}
        SD_PlayA18("0:/MUSIC0/MUSIC000.A18",SD_MusicNum++);
				break;

			case 0x04:	                                         // IOA26+VDD: Play prev song
				PlayCon = 0;
				SACM_DVR1801_Stop();
				if((SACM_DVR1801_GetStatus() != 0))
				{
					WDT_Clear();
					SACM_DVR1801_ServiceLoop();
				}
				if(SD_MusicNum == 0)
				{
          SD_MusicNum = MaxSpeechNum-1;
				}
				SD_PlayA18("0:/MUSIC0/MUSIC000.A18",SD_MusicNum--);
				break;

			case 0x08:                                           // IOA27+VDD: Play concatenate
				PlayCon = 1;
			  SACM_DVR1801_Stop();
 				PlaySD = 0;
			  A1801PlayIdx = 1;
			  SACM_DVR1801_Play(DVR1801_User_GetA1801StartAddr(A1801PlayIdx++), DVR1801_DAC_CH0, DVR1801_AUTO_RAMP_UP + DVR1801_AUTO_RAMP_DOWN);
			  break;

			case 0x10:                                           // IOA28+VDD:
				SwVolGain = ++SwVolGain > MAX_VOL_GAIN ? 1 : SwVolGain;
			  APP_SwVolCtrl_SetVolGain(0, SwVolGain);
				break;

      case 0x20:                                           // IOA29+VDD: Record
				PlayCon = 0;
				SACM_DVR1801_Stop();
				if((SACM_DVR1801_GetStatus() != 0))
				{
					WDT_Clear();
					SACM_DVR1801_ServiceLoop();
				}
				SD_RECRNum = SD_RECWNum;
				SD_WriteA18("0:/RECORD/REC00000.A18",SD_RECWNum++);
				break;

			case 0x40:                                           // IOA30+VDD: Playback current
				PlayCon = 0;
				SACM_DVR1801_Stop();
				if((SACM_DVR1801_GetStatus() != 0))
				{
					WDT_Clear();
					SACM_DVR1801_ServiceLoop();
				}
				SD_PlayA18("0:/RECORD/REC00000.A18",SD_RECRNum);
			  break;

			case 0x80:                                           // IOA31+VDD: Playback next
				PlayCon = 0;
				SACM_DVR1801_Stop();
				if((SACM_DVR1801_GetStatus() != 0))
				{
					WDT_Clear();
					SACM_DVR1801_ServiceLoop();
				}
				if(SD_RECRNum < (SD_RECWNum -1))
				{
          SD_RECRNum +=1;
				}
				else
				{
          SD_RECRNum = 0;
				}
				/*
				if(SD_RECRNum > 0)
				{
          SD_RECRNum--;
				}
				else
				{
          SD_RECRNum = SD_RECWNum -1;
				}
				*/
				SD_PlayA18("0:/RECORD/REC00000.A18",SD_RECRNum);
				break;
	  }

		if((PlayCon != 0) && (SACM_DVR1801_Check_Con() == 0))
		{
			SACM_DVR1801_Play_Con(DVR1801_User_GetA1801StartAddr(A1801PlayIdx++), DVR1801_DAC_CH0, DVR1801_AUTO_RAMP_UP + DVR1801_AUTO_RAMP_DOWN);
			A1801PlayIdx = A1801PlayIdx > A1801Num ? 1 : A1801PlayIdx;
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



const unsigned DVR18_header1[8] = {0xff00, 0xff00, 0x4547, 0x454E, 0x4152, 0x504C, 0x554C, 0x2053};
unsigned int DVR18HeaderCheck()
{
	int i;
	for(i = 0; i < 8; i++)
	{
		if(SDbuf[i] != DVR18_header1[i])
		{
			return 0;
		}
	}
	return 1;
}


void SD_PlayA18(char *filname,u8 SDReadInx)
{
		int i;
		char SDFile[23];
		for(i=0;i<22;i++)
			{
				SDFile[i] = filname[i];
			}
		SDFile[22] = ' ';		//ÎÄ¼þÃû½áÊø
		SDFile[15] = 0x30+SDReadInx/100;
		SDFile[16] = 0x30+SDReadInx/10-(SDReadInx/100)*10;
		SDFile[17] = 0x30+SDReadInx%10;

	PlaySD = 1;
	res = f_open(&file,SDFile,FA_READ);
	if(res == 0)
	{
		ReadA18SDLoop();
		if(DVR18HeaderCheck())
					SDDataInx = 0x0018;
				else
					SDDataInx = 0x0000;
		PlayCon = 0;
		SACM_DVR1801_Stop();
		// SACM_DVR1801_Play(GetSpeechStartAddr(1), DVR1801_DAC_CH0 , DVR1801_AUTO_RAMP_UP + DVR1801_AUTO_RAMP_DOWN);
		SACM_DVR1801_Play(DVR1801_User_GetA1801StartAddr(A1801PlayIdx++), DVR1801_DAC_CH0, DVR1801_AUTO_RAMP_UP + DVR1801_AUTO_RAMP_DOWN);
	}
}

void SD_WriteA18(char *filname,u8 SDWriteInx)
{
				u8 i=0;
				char SDFile[23];
			  for(i=0;i<22;i++)
					{
						SDFile[i] = filname[i];
					}
				SDFile[22] = ' ';		//'Space'
				SDFile[15] = 0x30+SDWriteInx/100;
				SDFile[16] = 0x30+SDWriteInx/10-(SDWriteInx/100)*10;
				SDFile[17] = 0x30+SDWriteInx%10;

				PlayCon = 0;
				PlaySD = 1;
				SACM_DVR1801_Stop();
        SACM_DVR1801_Initial(&Dvr1801WorkingRam, &Dvr1801TempBuffer, &Dvr1801PcmBuffer);
			  SACM_DVR1801_Rec((int16_t *) 0x00200000, DVR1801_RECORD_BITRATE_16000);

		 	while(f_opendir(&dir,"0:/RECORD")) // Open & make dir
				{
						f_mkdir("0:/RECORD");
				}

					res=f_open (&file,SDFile, FA_CREATE_NEW|FA_WRITE);

				if(res==8) res=f_open (&file,SDFile, FA_WRITE);
				SDDataInx =3;
				SDbuf[0] = 0xffff;
				SDbuf[1] = 0;
				SDbuf[2] = 0x3E80;
}


void ReadA18SDLoop(void)
{
		u8 buf[512];//SD¿¨Êý¾Ý»º´æÇø
		int i;
		// GPIOA_OBIT->OBIT08 |= 0x00000100;		//for test 0.911ms

		f_read (&file, buf,512,&bww);
		if(bww<512)
		{
			f_close(&file);
		}
		for(i=0;i<256;i++)
		{
		SDbuf[i]= (buf[2*i+1]<<8) + buf[2*i];
		}
		SDDataInx = 0;
		// GPIOA_OBIT->OBIT08 &= ~0x00000100;
}

void WriteA18SDLoop(void)
{
		u8 buf[512];//SD¿¨Êý¾Ý»º´æÇø
		int i;

		// GPIOA_OBIT->OBIT08 |= 0x00000100;		//for test 0.807ms
		for(i=0;i<256;i++)
		{
		buf[2*i] = SDbuf[i];
		buf[2*i+1] = SDbuf[i]>>8;
		}
		SDDataInx = 0;


		f_write (&file, buf, 512, &bww);
		// GPIOA_OBIT->OBIT08 &= ~0x00000100;
}

void WriteA18SDEnd(uint8_t *datalen)
{

		u8 buf[512];//SD¿¨Êý¾Ý»º´æÇø
		int i;

//		GPIOA_OBIT->OBIT08 |= 0x00000100;		//for test 0.911ms
		for(i=0;i<SDDataInx;i++)
		{
		buf[2*i] = SDbuf[i];
		buf[2*i+1] = SDbuf[i]>>8;
		}
		f_write (&file, buf, SDDataInx*2, &bww);	//Ð´×îºóµÄÊý¾Ý¡£

		f_lseek(&file,0);							//Æ«ÒÆµ½ÎÄ¼þÍ·.
		f_write (&file, datalen, 4, &bww);
		f_close(&file);
		SDDataInx = 0;
}

