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
#include "SACM_A1801_User.h"
#include "APP_SwVolumeControl.h"
#include "KeyScan.h"
#include "FW_Update.h"

// For SPI flash
#define DmaHandle             DMA0
#define SPIFLASH_RW_STARTADDR 0x100
#define SPIFC_FW_ADDR         0x000FF000


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#if defined(__CC_ARM)
#define Aligned4B __align(4)
__align(4) SACM_A1801_WORKING_RAM A1801WorkingRam;
__align(4) SACM_A1801_TEMP_RAM A1801TempBuffer;
__align(4) SACM_A1801_PCM_BUFFER A1801PcmBuffer;
__align(4) KEYSCAN_WORKING_RAM KeyScanWorkingRam;
#elif defined(__GNUC__)
extern unsigned int __boot_table_start; //in GPCM2_CM3.ld
#define Aligned4B __attribute__ ((aligned (4)))
__attribute__ ((aligned (4))) SACM_A1801_WORKING_RAM A1801WorkingRam;
__attribute__ ((aligned (4))) SACM_A1801_TEMP_RAM A1801TempBuffer;
__attribute__ ((aligned (4))) SACM_A1801_PCM_BUFFER A1801PcmBuffer;
__attribute__ ((aligned (4))) KEYSCAN_WORKING_RAM KeyScanWorkingRam;
#endif

uint32_t ScanedKey;
uint16_t SpeechNum = 0;
int16_t A1801PlayIdx = 3;
int8_t PlayCon = 0;
int8_t SwVolGain = 7;

// for SPI1 flash & DMA test
uint8_t SrcBuf1[52] = {0x80, 0xb5, 0x82, 0xb0, 0x00, 0xaf, 0xbb, 0x1d, 0x00, 0x22, 0x1A, 0x80, 0x05, 0xe0, 0xc0, 0x46,
                       0xbb, 0x1d, 0x1a, 0x88, 0xbb, 0x1d, 0x01, 0x32, 0x1a, 0x80, 0xbb, 0x1d, 0x1b, 0x88, 0x04, 0x4a,
                       0x93, 0x42, 0xf4, 0xd9, 0xbb, 0x1d, 0x1b, 0x88, 0x18, 0x1c, 0xbd, 0x46, 0x02, 0xb0, 0x80, 0xbd,
                       0xff, 0x01, 0x00, 0x00};


uint8_t DstBuf1[52];
uint16_t R_Temp;

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

 /**
 * @brief
 *	 Read bytes data from SPI1 flash IOA[12:9] with DMA
 * @param
 *   SpiAddr [in]: SPI flash address
 *  *pDstBuf [out]: Destination buffer address
 *   NumByteToRead [in]: Data length (Byte)
 * @return
 *        None
 */
void SPI_Flash_ReadNBytes_DMA(uint32_t SpiAddr, uint8_t *pDstBuf, uint32_t NumByteToRead)
{
  SPI_TYPE_DEF 	*mSpiHandle = SPI1;
  __IO uint32_t *mSpiCsPin = &GPIOA_OBIT->OBIT11;

	*mSpiCsPin = 0;                                                    // CS low
	SPI_Write(SPI_FLASH_READ);

	#if SPI_FLASH_4BYTE_ADDR_EN
	SPI_Write((ReadAddr & 0xFF000000) >> 24);                          // for 4-bytes address
	#endif

	SPI_Write((SpiAddr & 0xFF0000) >> 16);                             // addressH
	SPI_Write((SpiAddr & 0xFF00) >> 8);                                // addressM
	SPI_Write(SpiAddr & 0xFF);                                         // addressL

  /*
   * SPI RX DMA
   */
  DMA_Init(DmaHandle, DMA_REQSEL_SPI1, DMA_SRC_DATA_8B, DMA_SRC_ADDR_FIX, DMA_DST_DATA_8B, DMA_DST_ADDR_INC);
  while(DMA_CheckBusy(DmaHandle) != 0);                                   // Check DMAx busy
  DMA_Trigger(DmaHandle, (uint32_t)&mSpiHandle->RX_DATA , (uint32_t)&pDstBuf[0], NumByteToRead);
  while(DMA_CheckBusy(DmaHandle) != 0);

	*mSpiCsPin = 0xFFFFFFFF;                                           // CS high
}

/**
 * @brief
 *   Write N bytes data to SPI flash IOA[12:9] with DMA
 * @param
 *   SpiAddr [in]: SPI flash address
 *  *pSrcBuf [in]: Source buffer address
 *   NumBytes [in]: Data length (Byte)
 * @return
 *   None
 */
void SPI_Flash_WritePage_DMA(uint32_t SpiAddr, uint8_t* pSrcBuf, uint32_t NumBytes)
{
  SPI_TYPE_DEF 	*mSpiHandle = SPI1;
  __IO uint32_t *mSpiCsPin = &GPIOA_OBIT->OBIT11;

	Flash_Write_Enable();
	*mSpiCsPin = 0;                                                    // CS low
	SPI_Write(SPI_FLASH_PP);

	#if SPI_FLASH_4BYTE_ADDR_EN
	SPI_Write((WriteAddr & 0xFF000000) >> 24);                         // for 4-bytes address
	#endif

	SPI_Write((SpiAddr & 0xFF0000) >> 16);                             // addressH
	SPI_Write((SpiAddr & 0xFF00) >> 8);                                // addressM
	SPI_Write(SpiAddr & 0xFF);                                         // addressL

	/*
   * SPI TX DMA
	 */
  DMA_Init(DmaHandle, DMA_REQSEL_SPI1, DMA_SRC_DATA_8B, DMA_SRC_ADDR_INC, DMA_DST_DATA_8B, DMA_DST_ADDR_FIX);
  while(DMA_CheckBusy(DmaHandle) != 0);                                   // Check DMAx busy
  DMA_Trigger(DmaHandle, (uint32_t)&pSrcBuf[0], (uint32_t)&mSpiHandle->TX_DATA, NumBytes);
  while(DMA_CheckBusy(DmaHandle) != 0);

	while(((mSpiHandle->STS) & SPI_CTRL_TX_DONE_FLAG) == 0);
	mSpiHandle->STS |= SPI_CTRL_TX_DONE_FLAG;
	*mSpiCsPin = 0xFFFFFFFF;                                           // CS high
	while((SPI_Flash_ReadSR() & SPI_FLASH_SR_WIP_BIT) != 0);
}


/*---------------------------------------------------------------------------------------
 * Main Function
 *---------------------------------------------------------------------------------------*/
int main()
{
	WRITE_REG(RCU->STS,0x0000FFFF);                             // Clear Reset status
	MoveSpifcRamCode();
	MODIFY_REG(SPIFC->SCRAMBLE_CTRL,SPIFC_SCRAMBLE_CTRL_ENC_EN_MSK,SPIFC_SCRAMBLE_CTRL_ENC_ENABLE);   // Encode enabled in manual mode.

  // GPIO initialize
  GPIO_SetMode(GPIOA, 0xFFFFFFFF, GPIO_MODE_INPUT);         //IOA[31:0] Input pull-low, bypass IOA[14:13] UART & IOA.8 Output
  MODIFY_REG(GPIOA->OBUF, 0xFFFFFFFF, 0);
  GPIO_SetMode(GPIOC, 0xFFFFFFFF, GPIO_MODE_INPUT);         //IOC Input pull-low
  MODIFY_REG(GPIOC->OBUF, 0xFFFFFFFF, 0);
  GPIO_SetMode(GPIOD, 0xFFFFFFFF, GPIO_MODE_INPUT);         //IOD Input pull-low
  MODIFY_REG(GPIOD->OBUF, 0xFFFFFFFF, 0);

	SpeechNum = A1801_User_GetA1801Num();
	SACM_A1801_Initial(&A1801WorkingRam, &A1801TempBuffer, &A1801PcmBuffer);

	APP_SwVolCtrl_Init();                                    // Software volume control init.
	APP_SwVolCtrl_SetVolGain(0, SwVolGain);
	KeyScan_Initial(&KeyScanWorkingRam);                     // key scan init.

  SACM_A1801_Play(A1801_User_GetA1801StartAddr(A1801PlayIdx), A1801_DAC_CH0 , A1801_AUTO_RAMP_UP + A1801_AUTO_RAMP_DOWN);

  while(1)
	{
		WDT_Clear();
		SACM_A1801_ServiceLoop();
		KeyScan_ServiceLoop();

		if(SACM_A1801_CheckCpuOverload() != 0)
		{
			SACM_A1801_ClearCpuOverload();
		}

		ScanedKey = KeyScan_GetCh();
	  switch(ScanedKey)
		{
		  case 0x01:                                           // IOA24+VDD: Stop
				PlayCon = 0;
				SACM_A1801_Stop();
        break;

			case 0x02:                                           // IOA25+VDD: Play next song
				PlayCon = 0;
				SACM_A1801_Stop();
			  A1801PlayIdx = ++A1801PlayIdx > SpeechNum ? 1 : A1801PlayIdx;
				SACM_A1801_Play(A1801_User_GetA1801StartAddr(A1801PlayIdx), A1801_DAC_CH0 , A1801_AUTO_RAMP_UP + A1801_AUTO_RAMP_DOWN);
				break;

			case 0x04:	                                         // IOA26+VDD: Play prev song
				PlayCon = 0;
				SACM_A1801_Stop();
			  A1801PlayIdx = --A1801PlayIdx < 1 ? SpeechNum : A1801PlayIdx;
				SACM_A1801_Play(A1801_User_GetA1801StartAddr(A1801PlayIdx), A1801_DAC_CH1, A1801_AUTO_RAMP_UP + A1801_AUTO_RAMP_DOWN);
				break;

			case 0x08:                                           // IOA27+VDD: Play concatenate
				PlayCon = 1;
			  SACM_A1801_Stop();
			  A1801PlayIdx = 1;
			  SACM_A1801_Play(A1801_User_GetA1801StartAddr(A1801PlayIdx++), A1801_DAC_CH0, A1801_AUTO_RAMP_UP + A1801_AUTO_RAMP_DOWN);
			  break;

			case 0x10:                                           // IOA28+VDD:	SPI flash RW with DMA
        SPI_Flash_Open();                                  // Default: SPI_OUTPUT_SEL = SPI1_IOA9_12
        SPI_Flash_Sector_Erase(0);
        // SPI_Flash_SendNBytes(SPIFLASH_RW_STARTADDR, (uint8_t *)SrcBuf1, sizeof(SrcBuf1));
        // SPI_Flash_ReadNBytes(SPIFLASH_RW_STARTADDR, (uint8_t *)DstBuf1, sizeof(DstBuf1));
        SPI_Flash_WritePage_DMA(SPIFLASH_RW_STARTADDR, (uint8_t *)SrcBuf1, sizeof(SrcBuf1));
        SPI_Flash_ReadNBytes_DMA(SPIFLASH_RW_STARTADDR, (uint8_t *)DstBuf1, sizeof(DstBuf1));
				break;

			case 0x20:                                           // IOA29+VDD: FW updated
			  SACM_A1801_Stop();
			  R_Temp = FW_Open();
				SPIFC_SectorErase(SPIFC_FW_ADDR);
				SPIFC_WriteNWords((uint32_t)SPIFC_FW_ADDR, DstBuf1, (52 << 1));
				break;

      case 0x40:                                           // IOA30+VDD: Volume Up
        R_Temp = FW_Open();
				break;

			case 0x80:                                           // IOA31+VDD: Volume Dn
			  SwVolGain = --SwVolGain <= MIN_VOL_GAIN ? MIN_VOL_GAIN : SwVolGain;
			  APP_SwVolCtrl_SetVolGain(0, SwVolGain);
			  break;
	  }

		if((PlayCon != 0) && (SACM_A1801_Check_Con() == 0))
		{
			SACM_A1801_Play_Con(A1801_User_GetA1801StartAddr(A1801PlayIdx++), A1801_DAC_CH0, A1801_AUTO_RAMP_UP + A1801_AUTO_RAMP_DOWN);
			A1801PlayIdx = A1801PlayIdx > SpeechNum ? 1 : A1801PlayIdx;
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
