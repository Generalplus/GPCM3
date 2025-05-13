/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   BootCode_SPIFC_GPCM3_FM1.c
 * @Version:
 *   V0.9.2
 * @Date:
 *   Sep. 10, 2024
 * @Abstract:
 *
 **************************************************************************************************/


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "GPCM3_FM1.h"
#include "BootCode_SPIFC_GPCM3_FM1.h"


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
void SPIFC_EnhanModeReset(void);


/*---------------------------------------------------------------------------------------
 * Table Declaration Area
 *---------------------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------------------
 * Gobal Variable Declaration Area
 *---------------------------------------------------------------------------------------*/
uint32_t mSpiFcBackupCTRL0;
uint32_t mSpiFcBackupCTRL1;
uint32_t mSpiFcBackupCTRL2;
uint32_t mSpiFcBackupCMD;
uint32_t mSpiFcBackupPARA;
uint32_t mSpiFcBackupTIMING;


/*---------------------------------------------------------------------------------------
 * Constant Declaration Area
 *---------------------------------------------------------------------------------------*/
 #define C_ManualClockDiv       SPIFC_CLKSEL_HCLK_DIV4
 #define C_ManualTimingSetting  0x60


/*---------------------------------------------------------------------------------------
 * Subroutine Area
 *---------------------------------------------------------------------------------------*/
 /**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void SPIFC_SwitchToManualMode()
{
    mSpiFcBackupCTRL0 = SPIFC->CTRL0;
    mSpiFcBackupCTRL1 = SPIFC->CTRL1;
    mSpiFcBackupCTRL2 = SPIFC->CTRL2;
    mSpiFcBackupCMD = SPIFC->CMD;
    mSpiFcBackupPARA = SPIFC->PARA;
    mSpiFcBackupTIMING = SPIFC->TIMING;
    SPIFC->CTRL0 = 0;
    SPIFC->CTRL1 = 0;
    MODIFY_REG(SPIFC->TIMING, (SPIFC_TIMING_SMP_DELAY_MSK | SPIFC_TIMING_SMP_CLK_EDGE_SEL_MSK), C_ManualTimingSetting);
    SET_BIT(SPIFC->CTRL1, SPIFC_CTRL1_SPIFC_ENABLE);	                           // SPIFC enable
    MODIFY_REG(SPIFC->CTRL2, SPIFC_CTRL2_CLK_SEL_MSK, C_ManualClockDiv);
    SPIFC->CTRL0 = SPIFC_CTRL0_MIO_1BIT + SPIFC_CTRL0_MANUAL_MODE + SPIFC_CTRL0_CMIO_1BIT + SPIFC_CTRL0_AMIO_1BIT + SPIFC_CTRL0_IGNORE_LAST_CLK_DISABLE + SPIFC_CTRL0_CLK_STATE_LOW;
	SPIFC->RX_BC = 2;
	SPIFC->TX_BC = 0;
	SPIFC->PARA = SPIFC_PARA_ENHAN_DISABLE | SPIFC_PARA_WITHOUT_ADDR_ENABLE | SPIFC_PARA_ONLY_ADDR_DISABLE | SPIFC_PARA_DUMMY_CLK_0_CYCLE;  // wo_enhan = 1, wo_addr = 1, to_addr = 0
	SPIFC->CMD = 0x0 | SPIFC_CMD_WITHOUTCMD_DISABLE | SPIFC_CMD_ONLYCMD_DISABLE | SPIFC_CMD_1ST_TRANS_WITHCMD_DISABLE;                   // wo_cmd = 0, cmd_only = 0, one_cmd = 0
	while((SPIFC->CTRL0 & SPIFC_CTRL0_PENDING_FLAG) == 0);
    SPIFC_EnhanModeReset();
}


/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void SPIFC_ReturnToAutoMode()
{
    SPIFC->CTRL0 = 0;
    SPIFC->CTRL1 = 0;
    SPIFC->TX_BC = 0x0000;
    SPIFC->RX_BC = 0x0080;
    SPIFC->CTRL0 = mSpiFcBackupCTRL0;
    SPIFC->CTRL1 = mSpiFcBackupCTRL1;
    SPIFC->CTRL2 = mSpiFcBackupCTRL2;
    SPIFC->CMD = mSpiFcBackupCMD;
    SPIFC->PARA = mSpiFcBackupPARA;
    SPIFC->TIMING = mSpiFcBackupTIMING;
}


/**
 * @brief
 *   Enhance mode reset
 * @param
 *   None.
 * @return
 *   None.
 */
void SPIFC_EnhanModeReset(void)
{
	uint32_t iCount;

	SPIFC->CTRL0 = SPIFC_CTRL0_MIO_1BIT + SPIFC_CTRL0_MANUAL_MODE + SPIFC_CTRL0_CMIO_1BIT + SPIFC_CTRL0_AMIO_1BIT + SPIFC_CTRL0_IGNORE_LAST_CLK_DISABLE + SPIFC_CTRL0_CLK_STATE_LOW;

	SPIFC->RX_BC = 0;
	SPIFC->TX_BC = 1;

	SPIFC->PARA = SPIFC_PARA_ENHAN_DISABLE | SPIFC_PARA_WITHOUT_ADDR_ENABLE | SPIFC_PARA_ONLY_ADDR_DISABLE | SPIFC_PARA_DUMMY_CLK_0_CYCLE;  // wo_enhan = 1, wo_addr = 1, to_addr = 0
	SPIFC->CMD = 0xFF | SPIFC_CMD_WITHOUTCMD_DISABLE | SPIFC_CMD_ONLYCMD_DISABLE | SPIFC_CMD_1ST_TRANS_WITHCMD_DISABLE;                     // wo_cmd = 0, cmd_only = 0, one_cmd = 0
	while((SPIFC->CTRL0 & SPIFC_CTRL0_PENDING_FLAG) == 0);

	//for(iCount = 0; iCount < 1; iCount++)
	//{
	  SPIFC->TX_DATA = 0xFF;
	  while((SPIFC->CTRL0 & SPIFC_CTRL0_TX_DONE_FLAG) == 0);
	//}
}


/**
 * @brief
 *   SPI flash write enable
 * @param
 *   None.
 * @return
 *   None.
 */
void SPIFC_WriteEnable()
{
  SPIFC_Cmd(SPI_FLASH_WREN);
}


/**
 * @brief
 *   Write SPI flash status register
 * @param
 *   SpiStatusRegister [in]: SPI status register value
 * @return
 *   None.
 */
void SPIFC_WriteSR(uint8_t SpiStatusRegister)
{
  SPIFC_SwitchToManualMode();
	SPIFC_WriteEnable();
  SPIFC_CmdTx(SPIFC_FLASH_WRSR, &SpiStatusRegister, 1);
	while((SPIFC_ReadSR() & SPIFC_SR_WIP_BIT) != 0);
	SPIFC_ReturnToAutoMode();
}


/**
 * @brief
 *   Read Status Register
 * @param
 *   None.
 * @return
 *   SPI status register value
 */
uint16_t SPIFC_ReadSR()
{
	uint8_t SpiStatusRegister[1];

  SPIFC_CmdRx(SPIFC_FLASH_RDSR, SpiStatusRegister, 1);

	return (*((uint16_t*) SpiStatusRegister) & 0x00FF);
}


/**
 * @brief
 *   Read configuration Register
 * @param
 *   None.
 * @return
 *   SPI configuration register
 */
uint8_t SPIFC_ReadCR()
{
	uint8_t SpiConfigRegister[1];

	SPIFC_SwitchToManualMode();
  SPIFC_CmdRx(SPIFC_FLASH_RDCR, SpiConfigRegister, 1);
  SPIFC_ReturnToAutoMode();

	return SpiConfigRegister[0];
}


/**
 * @brief
 *
 * @param
 *   SpiAddr [in]: SPI flash address
 * @return
 *
 */
void SPIFC_SectorErase(uint32_t SpiAddr)
{
  SPIFC_SwitchToManualMode();
	SPIFC_WriteEnable();
  SPIFC_CmdAddr(SPIFC_FLASH_SE, SpiAddr);
	while((SPIFC_ReadSR() & SPIFC_SR_WIP_BIT) != 0);
	SPIFC_ReturnToAutoMode();
}


/**
 * @brief
 *
 * @param
 *   SpiAddr [in]: SPI flash address
 * @return
 *
 */
void SPIFC_BlockErase(uint32_t SpiAddr)
{
  SPIFC_SwitchToManualMode();
	SPIFC_WriteEnable();
  SPIFC_CmdAddr(SPIFC_FLASH_BE, SpiAddr);
	while((SPIFC_ReadSR() & SPIFC_SR_WIP_BIT) != 0);
	SPIFC_ReturnToAutoMode();
}

/**
 * @brief
 *   Read bytes data from flash
 * @param
 *   SpiAddr [in]: SPI flash address
 *  *pDstBuf [out]: Destination buffer address
 *   NumBytes [in]: Data length (Byte)
 * @return
 *
 */
void SPIFC_ReadNBytes(uint32_t SpiAddr, uint8_t *pDstBuf, uint32_t NumBytes)
{
	uint32_t DstBufIdx = 0;

	SPIFC_SwitchToManualMode();
  while(NumBytes != 0)
	{
		if(NumBytes > 256)
		{
	    SPIFC_CmdAddrRx(SPIFC_FLASH_READ, SpiAddr, &pDstBuf[DstBufIdx], 256);
			SpiAddr += 256;
			DstBufIdx += 256;
			NumBytes -= 256;
		}
		else
		{
			SPIFC_CmdAddrRx(SPIFC_FLASH_READ, SpiAddr, &pDstBuf[DstBufIdx], NumBytes);
			NumBytes = 0;
		}
	}
	SPIFC_ReturnToAutoMode();
}

/**
 * @brief
 *   Write bytes data from flash
 * @param
 *   DstSpiAddr [out]: SPI flash address
 *  *pSrcBuf [in]: Source buffer address
 *   NumBytes [in]: Data length (Byte)
 * @return
 *
 */
void SPIFC_WriteNBytes(uint32_t DstSpiAddr, uint8_t *pSrcBuf, uint32_t NumBytes)
{
	uint32_t DstBufIdx = 0;
	uint32_t PageEmptyBytes;

	SPIFC_SwitchToManualMode();
  while(NumBytes != 0)
	{
		SPIFC_WriteEnable();
		PageEmptyBytes = 256 - (DstSpiAddr & BYTE0_MSK);
		if(NumBytes < PageEmptyBytes)
		{
			SPIFC_CmdAddrTx(SPIFC_FLASH_PP, DstSpiAddr, &pSrcBuf[DstBufIdx], NumBytes);
			NumBytes = 0;
			while((SPIFC_ReadSR() & SPIFC_SR_WIP_BIT) != 0);
		}
		else
		{
	    SPIFC_CmdAddrTx(SPIFC_FLASH_PP, DstSpiAddr, &pSrcBuf[DstBufIdx], PageEmptyBytes);
			DstSpiAddr += PageEmptyBytes;
			DstBufIdx += PageEmptyBytes;
			NumBytes -= PageEmptyBytes;
			while((SPIFC_ReadSR() & SPIFC_SR_WIP_BIT) != 0);
		}
	}
	SPIFC_ReturnToAutoMode();
}

/**
 * @brief
 *   Write words data to flash
 * @param
 *   DstSpiAddr [out]: SPI flash address
 *  *pSrcBuf [in]: Source buffer address
 *   NumBytes [in]: Data length (word)
 * @return
 *
 */
void SPIFC_WriteNWords(uint32_t DstSpiAddr, uint8_t *pSrcBuf, uint32_t NumBytes)
{
	uint32_t DstBufIdx = 0;
	uint32_t PageEmptyBytes;

	SPIFC_SwitchToManualMode();
    while(NumBytes != 0)
	{
		SPIFC_WriteEnable();
		PageEmptyBytes = 256 - (DstSpiAddr & BYTE0_MSK);
		if(NumBytes < PageEmptyBytes)
		{
			SPIFC_CmdAddrTx32(SPIFC_FLASH_PP, DstSpiAddr, (uint32_t *)&pSrcBuf[DstBufIdx], NumBytes);
			NumBytes = 0;
			while((SPIFC_ReadSR() & SPIFC_SR_WIP_BIT) != 0);
		}
		else
		{
            SPIFC_CmdAddrTx32(SPIFC_FLASH_PP, DstSpiAddr, (uint32_t *)&pSrcBuf[DstBufIdx], PageEmptyBytes);
			DstSpiAddr += PageEmptyBytes;
			DstBufIdx += PageEmptyBytes;
			NumBytes -= PageEmptyBytes;
			while((SPIFC_ReadSR() & SPIFC_SR_WIP_BIT) != 0);
		}
	}
	SPIFC_ReturnToAutoMode();
}

/**
 * @brief
 *
 * @param
 *   SpiCmd [in]: SPI flash command
 * @return
 *   None.
 */
void SPIFC_Cmd(uint8_t SpiCmd)
{
	SPIFC->CTRL0 = SPIFC_CTRL0_MIO_1BIT + SPIFC_CTRL0_MANUAL_MODE + SPIFC_CTRL0_CMIO_1BIT + SPIFC_CTRL0_AMIO_1BIT + SPIFC_CTRL0_IGNORE_LAST_CLK_DISABLE + SPIFC_CTRL0_CLK_STATE_LOW;
	SPIFC->CMD = SpiCmd | SPIFC_CMD_WITHOUTCMD_DISABLE | SPIFC_CMD_ONLYCMD_ENABLE | SPIFC_CMD_1ST_TRANS_WITHCMD_DISABLE;                    // wo_cmd = 0, cmd_only = 1, one_cmd = 0
	while((SPIFC->CTRL0 & SPIFC_CTRL0_PENDING_FLAG) == 0);
}

/**
 * @brief
 *
 * @param
 *   SpiCmd [in]: SPI flash command
 *   SpiAddr [in]: SPI flash address
 * @return
 *   None.
 */
void SPIFC_CmdAddr(uint8_t SpiCmd, uint32_t SpiAddr)
{
	SPIFC->CTRL0 = SPIFC_CTRL0_MIO_1BIT + SPIFC_CTRL0_MANUAL_MODE + SPIFC_CTRL0_CMIO_1BIT + SPIFC_CTRL0_AMIO_1BIT + SPIFC_CTRL0_IGNORE_LAST_CLK_DISABLE + SPIFC_CTRL0_CLK_STATE_LOW;

	SPIFC->ADDRH = SpiAddr >> 16;
	SPIFC->ADDRL = SpiAddr & 0xFFFF;

	SPIFC->PARA = SPIFC_PARA_ENHAN_DISABLE | SPIFC_PARA_WITHOUT_ADDR_DISABLE | SPIFC_PARA_ONLY_ADDR_ENABLE | SPIFC_PARA_DUMMY_CLK_0_CYCLE;  // wo_enhan = x, wo_addr = 0, to_addr = 1
	SPIFC->CMD = SpiCmd | SPIFC_CMD_WITHOUTCMD_DISABLE | SPIFC_CMD_ONLYCMD_DISABLE | SPIFC_CMD_1ST_TRANS_WITHCMD_DISABLE;                   // wo_cmd = 0, cmd_only = 0, one_cmd = 0
	while((SPIFC->CTRL0 & SPIFC_CTRL0_PENDING_FLAG) == 0);
}

/**
 * @brief
 *
 * @param
 *   SpiCmd [in]: SPI flash command
 *  *pDstBuf [out]: Destination buffer address
 *   NumBytes [in]: Data length (Byte)
 * @return
 *   None.
 */
void SPIFC_CmdRx(uint8_t SpiCmd, uint8_t *pDstBuf, uint32_t NumBytes)
{
	uint32_t iCount;
	uint32_t jCount;
	uint32_t SpiRxTemp;
	uint32_t ScrCtrlBackup;

	ScrCtrlBackup = SPIFC->SCRAMBLE_CTRL;
	MODIFY_REG(SPIFC->SCRAMBLE_CTRL,SPIFC_SCRAMBLE_CTRL_DEC_EN_MSK,SPIFC_SCRAMBLE_CTRL_DEC_DISABLE);

	SPIFC->CTRL0 = SPIFC_CTRL0_MIO_1BIT + SPIFC_CTRL0_MANUAL_MODE + SPIFC_CTRL0_CMIO_1BIT + SPIFC_CTRL0_AMIO_1BIT + SPIFC_CTRL0_IGNORE_LAST_CLK_DISABLE + SPIFC_CTRL0_CLK_STATE_LOW;

	SPIFC->RX_BC = NumBytes;
	SPIFC->TX_BC = 0;

	SPIFC->PARA = SPIFC_PARA_ENHAN_DISABLE | SPIFC_PARA_WITHOUT_ADDR_ENABLE | SPIFC_PARA_ONLY_ADDR_DISABLE | SPIFC_PARA_DUMMY_CLK_0_CYCLE;  // wo_enhan = 1, wo_addr = 1, to_addr = 0
	SPIFC->CMD = SpiCmd | SPIFC_CMD_WITHOUTCMD_DISABLE | SPIFC_CMD_ONLYCMD_DISABLE | SPIFC_CMD_1ST_TRANS_WITHCMD_DISABLE;                   // wo_cmd = 0, cmd_only = 0, one_cmd = 0
	while((SPIFC->CTRL0 & SPIFC_CTRL0_PENDING_FLAG) == 0);

	for(iCount = 0; iCount < NumBytes; iCount += 4)
	{
		SPIFC->RX_DATA = 0;																	                       // write dummy before read operation.
		while((SPIFC->CTRL0 & SPIFC_CTRL0_PENDING_FLAG) == 0);
		SpiRxTemp = SPIFC->RX_DATA;

		for(jCount = iCount; jCount < (iCount + 4); jCount++)
		{
      if(jCount == NumBytes)
      {
        break;
      }
	    pDstBuf[jCount] = (SpiRxTemp >> ((jCount % 4) << 3)) & BYTE0_MSK;
	  }
	}
	SPIFC->SCRAMBLE_CTRL = ScrCtrlBackup;
}

/**
 * @brief
 *
 * @param
 *   SpiCmd [in]: SPI flash command
 *   SpiAddr [in]: SPI flash address
 *  *pDstBuf [out]: Destination buffer address
 *   NumBytes [in]: Data length (Byte)
 * @return
 *   None.
 */
void SPIFC_CmdAddrRx(uint8_t SpiCmd, uint32_t SpiAddr, uint8_t *pDstBuf, uint32_t NumBytes)
{
	uint32_t iCount;
	uint32_t jCount;
	uint32_t SpiRxTemp;

	SPIFC->CTRL0 = SPIFC_CTRL0_MIO_1BIT + SPIFC_CTRL0_MANUAL_MODE + SPIFC_CTRL0_CMIO_1BIT + SPIFC_CTRL0_AMIO_1BIT + SPIFC_CTRL0_IGNORE_LAST_CLK_DISABLE + SPIFC_CTRL0_CLK_STATE_LOW;
	SPIFC->ADDRH = SpiAddr >> 16;
	SPIFC->ADDRL = SpiAddr & 0xFFFF;
	SPIFC->RX_BC = NumBytes;
	SPIFC->TX_BC = 0;
	SPIFC->PARA = SPIFC_PARA_ENHAN_DISABLE | SPIFC_PARA_WITHOUT_ADDR_DISABLE | SPIFC_PARA_ONLY_ADDR_DISABLE | SPIFC_PARA_DUMMY_CLK_0_CYCLE;  // wo_enhan = 1, wo_addr = 0, to_addr = 0
	SPIFC->CMD = SpiCmd | SPIFC_CMD_WITHOUTCMD_DISABLE | SPIFC_CMD_ONLYCMD_DISABLE | SPIFC_CMD_1ST_TRANS_WITHCMD_DISABLE;                    // wo_cmd = 0, cmd_only = 0, one_cmd = 0
	while((SPIFC->CTRL0 & SPIFC_CTRL0_PENDING_FLAG) == 0);

	for(iCount = 0; iCount < NumBytes; iCount += 4)
	{
		SPIFC->RX_DATA = 0;																	                       // write dummy before read operation.
		while((SPIFC->CTRL0 & SPIFC_CTRL0_PENDING_FLAG) == 0);
		SpiRxTemp = SPIFC->RX_DATA;

		for(jCount = iCount; jCount < (iCount + 4); jCount++)
		{
      if(jCount == NumBytes)
      {
        break;
      }
	    pDstBuf[jCount] = (SpiRxTemp >> ((jCount % 4) << 3)) & 0x00FF;
	  }
	}
}

/**
 * @brief
 *
 * @param
 *   SpiCmd [in]: SPI flash command
 *  *pSrcBuf [in]: Source buffer address
 *   NumBytes [in]: Data length (Byte)
 * @return
 *   None.
 */
void SPIFC_CmdTx(uint8_t SpiCmd, uint8_t *pSrcBuf, uint32_t NumBytes)
{
	uint32_t iCount;

	SPIFC->CTRL0 = SPIFC_CTRL0_MIO_1BIT + SPIFC_CTRL0_MANUAL_MODE + SPIFC_CTRL0_CMIO_1BIT + SPIFC_CTRL0_AMIO_1BIT + SPIFC_CTRL0_IGNORE_LAST_CLK_DISABLE + SPIFC_CTRL0_CLK_STATE_LOW;
	SPIFC->RX_BC = 0;
	SPIFC->TX_BC = NumBytes;
	SPIFC->PARA = SPIFC_PARA_ENHAN_DISABLE | SPIFC_PARA_WITHOUT_ADDR_ENABLE | SPIFC_PARA_ONLY_ADDR_DISABLE | SPIFC_PARA_DUMMY_CLK_0_CYCLE;  // wo_enhan = 1, wo_addr = 1, to_addr = 0
	SPIFC->CMD = SpiCmd | SPIFC_CMD_WITHOUTCMD_DISABLE | SPIFC_CMD_ONLYCMD_DISABLE | SPIFC_CMD_1ST_TRANS_WITHCMD_DISABLE;                   // wo_cmd = 0, cmd_only = 0, one_cmd = 0
	while((SPIFC->CTRL0 & SPIFC_CTRL0_PENDING_FLAG) == 0);

	for(iCount = 0; iCount < NumBytes; iCount++)
	{
	  SPIFC->TX_DATA = pSrcBuf[iCount];
	  while((SPIFC->CTRL0 & SPIFC_CTRL0_TX_DONE_FLAG) == 0);
	}
}

/**
 * @brief
 *
 * @param
 *   SpiCmd [in]: SPI flash command
 *   SpiAddr [in]: SPI flash address
 *  *pSrcBuf [in]: Source buffer address
 *   NumBytes [in]: Data length (Byte)
 * @return
 *   None.
 */
void SPIFC_CmdAddrTx(uint8_t SpiCmd, uint32_t SpiAddr, uint8_t *pSrcBuf, uint32_t NumBytes)
{
	uint32_t iCount;

	SPIFC->CTRL0 = SPIFC_CTRL0_MIO_1BIT + SPIFC_CTRL0_MANUAL_MODE + SPIFC_CTRL0_CMIO_1BIT + SPIFC_CTRL0_AMIO_1BIT + SPIFC_CTRL0_IGNORE_LAST_CLK_DISABLE + SPIFC_CTRL0_CLK_STATE_LOW;
	SPIFC->ADDRH = SpiAddr >> 16;
	SPIFC->ADDRL = SpiAddr & 0xFFFF;
	SPIFC->RX_BC = 0;
	SPIFC->TX_BC = NumBytes;
	SPIFC->PARA = SPIFC_PARA_ENHAN_DISABLE | SPIFC_PARA_WITHOUT_ADDR_DISABLE | SPIFC_PARA_ONLY_ADDR_DISABLE | SPIFC_PARA_DUMMY_CLK_0_CYCLE;  // wo_enhan = 1, wo_addr = 0, to_addr = 0
	SPIFC->CMD = SpiCmd | SPIFC_CMD_WITHOUTCMD_DISABLE | SPIFC_CMD_ONLYCMD_DISABLE | SPIFC_CMD_1ST_TRANS_WITHCMD_DISABLE;                    // wo_cmd = 0, cmd_only = 0, one_cmd = 0
  while((SPIFC->CTRL0 & SPIFC_CTRL0_PENDING_FLAG) == 0);

	for(iCount = 0; iCount < NumBytes; iCount++)
	{
	  SPIFC->TX_DATA = pSrcBuf[iCount];
	  while((SPIFC->CTRL0 & SPIFC_CTRL0_TX_DONE_FLAG) == 0);
	}
}

/**
 * @brief
 *
 * @param
 *   SpiCmd [in]: SPI flash command
 *   SpiAddr [in]: SPI flash address
 *  *pSrcBuf [in]: Source buffer address
 *   NumBytes [in]: Data length (Byte)
 * @return
 *   None.
 */
void SPIFC_CmdAddrTx32(uint8_t SpiCmd, uint32_t SpiAddr, uint32_t *pSrcBuf, uint32_t NumBytes)
{
	uint32_t iCount;

	SPIFC->CTRL0 = SPIFC_CTRL0_MIO_1BIT + SPIFC_CTRL0_MANUAL_MODE + SPIFC_CTRL0_CMIO_1BIT + SPIFC_CTRL0_AMIO_1BIT + SPIFC_CTRL0_IGNORE_LAST_CLK_DISABLE + SPIFC_CTRL0_CLK_STATE_LOW;
	SPIFC->ADDRH = SpiAddr >> 16;
	SPIFC->ADDRL = SpiAddr & 0xFFFF;
	SPIFC->RX_BC = 0;
	SPIFC->TX_BC = NumBytes;
	SPIFC->PARA = SPIFC_PARA_ENHAN_DISABLE | SPIFC_PARA_WITHOUT_ADDR_DISABLE | SPIFC_PARA_ONLY_ADDR_DISABLE | SPIFC_PARA_DUMMY_CLK_0_CYCLE;  // wo_enhan = 1, wo_addr = 0, to_addr = 0
	SPIFC->CMD = SpiCmd | SPIFC_CMD_WITHOUTCMD_DISABLE | SPIFC_CMD_ONLYCMD_DISABLE | SPIFC_CMD_1ST_TRANS_WITHCMD_DISABLE;                    // wo_cmd = 0, cmd_only = 0, one_cmd = 0
    while((SPIFC->CTRL0 & SPIFC_CTRL0_PENDING_FLAG) == 0);

	for(iCount = 0; iCount < NumBytes; iCount+=4)
	{
	  SPIFC->TX_DATA32 = pSrcBuf[iCount>>2];
	  while((SPIFC->CTRL0 & SPIFC_CTRL0_TX_DONE_FLAG) == 0);
	}
}

