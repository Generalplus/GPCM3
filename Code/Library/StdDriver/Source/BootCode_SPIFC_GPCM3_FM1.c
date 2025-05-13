/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   BootCode_SPIFC_GPCM3_FM1.c
 * @Version:
 *   V0.9.1
 * @Date:
 *   Nov. 10, 2023
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


/*---------------------------------------------------------------------------------------
 * Table Declaration Area
 *---------------------------------------------------------------------------------------*/
const uint8_t T_SPIFlash_REMS[] =             // DeviceID, ManufacturerID, RDSR3, RDSR2, WRSR3, WRSR2, QEbit
{
	0x13, 0xC2, 0x00, 0x00, 0x00, 0x00, 0x06,		// GPR25L081B, GPR25L0805E, default
	0x13, 0xC8, 0x00, 0x35, 0x00, 0x00, 0x09,		// GD25Q80C
	0x13, 0x5E, 0x15, 0x35, 0x11, 0x31, 0x09, 	// ZB25VQ80
	0x13, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x00,		// EN25Q80B
	0x13, 0x0B, 0x00, 0x35, 0x00, 0x00, 0x09,		// XT25W08B
	0x14, 0xC2, 0x00, 0x00, 0x00, 0x00, 0x00,		// GPR25L162B
	0x24, 0xC2, 0x00, 0x00, 0x00, 0x00, 0x06,		// GPR25L1603E
	0x14, 0xC8, 0x00, 0x35, 0x00, 0x00, 0x09,		// GD25Q16C
	0x14, 0x5E, 0x15, 0x35, 0x11, 0x31, 0x09,		// ZB25VQ16
	0x14, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x00,		// EN25QH16A
	0x14, 0x0B, 0x00, 0x35, 0x00, 0x00, 0x09,		// XT25F16B
	0x14, 0x68, 0x15, 0x35, 0x11, 0x31, 0x09,		// BY25Q16BS
	0x15, 0xC2, 0x00, 0x00, 0x00, 0x00, 0x06,		// GPR25L322B, GPR25L3203F
	0x15, 0xC8, 0x15, 0x35, 0x11, 0x31, 0x09,		// GD25Q32C
	0x15, 0x20, 0x15, 0x35, 0x11, 0x31, 0x09,		// XM25QH32B
	0x15, 0x1C, 0x00, 0x09, 0x00, 0x00, 0x00,		// EN25QH32A
	0x16, 0xC2, 0x00, 0x00, 0x00, 0x00, 0x06,		// GPR25L642B, GPR25L6403F
	0x16, 0xC8, 0x15, 0x35, 0x11, 0x31, 0x09,		// GD25Q64C
	0x16, 0x20, 0x95, 0x09, 0xC0, 0x00, 0x00,		// XM25QH64A
	0x16, 0x1C, 0x95, 0x05, 0xC0, 0x00, 0x00,		// EN25QH64A
	0x16, 0xEF, 0x15, 0x35, 0x11, 0x31, 0x09, 	// T25S64
	0x16, 0x5E, 0x15, 0x35, 0x11, 0x31, 0x09,   // ZB25VQ64
	0x16, 0x68, 0x15, 0x35, 0x11, 0x31, 0x09,   // BY25Q64AS
	0x17, 0xC2, 0x00, 0x00, 0x00, 0x00, 0x06,		// GPR25L12805F, KH25L12833F
	0x17, 0xC8, 0x15, 0x35, 0x11, 0x31, 0x09,		// GD25Q127C
	0x17, 0x20, 0x95, 0x09, 0xC0, 0x00, 0x00,		// XM25QH128A
	0x17, 0x1C, 0x95, 0x09, 0xC0, 0x00, 0x00,		// EN25QH128A
	0x17, 0x68, 0x15, 0x35, 0x11, 0x31, 0x09,		// BY25Q128AS
	0x18, 0xC2, 0x00, 0x00, 0x00, 0x00, 0x06,		// GPR25L25605F
	0x18, 0xC8, 0x15, 0x35, 0x11, 0x31, 0x06 		// GPR25L25606F
};


/*---------------------------------------------------------------------------------------
 * Gobal Variable Declaration Area
 *---------------------------------------------------------------------------------------*/
uint8_t mSpiFcTimingSetting = 0x60;
uint8_t mSpiFcReadStatusReg2;
uint8_t mSpiFcReadStatusReg3;
uint8_t mSpiFcWriteStatusReg2;
uint8_t mSpiFcWriteStatusReg3;
uint8_t mSpiFcQuadModeEnBit;

#if defined(__GNUC__)
extern unsigned int __SPIFC_Timing_Calibration_Data;  //in GPCM3_FM1.ld
#endif

/*---------------------------------------------------------------------------------------
 * Subroutine Area
 *---------------------------------------------------------------------------------------*/
/**
 * @brief
 *   Fine-tune SPIFC clock timing for SPI Flash and PCB.
 *   User must define SPIFC_CHECK_SUM_START_ADDRESS, SPIFC_CHECK_SUM_LENGTH, SPIFC_CHECK_SUM_VALUE in the SPIFC_GPCM1Fx.h.
 *   If user do not define the correct SPIFC_CHECK_SUM_VALUE in the SPIFC_GPCM1Fx.h, SPIFC_TimingFineTune() maybe not work normally.
 * @param
 *   None.
 * @return
 *   - 0  => pass
 *   - -1 => fail
 */
int32_t BootCode_SPIFC_TimingFineTune()
{
	uint32_t *ReadDataStartAddr;
	uint32_t ReadDataIndex;
	uint32_t iCount;
	uint32_t CheckSumValue;
	uint32_t CheckSum[16];
	int32_t ClkEdgeSelTemp1;
	int32_t ClkEdgeSelTemp2 = -1;
	int32_t TimingSmpDelayTemp1;
	int32_t TimingSmpDelayTemp2 = -1;
	int32_t TimingSmpDelayTemp3 = -1;
	int32_t TimingFineTuneResult = -1;
	int32_t PllClkFreq;
	int32_t SpiClkDiv;

	SMU->CACHE_CTRL &= ~SMU_CACHE_CTRL_CACHE_ENABLE;  //Cache disable
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	if(SPIFC_CHECK_SUM_VALUE != 0)
	{
		CheckSumValue = SPIFC_CHECK_SUM_VALUE;
	}
	else
	{
		SpiClkDiv = (SPIFC->CTRL2 & SPIFC_CTRL2_CLK_SEL_MSK);
		MODIFY_REG(SPIFC->CTRL2, SPIFC_CTRL2_CLK_SEL_MSK, SPIFC_CTRL2_CLK_SEL_HCLK_DIV_4);

		MODIFY_REG(SMU->SYSLOCK, SMU_SYSLOCK_UNLOCK_KEY_MSK, 0);
		MODIFY_REG(SMU->SYSLOCK, SMU_SYSLOCK_UNLOCK_KEY_MSK, SMU_SYSLOCK_UNLOCK_KEY1);
		MODIFY_REG(SMU->SYSLOCK, SMU_SYSLOCK_UNLOCK_KEY_MSK, SMU_SYSLOCK_UNLOCK_KEY2);	       // SMU nulock
		PllClkFreq = (ACU->PLL_CTRL & ACU_PLL_CTRL_SYS_DIVC_MSK);                              // Kim modified 2022.07.29
		MODIFY_REG(SMU->SYSLOCK, SMU_SYSLOCK_UNLOCK_KEY_MSK, 0);	                             // SMU lock
		PLL_ChangeClk(PLL_24D576M);

		ReadDataStartAddr = (uint32_t*)&__SPIFC_Timing_Calibration_Data;
		CheckSumValue = 0;
		for(ReadDataIndex = 0; ReadDataIndex < SPIFC_CHECK_SUM_LENGTH; ReadDataIndex++)
		{
			CheckSumValue += ReadDataStartAddr[ReadDataIndex];
		}
		PLL_ChangeClk(PllClkFreq);
		MODIFY_REG(SPIFC->CTRL2, SPIFC_CTRL2_CLK_SEL_MSK, SpiClkDiv);
	}

	for(ClkEdgeSelTemp1 = 0; ClkEdgeSelTemp1 <= SPIFC_TIMING_SMP_CLK_EDGE_SEL_2; ClkEdgeSelTemp1++)
	{
		if(TimingSmpDelayTemp3 >= (TimingSmpDelayTemp2 + 4))
		{
			mSpiFcTimingSetting = ClkEdgeSelTemp2 | (((TimingSmpDelayTemp2 + TimingSmpDelayTemp3) >> 1) << SPIFC_TIMING_SMP_DELAY_POS);
			MODIFY_REG(SPIFC->TIMING, (SPIFC_TIMING_SMP_DELAY_MSK | SPIFC_TIMING_SMP_CLK_EDGE_SEL_MSK), mSpiFcTimingSetting);

			if(SPIFC_CHECK_SUM_VALUE != 0)
			{
				TimingFineTuneResult = 0;
			}
			break;
		}

		ClkEdgeSelTemp2 = -1;
		TimingSmpDelayTemp2 = -1;
		TimingSmpDelayTemp3 = -1;
		MODIFY_REG(SPIFC->TIMING, SPIFC_TIMING_SMP_CLK_EDGE_SEL_MSK, (ClkEdgeSelTemp1 << SPIFC_TIMING_SMP_CLK_EDGE_SEL_POS));
		for(TimingSmpDelayTemp1 = 0; TimingSmpDelayTemp1 <= (SPIFC_TIMING_SMP_DELAY_15 >> SPIFC_TIMING_SMP_DELAY_POS); TimingSmpDelayTemp1++)
		{
	    MODIFY_REG(SPIFC->TIMING, SPIFC_TIMING_SMP_DELAY_MSK, (TimingSmpDelayTemp1 << SPIFC_TIMING_SMP_DELAY_POS));
	    ReadDataStartAddr = (uint32_t*) &__SPIFC_Timing_Calibration_Data;
			CheckSum[TimingSmpDelayTemp1] = 0;
			for(ReadDataIndex = 0; ReadDataIndex < SPIFC_CHECK_SUM_LENGTH; ReadDataIndex++)
			{
				CheckSum[TimingSmpDelayTemp1] += ReadDataStartAddr[ReadDataIndex];
			}
		}

		for(iCount = 0; iCount < 16; iCount++)
		{
		  if(CheckSum[iCount] == CheckSumValue)
		  {
		    if(TimingSmpDelayTemp2 == -1)
		    {
		      TimingSmpDelayTemp2 = iCount;
					ClkEdgeSelTemp2 = ClkEdgeSelTemp1;
			  }
			  else
			  {
				  TimingSmpDelayTemp3 = iCount;
			  }
			}
			else
			{
			  if(TimingSmpDelayTemp2 != -1)
				{
				  break;
				}
			}
		}
  }
	SMU->CACHE_CTRL |= SMU_CACHE_CTRL_CACHE_ENABLE;

	return TimingFineTuneResult;
}


/**
 * @brief
 *   Read Status Register
 * @param
 *   None.
 * @return
 *   SPI status register value
 */
uint16_t BootCode_SPIFC_ReadREMS()
{
	uint8_t SpiStatusRegister[2];

  BootCode_SPIFC_CmdAddrRx(SPIFC_FLASH_REMS, 0x000000, SpiStatusRegister, 2);

	return (*((uint16_t*) SpiStatusRegister));
}


/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void BootCode_SPIFC_Open()
{
    uint16_t l_FlashNum = sizeof(T_SPIFlash_REMS);
    uint16_t l_SPIFlashREMS, l_temp, l_index;

    SET_BIT(CLOCK->AHBCKEN, CLOCK_AHBCKEN_SPIFC_CLK_ENABLE);	                   // SPIFC clock enable
    MODIFY_REG(SPIFC->TIMING, (SPIFC_TIMING_SMP_DELAY_MSK | SPIFC_TIMING_SMP_CLK_EDGE_SEL_MSK), mSpiFcTimingSetting);
    SET_BIT(SPIFC->CTRL1, SPIFC_CTRL1_SPIFC_ENABLE);	                           // SPIFC enable
    BootCode_SPIFC_SetClkDiv(SPIFC_CLKSEL_HCLK_DIV4);
    l_SPIFlashREMS = BootCode_SPIFC_ReadREMS();
    BootCode_SPIFC_SetClkDiv(SPIFC_CLKSEL_HCLK_DIV1);

    for(l_index = 0; l_index < l_FlashNum; l_index += 7)
    {
        l_temp = (T_SPIFlash_REMS[l_index] << 8) | T_SPIFlash_REMS[l_index + 1];
        if(l_SPIFlashREMS == l_temp)
        {
          mSpiFcReadStatusReg2 = T_SPIFlash_REMS[l_index + 3];
          mSpiFcWriteStatusReg2 = T_SPIFlash_REMS[l_index + 5];
          mSpiFcReadStatusReg3 = T_SPIFlash_REMS[l_index + 2];
          mSpiFcWriteStatusReg3 = T_SPIFlash_REMS[l_index + 4];
          mSpiFcQuadModeEnBit = T_SPIFlash_REMS[l_index + 6];
          break;
        }
    }
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void BootCode_SPIFC_Close()
{
	SPIFC->CTRL0 = 0;
	SPIFC->CTRL1 = 0;
	// SPIFC->CTRL2 = 0;
	CLEAR_BIT(CLOCK->AHBCKEN, CLOCK_AHBCKEN_SPIFC_CLK_ENABLE);	                 // SPIFC clock disable
}

/**
 * @brief
 *
 * @param
 *   SpiClkDiv [in]:
 *    - SPIFC_CLKSEL_HCLK_DIV1, SPIFC_CLKSEL_HCLK_DIV2, SPIFC_CLKSEL_HCLK_DIV3, SPIFC_CLKSEL_HCLK_DIV4
 * @return
 *   None.
 */
void BootCode_SPIFC_SetClkDiv(uint32_t SpiClkDiv)
{
    MODIFY_REG(SPIFC->CTRL2, SPIFC_CTRL2_CLK_SEL_MSK, SpiClkDiv);
}

/**
 * @brief
 *
 * @param
 *   MultiIoMode [in]:
 *    - SPIFC_1IO_MODE, SPIFC_2IO_MODE, SPIFC_4IO_MODE, SPIFC_4IO_ENHANCE_MODE
 * @return
 *   None
 */
void BootCode_SPIFC_AutoMode(uint32_t MultiIoMode)
{
  uint16_t l_temp, l_QEtemp;
  uint8_t l_StatusReg[2];

	if((MultiIoMode == SPIFC_4IO_MODE) || (MultiIoMode == SPIFC_4IO_ENHANCE_MODE))
	{
    if(mSpiFcQuadModeEnBit != 0)                                               // QE bit exist
    {
      if(mSpiFcReadStatusReg2 == 0)
      {
        BootCode_SPIFC_CmdRx(SPIFC_FLASH_RDSR, l_StatusReg, 2);
      }
      else
      {
        BootCode_SPIFC_CmdRx(SPIFC_FLASH_RDSR, l_StatusReg, 1);
        BootCode_SPIFC_CmdRx(mSpiFcReadStatusReg2, &l_StatusReg[1], 1);
      }
      l_temp = (l_StatusReg[1] << 8) | l_StatusReg[0];
      l_QEtemp = 0x1 << mSpiFcQuadModeEnBit;
      if((l_temp & l_QEtemp) == 0)
      {
        l_temp |= l_QEtemp;
        l_StatusReg[0] = (uint8_t)l_temp;
        l_StatusReg[1] = (uint8_t)(l_temp >> 8);
        BootCode_SPIFC_WriteEnable();
        if(mSpiFcWriteStatusReg2 == 0)
        {
          BootCode_SPIFC_CmdTx(SPIFC_FLASH_WRSR, l_StatusReg, 2);
        }
        else
        {
          if(mSpiFcQuadModeEnBit > 7)
					{
            BootCode_SPIFC_CmdTx(mSpiFcWriteStatusReg2, &l_StatusReg[1], 1);
					}
          else
					{
            BootCode_SPIFC_CmdTx(mSpiFcWriteStatusReg2, &l_StatusReg[0], 1);
					}
        }
        while((BootCode_SPIFC_ReadSR() & SPIFC_SR_WIP_BIT) != 0);
      }
    }
	}

	SPIFC->CTRL1 &= ~SPIFC_CTRL1_SPIFC_ENABLE; 													         // SPIFC disable
	SPIFC->ADDRL = 0x0000;
	SPIFC->ADDRH = 0x0000;
	SPIFC->TX_BC = 0x0000;
	SPIFC->RX_BC = 0x0080;

  if(MultiIoMode == SPIFC_1IO_MODE)
  {
    SPIFC->PARA = SPIFC_PARA_ENHAN_DISABLE + SPIFC_PARA_DUMMY_CLK_0_CYCLE;
	  if((SPIFC->CTRL1 & SPIFC_CTRL1_4BYTES_ADDR_EN_MSK) == 0)
	  {
	    SPIFC->CMD = SPIFC_FLASH_READ;                                           // 1 x I/O read command for 3byte addr mode
	  }
	  else
	  {
		  SPIFC->CMD = SPIFC_FLASH_READ4B;                                         // 1 x I/O read command for 4byte addr mode
	  }
	  SPIFC->CTRL0 = SPIFC_CTRL0_MIO_1BIT + SPIFC_CTRL0_AUTO_MODE + SPIFC_CTRL0_CMIO_1BIT + SPIFC_CTRL0_AMIO_1BIT + SPIFC_CTRL0_IGNORE_LAST_CLK_DISABLE + SPIFC_CTRL0_CLK_STATE_LOW;
  }
  else if(MultiIoMode == SPIFC_2IO_MODE)
  {
    SPIFC->PARA = SPIFC_PARA_ENHAN_DISABLE + SPIFC_PARA_DUMMY_CLK_4_CYCLE;     // 2bit mio, 1bit cio, 2bit aio, 4 dummy cycle
	  if((SPIFC->CTRL1 & SPIFC_CTRL1_4BYTES_ADDR_EN_MSK) == 0)
	  {
	    SPIFC->CMD = SPIFC_FLASH_2READ;                                          // 2 x I/O read command for 3byte addr mode
	  }
	  else
	  {
		  SPIFC->CMD = SPIFC_FLASH_2READ4B;                                        // 2 x I/O read command for 4byte addr mode
	  }
	  SPIFC->CTRL0 = SPIFC_CTRL0_MIO_2BIT + SPIFC_CTRL0_AUTO_MODE + SPIFC_CTRL0_CMIO_1BIT + SPIFC_CTRL0_AMIO_2BIT + SPIFC_CTRL0_IGNORE_LAST_CLK_DISABLE + SPIFC_CTRL0_CLK_STATE_LOW;
  }
  else if(MultiIoMode == SPIFC_4IO_MODE)
  {
    SPIFC->PARA = SPIFC_PARA_DUMMY_CLK_6_CYCLE + SPIFC_PARA_ENHAN_DISABLE;
		if((SPIFC->CTRL1 & SPIFC_CTRL1_4BYTES_ADDR_EN_MSK) == 0)
	  {
	    SPIFC->CMD = SPIFC_FLASH_4READ;                                          // 4 x I/O read command for 3byte addr mode
	  }
	  else
	  {
		  SPIFC->CMD = SPIFC_FLASH_4READ4B;                                        // 4 x I/O read command for 4byte addr mode
	  }
	  SPIFC->CTRL0 = SPIFC_CTRL0_MIO_4BIT + SPIFC_CTRL0_AUTO_MODE + SPIFC_CTRL0_CMIO_1BIT + SPIFC_CTRL0_AMIO_4BIT + SPIFC_CTRL0_IGNORE_LAST_CLK_DISABLE + SPIFC_CTRL0_CLK_STATE_LOW;
    GPIOB->FST = 0;
  }
  else if(MultiIoMode == SPIFC_2IO_ENHANCE_MODE)
  {
    SPIFC->PARA = SPIFC_PARA_DUMMY_CLK_0_CYCLE + SPIFC_PARA_ENHAN_ENABLE + SPIFC_PARA_ENHANCE_BIT0 + SPIFC_PARA_ENHANCE_BIT2 + SPIFC_PARA_ENHANCE_BIT5 + SPIFC_PARA_ENHANCE_BIT7;
	  if((SPIFC->CTRL1 & SPIFC_CTRL1_4BYTES_ADDR_EN_MSK) == 0)
	  {
	    SPIFC->CMD = SPIFC_CMD_1ST_TRANS_WITHCMD_ENABLE | SPIFC_FLASH_2READ;     // 2 x I/O read command for 3byte addr mode
	  }
	  else
	  {
		  SPIFC->CMD = SPIFC_CMD_1ST_TRANS_WITHCMD_ENABLE | SPIFC_FLASH_2READ4B;   // 2 x I/O read command for 4byte addr mode
	  }
	  SPIFC->CTRL0 = SPIFC_CTRL0_MIO_2BIT + SPIFC_CTRL0_AUTO_MODE + SPIFC_CTRL0_CMIO_1BIT + SPIFC_CTRL0_AMIO_2BIT + SPIFC_CTRL0_IGNORE_LAST_CLK_DISABLE + SPIFC_CTRL0_CLK_STATE_LOW;
  }
  else if(MultiIoMode == SPIFC_4IO_ENHANCE_MODE)
  {
    SPIFC->PARA = SPIFC_PARA_DUMMY_CLK_4_CYCLE + SPIFC_PARA_ENHAN_ENABLE + SPIFC_PARA_ENHANCE_BIT0 + SPIFC_PARA_ENHANCE_BIT2 + SPIFC_PARA_ENHANCE_BIT5 + SPIFC_PARA_ENHANCE_BIT7;
	  if((SPIFC->CTRL1 & SPIFC_CTRL1_4BYTES_ADDR_EN_MSK) == 0)
	  {
	    SPIFC->CMD = SPIFC_CMD_1ST_TRANS_WITHCMD_ENABLE | SPIFC_FLASH_4READ;     // 4 x I/O read command for 3byte addr mode
	  }
	  else
	  {
		  SPIFC->CMD = SPIFC_CMD_1ST_TRANS_WITHCMD_ENABLE | SPIFC_FLASH_4READ4B;   // 4 x I/O read command for 4byte addr mode
	  }
	  SPIFC->CTRL0 = SPIFC_CTRL0_MIO_4BIT + SPIFC_CTRL0_AUTO_MODE + SPIFC_CTRL0_CMIO_1BIT + SPIFC_CTRL0_AMIO_4BIT + SPIFC_CTRL0_IGNORE_LAST_CLK_DISABLE + SPIFC_CTRL0_CLK_STATE_LOW;
	  GPIOB->FST = 0;
  }

	SPIFC->CTRL1 |= SPIFC_CTRL1_SPIFC_ENABLE; 													         // SPIFC enable
}

/**
 * @brief
 *   Enhance mode reset
 * @param
 *   None.
 * @return
 *   None.
 */
void BootCode_SPIFC_EnhanModeReset(void)
{
	uint32_t iCount;

	SPIFC->CTRL0 = SPIFC_CTRL0_MIO_1BIT + SPIFC_CTRL0_MANUAL_MODE + SPIFC_CTRL0_CMIO_1BIT + SPIFC_CTRL0_AMIO_1BIT + SPIFC_CTRL0_IGNORE_LAST_CLK_DISABLE + SPIFC_CTRL0_CLK_STATE_LOW;

	SPIFC->RX_BC = 0;
	SPIFC->TX_BC = 7;

	SPIFC->PARA = SPIFC_PARA_ENHAN_DISABLE | SPIFC_PARA_WITHOUT_ADDR_ENABLE | SPIFC_PARA_ONLY_ADDR_DISABLE | SPIFC_PARA_DUMMY_CLK_0_CYCLE;  // wo_enhan = 1, wo_addr = 1, to_addr = 0
	SPIFC->CMD = 0xFF | SPIFC_CMD_WITHOUTCMD_DISABLE | SPIFC_CMD_ONLYCMD_DISABLE | SPIFC_CMD_1ST_TRANS_WITHCMD_DISABLE;                     // wo_cmd = 0, cmd_only = 0, one_cmd = 0
	while((SPIFC->CTRL0 & SPIFC_CTRL0_PENDING_FLAG) == 0);

	for(iCount = 0; iCount < 7; iCount++)
	{
	  SPIFC->TX_DATA = 0xFF;
	  while((SPIFC->CTRL0 & SPIFC_CTRL0_TX_DONE_FLAG) == 0);
	}
}

/**
 * @brief
 *  Enable 4-bytes address mode.
 * @param
 *   None.
 * @return
 *   None.
 */
void BootCode_SPIFC_EN4B()
{
	BootCode_SPIFC_Cmd(SPIFC_FLASH_EN4B);
	MODIFY_REG(SPIFC->CTRL1, SPIFC_CTRL1_4BYTES_ADDR_EN_MSK, SPIFC_CTRL1_4BYTES_ADDR_ENABLE);
}

/**
 * @brief
 *   Exit 4-bytes address mode.
 * @param
 *   None.
 * @return
 *   None.
 */
void BootCode_SPIFC_EX4B()
{
	BootCode_SPIFC_Cmd(SPIFC_FLASH_EX4B);
	MODIFY_REG(SPIFC->CTRL1, SPIFC_CTRL1_4BYTES_ADDR_EN_MSK, SPIFC_CTRL1_4BYTES_ADDR_DISABLE);
}

/**
 * @brief
 *   Read SPI Flash ID
 * @param
 *   None.
 * @return
 *   SPI flash ID
 */
uint32_t BootCode_SPIFC_ReadID()
{
	uint8_t SpiFlashId[3];

  BootCode_SPIFC_CmdRx(SPIFC_FLASH_RDID, SpiFlashId, 3);

	return (*((uint32_t*) SpiFlashId) & 0x00FFFFFF);
}

/**
 * @brief
 *   SPI flash write enable
 * @param
 *   None.
 * @return
 *   None.
 */
void BootCode_SPIFC_WriteEnable()
{
  BootCode_SPIFC_Cmd(SPI_FLASH_WREN);
}

/**
 * @brief
 *   SPI flash write disable
 * @param
 *   None.
 * @return
 *   None.
 */
void BootCode_SPIFC_WriteDisable()
{
  BootCode_SPIFC_Cmd(SPIFC_FLASH_WRDI);
}

/**
 * @brief
 *   Write SPI flash status register
 * @param
 *   SpiStatusRegister [in]: SPI status register value
 * @return
 *   None.
 */
void BootCode_SPIFC_WriteSR(uint8_t SpiStatusRegister)
{
	BootCode_SPIFC_WriteEnable();
  BootCode_SPIFC_CmdTx(SPIFC_FLASH_WRSR, &SpiStatusRegister, 1);
	while((BootCode_SPIFC_ReadSR() & SPIFC_SR_WIP_BIT) != 0);
}

/**
 * @brief
 *   Read Status Register
 * @param
 *   None.
 * @return
 *   SPI status register value
 */
uint16_t BootCode_SPIFC_ReadSR()
{
	uint8_t SpiStatusRegister[1];

  BootCode_SPIFC_CmdRx(SPIFC_FLASH_RDSR, SpiStatusRegister, 1);

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
uint8_t BootCode_SPIFC_ReadCR()
{
	uint8_t SpiConfigRegister[1];

  BootCode_SPIFC_CmdRx(SPIFC_FLASH_RDCR, SpiConfigRegister, 1);

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
void BootCode_SPIFC_SectorErase(uint32_t SpiAddr)
{
	BootCode_SPIFC_WriteEnable();
  BootCode_SPIFC_CmdAddr(SPIFC_FLASH_SE, SpiAddr);
	while((BootCode_SPIFC_ReadSR() & SPIFC_SR_WIP_BIT) != 0);
}

/**
 * @brief
 *
 * @param
 *   SpiAddr [in]: SPI flash address
 * @return
 *
 */
void BootCode_SPIFC_BlockErase(uint32_t SpiAddr)
{
	BootCode_SPIFC_WriteEnable();
  BootCode_SPIFC_CmdAddr(SPIFC_FLASH_BE, SpiAddr);
	while((BootCode_SPIFC_ReadSR() & SPIFC_SR_WIP_BIT) != 0);
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
void BootCode_SPIFC_ReadNBytes(uint32_t SpiAddr, uint8_t *pDstBuf, uint32_t NumBytes)
{
	uint32_t DstBufIdx = 0;

  while(NumBytes != 0)
	{
		if(NumBytes > 256)
		{
	    BootCode_SPIFC_CmdAddrRx(SPIFC_FLASH_READ, SpiAddr, &pDstBuf[DstBufIdx], 256);
			SpiAddr += 256;
			DstBufIdx += 256;
			NumBytes -= 256;
		}
		else
		{
			BootCode_SPIFC_CmdAddrRx(SPIFC_FLASH_READ, SpiAddr, &pDstBuf[DstBufIdx], NumBytes);
			NumBytes = 0;
		}
	}
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
void BootCode_SPIFC_WriteNBytes(uint32_t DstSpiAddr, uint8_t *pSrcBuf, uint32_t NumBytes)
{
	uint32_t DstBufIdx = 0;
	uint32_t PageEmptyBytes;

  while(NumBytes != 0)
	{
		BootCode_SPIFC_WriteEnable();
		PageEmptyBytes = 256 - (DstSpiAddr & BYTE0_MSK);
		if(NumBytes < PageEmptyBytes)
		{
			BootCode_SPIFC_CmdAddrTx(SPIFC_FLASH_PP, DstSpiAddr, &pSrcBuf[DstBufIdx], NumBytes);
			NumBytes = 0;
			while((BootCode_SPIFC_ReadSR() & SPIFC_SR_WIP_BIT) != 0);
		}
		else
		{
	    BootCode_SPIFC_CmdAddrTx(SPIFC_FLASH_PP, DstSpiAddr, &pSrcBuf[DstBufIdx], PageEmptyBytes);
			DstSpiAddr += PageEmptyBytes;
			DstBufIdx += PageEmptyBytes;
			NumBytes -= PageEmptyBytes;
			while((BootCode_SPIFC_ReadSR() & SPIFC_SR_WIP_BIT) != 0);
		}
	}
}

/**
 * @brief
 *
 * @param
 *   SpiCmd [in]: SPI flash command
 * @return
 *   None.
 */
void BootCode_SPIFC_Cmd(uint8_t SpiCmd)
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
void BootCode_SPIFC_CmdAddr(uint8_t SpiCmd, uint32_t SpiAddr)
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
void BootCode_SPIFC_CmdRx(uint8_t SpiCmd, uint8_t *pDstBuf, uint32_t NumBytes)
{
	uint32_t iCount;
	uint32_t jCount;
	uint32_t SpiRxTemp;

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
void BootCode_SPIFC_CmdAddrRx(uint8_t SpiCmd, uint32_t SpiAddr, uint8_t *pDstBuf, uint32_t NumBytes)
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
void BootCode_SPIFC_CmdTx(uint8_t SpiCmd, uint8_t *pSrcBuf, uint32_t NumBytes)
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
void BootCode_SPIFC_CmdAddrTx(uint8_t SpiCmd, uint32_t SpiAddr, uint8_t *pSrcBuf, uint32_t NumBytes)
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
