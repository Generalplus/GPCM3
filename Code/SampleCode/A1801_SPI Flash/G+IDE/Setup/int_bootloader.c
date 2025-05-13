/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include <stdio.h>
#include "GPCM2_CM3.h"
#include "Serial.h"

/*********************************************************************
 *********************************************************************/
typedef struct
{
	__IO uint32_t HEADER_TAG;							      //
	__IO uint8_t *BOOTCODE_LMA;							      //
	__IO uint8_t *BOOTCODE_VMA;							      //
	__IO uint32_t BOOTCODE_SIZE;				            //
	__IO uint32_t BOOTCODE_CHECKSUM;							    			//
	__IO uint32_t BODYOPT_CHECKSUM;							    			//
	__IO uint32_t EncRawData[58];
} BOOT_HEADER_DEF;

enum BootCodeMode
{
  UART_MODE = 1,
  TEST_MODE,
  SPI0_SLAVE_MODE,
  SPIFC_AUTO_MODE
};

#define MODE_Flag_REG         ((uint32_t *)(0x50000020))
#define ICE_MODE_Flag         0x00000001
#define WRITER_MODE_Flag      0x00000002
#define TEST_MODE_Flag        0x00000004

const uint8_t T_ShiftTable[] =
{
  16,20,24,12,28,
  16,20,28,12,24,
  16,24,28,12,20,
  20,24,28,12,16,
  12,20,24,16,28,
  12,20,28,16,24,
  12,24,28,16,20,
  12,16,24,20,28,
  12,16,28,20,24,
  12,16,20,24,28
};

const uint8_t T_SPIFCSetting[] =
{
  2,3,4,1,5,
  2,4,3,1,5,
  3,2,4,1,5,
  3,4,2,1,5,
  4,2,3,1,5,
  4,3,2,1,5
};

void BodyOptDecProcess(BOOT_HEADER_DEF* pBootTable);
void ResetProcess(void);
void EnterPowerDownMode(void);
void GPCM3800_TestMode(void);
uint32_t SPI0_Flash_ReadID(void);
void UART0_DMA_Enable(void);
void SPI0_Flash_ReadNBytes(uint8_t *pBuffer, uint32_t ReadAddr, uint32_t NumByteToRead);

void BootLoaderProcess()
{
  uint32_t iCount,jCount;
  uint8_t temp;
  uint32_t *RamStrAddr = (uint32_t *)0x20000000;
  uint32_t *FlashStrAddr = (uint32_t *)0x04000000;
  uint16_t RetryCnt = 256;

  SET_BIT(CLOCK->AHBCKEN, CLOCK_AHBCKEN_GPIO_CLK_ENABLE + CLOCK_AHBCKEN_DMA_CLK_ENABLE);
  //DMA Enable
  DMA_INT->INTSTS = DMA_INTSTS_DMA0_DONE_INT_FLAG;
  DMA0->DSTADR = 0x20000000;
  DMA0->DTN = 0x100;  //256 bytes for boot table

  if(MODE_Flag_REG[0] & WRITER_MODE_Flag)  //UART mode
  {
    RamStrAddr[64] = UART_MODE;
    SET_BIT(CLOCK->APBCKEN, CLOCK_APBCKEN_UART0_CLK_ENABLE);	//	APB UART0 Clock enable
    MODIFY_REG(IOFUNC->CTRL0, GPIOFUNC_CTRL0_ICE_EN_MSK + GPIOFUNC_CTRL0_UART0_IOSEL_MSK, GPIOFUNC_CTRL0_ICE_DISABLE + GPIOFUNC_CTRL0_UART0_IOC2_RST);
    //MODIFY_REG(IOFUNC->CTRL0, IOFUNC_CTRL0_UART0_IOSEL_MSK, IOFUNC_CTRL0_UART0_IOA13_14);
    UART0->CTRL = (UART_CTRL_UART0_FIFO_ENABLE | UART_CTRL_UART_ENABLE | UART_CTRL_UART_RX_ENABLE);                                                     // Initialize UART Controller
    CLEAR_FLAG(UART0->STS, (UART_STS_RX_IDLE_MSK | UART_STS_TX_DONE_MSK | UART_STS_RX_BRK_MSK), (UART_STS_RX_IDLE_FLAG | UART_STS_TX_DONE_FLAG | UART_STS_RX_BRK_FLAG));	  // Reset UART status
    UART0->BAUD_RATE = 0x00130007;  //Baud rate 115200 in 12.288MHz system clock

    SET_BIT(UART0->CTRL, UART_CTRL_UART_TX_ENABLE);             // Enable Uart Tx
    UART0->DATA = 'U';
    UART0->DATA = 'A';
    UART0->DATA = 'R';
    UART0->DATA = 'T';
    while(READ_BIT(UART0->STS, UART_STS_TX_FIFO_EMPTY_FLAG) == 0);
    while(READ_BIT(UART0->STS, UART_STS_TX_DONE_FLAG) == 0);     // Wait Tx empty and Time-out manner

    CLEAR_FLAG(UART0->STS, UART_STS_TX_DONE_MSK, UART_STS_TX_DONE_FLAG);
    CLEAR_BIT(UART0->CTRL, UART_CTRL_UART_TX_ENABLE);             // Disable Uart Tx

    DMA0->SRCADR = (uint32_t)&UART0->DATA;
    DMA0->CTRL = DMA_CTRL_DMA_ENABLE | DMA_CTRL_DMA_REQSEL_UART0 | DMA_CTRL_DMA_SRCSIZE_8B | DMA_CTRL_DMA_DSTSIZE_8B | DMA_CTRL_DMA_SRCINC_DISABLE | DMA_CTRL_DMA_DSTINC_ENABLE;
    UART0->CTRL |= UART_CTRL_RX_DMA_ENABLE;
    //while(DMA0->CTRL & DMA_CTRL_DMA_BUSY_FLAG) WDG->KEYCODE = WDG_KEYCODE_WDG_CLEAR;
    WDG->KEYCODE = WDG_KEYCODE_WDG_CLEAR;
    while(DMA0->CTRL & DMA_CTRL_DMA_BUSY_FLAG)
    {
      if((UART0->STS & UART_STS_RX_IDLE_MSK) == 0)
      {
        WDG->KEYCODE = WDG_KEYCODE_WDG_CLEAR;
      }
    }
    CLEAR_FLAG(UART0->STS, UART_STS_RX_DAT_NEMP_MSK, UART_STS_RX_DAT_NEMP_FLAG);
    UART0->CTRL &= ~UART_CTRL_RX_DMA_ENABLE;
  }
  else if(MODE_Flag_REG[0] & TEST_MODE_Flag) //TEST mode
  {
    RamStrAddr[64] = TEST_MODE;
    GPCM3800_TestMode();
  }
  else //SPIFC mode
  {
    WRITE_REG(GPIOB->OBUF, BIT5 + BIT4 + BIT3 + BIT2 + BIT1 + BIT0); ////Set all of SPIFC pin to output high and clk pin to pull high(IOB1).
    //WRITE_REG(GPIOB->CFG0, 0x00000FFF); // Set all of SPIFC pin to output high.
    WRITE_REG(GPIOB->CFG0, 0x00000000); // Set all of SPIFC pin to input pull high.
    MODIFY_REG(SPIFC->TIMING, (SPIFC_TIMING_SMP_DELAY_MSK | SPIFC_TIMING_SMP_CLK_EDGE_SEL_MSK), 0x00); //Need to confirm
    WDG->KEYCODE = WDG_KEYCODE_WDG_CLEAR;
    while((GPIOB->IDATA & 0x0000003F) == 0); // Wait CS Pin to high level.
    for(iCount=0;iCount<512;iCount++)
    {
      if((GPIOB->IDATA & (BIT4 + BIT3 + BIT2 + BIT1)) == BIT3)  // SPI0 Slave mode
      {
        RamStrAddr[64] = SPI0_SLAVE_MODE;
      }
      else
      {
        RamStrAddr[64] = SPIFC_AUTO_MODE;
        break;
      }
    }
    if(RamStrAddr[64] == SPI0_SLAVE_MODE)  // SPI0 Slave mode
    {
      SET_BIT(CLOCK->APBCKEN, CLOCK_APBCKEN_SPI0_CLK_ENABLE);
      WRITE_REG(GPIOB->CFG0, 0x515); //Set CS pin to input pull low, and the other pin to floating
      WRITE_REG(GPIOB->OBUF, 0);
      MODIFY_REG(IOFUNC->CTRL0, GPIOFUNC_CTRL0_SPI0_IOSEL_MSK + GPIOFUNC_CTRL0_SPI0_HWCS_EN_MSK, GPIOFUNC_CTRL0_SPI0_IOB1_4 + GPIOFUNC_CTRL0_SPI0_HWCS_ENABLE);
      WRITE_REG(SPI0->CTRL, SPI_CTRL_SLAVE_MODE + SPI_CTRL_MAUT_RW_EN_ENABLE + SPI_CTRL_LATCH_SEL_DLY_1 + SPI_CTRL_SPI_ENABLE);

      DMA0->SRCADR = (uint32_t)&SPI0->RX_DATA;
      DMA0->CTRL = DMA_CTRL_DMA_ENABLE | DMA_CTRL_DMA_REQSEL_SPI0 | DMA_CTRL_DMA_SRCSIZE_8B | DMA_CTRL_DMA_DSTSIZE_8B | DMA_CTRL_DMA_SRCINC_DISABLE | DMA_CTRL_DMA_DSTINC_ENABLE;
      SPI0->CTRL |= SPI_CTRL_RX_DMA_ENABLE;
      //while(DMA0->CTRL & DMA_CTRL_DMA_BUSY_FLAG) WDG->KEYCODE = WDG_KEYCODE_WDG_CLEAR;
      WDG->KEYCODE = WDG_KEYCODE_WDG_CLEAR;
      while(DMA0->CTRL & DMA_CTRL_DMA_BUSY_FLAG);
      WDG->KEYCODE = WDG_KEYCODE_WDG_CLEAR;
      SPI0->CTRL &= ~SPI_CTRL_RX_DMA_ENABLE;
    }
    else  // SPIFC Auto mode
    {
      SPIFC->CTRL0 = 0;
      SPIFC->CTRL1 = 0;
      SET_BIT(CLOCK->AHBCKEN, CLOCK_AHBCKEN_SPIFC_CLK_ENABLE);
      SET_BIT(SPIFC->CTRL1, SPIFC_CTRL1_SPIFC_ENABLE);
      MODIFY_REG(SPIFC->SCRAMBLE_CTRL, SPIFC_SCRAMBLE_CTRL_DEC_EN_MSK + SPIFC_SCRAMBLE_CTRL_ENC_EN_MSK, SPIFC_SCRAMBLE_CTRL_DEC_DISABLE + SPIFC_SCRAMBLE_CTRL_ENC_DISABLE);
      //MODIFY_REG(SPIFC->CTRL2, SPIFC_CTRL2_CLK_SEL_MSK, SPIFC_CLKSEL_HCLK_DIV4);
      iCount = 0;
      jCount = 0;
      while(RetryCnt != 0)
      {
        SPIFC->CTRL0 = SPIFC_CTRL0_MIO_1BIT + SPIFC_CTRL0_MANUAL_MODE + SPIFC_CTRL0_CMIO_1BIT + SPIFC_CTRL0_AMIO_1BIT + SPIFC_CTRL0_IGNORE_LAST_CLK_DISABLE + SPIFC_CTRL0_CLK_STATE_LOW;
        WDG->KEYCODE = WDG_KEYCODE_WDG_CLEAR;
        IOFUNC->SPIFC_CFG = 0;
        IOFUNC->SPIFC_CFG |= ((uint32_t)T_SPIFCSetting[jCount] << T_ShiftTable[iCount]);
        IOFUNC->SPIFC_CFG |= ((uint32_t)T_SPIFCSetting[jCount+1] << T_ShiftTable[iCount+1]);
        IOFUNC->SPIFC_CFG |= ((uint32_t)T_SPIFCSetting[jCount+2] << T_ShiftTable[iCount+2]);
        IOFUNC->SPIFC_CFG |= ((uint32_t)T_SPIFCSetting[jCount+3] << T_ShiftTable[iCount+3]);
        IOFUNC->SPIFC_CFG |= ((uint32_t)T_SPIFCSetting[jCount+4] << T_ShiftTable[iCount+4]);

        //Exit enhance mode
        SPIFC->RX_BC = 0;
        SPIFC->TX_BC = 1;
        SPIFC->PARA = SPIFC_PARA_ENHAN_DISABLE | SPIFC_PARA_WITHOUT_ADDR_ENABLE | SPIFC_PARA_ONLY_ADDR_DISABLE | SPIFC_PARA_DUMMY_CLK_0_CYCLE;  // wo_enhan = 1, wo_addr = 1, to_addr = 0
        SPIFC->CMD = 0xFF | SPIFC_CMD_WITHOUTCMD_DISABLE | SPIFC_CMD_ONLYCMD_DISABLE | SPIFC_CMD_1ST_TRANS_WITHCMD_DISABLE;                     // wo_cmd = 0, cmd_only = 0, one_cmd = 0
        while((SPIFC->CTRL0 & SPIFC_CTRL0_PENDING_FLAG) == 0);
        SPIFC->TX_DATA = 0xFF;
        while((SPIFC->CTRL0 & SPIFC_CTRL0_TX_DONE_FLAG) == 0);

        //Releae deep power down
        SPIFC->RX_BC = 1;
        SPIFC->TX_BC = 0;
        SPIFC->PARA = SPIFC_PARA_ENHAN_DISABLE | SPIFC_PARA_WITHOUT_ADDR_DISABLE | SPIFC_PARA_ONLY_ADDR_DISABLE | SPIFC_PARA_DUMMY_CLK_0_CYCLE;  // wo_enhan = 1, wo_addr = 0, to_addr = 0
        SPIFC->CMD = 0xAB | SPIFC_CMD_WITHOUTCMD_DISABLE | SPIFC_CMD_ONLYCMD_DISABLE | SPIFC_CMD_1ST_TRANS_WITHCMD_DISABLE;                    // wo_cmd = 0, cmd_only = 0, one_cmd = 0
        while((SPIFC->CTRL0 & SPIFC_CTRL0_PENDING_FLAG) == 0);
        SPIFC->RX_DATA = 0;																	                       // write dummy before read operation.
        while((SPIFC->CTRL0 & SPIFC_CTRL0_PENDING_FLAG) == 0);
        temp = SPIFC->RX_DATA;
        if((temp >= 0x10) && (temp <= 0x20)) //Read ID pass
        {
          for(temp = 0; temp < 70; temp++); // delay for release deep power down

          // Exit SPI Addr 4Bytes mode
          SPIFC->CMD = 0xE9 | SPIFC_CMD_WITHOUTCMD_DISABLE | SPIFC_CMD_ONLYCMD_ENABLE | SPIFC_CMD_1ST_TRANS_WITHCMD_DISABLE;                    // wo_cmd = 0, cmd_only = 1, one_cmd = 0
          while((SPIFC->CTRL0 & SPIFC_CTRL0_PENDING_FLAG) == 0);

          // Enable SPIFC 1IO auto mode
          SPIFC->CTRL1 &= ~SPIFC_CTRL1_SPIFC_ENABLE; 													         // SPIFC disable
          SPIFC->ADDRL = 0x0000;
          SPIFC->ADDRH = 0x0000;
          SPIFC->TX_BC = 0x0000;
          SPIFC->RX_BC = 0x0080;

          SPIFC->PARA = SPIFC_PARA_ENHAN_DISABLE + SPIFC_PARA_DUMMY_CLK_0_CYCLE;
          SPIFC->CMD = SPIFC_FLASH_READ;
          SPIFC->CTRL0 = SPIFC_CTRL0_MIO_1BIT + SPIFC_CTRL0_AUTO_MODE + SPIFC_CTRL0_CMIO_1BIT + SPIFC_CTRL0_AMIO_1BIT + SPIFC_CTRL0_IGNORE_LAST_CLK_DISABLE + SPIFC_CTRL0_CLK_STATE_LOW;
          SPIFC->CTRL1 |= SPIFC_CTRL1_SPIFC_ENABLE;

          if(FlashStrAddr[0] == 0x58334D43) //Header tag
          {
            DMA0->SRCADR = (uint32_t)FlashStrAddr;
            DMA0->CTRL = DMA_CTRL_DMA_ENABLE | DMA_CTRL_DMA_REQSEL_MEM | DMA_CTRL_DMA_SRCSIZE_8B | DMA_CTRL_DMA_DSTSIZE_8B | DMA_CTRL_DMA_SRCINC_ENABLE | DMA_CTRL_DMA_DSTINC_ENABLE | DMA_CTRL_DMA_BURST_ENABLE;
            DMA0->CTRL |= DMA_CTRL_DMA_START;
            while(DMA0->CTRL & DMA_CTRL_DMA_BUSY_FLAG) WDG->KEYCODE = WDG_KEYCODE_WDG_CLEAR;
            DMA0->CTRL &= ~DMA_CTRL_DMA_START;
            break;
          }
        }
        jCount += 5;
        if(jCount == sizeof(T_SPIFCSetting))
        {
          jCount = 0;
          iCount += 5;
          if(iCount == sizeof(T_ShiftTable))
          {
            iCount = 0;
            RetryCnt -= 1;
          }
        }
      }
      if(RetryCnt == 0)
      {
        EnterPowerDownMode();
      }
    }
  }
  BodyOptDecProcess((BOOT_HEADER_DEF*)RamStrAddr);
}

void ReadBootCodeProcess()
{
  uint32_t *RamStrAddr = (uint32_t *)0x20000000;
  uint32_t iCount,checksum,BootCodeMode;
  uint32_t BOOTCODE_LMA, BOOTCODE_SIZE, BOOTCODE_CHECKSUM;
  uint8_t *BOOTCODE_VMA;

  BOOTCODE_LMA = RamStrAddr[1];
  BOOTCODE_VMA = (uint8_t*)RamStrAddr[2];
  BOOTCODE_SIZE = RamStrAddr[3];
  BOOTCODE_CHECKSUM = RamStrAddr[4];
  BootCodeMode = RamStrAddr[64];

  if(RamStrAddr[0] != 0x58334D43)
  {
    if(BootCodeMode == UART_MODE)
    {
      SET_BIT(UART0->CTRL, UART_CTRL_UART_TX_ENABLE);             // Disable Uart Tx
      UART0->DATA = 0xFF;
      while(READ_BIT(UART0->STS, UART_STS_TX_DONE_FLAG) == 0);     // Wait Tx empty and Time-out manner
      CLEAR_FLAG(UART0->STS, UART_STS_TX_DONE_MSK, UART_STS_TX_DONE_FLAG);
      CLEAR_BIT(UART0->CTRL, UART_CTRL_UART_TX_ENABLE);             // Disable Uart Tx
    }
    ResetProcess();
  }
  DMA0->CTRL &= ~DMA_CTRL_DMA_ENABLE;
  DMA_INT->INTSTS = DMA_INTSTS_DMA0_DONE_INT_FLAG;
  DMA0->DSTADR = (uint32_t)BOOTCODE_VMA;
  DMA0->DTN = BOOTCODE_SIZE;
  DMA0->CTRL |= DMA_CTRL_DMA_ENABLE;

  switch(BootCodeMode)
  {
    case UART_MODE: //UART mode
      // Send Boot code size out
      SET_BIT(UART0->CTRL, UART_CTRL_UART_TX_ENABLE);             // Disable Uart Tx
      UART0->DATA = BOOTCODE_SIZE & 0x000000FF;
      UART0->DATA = (BOOTCODE_SIZE >> 8) & 0x000000FF;
      UART0->DATA = (BOOTCODE_SIZE >> 16) & 0x000000FF;
      UART0->DATA = (BOOTCODE_SIZE >> 24) & 0x000000FF;
      while(READ_BIT(UART0->STS, UART_STS_TX_FIFO_EMPTY_FLAG) == 0);
      while(READ_BIT(UART0->STS, UART_STS_TX_DONE_FLAG) == 0);     // Wait Tx empty and Time-out manner
      CLEAR_FLAG(UART0->STS, UART_STS_TX_DONE_MSK, UART_STS_TX_DONE_FLAG);
      CLEAR_BIT(UART0->CTRL, UART_CTRL_UART_TX_ENABLE);             // Disable Uart Tx

      // Received boot code
      UART0->CTRL |= UART_CTRL_RX_DMA_ENABLE;
      //while(DMA0->CTRL & DMA_CTRL_DMA_BUSY_FLAG) WDG->KEYCODE = WDG_KEYCODE_WDG_CLEAR;
      WDG->KEYCODE = WDG_KEYCODE_WDG_CLEAR;
      while(DMA0->CTRL & DMA_CTRL_DMA_BUSY_FLAG)
      {
        if((UART0->STS & UART_STS_RX_IDLE_MSK) == 0)
        {
          WDG->KEYCODE = WDG_KEYCODE_WDG_CLEAR;
        }
      }
      CLEAR_FLAG(UART0->STS, UART_STS_RX_DAT_NEMP_MSK, UART_STS_RX_DAT_NEMP_FLAG);
      UART0->CTRL &= ~UART_CTRL_RX_DMA_ENABLE;
    break;

    case TEST_MODE: //Test mode
    break;

    case SPI0_SLAVE_MODE: //SPI0 Slave mode
      // Received Boot header
      WRITE_REG(GPIOB->OBUF, BIT3); //Set CS Pin to input pull high
      SPI0->CTRL |= SPI_CTRL_RX_DMA_ENABLE;
      while(DMA0->CTRL & DMA_CTRL_DMA_BUSY_FLAG) WDG->KEYCODE = WDG_KEYCODE_WDG_CLEAR;
      SPI0->CTRL &= ~SPI_CTRL_RX_DMA_ENABLE;
      WRITE_REG(GPIOB->OBUF, 0); //Set CS Pin to input pull low
    break;

    case SPIFC_AUTO_MODE:
      DMA0->SRCADR = BOOTCODE_LMA;
      DMA0->CTRL |= DMA_CTRL_DMA_START;
      while(DMA0->CTRL & DMA_CTRL_DMA_BUSY_FLAG) WDG->KEYCODE = WDG_KEYCODE_WDG_CLEAR;
      DMA0->CTRL &= ~DMA_CTRL_DMA_START;
    break;

    default:
    break;
  }
  for(iCount = 0, checksum = 0; iCount < BOOTCODE_SIZE; iCount++)
  {
    checksum += BOOTCODE_VMA[iCount];
  }
  if(checksum != BOOTCODE_CHECKSUM)
  {
    if(BootCodeMode == UART_MODE)
    {
      SET_BIT(UART0->CTRL, UART_CTRL_UART_TX_ENABLE);             // Disable Uart Tx
      UART0->DATA = 0xFF;
      while(READ_BIT(UART0->STS, UART_STS_TX_DONE_FLAG) == 0);     // Wait Tx empty and Time-out manner
      CLEAR_FLAG(UART0->STS, UART_STS_TX_DONE_MSK, UART_STS_TX_DONE_FLAG);
      CLEAR_BIT(UART0->CTRL, UART_CTRL_UART_TX_ENABLE);             // Disable Uart Tx
    }
    ResetProcess();
  }
  if(BootCodeMode == UART_MODE)
  {
    SET_BIT(UART0->CTRL, UART_CTRL_UART_TX_ENABLE);             // Disable Uart Tx
    UART0->DATA = checksum & 0x000000FF;
    UART0->DATA = (checksum >> 8) & 0x000000FF;
    UART0->DATA = (checksum >> 16) & 0x000000FF;
    UART0->DATA = (checksum >> 24) & 0x000000FF;
    while(READ_BIT(UART0->STS, UART_STS_TX_FIFO_EMPTY_FLAG) == 0);
    while(READ_BIT(UART0->STS, UART_STS_TX_DONE_FLAG) == 0);     // Wait Tx empty and Time-out manner
    CLEAR_FLAG(UART0->STS, UART_STS_TX_DONE_MSK, UART_STS_TX_DONE_FLAG);
    CLEAR_BIT(UART0->CTRL, UART_CTRL_UART_TX_ENABLE);             // Disable Uart Tx
  }
}


void BodyOptDecProcess(BOOT_HEADER_DEF* pBootTable)
{
  uint32_t iCount;
  uint32_t checksum;
  uint32_t VirtualBodyTag_0;
  uint32_t VirtualBodyTag_1;
  uint32_t LastValue, u32_temp;
  //uint32_t* pBodyOpt = (uint32_t *)&BODYOPT->BODYOPT_0;
  uint32_t* pBodyOpt = (uint32_t *)0x50005044;

  MODIFY_REG(BODYOPT->BODYOPTLOCK, BODYOPT_OPTLOCK_UNLOCK_KEY_MSK, BODYOPT_OPTLOCK_UNLOCK_KEY1);
  MODIFY_REG(BODYOPT->BODYOPTLOCK, BODYOPT_OPTLOCK_UNLOCK_KEY_MSK, BODYOPT_OPTLOCK_UNLOCK_KEY2);	       // OPT unlock

  for(iCount=0,checksum=0,LastValue = pBootTable->HEADER_TAG;iCount<16;iCount++)
  {
    u32_temp = pBootTable->EncRawData[iCount] ^ pBootTable->BODYOPT_CHECKSUM ^ LastValue;
    pBodyOpt[iCount] = u32_temp;
    checksum += u32_temp;
    LastValue = pBootTable->EncRawData[iCount];
  }
  MODIFY_REG(BODYOPT->BODYOPTLOCK, BODYOPT_OPTLOCK_UNLOCK_KEY_MSK, 0);  //lock
  VirtualBodyTag_0 = pBootTable->EncRawData[iCount] ^ pBootTable->BODYOPT_CHECKSUM ^ LastValue;
  checksum += VirtualBodyTag_0;
  LastValue = pBootTable->EncRawData[iCount++];
  VirtualBodyTag_1 = pBootTable->EncRawData[iCount] ^ pBootTable->BODYOPT_CHECKSUM ^ LastValue;
  checksum += VirtualBodyTag_1;

  if(checksum != pBootTable->BODYOPT_CHECKSUM)
  {
    ResetProcess();
  }

  //cmpare virtual tag with EFUSE->EFUSE_A
  iCount = 0x00000001 << 25;
  while(iCount != 0)
  {
    if(EFUSE->EFUSE_A & iCount)
    {
      if(VirtualBodyTag_0 & iCount)
      {
        ResetProcess();
      }
    }
    iCount = iCount << 1;
  }
  MODIFY_REG(SMU->SYSLOCK, SMU_SYSLOCK_UNLOCK_KEY_MSK, SMU_SYSLOCK_UNLOCK_KEY1);
	MODIFY_REG(SMU->SYSLOCK, SMU_SYSLOCK_UNLOCK_KEY_MSK, SMU_SYSLOCK_UNLOCK_KEY2);	       // SMU nulock
  ACU->PLL_CTRL = pBootTable->EncRawData[18];
  if(ACU->PLL_CTRL & ACU_PLL_CTRL_PLL_ENABLE)
  {
    MODIFY_REG(CLOCK->AHBCKSEL, CLOCK_AHBCKSEL_HCLK_SEL_MSK, (HCLK_SEL_PLL & CLOCK_AHBCKSEL_HCLK_SEL_MSK));
  }
  GPIOB->DRV = pBootTable->EncRawData[19];
  SPIFC->CTRL2 = pBootTable->EncRawData[20];
  SPIFC->TIMING = pBootTable->EncRawData[21];
  UART0->BAUD_RATE = pBootTable->EncRawData[22];
  MODIFY_REG(SMU->SYSLOCK, SMU_SYSLOCK_UNLOCK_KEY_MSK, 0);

  return;
}


void ResetProcess(void)
{
  MODIFY_REG(SMU->SYSLOCK, SMU_SYSLOCK_UNLOCK_KEY_MSK, SMU_SYSLOCK_UNLOCK_KEY1);
  MODIFY_REG(SMU->SYSLOCK, SMU_SYSLOCK_UNLOCK_KEY_MSK, SMU_SYSLOCK_UNLOCK_KEY2);	       // SMU nulock
  RCU->CTRL = RCU_CTRL_MRST_EN_TRIGGER;
}


#define TEST_Mode_Sel         (0x0000001F << 3)     // 0x50000020[7:3]
#define TEST_NormalSleep      (0x00000001 << 3)
#define TEST_DeepSleep        (0x00000002 << 3)
#define TEST_ShutDown         (0x00000003 << 3)
#define TEST_GPIO             (0x00000004 << 3)
#define TEST_DumpROM          (0x00000005 << 3)
#define UART_DumpROM          (0x00000006 << 3)
#define UART_DumpFuse         (0x00000007 << 3)
#define UART_DumpFlashSPIFC   (0x00000008 << 3)
#define UART_DumpFlashSPI0    (0x00000009 << 3)
#define UART_DumpFlashID      (0x000000010 << 3)

#define REG_0x50005004          ((uint32_t *)(0x50005004))
#define REG_0x50005044          ((uint32_t *)(0x50005044))
#define REG_0x50005058          ((uint32_t *)(0x50005058))
#define InternalROM             ((uint32_t *)(0x00000000))
#define InternalROM_Byte        ((uint8_t *)(0x00000000))

void GPCM3800_TestMode(void)
{
    uint8_t Test_Cnt;
    uint32_t iCount;
    uint32_t R_IOA_Data;
    uint32_t R_IOB_Data;
    uint32_t R_IOC_Data;
    uint32_t R_IOD_Data;
    uint32_t ROMData;
    uint32_t temp;
    uint8_t *Ptr;
    uint8_t FlashData[256];

    WRITE_REG(GPIOA->CFG0, 0x0000000F);
    WRITE_REG(GPIOA->CFG1, 0x00000000);
    WRITE_REG(GPIOA->OBUF, 0x00000000);
    WRITE_REG(GPIOB->CFG0, 0x00000000);
    WRITE_REG(GPIOB->OBUF, 0x00000000);
    WRITE_REG(GPIOC->CFG0, 0x00000000);
    WRITE_REG(GPIOC->OBUF, 0x00000000);
    WRITE_REG(GPIOD->CFG0, 0x00000000);
    WRITE_REG(GPIOD->OBUF, 0x00000000);

    switch(MODE_Flag_REG[0] & TEST_Mode_Sel)
    {
        case TEST_NormalSleep:
            SET_BIT(GPIOA->OBUF, 0x00000001);
            READ_REG(GPIOA->IDATA);
            READ_REG(GPIOB->IDATA);
            READ_REG(GPIOC->IDATA);
            READ_REG(GPIOD->IDATA);
            __WFI();
            __NOP();
            __NOP();
            break;

        case TEST_DeepSleep:
            MODIFY_REG(BODYOPT->BODYOPTLOCK, BODYOPT_OPTLOCK_UNLOCK_KEY_MSK, BODYOPT_OPTLOCK_UNLOCK_KEY1);
            MODIFY_REG(BODYOPT->BODYOPTLOCK, BODYOPT_OPTLOCK_UNLOCK_KEY_MSK, BODYOPT_OPTLOCK_UNLOCK_KEY2);	       // OPT unlock
            REG_0x50005058[0] |= (READ_REG(GPIOA->IDATA) & (BIT8 + BIT9));

            MODIFY_REG(SMU->SYSLOCK, SMU_SYSLOCK_UNLOCK_KEY_MSK, SMU_SYSLOCK_UNLOCK_KEY1);
            MODIFY_REG(SMU->SYSLOCK, SMU_SYSLOCK_UNLOCK_KEY_MSK, SMU_SYSLOCK_UNLOCK_KEY2);	       // SMU nulock
            if(GPIOA->IDATA & BIT10)
            {
              REG_0x50005004[0] |= BIT3;
            }
            else
            {
              REG_0x50005004[0] &= ~BIT3;
            }

            SET_BIT(GPIOA->OBUF, 0x00000002);
            READ_REG(GPIOA->IDATA);
            READ_REG(GPIOB->IDATA);
            READ_REG(GPIOC->IDATA);
            READ_REG(GPIOD->IDATA);
            SCB->SCR = (1UL << 2); // Deep Sleep Mode
            __WFI();
            __NOP();
            __NOP();
            break;

        case TEST_ShutDown:
            SET_BIT(GPIOA->OBUF, 0x00000003);
            READ_REG(GPIOA->IDATA);
            READ_REG(GPIOB->IDATA);
            READ_REG(GPIOC->IDATA);
            READ_REG(GPIOD->IDATA);
            // MODIFY_REG(SMU->SHUTDOWN_CTRL, 0xFF, 0x55);
            MODIFY_REG(SMU->SHUTDOWN_ON, 0xFF, 0x55);
            SCB->SCR = (1UL << 2); // SHUTDOWN
            __WFI();
            __NOP();
            __NOP();
            break;

        case TEST_GPIO:
            MODIFY_REG(IOFUNC->CTRL0, GPIOFUNC_CTRL0_ICE_EN_MSK, GPIOFUNC_CTRL0_ICE_DISABLE);
            WRITE_REG(GPIOA->CFG0, 0xFFFFFFFF);
            WRITE_REG(GPIOA->CFG1, 0xFFFFFFFF);
            WRITE_REG(GPIOA->OBUF, 0x55555555);// 32 IO
            WRITE_REG(GPIOB->CFG0, 0xFFFFFFFF);
            WRITE_REG(GPIOB->OBUF, 0x00000015);// 6IO
            WRITE_REG(GPIOC->CFG0, 0xFFFFFFFF);
            WRITE_REG(GPIOC->OBUF, 0x00000005);// 3IO
            WRITE_REG(GPIOD->CFG0, 0xFFFFFFFF);
            WRITE_REG(GPIOD->OBUF, 0x00000001);// 2IO
            Test_Cnt = 10;
            while(Test_Cnt--);
            R_IOA_Data = READ_REG(GPIOA->IDATA);
            R_IOB_Data = READ_REG(GPIOB->IDATA);
            R_IOC_Data = READ_REG(GPIOC->IDATA);
            R_IOD_Data = READ_REG(GPIOD->IDATA);
            if((R_IOA_Data != 0x55555555) | (R_IOB_Data != 0x15) | (R_IOC_Data != 0x5) | (R_IOD_Data != 0x1))
            {
                // test fail
                WRITE_REG(GPIOA->OBUF, 0xFFFFFFFF);// 32 IO
                break;
            }
            WRITE_REG(GPIOA->CFG0, 0xFFFFFFFF);
            WRITE_REG(GPIOA->CFG1, 0xFFFFFFFF);
            WRITE_REG(GPIOA->OBUF, 0xAAAAAAAA);// 32 IO
            WRITE_REG(GPIOB->CFG0, 0xFFFFFFFF);
            WRITE_REG(GPIOB->OBUF, 0x0000002A);// 6IO
            WRITE_REG(GPIOC->CFG0, 0xFFFFFFFF);
            WRITE_REG(GPIOC->OBUF, 0x00000002);// 3IO
            WRITE_REG(GPIOD->CFG0, 0xFFFFFFFF);
            WRITE_REG(GPIOD->OBUF, 0x00000002);// 2IO
            Test_Cnt = 10;
            while(Test_Cnt--);
            R_IOA_Data = READ_REG(GPIOA->IDATA);
            R_IOB_Data = READ_REG(GPIOB->IDATA);
            R_IOC_Data = READ_REG(GPIOC->IDATA);
            R_IOD_Data = READ_REG(GPIOD->IDATA);
            if((R_IOA_Data != 0xAAAAAAAA) | (R_IOB_Data != 0x2A) | (R_IOC_Data != 0x2) | (R_IOD_Data != 0x2))
            {
                // test fail
                WRITE_REG(GPIOA->OBUF, 0xFFFFFFFF);// 32 IO
                break;
            }
            else
            {
                WRITE_REG(GPIOA->OBUF, 0x4);// 32 IO
            }
            break;

        case TEST_DumpROM:
            temp = (READ_REG(GPIOA->IDATA) & (BIT8 + BIT9));
            if( temp == 0)
            {
                temp = 3 << 3;
            }
            else if (temp == BIT8)
            {
                temp = 4 << 3;
            }
            else if (temp == BIT9)
            {
                temp = 5 << 3;
            }
            else if (temp == (BIT8 + BIT9))
            {
                temp = 7 << 3;
            }
            MODIFY_REG(BODYOPT->BODYOPTLOCK, BODYOPT_OPTLOCK_UNLOCK_KEY_MSK, BODYOPT_OPTLOCK_UNLOCK_KEY1);
            MODIFY_REG(BODYOPT->BODYOPTLOCK, BODYOPT_OPTLOCK_UNLOCK_KEY_MSK, BODYOPT_OPTLOCK_UNLOCK_KEY2);
            REG_0x50005044[0] &= ~BIT0;     // 12MHz XTAL Mode
            // PLL Enable
           	MODIFY_REG(SMU->SYSLOCK, SMU_SYSLOCK_UNLOCK_KEY_MSK, SMU_SYSLOCK_UNLOCK_KEY1);
            MODIFY_REG(SMU->SYSLOCK, SMU_SYSLOCK_UNLOCK_KEY_MSK, SMU_SYSLOCK_UNLOCK_KEY2);	       // SMU nulock
            while((CLOCK->CLKSTS & CLOCK_CLKSTS_IOSC12M_RDY) == 0);                                // Wait IOSC12M stable
            SET_BIT(ACU->PLL_CTRL, ACU_PLL_CTRL_PLL_ENABLE | temp);                                // PLL enable
            for(iCount = 0; iCount < 20; iCount++);                                                // Wait H/W update PLL Ready Flag
            while((CLOCK->CLKSTS & CLOCK_CLKSTS_PLL_RDY) == 0);                                    // Wait PLL stable
            MODIFY_REG(SMU->SYSLOCK, SMU_SYSLOCK_UNLOCK_KEY_MSK, 0);
            // Sel PLL
            MODIFY_REG(CLOCK->AHBCKSEL, CLOCK_AHBCKSEL_HCLK_SEL_MSK, (CLOCK_AHBCKSEL_HCLK_SEL_PLL & CLOCK_AHBCKSEL_HCLK_SEL_MSK));
            WRITE_REG(GPIOA->CFG0, 0xFFFFFFFF);
            WRITE_REG(GPIOA->CFG1, 0xFFFFFFFF);
            WRITE_REG(GPIOA->OBUF, 0x00000000);// 32 IO
            WRITE_REG(GPIOB->CFG0, 0xFFFFFFFF);
            WRITE_REG(GPIOB->OBUF, 0x00000000);// 6IO
            for (iCount = 0; iCount < 2048; iCount++)
            {
                GPIOB->OBUF &= ~BIT0;
                ROMData = InternalROM[iCount];
                WRITE_REG(GPIOA->OBUF, ROMData);// 32 IO
                __NOP();
                GPIOB->OBUF |= BIT0;
                __NOP();
                __NOP();
            }
            break;

        case UART_DumpROM:
            UART0_DMA_Enable();
            Ptr = (uint8_t*)0x20000000;
            for (iCount = 0; iCount < 0x2000; iCount++)
            {
                Ptr[iCount] = InternalROM_Byte[iCount];
            }
            DMA0->SRCADR = 0x20000000;
            DMA0->DSTADR = (uint32_t)&UART0->DATA;
            DMA0->DTN = 0x2000;
            DMA0->CTRL = DMA_CTRL_DMA_DSTSIZE_8B | DMA_CTRL_DMA_SRCSIZE_8B | DMA_CTRL_DMA_SRCINC_ENABLE | DMA_CTRL_DMA_REQSEL_UART0;
            DMA_INT->INTSTS = DMA_INTSTS_DMA0_DONE_INT_FLAG;
            UART0->CTRL |= UART_CTRL_TX_DMA_ENABLE;
            DMA0->CTRL |= DMA_CTRL_DMA_ENABLE;
            while((DMA0->CTRL & DMA_CTRL_DMA_ENABLE) != 0);
            while((UART0->CTRL & UART_CTRL_TX_DMA_ENABLE) != 0);
            while(READ_BIT(UART0->STS, UART_STS_TX_DONE_FLAG) == 0);
            break;

        case UART_DumpFlashSPIFC:
            SET_BIT(CLOCK->AHBCKEN, CLOCK_AHBCKEN_SPIFC_CLK_ENABLE);
            SET_BIT(SPIFC->CTRL1, SPIFC_CTRL1_SPIFC_ENABLE);
            SPIFC->CTRL0 = SPIFC_CTRL0_MIO_1BIT + SPIFC_CTRL0_MANUAL_MODE + SPIFC_CTRL0_CMIO_1BIT + SPIFC_CTRL0_AMIO_1BIT + SPIFC_CTRL0_IGNORE_LAST_CLK_DISABLE + SPIFC_CTRL0_CLK_STATE_LOW;
            //Exit enhance mode
            SPIFC->RX_BC = 0;
            SPIFC->TX_BC = 1;
            SPIFC->PARA = SPIFC_PARA_ENHAN_DISABLE | SPIFC_PARA_WITHOUT_ADDR_ENABLE | SPIFC_PARA_ONLY_ADDR_DISABLE | SPIFC_PARA_DUMMY_CLK_0_CYCLE;  // wo_enhan = 1, wo_addr = 1, to_addr = 0
            SPIFC->CMD = 0xFF | SPIFC_CMD_WITHOUTCMD_DISABLE | SPIFC_CMD_ONLYCMD_DISABLE | SPIFC_CMD_1ST_TRANS_WITHCMD_DISABLE;                     // wo_cmd = 0, cmd_only = 0, one_cmd = 0
            while((SPIFC->CTRL0 & SPIFC_CTRL0_PENDING_FLAG) == 0);
            SPIFC->TX_DATA = 0xFF;
            while((SPIFC->CTRL0 & SPIFC_CTRL0_TX_DONE_FLAG) == 0);

            //Releae deep power down
            SPIFC->RX_BC = 1;
            SPIFC->TX_BC = 0;
            SPIFC->PARA = SPIFC_PARA_ENHAN_DISABLE | SPIFC_PARA_WITHOUT_ADDR_DISABLE | SPIFC_PARA_ONLY_ADDR_DISABLE | SPIFC_PARA_DUMMY_CLK_0_CYCLE;  // wo_enhan = 1, wo_addr = 0, to_addr = 0
            SPIFC->CMD = 0xAB | SPIFC_CMD_WITHOUTCMD_DISABLE | SPIFC_CMD_ONLYCMD_DISABLE | SPIFC_CMD_1ST_TRANS_WITHCMD_DISABLE;                    // wo_cmd = 0, cmd_only = 0, one_cmd = 0
            while((SPIFC->CTRL0 & SPIFC_CTRL0_PENDING_FLAG) == 0);
            SPIFC->RX_DATA = 0;																	                       // write dummy before read operation.
            while((SPIFC->CTRL0 & SPIFC_CTRL0_PENDING_FLAG) == 0);
            temp = SPIFC->RX_DATA;
            if((temp >= 0x10) && (temp <= 0x20)) //Read ID pass
            {
              for(temp = 0; temp < 70; temp++); // delay for release deep power down

              // Exit SPI Addr 4Bytes mode
              SPIFC->CMD = 0xE9 | SPIFC_CMD_WITHOUTCMD_DISABLE | SPIFC_CMD_ONLYCMD_ENABLE | SPIFC_CMD_1ST_TRANS_WITHCMD_DISABLE;                    // wo_cmd = 0, cmd_only = 1, one_cmd = 0
              while((SPIFC->CTRL0 & SPIFC_CTRL0_PENDING_FLAG) == 0);

              // Enable SPIFC 1IO auto mode
              SPIFC->CTRL1 &= ~SPIFC_CTRL1_SPIFC_ENABLE; 													         // SPIFC disable
              SPIFC->ADDRL = 0x0000;
              SPIFC->ADDRH = 0x0000;
              SPIFC->TX_BC = 0x0000;
              SPIFC->RX_BC = 0x0080;
              SPIFC->PARA = SPIFC_PARA_ENHAN_DISABLE + SPIFC_PARA_DUMMY_CLK_0_CYCLE;
              SPIFC->CMD = SPIFC_FLASH_READ;
              SPIFC->CTRL0 = SPIFC_CTRL0_MIO_1BIT + SPIFC_CTRL0_AUTO_MODE + SPIFC_CTRL0_CMIO_1BIT + SPIFC_CTRL0_AMIO_1BIT + SPIFC_CTRL0_IGNORE_LAST_CLK_DISABLE + SPIFC_CTRL0_CLK_STATE_LOW;
              SPIFC->CTRL1 |= SPIFC_CTRL1_SPIFC_ENABLE;
            }
            UART0_DMA_Enable();
            for (iCount = 0x4000000; iCount < 0x6000000; iCount += 0x1000)
            {
                WDG->KEYCODE = WDG_KEYCODE_WDG_CLEAR;
                DMA0->SRCADR = iCount;
                DMA0->DSTADR = (uint32_t)&UART0->DATA;
                DMA0->DTN = 0x1000;
                DMA0->CTRL = DMA_CTRL_DMA_DSTSIZE_8B | DMA_CTRL_DMA_SRCSIZE_8B | DMA_CTRL_DMA_SRCINC_ENABLE | DMA_CTRL_DMA_REQSEL_UART0;
                DMA_INT->INTSTS = DMA_INTSTS_DMA0_DONE_INT_FLAG;
                UART0->CTRL |= UART_CTRL_TX_DMA_ENABLE;
                DMA0->CTRL |= DMA_CTRL_DMA_ENABLE;
                while((DMA0->CTRL & DMA_CTRL_DMA_ENABLE) != 0);
                while((UART0->CTRL & UART_CTRL_TX_DMA_ENABLE) != 0);
                while(READ_BIT(UART0->STS, UART_STS_TX_DONE_FLAG) == 0);
            }
            break;

        case UART_DumpFlashSPI0:
            UART0_DMA_Enable();
            SET_BIT(CLOCK->AHBCKEN, CLOCK_AHBCKEN_GPIO_CLK_ENABLE);
            SET_BIT(CLOCK->APBCKEN, CLOCK_APBCKEN_SPI0_CLK_ENABLE);
            WRITE_REG(GPIOB->CFG0, 0x00000000); // Set all of SPIFC pin to input pull high.
            WRITE_REG(GPIOB->OBUF, BIT5 + BIT4 + BIT3 + BIT2 + BIT1 + BIT0); ////Set all of SPIFC pin to output high and clk pin to pull high(IOB1).
            GPIOB->CFG0 |= ((GPIO_MODE_OUTPUT << BIT3) << BIT3);
            SET_BIT(GPIOB->OBUF, BIT3);
            MODIFY_REG(IOFUNC->CTRL0, GPIOFUNC_CTRL0_SPI0_HWCS_EN_MSK, GPIOFUNC_CTRL0_SPI0_HWCS_DISABLE);    // SPI0 S/W CS
            MODIFY_REG(IOFUNC->CTRL0, GPIOFUNC_CTRL0_SPI0_IOSEL_MSK, GPIOFUNC_CTRL0_SPI0_IOB1_4);
            SPI0->CTRL = SPI_CTRL_MASTER_MODE | SPI_CTRL_SPI_ENABLE;
            SPI0->CLK_DIV = 0;
            for (iCount = 0; iCount < 0x2000000; iCount += 256)              // 256MBit
            {
                WDG->KEYCODE = WDG_KEYCODE_WDG_CLEAR;
                SPI0_Flash_ReadNBytes(FlashData, iCount, 256);
                DMA0->SRCADR = (uint32_t)&FlashData;
                DMA0->DSTADR = (uint32_t)&UART0->DATA;
                DMA0->DTN = 256;
                DMA0->CTRL = DMA_CTRL_DMA_DSTSIZE_8B | DMA_CTRL_DMA_SRCSIZE_8B | DMA_CTRL_DMA_SRCINC_ENABLE | DMA_CTRL_DMA_REQSEL_UART0;
                DMA_INT->INTSTS = DMA_INTSTS_DMA0_DONE_INT_FLAG;
                UART0->CTRL |= UART_CTRL_TX_DMA_ENABLE;
                DMA0->CTRL |= DMA_CTRL_DMA_ENABLE;
                while((DMA0->CTRL & DMA_CTRL_DMA_ENABLE) != 0);
                while((UART0->CTRL & UART_CTRL_TX_DMA_ENABLE) != 0);
                while(READ_BIT(UART0->STS, UART_STS_TX_DONE_FLAG) == 0);
            }
            break;

        case UART_DumpFuse:
            SET_BIT(CLOCK->APBCKEN, CLOCK_APBCKEN_UART0_CLK_ENABLE);	//	APB UART0 Clock enable
            MODIFY_REG(IOFUNC->CTRL0, GPIOFUNC_CTRL0_ICE_EN_MSK, GPIOFUNC_CTRL0_ICE_DISABLE);
            MODIFY_REG(IOFUNC->CTRL0, GPIOFUNC_CTRL0_UART0_IOSEL_MSK, GPIOFUNC_CTRL0_UART0_IOC2_RST);
//            MODIFY_REG(IOFUNC->CTRL0, IOFUNC_CTRL0_UART0_IOSEL_MSK, IOFUNC_CTRL0_UART0_IOA13_14);
            UART0->CTRL = (UART_CTRL_UART0_FIFO_ENABLE | UART_CTRL_UART_ENABLE | UART_CTRL_UART_TX_ENABLE);                                                     // Initialize UART Controller
            CLEAR_FLAG(UART0->STS, (UART_STS_RX_IDLE_MSK | UART_STS_TX_DONE_MSK), (UART_STS_RX_IDLE_FLAG | UART_STS_TX_DONE_FLAG | UART_STS_RX_BRK_FLAG));	  // Reset UART status
            UART0->BAUD_RATE = 0x00130007;  //Baut rate 115200 in 12MHz system clock

            UART0->DATA = (READ_REG(EFUSE->EFUSE_A) & 0xFF);
            UART0->DATA = (READ_REG(EFUSE->EFUSE_A) & 0xFF00) >> 8;
            UART0->DATA = (READ_REG(EFUSE->EFUSE_A) & 0xFF0000) >> 16;
            UART0->DATA = (READ_REG(EFUSE->EFUSE_A) & 0xFF000000) >> 24;
            UART0->DATA = (READ_REG(EFUSE->EFUSE_B) & 0xFF);
            UART0->DATA = (READ_REG(EFUSE->EFUSE_B) & 0xFF00) >> 8;
            UART0->DATA = (READ_REG(EFUSE->EFUSE_B) & 0xFF0000) >> 16;
            UART0->DATA = (READ_REG(EFUSE->EFUSE_B) & 0xFF000000) >> 24;
            while(READ_BIT(UART0->STS, UART_STS_TX_FIFO_EMPTY_FLAG) == 0);
            while(READ_BIT(UART0->STS, UART_STS_TX_DONE_FLAG) == 0);     // Wait Tx empty and Time-out manner
            CLEAR_FLAG(UART0->STS, UART_STS_TX_DONE_MSK, UART_STS_TX_DONE_FLAG);
            break;

        case UART_DumpFlashID:
            SET_BIT(CLOCK->AHBCKEN, CLOCK_AHBCKEN_GPIO_CLK_ENABLE + CLOCK_AHBCKEN_DMA_CLK_ENABLE);
            SET_BIT(CLOCK->APBCKEN, CLOCK_APBCKEN_SPI0_CLK_ENABLE + CLOCK_APBCKEN_UART0_CLK_ENABLE);
            MODIFY_REG(IOFUNC->CTRL0, GPIOFUNC_CTRL0_ICE_EN_MSK, GPIOFUNC_CTRL0_ICE_DISABLE);
            MODIFY_REG(IOFUNC->CTRL0, GPIOFUNC_CTRL0_UART0_IOSEL_MSK, GPIOFUNC_CTRL0_UART0_IOC2_RST);
//            MODIFY_REG(IOFUNC->CTRL0, IOFUNC_CTRL0_UART0_IOSEL_MSK, IOFUNC_CTRL0_UART0_IOA13_14);
            UART0->CTRL = (UART_CTRL_UART0_FIFO_ENABLE | UART_CTRL_UART_ENABLE | UART_CTRL_UART_TX_ENABLE);                                                     // Initialize UART Controller
            CLEAR_FLAG(UART0->STS, (UART_STS_RX_IDLE_MSK | UART_STS_TX_DONE_MSK), (UART_STS_RX_IDLE_FLAG | UART_STS_TX_DONE_FLAG | UART_STS_RX_BRK_FLAG));	  // Reset UART status
            UART0->BAUD_RATE = 0x00130007;  //Baut rate 115200 in 12MHz system clock

            WRITE_REG(GPIOB->CFG0, 0x00000000); // Set all of SPIFC pin to input pull high.
            WRITE_REG(GPIOB->OBUF, BIT5 + BIT4 + BIT3 + BIT2 + BIT1 + BIT0); ////Set all of SPIFC pin to output high and clk pin to pull high(IOB1).
            GPIOB->CFG0 |= ((GPIO_MODE_OUTPUT << BIT3) << BIT3);
            SET_BIT(GPIOB->OBUF, BIT3);

            MODIFY_REG(IOFUNC->CTRL0, GPIOFUNC_CTRL0_SPI0_HWCS_EN_MSK, GPIOFUNC_CTRL0_SPI0_HWCS_DISABLE);    // SPI0 S/W CS
            MODIFY_REG(IOFUNC->CTRL0, GPIOFUNC_CTRL0_SPI0_IOSEL_MSK, GPIOFUNC_CTRL0_SPI0_IOB1_4);
            SPI0->CTRL = SPI_CTRL_MASTER_MODE | SPI_CTRL_SPI_ENABLE;
            SPI0->CLK_DIV = 0;

            temp = SPI0_Flash_ReadID();
            UART0->DATA = temp & 0xFF;
            UART0->DATA = (temp & 0xFF00) >> 8;
            UART0->DATA = (temp & 0xFF0000) >> 16;
            while(READ_BIT(UART0->STS, UART_STS_TX_FIFO_EMPTY_FLAG) == 0);
            while(READ_BIT(UART0->STS, UART_STS_TX_DONE_FLAG) == 0);     // Wait Tx empty and Time-out manner
            CLEAR_FLAG(UART0->STS, UART_STS_TX_DONE_MSK, UART_STS_TX_DONE_FLAG);
            CLEAR_BIT(UART0->CTRL, UART_CTRL_UART0_FIFO_ENABLE);             // Disable Uart Tx

            break;

        default:
			break;
    }
    while(1)
    {
        WDG->KEYCODE = WDG_KEYCODE_WDG_CLEAR;
    }
}

void UART0_DMA_Enable(void)
{
    SET_BIT(CLOCK->APBCKEN, CLOCK_APBCKEN_UART0_CLK_ENABLE);	//	APB UART0 Clock enable
    SET_BIT(CLOCK->AHBCKEN, CLOCK_AHBCKEN_DMA_CLK_ENABLE);
    MODIFY_REG(IOFUNC->CTRL0, GPIOFUNC_CTRL0_ICE_EN_MSK, GPIOFUNC_CTRL0_ICE_DISABLE);
    MODIFY_REG(IOFUNC->CTRL0, GPIOFUNC_CTRL0_UART0_IOSEL_MSK, GPIOFUNC_CTRL0_UART0_IOC2_RST);
//    MODIFY_REG(IOFUNC->CTRL0, IOFUNC_CTRL0_UART0_IOSEL_MSK, IOFUNC_CTRL0_UART0_IOA13_14);
    UART0->CTRL = (UART_CTRL_UART_ENABLE | UART_CTRL_UART_TX_ENABLE);                                                     // Initialize UART Controller
    CLEAR_FLAG(UART0->STS, (UART_STS_RX_IDLE_MSK | UART_STS_TX_DONE_MSK), (UART_STS_RX_IDLE_FLAG | UART_STS_TX_DONE_FLAG | UART_STS_RX_BRK_FLAG));	  // Reset UART status
    UART0->BAUD_RATE = 0x00130007;  //Baut rate 115200 in 12MHz system clock
}

void SPI0_Write(uint8_t Data)
{
	SPI0->TX_DATA = Data;
	while(!(READ_BIT(SPI0->STS, SPI_CTRL_TX_DONE_FLAG)));				//	Wait until SPI transmission is completed.
	SPI0->STS |= SPI_CTRL_TX_DONE_FLAG;													//	Clear transmission complete flag
}

uint8_t SPI0_Read(void)
{
	uint32_t temp;
	SPI0->CTRL |= SPI_CTRL_MASTER_RX_TRIG_ENABLE;
	while(!(READ_BIT(SPI0->STS, SPI_CTRL_TX_DONE_FLAG)));				//	Wait until SPI transmission is completed.
	SPI0->STS |= SPI_CTRL_TX_DONE_FLAG;													//	Clear transmission complete flag
	temp = SPI0->RX_DATA;
	return temp;
}

void SPI0_Flash_ReadNBytes(uint8_t *pBuffer, uint32_t ReadAddr, uint32_t NumByteToRead)
{
	uint32_t index;

	CLEAR_BIT(GPIOB->OBUF, BIT3);								// CS low

	SPI0->CTRL |= SPI_CTRL_MAUT_RW_EN_ENABLE;
	SPI0_Write(0x03);											// send cmd

	SPI0_Write((ReadAddr & 0xFF0000) >> 16);								// addressH
	SPI0_Write((ReadAddr & 0xFF00) >> 8);									// addressM
	SPI0_Write(ReadAddr & 0xFF);														// addressL

	for(index = 0; index < NumByteToRead; index++)			// (NumByteToRead-1)
	{
		*pBuffer++ = SPI0_Read();														// read data
	}
	SPI0->CTRL &= ~SPI_CTRL_MAUT_RW_EN_ENABLE;
	SET_BIT(GPIOB->OBUF, BIT3);									// CS high
}

uint32_t SPI0_Flash_ReadID(void)
{
  uint32_t Temp = 0;
	uint32_t Temp0 = 0;
	uint32_t Temp1 = 0;
	uint32_t Temp2 = 0;

	CLEAR_BIT(GPIOB->OBUF, BIT3);							// CS low

	SPI0->CTRL |= SPI_CTRL_MAUT_RW_EN_ENABLE;
	SPI0_Write(0x9F);												              // Read ID command
	Temp0 = SPI0_Read();																		// 2nd clock, read dummy data
	Temp1 = SPI0_Read();																		// 3nd clock, read 1st data
	Temp2 = SPI0_Read();																		// 4th clock, read 2nd data
	SPI0->CTRL &= ~SPI_CTRL_MAUT_RW_EN_ENABLE;										// disable clock generator

	SET_BIT(GPIOB->OBUF, BIT3);							// CS high

	Temp = (Temp0 << 16) | (Temp1 << 8) | Temp2;
	return Temp;
}


