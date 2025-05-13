/************************************************************************************
 *  File:     main.c
 *  Purpose:  GPCM3/GPFM1 Example Code
 *            Replace with your code.
 *  Date:     21 March 2023
 *  Info:
 *
 *
 ************************************************************************************/

/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include <stdio.h>
#include "GPCM3_FM1.h"
#include "KeyScan.h"


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#if defined(__CC_ARM)
#define Aligned4B __align(4)
#elif defined(__GNUC__)
extern unsigned int __boot_table_start; //in GPCM3Test.ld
#define Aligned4B __attribute__ ((aligned (4)))
#endif

KEYSCAN_WORKING_RAM     KeyScanWorkingRam;
uint32_t ScanedKey;

#define C_I2C_Address       0xA0

uint8_t WriteDataBuf1[16] = {0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xAB, 0xAC, 0xAD, 0xAE, 0xAF};
uint8_t WriteDataBuf2[16] = {0xCF, 0xCE, 0xCD, 0xCC, 0xCB, 0xCA, 0xC9, 0xC8, 0xC7, 0xC6, 0xC5, 0xC4, 0xC3, 0xC2, 0xC1, 0xC0};
uint8_t ReadDataBuf[16];

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
    uint8_t temp;
    uint8_t *bufptr;

    WDT_Close();

    GPIO_SetMode(GPIOA, (BIT8 | BIT9), GPIO_MODE_OUTPUT);
    CLEAR_BIT(GPIOA->OBUF, (BIT8 | BIT9));

    KeyScan_Initial(&KeyScanWorkingRam);                  // Key scan init

    /*
	 * Init I2C Master Mode
	 */
	I2C_Init(I2C_Master, I2C_IOA15_16);
	I2C_SetClkDiv(I2C_CLK_SEL_HCLK_DIV_1024);
    I2C_SetSlaveAddr(C_I2C_Address);

    while(1)
	{
		WDT_Clear();
		KeyScan_ServiceLoop();
		ScanedKey = KeyScan_GetCh();

        switch(ScanedKey)
		{
            case 0x01:  // IOA0+VDD: Init I2C Master Mode
                I2C_Init(I2C_Master, I2C_IOA15_16);
                I2C_SetClkDiv(I2C_CLK_SEL_HCLK_DIV_1024);
                I2C_SetSlaveAddr(C_I2C_Address);
            break;

            case 0x02:  // IOA1+VDD: Init I2C Slave Mode
                I2C_Init(I2C_Slave, I2C_IOA15_16);
                I2C_SetSlaveAddr(C_I2C_Address);
            break;

            case 0x04:  // IOA2+VDD: Slave mode - auto response
                if (I2C->CTRL & I2C_CTRL_MODE_SEL_MASTER)
                {
                    break;
                }

                while(1)
                {
                    WDT_Clear();

                    //Polling master send slave address.
                    if ((I2C->STS & I2C_STS_SLV_DATA_DONE_FLAG) != 0)
                    {
                        I2C->STS |= I2C_STS_SLV_DATA_DONE_FLAG;

                        if ((I2C->ADDR & I2C_RW_SEL_MSK) == I2C_RW_SEL_READ)
                        {
                            //Master read
                            temp = sizeof(ReadDataBuf);
                            bufptr = ReadDataBuf;
                            while(temp--)
                            {
                                if (I2C_Slave_SendByte(*bufptr++) == 0)
                                {
                                    break;
                                }
                            }

                            while((I2C->STS & I2C_STS_RX_STOP_CMD_FLAG) == 0);
                            I2C->STS |= I2C_STS_RX_STOP_CMD_FLAG;
                        }
                        else
                        {
                            //Master write
                            temp = sizeof(ReadDataBuf);
                            bufptr = ReadDataBuf;
                            while(temp--)
                            {
                                *bufptr++ = I2C_Slave_ReadByte();
                            }

                            while((I2C->STS & I2C_STS_RX_STOP_CMD_FLAG) == 0);
                            I2C->STS |= I2C_STS_RX_STOP_CMD_FLAG;
                        }
                    }
                }
            break;

            case 0x08:  // IOA3+VDD
            break;

            case 0x10:  // IOA4+VDD: Write Mode
                if (I2C->CTRL & I2C_CTRL_MODE_SEL_MASTER)
                {
                    I2C_Master_WriteNBytes(WriteDataBuf1, sizeof(WriteDataBuf1));
                }
                else
                {
                    I2C_Slave_WriteNBytes(WriteDataBuf1, sizeof(WriteDataBuf1));
                }
            break;

            case 0x20:  // IOA5+VDD: Read Mode
                if (I2C->CTRL & I2C_CTRL_MODE_SEL_MASTER)
                {
                    I2C_Master_ReadNBytes(ReadDataBuf, sizeof(ReadDataBuf));
                }
                else
                {
                    I2C_Slave_ReadNBytes(ReadDataBuf, sizeof(ReadDataBuf));
                }
            break;

            case 0x40:   // IOA6+VDD: Write Mode with DMA
                if (I2C->CTRL & I2C_CTRL_MODE_SEL_MASTER)
                {
                    I2C_Master_WriteNBytes_DMA(DMA0, WriteDataBuf2, sizeof(WriteDataBuf2));
                }
                else
                {
                    I2C_Slave_WriteNBytes_DMA(DMA0, WriteDataBuf2, sizeof(WriteDataBuf2));
                }
            break;

            case 0x80:  // IOA7+VDD: Read Mode with DMA
                if (I2C->CTRL & I2C_CTRL_MODE_SEL_MASTER)
                {
                    I2C_Master_ReadNBytes_DMA(DMA0, ReadDataBuf, sizeof(ReadDataBuf));
                }
                else
                {
                    I2C_Slave_ReadNBytes_DMA(DMA0, ReadDataBuf, sizeof(ReadDataBuf));
                }
            break;
		}
	}
  return 0;
}


