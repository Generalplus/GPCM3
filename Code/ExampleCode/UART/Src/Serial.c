/**************************************************************************************************
 * Copyright(c) 2018 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *
 * @Version:
 *
 * @Date:
 *
 * @Abstract:
 *
 **************************************************************************************************/

/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include <stdio.h>
#include "GPCM3_FM1.h"
#include "Serial.h"


/*---------------------------------------------------------------------------------------
 * Code Area
 *---------------------------------------------------------------------------------------*/

/**
 * @brief
 *         Conver 0~F char to uint8_t
 * @param
 *         *DstNum    -[out] uint8_t
 *          KeyBuffer -[in]  Ascii code
 * @return
 *         1 : Done
 *         0 : Input are not 0~F char
 */
_Bool AsciiCodeToUINT(uint8_t *DstNum, uint8_t *KeyBuffer)
{
  if((*KeyBuffer >= '0') && (*KeyBuffer <= '9'))
	{
    *DstNum = *KeyBuffer - 48;
    return 1;
  }
	else if((*KeyBuffer >= 'A') && (*KeyBuffer <= 'F'))
	{
    *DstNum = *KeyBuffer - 55;
    return 1;
  }
	else if((*KeyBuffer >= 'a') && (*KeyBuffer <= 'f'))
	{
    *DstNum = *KeyBuffer - 87;
    return 1;
  }
	else
	{
    return 0;
  }
}


/**
 * @brief
 *
 * @param
 *         None
 * @return
 *         None
 */
int32_t DrvUART_Read(uint8_t	*pu8RxBuf, uint32_t	u32ReadBytes)
{
  uint32_t  iCount;
  uint32_t	u32delayno;

  for(iCount = 0; iCount < u32ReadBytes; iCount++)
	{
    u32delayno = 0;

	  //while(UART->FSR.RX_EMPTY == 1)					   // Check RX empty => failed
		while(READ_BIT(UART0->STS, UART_STS_RX_DAT_NEMP_FLAG) == 0)
		{
			WDT_Clear();
	    u32delayno++;
	    if(u32delayno >= 0x40000000)
			{
	      return -1;
			}
    }

    CLEAR_FLAG(UART0->STS, UART_STS_RX_DAT_NEMP_MSK, UART_STS_RX_DAT_NEMP_FLAG);
    pu8RxBuf[iCount] = UART0->DATA;					     // Get Data from UART RX
  }
  return 0;
}


/**
 * @brief
 *         Get hex data from Uart
 * @param
 *         *DstNum  -[out] uint32_t
 *          DataLen -[in]  Input data length, unit: byte
 * @return
 *         1 : Done
 *         0 : Cancel
 */
_Bool Input0x(uint32_t *DstNum, uint8_t DataLen)
{
  uint8_t TmpNum = 0;
	uint8_t InputCnt = 0;
	uint8_t UartRxBuf;

  while(1)
	{
    DrvUART_Read(&UartRxBuf, 1);
    if(AsciiCodeToUINT(&TmpNum, &UartRxBuf) && (InputCnt < DataLen))
		{
			// printf("%c", UartRxBuf);               // Display the input string on the PC side
      *DstNum = *DstNum * 16 + TmpNum;
      InputCnt += 1;
      continue;
    }
		else if(UartRxBuf == 0x08)                   // BACKSPACE Key: 0x08
		{
      if(InputCnt == 0) {
        InputCnt = 0;
      } else {
				/*
				 * To clear 1 char
				 */
				printf("%c", UartRxBuf);
				printf("%c", 0x20);
        printf("%c", UartRxBuf);

        InputCnt -= 1;
      }
      *DstNum /= 16;
      continue;
    }
		else if(UartRxBuf == 0x0D)                   // Enter Key: 0x0D
		{
      DrvUart_SendString("\r\n");
      return 1;
    }
		else if(UartRxBuf == 0x1B)                   // ESC Key: 0x1B
		{
			DrvUart_SendString("\r\n");
			DrvUart_SendString("Return to Main Function Test Menu \r\n");
			DrvUart_SendString("Please select test item:\r\n");
			DrvUart_SendString("Test item is 0x");
      return 0;
    }
  }
}


/**
 * @brief
 *
 * @param
 *         None
 * @return
 *         None
 */
int32_t DrvUart_SendString(uint8_t *pu8TxBuf)
{
  uint32_t iCount;
	uint32_t u32delayno;

	iCount = 0;
	while(pu8TxBuf[iCount] != 0)
	{
    u32delayno = 0;

		UART0->DATA = pu8TxBuf[iCount];                             // Send UART Data from buffer
	  while(READ_BIT(UART0->STS, UART_STS_TX_DONE_FLAG) == 0)     // Wait Tx empty and Time-out manner
    {
			WDT_Clear();
      u32delayno++;
      if(u32delayno >= 0x40000000)
			{
        return -1;
      }
    }

		CLEAR_FLAG(UART0->STS, UART_STS_TX_DONE_MSK, UART_STS_TX_DONE_FLAG);
    iCount++;
  }

    return 0;
}

/**
 * @brief
 *
 * @param
 *          None.
 * @return
 *          None.
 */
void ShowMainTestList()
{
	DrvUart_SendString("\r\n");
	DrvUart_SendString("=========== GPCM Function Test Main Menu =========== \r\n");
	DrvUart_SendString("0x01: SRAM \r\n");
	DrvUart_SendString("0x02: GPIO \r\n");
	DrvUart_SendString("0x03: IR \r\n");
	DrvUart_SendString("0x04: System Clock \r\n");
	DrvUart_SendString("0x05: TimeBase \r\n");
	DrvUart_SendString("0x06: Timer \r\n");
	DrvUart_SendString("0x07: SPI \r\n");
	DrvUart_SendString("0x08: SPIFC \r\n");
	DrvUart_SendString("0x09: CCP \r\n");
	DrvUart_SendString("0x0A: IOPWM \r\n");
	DrvUart_SendString("0x0B: Quadrature Decoder \r\n");
	DrvUart_SendString("0x0C: SAR-ADC \r\n");
	DrvUart_SendString("0x0D: DS-ADC(Mic) \r\n");
	DrvUart_SendString("0x0E: AGC \r\n");
	DrvUart_SendString("0x0F: Audio Output \r\n");
	DrvUart_SendString("0x10: UART \r\n");
	DrvUart_SendString("0x11: I2C \r\n");
	DrvUart_SendString("0x12: I2S \r\n");
	DrvUart_SendString("0x13: EXT INT \r\n");
	DrvUart_SendString("0x14: MAC \r\n");
	DrvUart_SendString("0x15: DMA \r\n");
	DrvUart_SendString("0x16: Reset \r\n");
	DrvUart_SendString("0x17: Sleep & Wakeup \r\n");
	DrvUart_SendString("0x18: Info. Block & Body Option \r\n");
	DrvUart_SendString("0x19: Touch \r\n");
	DrvUart_SendString("0x1A: Cache \r\n");
	DrvUart_SendString("0x1B: Others \r\n");
	DrvUart_SendString("0x1C: Combination Test \r\n");
	DrvUart_SendString("0x1D: Writer I/F \r\n");
	DrvUart_SendString("0x1E: USB \r\n");
	DrvUart_SendString("0x1F: SPU \r\n");
	DrvUart_SendString("0x20: Velocity Key \r\n");
	DrvUart_SendString("0x21: PDM \r\n");
	DrvUart_SendString("0x22: SRAM Run Code Test \r\n");
	DrvUart_SendString("\r\n");
	DrvUart_SendString("Please select test item:\r\n");
}
