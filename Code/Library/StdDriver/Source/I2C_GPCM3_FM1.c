/**************************************************************************************************
 * Copyright(c) 2022 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   I2C_GPCM3_FM1.c
 * @Version:
 *   V0.9.1
 * @Date:
 *   July 30, 2021
 * @Abstract:
 *
 **************************************************************************************************/


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "GPCM3_FM1.h"

/*---------------------------------------------------------------------------------------
 * Subroutine Area
 *---------------------------------------------------------------------------------------*/

/**
 * @brief
 *
 * @param
 *   I2cModeSel [in]:
 *    - I2C_Master, I2C_Slave
 *   I2cSelIo [in]:
 *    - I2C_IOA0_1, I2C_IOA15_16, I2C_IOA20_21
 * @return
 *   None.
 */
void I2C_Init(uint32_t I2cModeSel, uint32_t I2cSelIo)
{
	MODIFY_REG(IOFUNC->CTRL0, GPIOFUNC_CTRL0_I2C_IOSEL_MSK, I2cSelIo);
	SET_BIT(CLOCK->APBCKEN, CLOCK_APBCKEN_I2C_CLK_ENABLE);             // I2C Clock enable

    switch(I2cSelIo)
	{
		case I2C_IOA0_1:
			GPIO_SetMode(GPIOA, (BIT0 | BIT1), GPIO_MODE_FLOATING);
			break;

		case I2C_IOA15_16:
			GPIO_SetMode(GPIOA, (BIT15 | BIT16), GPIO_MODE_FLOATING);
			break;

        case I2C_IOA20_21:
			GPIO_SetMode(GPIOA, (BIT20 | BIT21), GPIO_MODE_FLOATING);
			break;
	}

	I2C->CTRL = I2C_CTRL_I2C_ENABLE | I2C_CTRL_CLK_SEL_HCLK_DIV_1024 | I2cModeSel;
	I2C->ADDR = I2C_ADDR_MODE_7BIT;
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void I2C_Close()
{
	I2C->CTRL = 0;
	CLEAR_BIT(CLOCK->APBCKEN, CLOCK_APBCKEN_I2C_CLK_ENABLE);           // I2C Clock disable
}

/**
 * @brief
 *
 * @param
 *   I2cClkDiv [in]:
 *    - I2C_CLK_SEL_HCLK_DIV_16,  I2C_CLK_SEL_HCLK_DIV_32,  I2C_CLK_SEL_HCLK_DIV_64, I2C_CLK_SEL_HCLK_DIV_128,
 *    - I2C_CLK_SEL_HCLK_DIV_256, I2C_CLK_SEL_HCLK_DIV_768, I2C_CLK_SEL_HCLK_DIV_1024
 * @return
 *   None.
 */
void I2C_SetClkDiv(uint32_t I2cClkDiv)
{
    MODIFY_REG(I2C->CTRL, I2C_CTRL_CLK_SEL_MSK, I2cClkDiv);
}

/**
 * @brief
 *
 * @param
 *   Addr [in]: Slave address
 * @return
 *   None.
 */
void I2C_SetSlaveAddr(uint8_t Addr)
{
    MODIFY_REG(I2C->ADDR, I2C_ADDR_ADDRESS_MSK, Addr);
}

/**
 * @brief
 *   Check I2C busy status
 * @param
 *   None.
 * @return
 *   0 -> I2C done.
 *   nonzero -> I2C busy.
 */
uint32_t I2C_CheckBusy(void)
{
	return (I2C->STS & I2C_STS_BUSY_FLAG);
}

/**
 * @brief
 *
 * @param
 *   RW_Bit [in]:
 *    - I2C_WRITE, I2C_READ
 * @return
 *   1: ACK
 *   0: NAK
 */
uint8_t I2C_Master_Start(uint8_t RW_Bit)
{
	MODIFY_REG(I2C->ADDR, I2C_RW_SEL_MSK, RW_Bit);
	I2C->CTRL &= ~I2C_CTRL_MST_NACK;
	I2C->CTRL |= I2C_CTRL_MST_STR | I2C_CTRL_I2C_TRIGGER;
	while((I2C->STS & I2C_STS_TRS_DONE_FLAG) == 0);
	I2C->STS |= I2C_STS_TRS_DONE_FLAG;

	if((I2C->STS & I2C_STS_RX_NO_ACK_FLAG) != 0)
	{
		I2C->STS = I2C_STS_RX_NO_ACK_FLAG;

		return 0;
	}

	return 1;
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void I2C_Master_Stop()
{
	if(I2C_CheckBusy() != 0)
	{
	  while((I2C->STS & I2C_STS_TRS_DONE_FLAG) == 0);
	  I2C->STS |= I2C_STS_TRS_DONE_FLAG;
	}

	I2C->CTRL |= I2C_CTRL_MST_STP | I2C_CTRL_I2C_TRIGGER;
	while(I2C_CheckBusy() != 0);
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void I2C_Master_SendByte(uint8_t DataByte)
{
	I2C->DATA = DataByte;
	while((I2C->STS & I2C_STS_TRS_DONE_FLAG) == 0);
	I2C->STS |= I2C_STS_TRS_DONE_FLAG;
    while(I2C_CheckBusy() != 0);
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   A byte data
 */
void I2C_Master_ReadByteTrigger()
{
  volatile uint32_t DataTemp;

	DataTemp = I2C->DATA;
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   A byte data
 */
uint8_t I2C_Master_ReadByteAck()
{
	I2C->CTRL &= ~I2C_CTRL_MST_NACK;
	while((I2C->STS & I2C_STS_TRS_DONE_FLAG) == 0);
	I2C->STS |= I2C_STS_TRS_DONE_FLAG;
    while(I2C_CheckBusy() != 0);

	return I2C->DATA;
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   A byte data
 */
uint8_t I2C_Master_ReadByteNack()
{
    uint8_t data;

	I2C->CTRL |= I2C_CTRL_MST_NACK;
	while((I2C->STS & I2C_STS_TRS_DONE_FLAG) == 0);
	I2C->STS |= I2C_STS_TRS_DONE_FLAG;

	I2C->CTRL |= I2C_CTRL_MST_STP | I2C_CTRL_I2C_TRIGGER;

    while(I2C_CheckBusy() != 0);

    I2C->CTRL &= ~I2C_CTRL_I2C_ENABLE;
	data = I2C->DATA;
    I2C->CTRL |= I2C_CTRL_I2C_ENABLE;

	return data;
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   1: ACK
 *   0: NAK
 */
uint8_t I2C_Slave_SendByte(uint8_t DataByte)
{
	I2C->DATA = DataByte;
	while((I2C->STS & I2C_STS_TRS_DONE_FLAG) == 0);
	I2C->STS |= I2C_STS_DATA_DONE_FLAG | I2C_STS_TRS_DONE_FLAG;

	if((I2C->STS & I2C_STS_RX_NO_ACK_FLAG) != 0)
	{
		I2C->STS = I2C_STS_RX_NO_ACK_FLAG;

		return 0;
	}
	return 1;
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
uint8_t I2C_Slave_ReadByte(void)
{
    uint8_t data;
	while((I2C->STS & I2C_STS_DATA_DONE_FLAG) == 0);
	data = I2C->DATA;
	I2C->STS |= I2C_STS_DATA_DONE_FLAG | I2C_STS_TRS_DONE_FLAG;

	return data;
}

/**
 * @brief
 *
 * @param
 *  *pSrcBuf [in]: Source buffer address
 *   DataLen [in]: Data length (Byte)
 * @return
 *   None.
 */
void I2C_Master_WriteNBytes(uint8_t *pSrcBuf, uint32_t DataLen)
{
    uint8_t ack;

    ack = I2C_Master_Start(I2C_WRITE);

    if (ack == 0x00)
    {
        //NACK.
        return;
    }

    while(DataLen--)
    {
        I2C_Master_SendByte(*pSrcBuf++);
    }

    I2C_Master_Stop();
}

/**
 * @brief
 *
 * @param
 *  *pDstBuf [out]: Destination buffer address
 *   DataLen [in]: Data length (Byte)
 * @return
 *   A byte data
 */
void I2C_Master_ReadNBytes(uint8_t *pDstBuf, uint32_t DataLen)
{
    uint8_t ack;

    ack = I2C_Master_Start(I2C_READ);
    if (ack == 0x00)
    {
        //NACK.
        return;
    }

    I2C_Master_ReadByteTrigger();
    while(--DataLen)
    {
        *pDstBuf++ = I2C_Master_ReadByteAck();
    }

    *pDstBuf = I2C_Master_ReadByteNack();
}

/**
 * @brief
 *
 * @param
 *  *DmaHandler [in]: The base address of DMAx module
 *    - DMA0, DMA1, DMA2, DMA3, DMA4
 *  *pSrcBuf [in]: Source buffer address
 *   DataLen [in]: Data length (Byte)
 * @return
 *   None.
 */
void I2C_Master_WriteNBytes_DMA(DMA_TYPE_DEF *DmaHandler, uint8_t *pSrcBuf, uint32_t DataLen)
{
    uint8_t ack;

	DMA_Init(DmaHandler, DMA_REQSEL_I2C, DMA_SRC_DATA_8B, DMA_SRC_ADDR_INC, DMA_DST_DATA_8B, DMA_DST_ADDR_FIX);
	DMA_Trigger(DmaHandler, (uint32_t)&pSrcBuf[1], (uint32_t)&I2C->DATA, DataLen-1);

    ack = I2C_Master_Start(I2C_WRITE);
    if (ack == 0x00)
    {
        //NACK.
        DMA_Close(DmaHandler);
        return;
    }
    I2C_Master_SendByte(pSrcBuf[0]);
    while(DMA_CheckBusy(DmaHandler) != 0);
    DMA_Close(DmaHandler);
    while(I2C_CheckBusy() != 0);
    I2C_Master_Stop();
}

/**
 * @brief
 *
 * @param
 *  *DmaHandler [in]: The base address of DMAx module
 *    - DMA0, DMA1, DMA2, DMA3, DMA4
 *  *pDstBuf [in]: Destination buffer address
 *   DataLen [in]: Data length (Byte)
 * @return
 *   A byte data
 */
void I2C_Master_ReadNBytes_DMA(DMA_TYPE_DEF *DmaHandler, uint8_t *pDstBuf, uint32_t DataLen)
{
    uint8_t ack;

    ack = I2C_Master_Start(I2C_READ);
    if (ack == 0x00)
    {
        //NACK.
        return;
    }

    DMA_Init(DmaHandler, DMA_REQSEL_I2C, DMA_SRC_DATA_8B, DMA_SRC_ADDR_FIX, DMA_DST_DATA_8B, DMA_DST_ADDR_INC);
    DMA_Trigger(DmaHandler, (uint32_t)&I2C->DATA, (uint32_t)pDstBuf, DataLen);
    I2C_Master_ReadByteTrigger();
    while(DMA_CheckBusy(DmaHandler) != 0);
    DMA_Close(DmaHandler);
    while(I2C_CheckBusy() != 0);
    I2C_Master_Stop();
}

/**
 * @brief
 *
 * @param
 *   DataAddr [in]:
 *  *pSrcBuf [in]: Source buffer address
 *   DataLen [in]: Data length (Byte)
 * @return
 *   None.
 */
void I2C_Slave_WriteNBytes(uint8_t *pSrcBuf, uint32_t DataLen)
{
    uint8_t ack;

    while((I2C->STS & I2C_STS_SLV_DATA_DONE_FLAG) == 0);
    I2C->STS |= I2C_STS_SLV_DATA_DONE_FLAG | I2C_STS_TRS_DONE_FLAG;

    while(DataLen--)
    {
        ack = I2C_Slave_SendByte(*pSrcBuf++);
        if (ack == 0)
        {
            //NACK
            break;
        }
    }

    while((I2C->STS & I2C_STS_RX_STOP_CMD_FLAG) == 0);
    I2C->STS |= I2C_STS_RX_STOP_CMD_FLAG;
}

/**
 * @brief
 *
 * @param
 *   DataAddr [in]:
 *  *pDstBuf [out]: Destination buffer address
 *   DataLen [in]: Data length (Byte)
 * @return
 *   A byte data
 */
void I2C_Slave_ReadNBytes(uint8_t *pDstBuf, uint32_t DataLen)
{
    while((I2C->STS & I2C_STS_SLV_DATA_DONE_FLAG) == 0);
    I2C->STS |= I2C_STS_SLV_DATA_DONE_FLAG | I2C_STS_TRS_DONE_FLAG;

    while(DataLen--)
    {
        *pDstBuf++ = I2C_Slave_ReadByte();
    }

    while((I2C->STS & I2C_STS_RX_STOP_CMD_FLAG) == 0);
    I2C->STS |= I2C_STS_RX_STOP_CMD_FLAG;
}

/**
 * @brief
 *
 * @param
 *  *DmaHandler [in]: The base address of DMAx module
 *    - DMA0, DMA1, DMA2, DMA3, DMA4
 *  *pSrcBuf [in]: Source buffer address
 *   DataLen [in]: Data length (Byte)
 * @return
 *   None.
 */
void I2C_Slave_WriteNBytes_DMA(DMA_TYPE_DEF *DmaHandler, uint8_t *pSrcBuf, uint32_t DataLen)
{
    CLEAR_BIT(I2C->ADDR, I2C_RW_SEL_READ);
    DMA_Init(DmaHandler, DMA_REQSEL_I2C, DMA_SRC_DATA_8B, DMA_SRC_ADDR_INC, DMA_DST_DATA_8B, DMA_DST_ADDR_FIX);
    DMA_Trigger(DmaHandler, (uint32_t)pSrcBuf, (uint32_t)&I2C->DATA, DataLen);
    while(DMA_CheckBusy(DmaHandler) != 0);
    DMA_Close(DmaHandler);
    while(I2C_CheckBusy() != 0);
    I2C->STS |= 0xFFFF;
}

/**
 * @brief
 *
 * @param
 *  *DmaHandler [in]: The base address of DMAx module
 *    - DMA0, DMA1, DMA2, DMA3, DMA4
 *  *pSrcBuf [in]: Source buffer address
 *   DataLen [in]: Data length (Byte)
 * @return
 *   None.
 */
void I2C_Slave_ReadNBytes_DMA(DMA_TYPE_DEF *DmaHandler, uint8_t *pSrcBuf, uint32_t DataLen)
{
    SET_BIT(I2C->ADDR, I2C_RW_SEL_READ);
    DMA_Init(DmaHandler, DMA_REQSEL_I2C, DMA_SRC_DATA_8B, DMA_SRC_ADDR_FIX, DMA_DST_DATA_8B, DMA_DST_ADDR_INC);
    DMA_Trigger(DmaHandler, (uint32_t)&I2C->DATA, (uint32_t)pSrcBuf, DataLen);
    while(DMA_CheckBusy(DmaHandler) != 0);
    DMA_Close(DmaHandler);
    while(I2C_CheckBusy() != 0);
    I2C->STS |= 0xFFFF;
}

