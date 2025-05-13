/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   I2S_GPCM1Fx.c
 * @Version:
 *   V0.9.1
 * @Date:
 *   Nov. 19, 2024
 * @Abstract:
 *        2024.11.19 Modify I2S_RX_Init() to match with I2S_TX_Initial
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
 *   None.
 * @return
 *   None.
 */
void I2S_RX_Init(void)
{
  MODIFY_REG(IOFUNC->CTRL0, GPIOFUNC_CTRL0_I2S_IN_IOSEL_MSK, GPIOFUNC_CTRL0_I2S_IN_IOA26_29);
	SET_BIT(CLOCK->APBCKEN, CLOCK_APBCKEN_I2S_CLK_ENABLE);	           // I2S clock enable
	WRITE_REG(I2S->RX_CTRL2, I2S_RX_CTRL2_BCLK_DIV_8 | I2S_RX_CTRL2_MCLK_DIV_8);
  WRITE_REG(I2S->RX_CTRL, I2S_RX_CTRL_MODE_SEL_SLAVE | I2S_RX_CTRL_FRAME_MODE_NORMAL | I2S_RX_CTRL_FRAME_SIZE_Slave
          | I2S_RX_CTRL_DATA_LENGTH_16BITS | I2S_RX_CTRL_ALIGN_LEFT |I2S_RX_CTRL_MONO_MODE_ENABLE | I2S_RX_CTRL_MERGE_ENABLE);
  // WRITE_REG(I2S->RX_CTRL, I2S_RX_CTRL_MODE_SEL_SLAVE | I2S_RX_CTRL_FRAME_MODE_NORMAL | I2S_RX_CTRL_FRAME_SIZE_Slave | I2S_RX_CTRL_DATA_LENGTH_16BITS | I2S_RX_CTRL_ALIGN_LEFT);
}

void I2S_RX_Enable(void)
{
	MODIFY_REG(I2S->RX_CTRL, I2S_RX_CTRL_EN_MSK, I2S_RX_CTRL_ENABLE);
}

void I2S_RX_Disable(void)
{
	MODIFY_REG(I2S->RX_CTRL, I2S_RX_CTRL_EN_MSK, I2S_RX_CTRL_DISABLE);
}


/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void I2S_TX_Initial(void)
{
	SET_BIT(CLOCK->APBCKEN, CLOCK_APBCKEN_I2S_CLK_ENABLE);	           // I2S clock enable
	WRITE_REG(I2S->TX_CTRL, I2S_TX_CTRL_MCLK_DIV_8 | I2S_TX_CTRL_BCLK_DIV_8 | I2S_TX_CTRL_ENABLE |I2S_TX_CTRL_ALIGN_LEFT | I2S_TX_CTRL_DATA_FIRST_MSB | I2S_TX_CTRL_FRAME_SIZE_16BITS | I2S_TX_CTRL_MCLK_ENABLE | I2S_TX_CTRL_DATA_FMT_SIGN | I2S_TX_CTRL_FRAME_MODE_NORMAL | I2S_TX_CTRL_DATA_SRC_POSTWAVE);

}

void I2S_TX_Enable(void)
{
	MODIFY_REG(I2S->TX_CTRL, I2S_TX_CTRL_EN_MSK, I2S_TX_CTRL2_DELAY_DISABLE);
}

void I2S_TX_Disable(void)
{
	MODIFY_REG(I2S->TX_CTRL, I2S_TX_CTRL_EN_MSK, I2S_TX_CTRL_ENABLE);
}

