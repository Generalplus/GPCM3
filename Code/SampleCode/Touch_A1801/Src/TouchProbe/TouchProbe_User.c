//=================================================================================================
// File Name	: TouchProbe_User.c
// Description	: Touch Probe setting
// Written by	: Porter Yang
// Last modified date:
//                2019/08/02
// Note:
//=================================================================================================

//***************************************************************************************
// Header File Included Area
//***************************************************************************************
#include "GPCM3_FM1.h"
#include "TouchProbe_User.h"

//***************************************************************************************
// Public Function List
//***************************************************************************************

//***************************************************************************************
// Constant Definition Area
//***************************************************************************************
#define	C_PadNumber					8	// Max. = 10

/*
 *  User Setting:
 *   - TP_OUTPUT_SEL: Select SPI0_IOA[12:9]/SPI0_IOB[4:1]/SPI1_IOA[3:0]/SPI1_IOA[25:22]
 */
#define  SPI0_IOA9_12				(1)
#define  SPI0_IOB1_4        (2)
#define  SPI1_IOA0_3        (3)
#define  SPI1_IOA22_25      (4)
#define  TP_OUTPUT_SEL      SPI1_IOA22_25             // User Setting
/*---------------------------------------------------------------------------------------
 * Module Vairable Declaration Area
 *---------------------------------------------------------------------------------------*/
SPI_TYPE_DEF 	*mTpHandle;
__IO uint32_t *mTpCsPin;
uint16_t R_TP_Buffer[C_PadNumber];				// For Touch Probe Used Only


//***************************************************************************************
// Table Definition Area (be used by Touch Library)
//***************************************************************************************
//for TP Library (Don't Modify)
#define C_Delay_AfterCSLow		30	//15			//unit:us
#define C_Delay_BeforeCSHigh	10	//5				//unit:us
#define C_Delay_AfterCSHigh		30	//15			//unit:us

const uint16_t T_TPDelay[3] = 		//for TP Library (Don't Modify)
{
	C_Delay_AfterCSLow,
	C_Delay_BeforeCSHigh,
	C_Delay_AfterCSHigh
};


//***************************************************************************************
// CODE Definition Area
//***************************************************************************************
/**
 * @brief
 *		Hardware initilazation for Touch Probe
 * @param
 *	 	None.
 * @return
 *   	None.
 */
void F_TP_HW_Init(void)
{
	#if TP_OUTPUT_SEL == SPI0_IOA9_12

	SET_BIT(CLOCK->APBCKEN, CLOCK_APBCKEN_SPI0_CLK_ENABLE);                                          // APB SPI0 Clock enable
	MODIFY_REG(IOFUNC->CTRL0, GPIOFUNC_CTRL0_SPI0_HWCS_EN_MSK, GPIOFUNC_CTRL0_SPI0_HWCS_DISABLE);    // SPI0 S/W CS
	MODIFY_REG(IOFUNC->CTRL0, GPIOFUNC_CTRL0_SPI0_IOSEL_MSK, GPIOFUNC_CTRL0_SPI0_IOA9_12);

	mTpHandle = SPI0;
  GPIO_SetMode(GPIOA, 0x00001E00, GPIO_MODE_INPUT);
  GPIO_SetMode(GPIOA, 0x00001E00, GPIO_MODE_INPUT);

	mTpCsPin = &GPIOA_OBIT->OBIT11;
	#endif

	#if TP_OUTPUT_SEL == SPI0_IOB1_4
	SET_BIT(CLOCK->APBCKEN, CLOCK_APBCKEN_SPI0_CLK_ENABLE);                                          // APB SPI0 Clock enable
	MODIFY_REG(IOFUNC->CTRL0, GPIOFUNC_CTRL0_SPI0_HWCS_EN_MSK, GPIOFUNC_CTRL0_SPI0_HWCS_DISABLE);    // SPI0 S/W CS
	MODIFY_REG(IOFUNC->CTRL0, GPIOFUNC_CTRL0_SPI0_IOSEL_MSK, GPIOFUNC_CTRL0_SPI0_IOB1_4);
  mTpHandle = SPI0;
	mTpCsPin = &GPIOB_OBIT->OBIT03;
	#endif

	#if TP_OUTPUT_SEL == SPI1_IOA0_3
	SET_BIT(CLOCK->APBCKEN, CLOCK_APBCKEN_SPI1_CLK_ENABLE);                                          // APB SPI1 Clock enable
	MODIFY_REG(IOFUNC->CTRL0, GPIOFUNC_CTRL0_SPI1_HWCS_EN_MSK, GPIOFUNC_CTRL0_SPI1_HWCS_DISABLE);    // SPI1 S/W CS
	MODIFY_REG(IOFUNC->CTRL0, GPIOFUNC_CTRL0_SPI1_IOSEL_MSK, GPIOFUNC_CTRL0_SPI1_IOA0_4);
  mTpHandle = SPI1;
	mTpCsPin = &GPIOA_OBIT->OBIT02;
	#endif

	#if TP_OUTPUT_SEL == SPI1_IOA22_25
	GPIO_SetMode(GPIOA, BIT25, GPIO_MODE_FLOATING);
	GPIO_SetMode(GPIOA, (BIT22 | BIT23 | BIT24), GPIO_MODE_OUTPUT);
	GPIOA->OBUF &= ~(BIT22 | BIT23);
	GPIOA->OBUF |= BIT24;

	SET_BIT(CLOCK->APBCKEN, CLOCK_APBCKEN_SPI1_CLK_ENABLE);                                          // APB SPI1 Clock enable
	MODIFY_REG(IOFUNC->CTRL0, GPIOFUNC_CTRL0_SPI1_HWCS_EN_MSK, GPIOFUNC_CTRL0_SPI1_HWCS_DISABLE);    // SPI1 S/W CS
	MODIFY_REG(IOFUNC->CTRL0, GPIOFUNC_CTRL0_SPI1_IOSEL_MSK, GPIOFUNC_CTRL0_SPI1_IOA22_25);
  mTpHandle = SPI1;
	mTpCsPin = &GPIOA_OBIT->OBIT24;

	mTpHandle->CTRL = SPI_CTRL_MASTER_MODE | SPI_CTRL_SPI_ENABLE | SPI_CTRL_MAUT_RW_EN_ENABLE;
	mTpHandle->CLK_DIV = SPI_CLK_DIV_PCLK_DIV_16;
	*mTpCsPin = 0xFFFFFFFF;                                                                          // CS high
	#endif
}


/**
 * @brief
 *		delay 1us after CS go HIGH
 * @param
 *
 * @return
 *   	None.
 */
void F_TP_SetCSHigh(uint32_t DelayT)
{
	*mTpCsPin = 0xFFFFFFFF;              // enable SPI Flash
	F_TP_Delay_1us(DelayT);								// delay R1 uS for touch probe process data
}


/**
 * @brief
 *		delay 1uS after CS go LOW
 * @param
 *
 * @return
 *   	None.
 */
void F_TP_SetCSLow(uint32_t lDelayTime)
{
	*mTpCsPin = 0x00000000;
	F_TP_Delay_1us(lDelayTime);											// delay R1 uS for touch probe process data
}


/**
 * @brief
 *		Delay about 1us
 * @param
 *	 	[IN]: Delay Count (Unit 1us)
 * @return
 *   	None
 */
void F_TP_Delay_1us(uint32_t DelayT)
{
	uint16_t i, j;

	for(i=0;i<DelayT;i++)
	{
		for(j=0;j<10;j++);
		WDT_Clear();
	}

}

/**
 * @brief
 *		Send Data to Touch Probe
 * @param
 *	 	[IN] Buffer Address,
 *    [IN] Data Length (bytes)
 * @return
 *   	[OUT] Checksum
 */
uint32_t F_TP_SendDataToTouchProbe(uint8_t *pSrcBuf, uint32_t DataLength)
{
	uint32_t index;
	uint32_t R_CheckSum;
	uint8_t R_Data;

	R_CheckSum = 0;
	for(index = 0; index < DataLength; index++)
	{
		R_Data = *pSrcBuf++;

		F_TP_SendAByte(R_Data);
//		F_TP_Delay_1us(1);
		R_CheckSum += R_Data;
	}
	return R_CheckSum;															// return checksum
}

/**
 * @brief
 *		Send A Byte
 * @param
 *	 	[IN] one byte data
 * @return
 *   	None
 */
void F_TP_SendAByte(uint8_t Data)
{
	mTpHandle->TX_DATA = Data;
	while((READ_BIT(mTpHandle->STS, SPI_CTRL_TX_DONE_FLAG)) == 0);    // Wait until SPI transmission is completed.
	mTpHandle->STS |= SPI_CTRL_TX_DONE_FLAG;                          // Clear transmission complete flag
}

/**
 * @brief
 *		Send A Word
 * @param
 *	 	[IN] one word data
 * @return
 *   	None
 */
void F_TP_SendAWord(uint16_t wData)
{
	mTpHandle->TX_DATA = (wData & 0x00FF);
	while((READ_BIT(mTpHandle->STS, SPI_CTRL_TX_DONE_FLAG)) == 0);    // Wait until SPI transmission is completed.
	mTpHandle->STS |= SPI_CTRL_TX_DONE_FLAG;                          // Clear transmission complete flag

	mTpHandle->TX_DATA = (wData >> 8);
	while((READ_BIT(mTpHandle->STS, SPI_CTRL_TX_DONE_FLAG)) == 0);    // Wait until SPI transmission is completed.
	mTpHandle->STS |= SPI_CTRL_TX_DONE_FLAG;                          // Clear transmission complete flag

}

/**
 * @brief
 *		Read A Byte
 * @param
 *	 	None
 * @return
 *   	[OUT]R1 = one byte data
 */
uint8_t F_TP_ReadAByte(void)
{
	mTpHandle->CTRL |= SPI_CTRL_MASTER_RX_TRIG_ENABLE;
	while((mTpHandle->STS&SPI_CTRL_TX_DONE_FLAG) == 0);               // Wait until SPI transmission is completed.
	mTpHandle->STS |= SPI_CTRL_TX_DONE_FLAG;                          // Clear transmission complete flag

	return mTpHandle->RX_DATA;

}

/**
 * @brief
 *		Read A Word
 * @param
 *	 	None
 * @return
 *   	[OUT]R1 = one word data
 */
uint16_t F_TP_ReadAWord(void)
{
	uint16_t temp;
	mTpHandle->CTRL |= SPI_CTRL_MASTER_RX_TRIG_ENABLE;
	while((mTpHandle->STS&SPI_CTRL_TX_DONE_FLAG) == 0);               // Wait until SPI transmission is completed.
	mTpHandle->STS |= SPI_CTRL_TX_DONE_FLAG;                          // Clear transmission complete flag
	temp = mTpHandle->RX_DATA;

	mTpHandle->CTRL |= SPI_CTRL_MASTER_RX_TRIG_ENABLE;
	while((mTpHandle->STS&SPI_CTRL_TX_DONE_FLAG) == 0);               // Wait until SPI transmission is completed.
	mTpHandle->STS |= SPI_CTRL_TX_DONE_FLAG;                          // Clear transmission complete flag
	temp |= (mTpHandle->RX_DATA<<8);
	return temp;
}



