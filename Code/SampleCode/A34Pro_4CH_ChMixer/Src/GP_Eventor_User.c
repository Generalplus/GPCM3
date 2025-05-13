/**************************************************************************************************
 * Copyright(c) 2020 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   GP_Eventor_User.c
 * @Version:
 *   V0.9.0
 * @Date:
 *   December 22, 2020
 * @Abstract:
 *
 **************************************************************************************************/


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "GPCM3_FM1.h"
#include "GP_Eventor_User.h"


/*---------------------------------------------------------------------------------------
 * Table Definition Area
 *---------------------------------------------------------------------------------------*/
/*
 * Eventor IO Definition
 *  - Defined by user.
 *  - Max. 4ch
 *  - Mapped to the number of Edit Event on G+ Eventor.
 */
#if defined(__CC_ARM)
#pragma diag_suppress 1296
#endif
const uint32_t DefEventorIo_Ch0[] =
{
	(uint32_t) &GPIOA_OBIT->OBIT09,
	(uint32_t) &GPIOA_OBIT->OBIT10,
	(uint32_t) &GPIOA_OBIT->OBIT11
};

const uint32_t DefEventorIo_Ch1[] =
{
	(uint32_t) &GPIOA_OBIT->OBIT12,
	(uint32_t) &GPIOA_OBIT->OBIT13,
	(uint32_t) &GPIOA_OBIT->OBIT14
};

const uint32_t DefEventorIo_Ch2[] =
{
	(uint32_t) &GPIOA_OBIT->OBIT16,
	(uint32_t) &GPIOA_OBIT->OBIT17,
	(uint32_t) &GPIOA_OBIT->OBIT18
};

const uint32_t DefEventorIo_Ch3[] =
{
	(uint32_t) &GPIOB_OBIT->OBIT00,
	(uint32_t) &GPIOB_OBIT->OBIT01,
	(uint32_t) &GPIOB_OBIT->OBIT02
};


/*---------------------------------------------------------------------------------------
 * Gobal Variable Declaration Area
 *---------------------------------------------------------------------------------------*/
TIMER_TYPE_DEF *mEventorTimerHandle;


/*---------------------------------------------------------------------------------------
 * IRQ Handler Area
 *---------------------------------------------------------------------------------------*/
/**
 * @brief
 *
 * @param
 *	 None.
 * @return
 *   None.
 */
void TIMER2_IRQHandler(void)
{
	GP_EVENTOR_ISR_Service(EVENTOR_CH0);
	GP_EVENTOR_ISR_Service(EVENTOR_CH1);

	TM_INT->INTSTS = TIMER_INTSTS_TM2_INT_FLAG;
}


/*---------------------------------------------------------------------------------------
 * User Subroutine Area
 *---------------------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------------------
 * Callback Function Area
 *---------------------------------------------------------------------------------------*/

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When user call API function GP_EVENTOR_Initial, this function is called by GP_EVENTOR Library.
 * @param
 *   EventorChannelSel [in]: Eventor channel selection
 *    - EVENTOR_CH0(0), EVENTOR_CH1(1), EVENTOR_CH2(2), EVENTOR_CH3(3).
 *  *GpEvtWorkingRam [in]: GP_EVENTOR library working RAM address.
 * @return
 *   None.
 */
void GP_EVENTOR_CB_Init(const uint8_t EventorChannelSel, const GP_EVENTOR_WORKING_RAM *GpEvtWorkingRam)
{
  mEventorTimerHandle = TM2;
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   This function will be called by library when IO event start. The state of IO pins can be set by user.
 * @param
 *   EventorChannelSel [in]: Eventor channel selection
 *    - EVENTOR_CH0(0), EVENTOR_CH1(1), EVENTOR_CH2(2), EVENTOR_CH3(3).
 *  *GpEvtWorkingRam [in]: GP_EVENTOR library working RAM address.
 * @return
 *   None.
 */
void GP_EVENTOR_CB_IoEvtStart(const uint8_t EventorChannelSel, const GP_EVENTOR_WORKING_RAM *GpEvtWorkingRam)
{
	uint32_t iCount;

	switch(EventorChannelSel)
	{
		case EVENTOR_CH0:
	    /*
	     * Set S/W PWM IO as output low
	     */
	    for(iCount = 0; iCount < EVENTOR_CH0_IO_NUM; iCount++)
	    {
		    GPIO_SetMode((GPIO_TYPE_DEF *)(DefEventorIo_Ch0[iCount] & 0xFFFFFF00), (0x1 << (((DefEventorIo_Ch0[iCount] & 0x000000FF) - (GPIOA_BIT_OPERATION_BASE - GPIOA_BASE)) >> 2)), GPIO_MODE_OUTPUT);
		    *(uint32_t*)DefEventorIo_Ch0[iCount] = 0;
	    }
			break;

		case EVENTOR_CH1:
	    /*
	     * Set S/W PWM IO as output low
	     */
	    for(iCount = 0; iCount < EVENTOR_CH0_IO_NUM; iCount++)
	    {
		    GPIO_SetMode((GPIO_TYPE_DEF *)(DefEventorIo_Ch1[iCount] & 0xFFFFFF00), (0x1 << (((DefEventorIo_Ch1[iCount] & 0x000000FF) - (GPIOA_BIT_OPERATION_BASE - GPIOA_BASE)) >> 2)), GPIO_MODE_OUTPUT);
		    *(uint32_t*)DefEventorIo_Ch1[iCount] = 0;
	    }
			break;
	}

	if(((GP_EVENTOR_GetStatus(EVENTOR_CH0) & EVENTOR_EN_FLAG) != 0) || (GP_EVENTOR_GetStatus(EVENTOR_CH1) & EVENTOR_EN_FLAG) != 0)
	{
	  /*
	   * Initialize Timer and enable Timer interrupt
	   */
	  TIMER_Open(mEventorTimerHandle);
	  TIMER_SetFreq(mEventorTimerHandle, EVENTOR_CH0_SW_PWM_TIMER_SETTING);
	  TIMER_EnableInt(mEventorTimerHandle);
	  NVIC_EnableIRQ(TIMER2_IRQn);
	}
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   This function will be called by library when IO event end. The state of IO pins can be set by user.
 * @param
 *   EventorChannelSel [in]: Eventor channel selection
 *    - EVENTOR_CH0(0), EVENTOR_CH1(1), EVENTOR_CH2(2), EVENTOR_CH3(3).
 *  *GpEvtWorkingRam [in]: GP_EVENTOR library working RAM address.
 * @return
 *   None.
 */
void GP_EVENTOR_CB_IoEvtEnd(const uint8_t EventorChannelSel, const GP_EVENTOR_WORKING_RAM *GpEvtWorkingRam)
{
	uint32_t iCount;

	if(((GP_EVENTOR_GetStatus(EVENTOR_CH0) & EVENTOR_EN_FLAG) == 0) && (GP_EVENTOR_GetStatus(EVENTOR_CH1) & EVENTOR_EN_FLAG) == 0)
	{
	  TIMER_Close(mEventorTimerHandle);
	}

	switch(EventorChannelSel)
	{
		case EVENTOR_CH0:
	    for(iCount = 0; iCount < EVENTOR_CH0_IO_NUM; iCount++)
	    {
		    *(uint32_t*)DefEventorIo_Ch0[iCount] = 0;
	    }
			break;

		case EVENTOR_CH1:
	    for(iCount = 0; iCount < EVENTOR_CH0_IO_NUM; iCount++)
	    {
		    *(uint32_t*)DefEventorIo_Ch1[iCount] = 0;
	    }
	}
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   When Eventor working, this function is called by GP_EVENTOR Library to get event data.
 * @param
 *   EventorChannelSel [in]: Eventor channel selection
 *    - EVENTOR_CH0(0), EVENTOR_CH1(1), EVENTOR_CH2(2), EVENTOR_CH3(3).
 *  *GpEvtWorkingRam [in]: GP_EVENTOR library working RAM address.
 *  *DstBufAddr [out]: Destination buffer address pointer.
 *  *SrcDataAddr [in]: Source data address pointer.
 *   DataLen [in]: Data length (byte)
 * @return
 *   None.
 */
void GP_EVENTOR_CB_GetData(const uint8_t EventorChannelSel, const GP_EVENTOR_WORKING_RAM *GpEvtWorkingRam, uint8_t *DstBufAddr, uint8_t *SrcDataAddr, uint16_t DataLen)
{
  ///*
	uint32_t iCount;

	for(iCount = 0; iCount < DataLen; iCount++)
  {
    DstBufAddr[iCount] = SrcDataAddr[iCount];
	}
  //*/

	/*
  DMA_Init(DMA2, DMA_REQSEL_MEM, DMA_SRC_DATA_8B, DMA_SRC_ADDR_INC, DMA_DST_DATA_8B, DMA_DST_ADDR_INC);
	DMA_Trigger(DMA2, (uint32_t)SrcDataAddr, (uint32_t)DstBufAddr, DataLen);
	while(DMA_CheckBusy(DMA2) != 0);
	DMA_Close(DMA2);
	*/

  //SPI_Flash_ReadNBytes((uint32_t)SrcDataAddr, (uint8_t *)DstBufAddr, DataLen);
}

/**
 * @brief
 * 	 When a user event is decoded, GP_EVENTOR_CB_EvtProcess will be executed.
 * 	 User can process the user event in this function.
 * @param
 *   EventorChannelSel [in]: Eventor channel selection
 *    - EVENTOR_CH0(0), EVENTOR_CH1(1), EVENTOR_CH2(2), EVENTOR_CH3(3).
 *  *GpEvtWorkingRam [in]: GP_EVENTOR library working RAM address.
 *   EventData = SubIndex(8-bits):MainIndex(8-bits)
 * @return
 *   None.
 */
void GP_EVENTOR_CB_UserEvtProcess(const uint8_t EventorChannelSel, const GP_EVENTOR_WORKING_RAM *GpEvtWorkingRam, uint16_t EventData)
{
	switch(EventorChannelSel)
	{
		case EVENTOR_CH0:
	  __NOP();
		break;

    case EVENTOR_CH1:
	  __NOP();
		break;
	}
}

/**
 * @brief
 *
 * @param
 *   EventorChannelSel [in]: Eventor channel selection
 *    - EVENTOR_CH0(0), EVENTOR_CH1(1), EVENTOR_CH2(2), EVENTOR_CH3(3).
 *  *GpEvtWorkingRam [in]: GP_EVENTOR library working RAM address.
 * @return
 *   None.
 */
void GP_EVENTOR_CB_IoEvt_ISR_Service(const uint8_t EventorChannelSel, const GP_EVENTOR_WORKING_RAM *GpEvtWorkingRam)
{
	uint32_t iCount;

	switch(EventorChannelSel)
	{
		case EVENTOR_CH0:
	    for(iCount = 0; iCount < EVENTOR_CH0_IO_NUM; iCount++)
	    {
		    if(GpEvtWorkingRam->SwPwmWorkingRamPtr[iCount] >= 0)
		    {
			    if(GpEvtWorkingRam->SwPwmWorkingRamPtr[iCount] >= GpEvtWorkingRam->SwPwmDutyCounter)
			    {
				    *(uint32_t*)DefEventorIo_Ch0[iCount] = 0xFFFFFFFF;
			    }
			    else
			    {
				    *(uint32_t*)DefEventorIo_Ch0[iCount] = 0;
			    }
		    }
			}
			break;

		case EVENTOR_CH1:
	    for(iCount = 0; iCount < EVENTOR_CH0_IO_NUM; iCount++)
	    {
		    if(GpEvtWorkingRam->SwPwmWorkingRamPtr[iCount] >= 0)
		    {
			    if(GpEvtWorkingRam->SwPwmWorkingRamPtr[iCount] >= GpEvtWorkingRam->SwPwmDutyCounter)
			    {
				    *(uint32_t*)DefEventorIo_Ch1[iCount] = 0xFFFFFFFF;
			    }
			    else
			    {
				    *(uint32_t*)DefEventorIo_Ch1[iCount] = 0;
			    }
		    }
			}
			break;
	}
}
