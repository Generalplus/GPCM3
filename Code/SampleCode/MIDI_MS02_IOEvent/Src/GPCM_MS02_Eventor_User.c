/**************************************************************************************************
 * Copyright(c) 2020 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   MS02_EVENTOR_User.c
 * @Version:
 *   V0.9.1
 * @Date:
 *   March 20, 2024
 * @Abstract:
 *   1. Modify for MS02 Event of the GPCMx series
 *
 **************************************************************************************************/


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "GPCM3_FM1.h"
#include "GPCM_MS02_Eventor_User.h"


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
const uint32_t DefMS02EventIo[] =
{
	(uint32_t) &GPIOA_OBIT->OBIT13,
	(uint32_t) &GPIOA_OBIT->OBIT14,
	(uint32_t) &GPIOA_OBIT->OBIT15
};


/*---------------------------------------------------------------------------------------
 * Gobal Variable Declaration Area
 *---------------------------------------------------------------------------------------*/
TIMER_TYPE_DEF *mMS02EventTimerHandle;


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
	MS02_EVENTOR_ISR_Service(MS02_EVENT_CH2);

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
 *   When user call API function MS02_EVENTOR_Initial, this function is called by MS02_EVENTOR Library.
 * @param
 *   EventorChannelSel [in]: Eventor channel selection
 *    - EVENTOR_CH0(0), EVENTOR_CH1(1), EVENTOR_CH2(2), EVENTOR_CH3(3).
 *  *MS02EvtWorkingRam [in]: MS02_EVENTOR library working RAM address.
 * @return
 *   None.
 */
void MS02_EVENTOR_CB_Init(const uint8_t EventorChannelSel, const MS02_EVENTOR_WORKING_RAM *MS02EvtWorkingRam)
{
  mMS02EventTimerHandle = TM2;
}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   This function will be called by library when IO event start. The state of IO pins can be set by user.
 * @param
 *   EventorChannelSel [in]: Eventor channel selection
 *    - EVENTOR_CH0(0), EVENTOR_CH1(1), EVENTOR_CH2(2), EVENTOR_CH3(3).
 *  *MS02EvtWorkingRam [in]: MS02_EVENTOR library working RAM address.
 * @return
 *   None.
 */
void MS02_EVENTOR_CB_IoEvtStart(const uint8_t EventorChannelSel, const MS02_EVENTOR_WORKING_RAM *MS02EvtWorkingRam)
{
	uint32_t iCount;

	switch(EventorChannelSel)
	{
		case MS02_EVENT_CH0:
			break;
		case MS02_EVENT_CH1:
			break;
		case MS02_EVENT_CH2:
	    /*
	     * Set S/W PWM IO as output low
	     */
	    for(iCount = 0; iCount < MS02_EVENT_IO_NUM; iCount++)
	    {
		    GPIO_SetMode((GPIO_TYPE_DEF *)(DefMS02EventIo[iCount] & 0xFFFFFF00), (0x1 << (((DefMS02EventIo[iCount] & 0x000000FF) - (GPIOA_BIT_OPERATION_BASE - GPIOA_BASE)) >> 2)), GPIO_MODE_OUTPUT);
		    *(uint32_t*)DefMS02EventIo[iCount] = 0;
	    }
			break;
	}

	/*
	 * Initialize Timer and enable Timer interrupt
	 */
	TIMER_Open(mMS02EventTimerHandle);
	TIMER_SetFreq(mMS02EventTimerHandle, MS02_EVENT_SW_PWM_TIMER_SETTING);
	TIMER_EnableInt(mMS02EventTimerHandle);
	NVIC_EnableIRQ(TIMER2_IRQn);

}

/**
 * @brief
 *   Callback function. Implemented by user.
 *   This function will be called by library when IO event end. The state of IO pins can be set by user.
 * @param
 *   EventorChannelSel [in]: Eventor channel selection
 *    - EVENTOR_CH0(0), EVENTOR_CH1(1), EVENTOR_CH2(2), EVENTOR_CH3(3).
 *  *MS02EvtWorkingRam [in]: MS02_EVENTOR library working RAM address.
 * @return
 *   None.
 */
void MS02_EVENTOR_CB_IoEvtEnd(const uint8_t EventorChannelSel, const MS02_EVENTOR_WORKING_RAM *MS02EvtWorkingRam)
{
	uint32_t iCount;

	TIMER_Close(mMS02EventTimerHandle);

	switch(EventorChannelSel)
	{
		case MS02_EVENT_CH0:
			break;

		case MS02_EVENT_CH1:
			break;

		case MS02_EVENT_CH2:
	    for(iCount = 0; iCount < MS02_EVENT_IO_NUM; iCount++)
	    {
		    *(uint32_t*)DefMS02EventIo[iCount] = 0;
	    }
			break;
	}
}


/**
 * @brief
 * 	 When a user event is decoded, MS02_EVENTOR_CB_EvtProcess will be executed.
 * 	 User can process the user event in this function.
 * @param
 *   EventorChannelSel [in]: Eventor channel selection
 *    - EVENTOR_CH0(0), EVENTOR_CH1(1), EVENTOR_CH2(2), EVENTOR_CH3(3).
 *  *MS02EvtWorkingRam [in]: MS02_EVENTOR library working RAM address.
 *   EventData = SubIndex(8-bits):MainIndex(8-bits)
 * @return
 *   None.
 */
void MS02_EVENTOR_CB_UserEvtProcess(const uint8_t EventorChannelSel, const MS02_EVENTOR_WORKING_RAM *MS02EvtWorkingRam, uint16_t EventData)
{
	switch(EventorChannelSel)
	{
		case MS02_EVENT_CH0:
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
 *  *MS02EvtWorkingRam [in]: MS02_EVENTOR library working RAM address.
 * @return
 *   None.
 */
void MS02_EVENTOR_CB_IoEvt_ISR_Service(const uint8_t EventorChannelSel, const MS02_EVENTOR_WORKING_RAM *MS02EvtWorkingRam)
{
	uint32_t iCount;

	switch(EventorChannelSel)
	{
		case MS02_EVENT_CH0:
      break;

 		case MS02_EVENT_CH1:
      break;

		case MS02_EVENT_CH2:
	    for(iCount = 0; iCount < MS02_EVENT_IO_NUM; iCount++)
	    {
		    if(MS02EvtWorkingRam->SwPwmWorkingRamPtr[iCount] >= 0)
		    {
			    if(MS02EvtWorkingRam->SwPwmWorkingRamPtr[iCount] >= MS02EvtWorkingRam->SwPwmDutyCounter)
			    {
				    *(uint32_t*)DefMS02EventIo[iCount] = 0xFFFFFFFF;
			    }
			    else
			    {
				    *(uint32_t*)DefMS02EventIo[iCount] = 0;
			    }
		    }
			}
			break;
	}
}
