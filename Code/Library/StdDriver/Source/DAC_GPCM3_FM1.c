/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   DAC_GPCM3_FM1.c
 * @Version:
 *   V1.0.5
 * @Date:
 *   May 08, 2025
 * @Update Information:
 *	2023.10.13 Correct the settings in the DAC_AudioPwm_Open function.
 *  2025.05.08 Set AUDPWM_CTRL_MUTE_TYPE_50DUTY = 1 in DAC mode(Add in DAC_VoltageDAC_CH0_Enable & DAC_VoltageDAC_CH1_Enable)
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
 *   Open DAC and Audio PWM
 * @param
 *   None.
 * @return
 *   None.
 */
void DAC_Open(void)
{
	SET_BIT(CLOCK->APBCKEN, CLOCK_APBCKEN_DAC_CLK_ENABLE);	           // DAC clock enable
}

/**
 * @brief
 *  Close DAC and AUDPWM
 * @param
 *   None.
 * @return
 *   None.
 */
void DAC_Close()
{
	DAC->CTRL = 0;
	CLEAR_BIT(CLOCK->APBCKEN, CLOCK_APBCKEN_DAC_CLK_ENABLE);	         // DAC clock disable
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void DAC_VoltageDac_Enable()
{
//	int16_t DacDataTemp1;
//	int16_t DacDataTemp2;
//	int16_t DacDataTemp3;
//	int16_t DacDataTemp4;
//	int16_t iCount;
//	uint32_t DelayCount;
//
//	SET_BIT(DAC->CTRL, DAC_CTRL_VOLTAGE_DAC_ENABLE);
//
//  DacDataTemp1 = DAC->DAC_CH0_DATA & 0xFFE0;
//	DacDataTemp2 = DacDataTemp1;
//
//  DacDataTemp3 = DAC->DAC_CH1_DATA & 0xFFE0;
//	DacDataTemp4 = DacDataTemp3;
//	do
//	{
//		DacDataTemp2 = DacDataTemp1;
//		DacDataTemp4 = DacDataTemp3;
//	  for(iCount = 0; iCount < 8; iCount++)
//	  {
//	    if(DacDataTemp1 > 0)
//	    {
//			  DacDataTemp1 -= 0x10;
//		  }
//	    else if(DacDataTemp1 < 0)
//		  {
//			  DacDataTemp1 += 0x10;
//		  }
//
//	    if(DacDataTemp3 > 0)
//	    {
//			  DacDataTemp3 -= 0x10;
//		  }
//	    else if(DacDataTemp3 < 0)
//		  {
//			  DacDataTemp3 += 0x10;
//		  }
//
//			DAC->DAC_CH0_DATA = DacDataTemp1;
//			DAC->DAC_CH1_DATA = DacDataTemp3;
//      for(DelayCount = 0; DelayCount < CURRENT_DAC_RAMP_UP_DELAY; DelayCount++);
//	  }
//	} while((DacDataTemp2 != 0) || (DacDataTemp4 != 0));
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void DAC_VoltageDac_Disable()
{
//	int16_t DacDataTemp1;
//	int16_t DacDataTemp2;
//	int16_t DacDataTemp3;
//	int16_t DacDataTemp4;
//	int16_t iCount;
//	uint32_t DelayCount;
//
//	MODIFY_REG(DAC->CTRL, (DAC_CTRL_DAC_CH0_TRG_SEL_MSK | DAC_CTRL_DAC_CH1_TRG_SEL_MSK), (DAC_CTRL_DAC_CH0_TRG_SEL_MANUAL | DAC_CTRL_DAC_CH1_TRG_SEL_Manual));
//
//  DacDataTemp1 = DAC->DAC_CH0_DATA & 0xFFC0;
//	DacDataTemp2 = DacDataTemp1;
//  DacDataTemp3 = DAC->DAC_CH1_DATA & 0xFFC0;
//	DacDataTemp4 = DacDataTemp3;
//	do
//	{
//		DacDataTemp2 = DacDataTemp1;
//		DacDataTemp4 = DacDataTemp3;
//	  for(iCount = 0; iCount < 8; iCount++)
//	  {
//      DacDataTemp1 ^= 0x8000;
//	    if(DacDataTemp1 != 0)
//	    {
//			  DacDataTemp1 -= 0x40;
//		  }
//			DacDataTemp1 ^= 0x8000;
//
//      DacDataTemp3 ^= 0x8000;
//	    if(DacDataTemp3 != 0)
//	    {
//			  DacDataTemp3 -= 0x40;
//		  }
//	    DacDataTemp3 ^= 0x8000;
//
//			DAC->DAC_CH0_DATA = DacDataTemp1;
//			DAC->DAC_CH1_DATA = DacDataTemp3;
//			for(DelayCount = 0; DelayCount < CURRENT_DAC_RAMP_DOWN_DELAY; DelayCount++);
//	  }
//	} while(((DacDataTemp2 ^= 0x8000) != 0) || ((DacDataTemp4 ^= 0x8000) != 0));
//
//	CLEAR_BIT(DAC->CTRL, DAC_CTRL_VOLTAGE_DAC_ENABLE);
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void DAC_VoltageDac_Reset()
{
	DAC->DAC_CH0_DATA = 0x8000;
	DAC->DAC_CH1_DATA = 0x8000;
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void DAC_Scale_Enable()
{
	SET_BIT(DAC->CTRL, DAC_CTRL_SCALE_ENABLE);
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void DAC_Scale_Disable()
{
	CLEAR_BIT(DAC->CTRL, DAC_CTRL_SCALE_ENABLE);
}

/**
 * @brief
 *
 * @param
 *   TriggerSel [in]:
 *    - DAC_TRG_SEL_TM0, DAC_TRG_SEL_TM1, DAC_TRG_SEL_TM2, DAC_TRG_SEL_CCP0_TM,
 *      DAC_TRG_SEL_CCP1_TM, DAC_TRG_SEL_CTS_TM0, DAC_TRG_SEL_CTS_TM1
 * @return
 *   None.
 */
void DAC_CH0_Init(uint32_t TriggerSel)
{
	MODIFY_REG(DAC->CTRL, DAC_CTRL_DAC_CH0_TRG_SEL_MSK, (TriggerSel << DAC_CTRL_DAC_CH0_TRG_SEL_POS));
	MODIFY_REG(DAC->CTRL, DAC_CTRL_DAC_CH0_UPSMP_MODE_MSK, DAC_CTRL_DAC_CH0_UPSMP_MODE_4X);
	SET_BIT(DAC->CTRL, DAC_CTRL_DAC_CH0_ENABLE);	                     //	Enable DAC channel0
}


/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void DAC_CH0_Disable()
{
	CLEAR_BIT(DAC->CTRL, DAC_CTRL_DAC_CH0_ENABLE);	                   //	Disable DAC channel0
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void DAC_CH0_HalfVol_Enable()
{
	SET_BIT(DAC->CTRL, DAC_CTRL_DAC_CH0_HALF_ENABLE);
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void DAC_CH0_HalfVol_Disable()
{
	CLEAR_BIT(DAC->CTRL, DAC_CTRL_DAC_CH0_HALF_ENABLE);
}

/**
 * @brief
 *
 * @param
 *   TriggerSel [in]:
 *    - DAC_TRG_SEL_TM0, DAC_TRG_SEL_TM1, DAC_TRG_SEL_TM2, DAC_TRG_SEL_CCP0_TM,
 *      DAC_TRG_SEL_CCP1_TM, DAC_TRG_SEL_CTS_TM0, DAC_TRG_SEL_CTS_TM1
 * @return
 *   None.
 */
void DAC_CH1_Init(uint32_t TriggerSel)
{
	MODIFY_REG(DAC->CTRL, DAC_CTRL_DAC_CH1_TRG_SEL_MSK, (TriggerSel << DAC_CTRL_DAC_CH1_TRG_SEL_POS));
	MODIFY_REG(DAC->CTRL, DAC_CTRL_DAC_CH1_UPSMP_MODE_MSK, DAC_CTRL_DAC_CH1_UPSMP_MODE_4X);
	SET_BIT(DAC->CTRL, DAC_CTRL_DAC_CH1_ENABLE);	                     //	Enable DAC channel1
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void DAC_CH1_Disable()
{
	CLEAR_BIT(DAC->CTRL, DAC_CTRL_DAC_CH1_ENABLE);	                   //	Disable DAC channel1
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void DAC_CH1_HalfVol_Enable()
{
	SET_BIT(DAC->CTRL, DAC_CTRL_DAC_CH1_HALF_ENABLE);
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void DAC_CH1_HalfVol_Disable()
{
	CLEAR_BIT(DAC->CTRL, DAC_CTRL_DAC_CH1_HALF_ENABLE);
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void DAC_AudioPwm_Open()
{
	SET_BIT(CLOCK->APBCKEN, CLOCK_APBCKEN_AUDPWM_CLK_ENABLE);          // AUDPWM clock enable
  MODIFY_REG(CLOCK->CLKDIV1, CLOCK_CLKDIV1_AUDCLK_SEL_MSK, CLOCK_CLKDIV1_AUDCLK_SEL_SYSCLK_DIV_12);

	// DS3 Setup
  DAC->AUDPWM_CTRL = AUDPWM_CTRL_AUDPWM_ENABLE | AUDPWM_CTRL_MUTE_CTRL_BY_DATACHANGE | AUDPWM_CTRL_MUTE_STATE_SEL_1MHZ | AUDPWM_CTRL_AUDPWM_IP_DISABLE | AUDPWM_CTRL_DS_TYPE_DS3 | AUDPWM_CTRL_AUDPWM_GAIN_LV31 |(0x7C <<21) ; // Auto Mute ON by user
  // DAC->AUDPWM_CTRL2 = AUDPWM_CTRL2_MUTE_OUT_CLEAR_TIME_512US | (0x7D << 2);
	// DAC->AUDPWM_CTRL = AUDPWM_CTRL_AUDPWM_ENABLE | AUDPWM_CTRL_MUTE_CTRL_OFF | AUDPWM_CTRL_MUTE_STATE_SEL_8MHZ | AUDPWM_CTRL_AUDPWM_IP_DISABLE | AUDPWM_CTRL_DS_TYPE_DS6 | AUDPWM_CTRL_AUDPWM_GAIN_LV31 ;
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void DAC_AudioPwm_Close()
{
	DAC->AUDPWM_CTRL = 0;
	CLEAR_BIT(CLOCK->APBCKEN, CLOCK_APBCKEN_AUDPWM_CLK_ENABLE);        // AUDPWM clock disable
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void DAC_AudioPwm_IP_Enable()
{
	MODIFY_REG(DAC->AUDPWM_CTRL, AUDPWM_CTRL_AUDPWM_IP_EN_MSK, AUDPWM_CTRL_AUDPWM_IP_ENABLE);
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void DAC_AudioPwm_IP_Disable()
{
	MODIFY_REG(DAC->AUDPWM_CTRL, AUDPWM_CTRL_AUDPWM_IP_EN_MSK, AUDPWM_CTRL_AUDPWM_IP_DISABLE);
}

/**
 * @brief
 *
 * @param
 *   AudPwmGain [in]: Audio PWM Gain level 0 ~ level 62.
 *    - input range: 0 ~ 62
 * @return
 *   None.
 */
void DAC_SetAudPwmGain(uint32_t AudPwmGain)
{
	MODIFY_REG(DAC->AUDPWM_CTRL, AUDPWM_CTRL_AUDPWM_GAIN_MSK, ((AudPwmGain << AUDPWM_CTRL_AUDPWM_GAIN_POS) & AUDPWM_CTRL_AUDPWM_GAIN_MSK));
}
/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void DAC_PostWaveWithSPU()
{
  CLEAR_BIT(DAC->CTRL, DAC_CTRL_CH0_POSTWAVE_OUT_CH0 + DAC_CTRL_CH1_POSTWAVE_OUT_CH1);
}
/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void DAC_PostWaveWithoutSPU()
{
  SET_BIT(DAC->CTRL, DAC_CTRL_CH0_POSTWAVE_OUT_CH0 + DAC_CTRL_CH1_POSTWAVE_OUT_CH1);
}
/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void DAC_VoltageDAC_CH0_Enable()
{
  SET_BIT(DAC->AUDPWM_CTRL, (AUDPWM_CTRL_DAC_CH0_ENABLE| AUDPWM_CTRL_MUTE_TYPE_50DUTY));
}


/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void DAC_VoltageDAC_CH0_Disable()
{
  CLEAR_BIT(DAC->AUDPWM_CTRL, (AUDPWM_CTRL_DAC_CH0_ENABLE| AUDPWM_CTRL_MUTE_TYPE_50DUTY));
}
/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void DAC_VoltageDAC_CH1_Enable()
{
  SET_BIT(DAC->AUDPWM_CTRL, (AUDPWM_CTRL_DAC_CH1_ENABLE| AUDPWM_CTRL_MUTE_TYPE_50DUTY));
}
/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void DAC_VoltageDAC_CH1_Disable()
{
  CLEAR_BIT(DAC->AUDPWM_CTRL, (AUDPWM_CTRL_DAC_CH1_ENABLE| AUDPWM_CTRL_MUTE_TYPE_50DUTY));
}



