/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   SPU_GPCM2_CM3.c
 * @Version:
 *   V0.9.1
 * @Date:
 *   Aug 18, 2021
 * @Abstract:
 *
 **************************************************************************************************/


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "GPCM3_FM1.h"

#define C_PCM_AutoEnd        0
#define C_PCM_LOOP           1
#define C_ADPCM_AutoEnd      2
#define C_ADPCM_LOOP         3
#define C_ADPCM_PCM_LOOP     4
#define SPU_CH_PAN_VELOCITY_DEFAULT		      (SPU_CH_VOLUME_PAN_DEFAULE + SPU_CH_VELOCITY_0x7F)
#define SPU_CH_WAVE_ADPCM36_DEFAULT				  (SPU_CH_WAVE_ADPCM36 + SPU_CH_WAVE_ADPCM_FRAMEPOINT_32P)
#define SPU_CH_WAVE_ADPCM_DEFAULT				    (SPU_CH_WAVE_ADPCM_NORMAL + SPU_CH_WAVE_ADPCM_FRAMEPOINT_32P)


// 8000*(2^19)/287425.15
// 8000*(2^19)/294323
// X * (1.781335)
#define SampleRate_32K                      0xDEAA
#define SampleRate_16K                      0x6F55
#define SampleRate_8K                       0x37AA



/*---------------------------------------------------------------------------------------
 * Subroutine Area
 *---------------------------------------------------------------------------------------*/
//SPU_Attrib_RAM *SPU_Attrib_RAMPtr;
/**
 * @brief
 *   SECTION 1. 8-bit Single Channel Test
 * @param
 *   None.
 * @return
 *   None.
 */
void SPU_Initial()
{
  NVIC_EnableIRQ(SPUBEAT_IRQn);
//  CLEAR_BIT(DAC->CTRL, BIT19 + BIT22);
	SET_BIT(CLOCK->AHBCKEN, CLOCK_AHBCKEN_SPU_CLK_ENABLE);
	MODIFY_REG(DAC->CTRL, DAC_CTRL_CH0_POSTWAVE_OUT_SRC_MSK + DAC_CTRL_CH1_POSTWAVE_OUT_SRC_MSK, DAC_CTRL_CH1_POSTWAVE_OUT_CH1ANDSPU + DAC_CTRL_CH0_POSTWAVE_OUT_CH0ANDSPU);
	WRITE_REG(SPU->SPU_EN_CH0_15, 0x0000);	   //clear channel enable
	while(READ_REG(SPU->CH_STATUS_CH0_15));
	WRITE_REG(SPU->SPU_EN_CH16_31, 0x0000);	   //clear channel enable
	while(READ_REG(SPU->CH_STATUS_CH16_31));
	WRITE_REG(SPU->SPU_EN_CH32_47, 0x0000);	   //clear channel enable
	while(READ_REG(SPU->CH_STATUS_CH32_47));
	WRITE_REG(SPU->SPU_EN_CH48_63, 0x0000);	   //clear channel enable
	while(READ_REG(SPU->CH_STATUS_CH48_63));

	SPU_SRAM_Initial();
// CTRL
  WRITE_REG(SPU->CTRL_FLAG, SPU_COMPRESSOR_DISABLE | SPU_INTERPOLATION_HQ_ENABLE | 0x0080 | SPU_PHASEADDER_INITIAL);

  WRITE_REG(SPU->MAIN_VOLUME, 0x1F);
// ENV
  WRITE_REG(SPU->ENV_MODE_CH0_15, 0x0000);
  WRITE_REG(SPU->ENV_REPEAT_CH0_15, 0xFFFF);   // Envelope Repeat Disable
  WRITE_REG(SPU->ENV_CLK_CH0_3, 0x3333);      // clock 0x3 Default
  WRITE_REG(SPU->ENV_CLK_CH4_7, 0x3333);      // clock 0x3 Default
  WRITE_REG(SPU->ENV_CLK_CH8_11, 0x3333);      // clock 0x3 Default
  WRITE_REG(SPU->ENV_CLK_CH12_15, 0x3333);      // clock 0x3 Default

  WRITE_REG(SPU->ENV_MODE_CH16_31, 0x0000);
  WRITE_REG(SPU->ENV_REPEAT_CH16_31, 0xFFFF);
  WRITE_REG(SPU->ENV_CLK_CH16_19, 0x3333);      // clock 0x3 Default
  WRITE_REG(SPU->ENV_CLK_CH20_23, 0x3333);      // clock 0x3 Default
  WRITE_REG(SPU->ENV_CLK_CH24_27, 0x3333);      // clock 0x3 Default
  WRITE_REG(SPU->ENV_CLK_CH28_31, 0x3333);      // clock 0x3 Default

  WRITE_REG(SPU->ENV_MODE_CH32_47, 0x0000);
  WRITE_REG(SPU->ENV_REPEAT_CH32_47, 0xFFFF);
  WRITE_REG(SPU->ENV_CLK_CH32_35, 0x3333);      // clock 0x3 Default
  WRITE_REG(SPU->ENV_CLK_CH36_39, 0x3333);      // clock 0x3 Default
  WRITE_REG(SPU->ENV_CLK_CH40_43, 0x3333);      // clock 0x3 Default
  WRITE_REG(SPU->ENV_CLK_CH44_47, 0x3333);      // clock 0x3 Default

  WRITE_REG(SPU->ENV_MODE_CH48_63, 0x0000);
  WRITE_REG(SPU->ENV_REPEAT_CH48_63, 0xFFFF);
  WRITE_REG(SPU->ENV_CLK_CH48_51, 0x3333);      // clock 0x3 Default
  WRITE_REG(SPU->ENV_CLK_CH52_55, 0x3333);      // clock 0x3 Default
  WRITE_REG(SPU->ENV_CLK_CH56_59, 0x3333);      // clock 0x3 Default
  WRITE_REG(SPU->ENV_CLK_CH60_63, 0x3333);      // clock 0x3 Default

// BEAT INT Test
  WRITE_REG(SPU->BEAT_BASE_COUNTER, 0x167);
  WRITE_REG(SPU->BEAT_COUNTER, 0xC001);
}


void SPU_CH_SRAM_Initial(SPU_CH_TYPE_DEF *SPU_CH)
{
  uint32_t *ptr;
  uint32_t i;
  ptr = SPU_CH;
  for (i = 0; i < 0x10; i += 1)
  {
    *(ptr + i) =  0x0000;
  }
  ptr = SPU_CH + 0x2000;
  for (i = 0; i < 0x8; i += 1)
  {
    *(ptr + i) =  0x0000;
  }
}

void SPU_SRAM_Initial()
{
  SPU_CH_SRAM_Initial(SPU_CH0);
  SPU_CH_SRAM_Initial(SPU_CH1);
  SPU_CH_SRAM_Initial(SPU_CH2);
  SPU_CH_SRAM_Initial(SPU_CH3);
  SPU_CH_SRAM_Initial(SPU_CH4);
  SPU_CH_SRAM_Initial(SPU_CH5);
  SPU_CH_SRAM_Initial(SPU_CH6);
  SPU_CH_SRAM_Initial(SPU_CH7);
  SPU_CH_SRAM_Initial(SPU_CH8);
  SPU_CH_SRAM_Initial(SPU_CH9);
  SPU_CH_SRAM_Initial(SPU_CH10);
  SPU_CH_SRAM_Initial(SPU_CH11);
  SPU_CH_SRAM_Initial(SPU_CH12);
  SPU_CH_SRAM_Initial(SPU_CH13);
  SPU_CH_SRAM_Initial(SPU_CH14);
  SPU_CH_SRAM_Initial(SPU_CH15);
  SPU_CH_SRAM_Initial(SPU_CH16);
  SPU_CH_SRAM_Initial(SPU_CH17);
  SPU_CH_SRAM_Initial(SPU_CH18);
  SPU_CH_SRAM_Initial(SPU_CH19);
  SPU_CH_SRAM_Initial(SPU_CH20);
  SPU_CH_SRAM_Initial(SPU_CH21);
  SPU_CH_SRAM_Initial(SPU_CH22);
  SPU_CH_SRAM_Initial(SPU_CH23);
  SPU_CH_SRAM_Initial(SPU_CH24);
  SPU_CH_SRAM_Initial(SPU_CH25);
  SPU_CH_SRAM_Initial(SPU_CH26);
  SPU_CH_SRAM_Initial(SPU_CH27);
  SPU_CH_SRAM_Initial(SPU_CH28);
  SPU_CH_SRAM_Initial(SPU_CH29);
  SPU_CH_SRAM_Initial(SPU_CH30);
  SPU_CH_SRAM_Initial(SPU_CH31);
  SPU_CH_SRAM_Initial(SPU_CH32);
  SPU_CH_SRAM_Initial(SPU_CH33);
  SPU_CH_SRAM_Initial(SPU_CH34);
  SPU_CH_SRAM_Initial(SPU_CH35);
  SPU_CH_SRAM_Initial(SPU_CH36);
  SPU_CH_SRAM_Initial(SPU_CH36);
  SPU_CH_SRAM_Initial(SPU_CH38);
  SPU_CH_SRAM_Initial(SPU_CH39);
  SPU_CH_SRAM_Initial(SPU_CH40);
  SPU_CH_SRAM_Initial(SPU_CH41);
  SPU_CH_SRAM_Initial(SPU_CH42);
  SPU_CH_SRAM_Initial(SPU_CH43);
  SPU_CH_SRAM_Initial(SPU_CH44);
  SPU_CH_SRAM_Initial(SPU_CH45);
  SPU_CH_SRAM_Initial(SPU_CH46);
  SPU_CH_SRAM_Initial(SPU_CH47);
  SPU_CH_SRAM_Initial(SPU_CH48);
  SPU_CH_SRAM_Initial(SPU_CH49);
  SPU_CH_SRAM_Initial(SPU_CH50);
  SPU_CH_SRAM_Initial(SPU_CH51);
  SPU_CH_SRAM_Initial(SPU_CH52);
  SPU_CH_SRAM_Initial(SPU_CH53);
  SPU_CH_SRAM_Initial(SPU_CH54);
  SPU_CH_SRAM_Initial(SPU_CH55);
  SPU_CH_SRAM_Initial(SPU_CH56);
  SPU_CH_SRAM_Initial(SPU_CH57);
  SPU_CH_SRAM_Initial(SPU_CH58);
  SPU_CH_SRAM_Initial(SPU_CH59);
  SPU_CH_SRAM_Initial(SPU_CH60);
  SPU_CH_SRAM_Initial(SPU_CH61);
  SPU_CH_SRAM_Initial(SPU_CH62);
  SPU_CH_SRAM_Initial(SPU_CH63);
}

void SPU_CH_Play(SPU_CH_TYPE_DEF *SPU_CH, uint32_t START_ADDR, uint32_t LOOP_ADDR, uint32_t AlgorithmType)
{
  uint32_t Channel_Number = 0;

  Channel_Number = SPU_CH;
  Channel_Number = ((Channel_Number-0x1000) & 0xFFFF) >> 6;

  if (Channel_Number < 16)
  {
    CLEAR_BIT(SPU->SPU_EN_CH0_15, 0x0001 << Channel_Number);	   //clear channel enable
  }
  else if (Channel_Number < 32)
  {
    CLEAR_BIT(SPU->SPU_EN_CH16_31, 0x0001 << (Channel_Number - 16));	   //clear channel enable
  }
  else if (Channel_Number < 48)
  {
    CLEAR_BIT(SPU->SPU_EN_CH32_47, 0x0001 << (Channel_Number - 32));	   //clear channel enable
  }
  else
  {
    CLEAR_BIT(SPU->SPU_EN_CH48_63, 0x0001 << (Channel_Number - 48));	   //clear channel enable
  }

  if (AlgorithmType == C_PCM_AutoEnd)
  {
    MODIFY_REG(SPU_CH->WAVE_MODE, 0xF000, 0x5000);     // 16-bitit PCM Auto End
//    MODIFY_REG(SPU_CH->WAVE_MODE, 0xF000, 0x1000);     // 8-bit PCM Auto End
  }
  else if (AlgorithmType == C_PCM_LOOP)
  {
    MODIFY_REG(SPU_CH->WAVE_MODE, 0xF000, 0x7000);     // 16-bit PCM Loop
//    MODIFY_REG(SPU_CH->WAVE_MODE, 0xF000, 0x3000);     // 8-bit PCM Loop
  }
  else if (AlgorithmType == C_ADPCM_AutoEnd)
  {
    MODIFY_REG(SPU_CH->WAVE_MODE, 0xF000, 0xD000);     //ADPCM Auto End
    WRITE_REG(SPU_CH->WAVE_ADPCM_CTRL, SPU_CH_WAVE_ADPCM36_DEFAULT);
  }
  else if (AlgorithmType == C_ADPCM_LOOP)
  {
    MODIFY_REG(SPU_CH->WAVE_MODE, 0xF000, 0xE000);     //ADPCM + ADPCM
    WRITE_REG(SPU_CH->WAVE_ADPCM_CTRL, SPU_CH_WAVE_ADPCM36_DEFAULT);
  }
  else if (AlgorithmType == C_ADPCM_PCM_LOOP)
  {
    MODIFY_REG(SPU_CH->WAVE_MODE, 0xF000, 0xF000);     //ADPCM + PCM
    WRITE_REG(SPU_CH->WAVE_ADPCM_CTRL, SPU_CH_WAVE_ADPCM36_DEFAULT);
  }

// Set Address
  SPU_SetStartAddr(SPU_CH, START_ADDR >> 1);
  SPU_SetLoopAddr(SPU_CH, LOOP_ADDR >> 1);

// Set PhaseAdder
  WRITE_REG(SPU_CH->PHASE_ACCH, 0x7);
	WRITE_REG(SPU_CH->PHASE_ACC, 0xFFFF);
  SPU_SetSampleRate(SPU_CH, SampleRate_16K);        // 32KHz

	WRITE_REG(SPU_CH->ENV_DATA, 0x007F);            //7 bit[6:0]
  WRITE_REG(SPU_CH->WAVE_DATA, 0X8000);  		         //16 bit data
  WRITE_REG(SPU_CH->WAVE_DATA0, 0X8000);  		       //16 bit data
  WRITE_REG(SPU_CH->PAN_VELOCITY, SPU_CH_PAN_VELOCITY_DEFAULT);                //{Pan,1'b0,Velocity}

  if (Channel_Number < 16)
  {
    WRITE_REG(SPU->STOP_STATUS_CH0_15, 0x0001 << Channel_Number);	   // clear stop flag
    SET_BIT(SPU->SPU_EN_CH0_15, 0x0001 << Channel_Number);	         // channel enable
  }
  else if (Channel_Number < 32)
  {
    WRITE_REG(SPU->STOP_STATUS_CH16_31, 0x0001 << (Channel_Number - 16));	   // clear stop flag
    SET_BIT(SPU->SPU_EN_CH16_31, 0x0001 << (Channel_Number - 16));	         // channel enable
  }
  else if (Channel_Number < 48)
  {
    WRITE_REG(SPU->STOP_STATUS_CH32_47, 0x0001 << (Channel_Number - 32));	   // clear stop flag
    SET_BIT(SPU->SPU_EN_CH32_47, 0x0001 << (Channel_Number - 32));	         // channel enable
  }
  else
  {
    WRITE_REG(SPU->STOP_STATUS_CH48_63, 0x0001 << (Channel_Number - 48));	   // clear stop flag
    SET_BIT(SPU->SPU_EN_CH48_63, 0x0001 << (Channel_Number - 48));	         // channel enable
  }
}

void SPU_SetStartAddr(SPU_CH_TYPE_DEF *SPU_CH, uint32_t START_ADDR)
{
  uint32_t ADDR_Bit30_28;
  uint32_t ADDR_Bit27_22;
  uint32_t ADDR_Bit21_16;
  uint32_t ADDR_Bit15_0;

  ADDR_Bit15_0 = START_ADDR & 0x0000FFFF;
  ADDR_Bit21_16 = START_ADDR & 0x003F0000;
  ADDR_Bit27_22 = START_ADDR & 0x0FC00000;
  ADDR_Bit30_28 = START_ADDR & 0x70000000;

  MODIFY_REG(SPU_CH->WAVE_ADDR, 0xFFFF, ADDR_Bit15_0);     //bit[15:0]
  MODIFY_REG(SPU_CH->WAVE_MODE, 0x003F, ADDR_Bit21_16 >> 16);     //bit[21:16]
  MODIFY_REG(SPU_CH->WAVE_ADDRH, 0x703F, (ADDR_Bit27_22 >> 22) + (ADDR_Bit30_28 >> 16));     //bit[30:28]
}

void SPU_SetLoopAddr(SPU_CH_TYPE_DEF *SPU_CH, uint32_t LOOP_ADDR)
{
  uint32_t ADDR_Bit30_28;
  uint32_t ADDR_Bit27_22;
  uint32_t ADDR_Bit21_16;
  uint32_t ADDR_Bit15_0;

  ADDR_Bit15_0 = LOOP_ADDR & 0x0000FFFF;
  ADDR_Bit21_16 = LOOP_ADDR & 0x003F0000;
  ADDR_Bit27_22 = LOOP_ADDR & 0x0FC00000;
  ADDR_Bit30_28 = LOOP_ADDR & 0x70000000;

  MODIFY_REG(SPU_CH->WAVE_LOOPADDR, 0xFFFF, ADDR_Bit15_0);     //bit[15:0]
  MODIFY_REG(SPU_CH->WAVE_MODE, 0x0FC0, ADDR_Bit21_16 >> 16);     //bit[21:16]
  MODIFY_REG(SPU_CH->WAVE_ADDRH, 0x0FC0, (ADDR_Bit27_22 >> 16) + (ADDR_Bit30_28 >> 16));     //bit[30:28]
}

void SPU_SetSampleRate(SPU_CH_TYPE_DEF *SPU_CH, uint32_t PhaseAdder)
{
  uint32_t PhaseAddrt18_16;
  uint32_t PhaseAdder15_0;

  PhaseAdder15_0 = PhaseAdder & 0x0000FFFF;
  PhaseAddrt18_16 = (PhaseAdder & 0x00070000) >> 16;
  WRITE_REG(SPU_CH->PHASE_VALUEH, PhaseAddrt18_16);     //bit[15:0]
  WRITE_REG(SPU_CH->PHASE_VALUE, PhaseAdder15_0);     //bit[15:0]
}

//void SPU_SetPitchBend(SPU_CH_TYPE_DEF *SPU_CH)
//{
//  WRITE_REG(SPU_CH->PITCHBEND_CTRL, 0x00FF);     //bit[15:0]
//  WRITE_REG(SPU_CH->PITCHBEND_TARGET, 0xABCD);     //bit[15:0]
//  WRITE_REG(SPU_CH->PITCHBEND_TARGETH, 0x0);     //bit[15:0]
//}

void SPU_WaveReleaseEN(SPU_CH_TYPE_DEF *SPU_CH)
{
  uint32_t Channel_Number = 0;
  Channel_Number = SPU_CH;
  Channel_Number = ((Channel_Number-0x1000) & 0xFFFF) >> 6;
  if (Channel_Number < 16)
  {
    SET_BIT(SPU->TONE_RELEASE_CH0_15, 0x0001 << Channel_Number);
  }
  else if (Channel_Number < 32)
  {
//    Channel_Number -= 16;
    SET_BIT(SPU->TONE_RELEASE_CH16_31, 0x0001 << (Channel_Number - 16));
  }
  else if (Channel_Number < 48)
  {
//    Channel_Number -= 32;
    SET_BIT(SPU->TONE_RELEASE_CH32_47, 0x0001 << (Channel_Number - 32));
  }
  else
  {
//    Channel_Number -= 48;
    SET_BIT(SPU->TONE_RELEASE_CH48_63, 0x0001 << (Channel_Number - 48));
  }
}


void SPU_SetEnvAddr(SPU_CH_TYPE_DEF *SPU_CH, uint32_t EnvAddr)
{
  uint32_t ADDR_Bit15_0;
  uint32_t ADDR_Bit21_16;
  uint32_t ADDR_Bit30_22;

  ADDR_Bit15_0 = (EnvAddr >> 1) & 0x0000FFFF;
  ADDR_Bit21_16 = (EnvAddr >> 1) & 0x003F0000;
  ADDR_Bit30_22 = (EnvAddr >> 1) & 0xFFC00000;
  WRITE_REG(SPU_CH->ENV_ADDR, ADDR_Bit15_0);
  MODIFY_REG(SPU_CH->ENV_INT_CTRL, 0x003F, (ADDR_Bit21_16) >> 16);
  WRITE_REG(SPU_CH->ENV_ADDRH, ADDR_Bit30_22 >> 22);
}


void SPU_ENV_RampDown(SPU_CH_TYPE_DEF *SPU_CH)
{
  uint32_t Channel_Number = 0;
  Channel_Number = (uint32_t) SPU_CH;
  Channel_Number = ((Channel_Number-0x1000) & 0xFFFF) >> 6;

  MODIFY_REG(SPU_CH->ENV_RAMPDOWN_STEP, SPU_CH_ENV_RAMPDOWN_STEP_MSK, SPU_CH_ENV_RAMPDOWN_STEP_0x00);
  WRITE_REG(SPU_CH->ENV_RAMPDOWN_CLK, 0x0001);
  SET_BIT(SPU->ENV_RAMPDOWN_CH0_15, 0x0001 << Channel_Number);
  CLEAR_BIT(SPU->ENV_MODE_CH0_15, 0x0001 << Channel_Number);  // Envelope Auto Mode
}

void SPU_Set_Velocity(SPU_CH_TYPE_DEF *SPU_CH, uint32_t Velocity)
{
  Velocity &= 0x007F;
  MODIFY_REG(SPU_CH->PAN_VELOCITY, SPU_CH_VELOCITY_MSK, Velocity);                //{Pan,1'b0,Velocity}
}

void SPU_Set_Pan(SPU_CH_TYPE_DEF *SPU_CH, uint32_t Pan)
{
  Pan &= 0x007F;
  MODIFY_REG(SPU_CH->PAN_VELOCITY, SPU_CH_VOLUME_PAN_MSK, Pan);                //{Pan,1'b0,Velocity}
}

void SPU_Set_BeatBaseCounter(uint32_t BeatBaseCnt)
{
  WRITE_REG(SPU->BEAT_BASE_COUNTER, BeatBaseCnt & SPU_BEAT_BASE_COUNTER_MSK);
}

void SPU_AllChannelStop()
{
  CLEAR_BIT(SPU->SPU_EN_CH0_15, 0xFFFF);
  CLEAR_BIT(SPU->SPU_EN_CH16_31, 0xFFFF);
  CLEAR_BIT(SPU->SPU_EN_CH32_47, 0xFFFF);
  CLEAR_BIT(SPU->SPU_EN_CH48_63, 0xFFFF);
}


///////////////////////////////////////////////
/**
 * @brief
 *   SPUBEAT_IRQHandler
 * @param
 *   None.
 * @return
 *   None.
 */
void SPUBEAT_IRQHandler(void)
{
//  GPIOA->OBUF ^= BIT8;                          // Toggle IOA[10]
  MIDI_BeatISR();
}

/**
 * @brief
 *   SPUCH_IRQHandler
 * @param
 *   None.
 * @return
 *   None.
 */
void SPUCH_IRQHandler(void)
{

}

/**
 * @brief
 *   SPUBEAT_IRQHandler
 * @param
 *   None.
 * @return
 *   None.
 */
void SPUENV_IRQHandler(void)
{

}




