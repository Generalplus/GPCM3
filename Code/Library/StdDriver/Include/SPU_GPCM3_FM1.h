/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *
 * @Version:
 *   V0.9.0
 * @Date:
 *   19, Aug 2021
 * @Abstract:
 *
 **************************************************************************************************/
#ifndef _SPU_GPCM3_FM1_H_
#define _SPU_GPCM3_FM1_H_


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C"{
#endif

void SPU_Initial(void);
void SPU_SRAM_Initial(void);
void SPU_CH_Play(SPU_CH_TYPE_DEF *SPU_CH, uint32_t START_ADDR, uint32_t LOOP_ADDR, uint32_t AlgorithmType);
void SPU_SetStartAddr(SPU_CH_TYPE_DEF *SPU_CH, uint32_t START_ADDR);
void SPU_SetLoopAddr(SPU_CH_TYPE_DEF *SPU_CH, uint32_t LOOP_ADDR);
void SPU_SetSampleRate(SPU_CH_TYPE_DEF *SPU_CH, uint32_t PhaseAdder);
void SPU_WaveReleaseEN(SPU_CH_TYPE_DEF *SPU_CH);
void SPU_SetEnvAddr(SPU_CH_TYPE_DEF *SPU_CH, uint32_t EnvAddr);
void SPU_ENV_RampDown(SPU_CH_TYPE_DEF *SPU_CH);
void SPU_Set_Velocity(SPU_CH_TYPE_DEF *SPU_CH, uint32_t Velocity);
void SPU_Set_Pan(SPU_CH_TYPE_DEF *SPU_CH, uint32_t Pan);
void SPU_Set_BeatBaseCounter(uint32_t BeatBaseCnt);
void SPU_AllChannelStop();


#ifdef __cplusplus
}
#endif

#endif
