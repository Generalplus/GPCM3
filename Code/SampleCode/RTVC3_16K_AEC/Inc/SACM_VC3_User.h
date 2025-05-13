/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:     
 *   SACM_VC3_User.h       
 * @Version: 
 *   V1.0.4
 * @Date: 
 *   December 14, 2021
 * @Abstract: 
 *
 **************************************************************************************************/
#ifndef _SACM_VC3_USER_H_
#define _SACM_VC3_USER_H_


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "SACM_VC3_API.h"


/*---------------------------------------------------------------------------------------
 * Extern Const Array Declaration Area
 *---------------------------------------------------------------------------------------*/
 
/* 
 * VC3 Filter Table 
 */
extern const int16_t VC3_ShiftPitchFilterTable1[VC3_N_INDEX]; 
extern const int16_t VC3_ShiftPitchFilterTable2[VC3_N_INDEX];
extern const int16_t VC3_ShiftPitchFilterTable3[VC3_N_INDEX];
extern const int16_t VC3_ShiftPitchFilterTable4[VC3_N_INDEX];
extern const int16_t VC3_ShiftPitchFilterTable5[VC3_N_INDEX];
extern const int16_t VC3_ShiftPitchFilterTable6[VC3_N_INDEX];
extern const int16_t VC3_ShiftPitchFilterTable7[VC3_N_INDEX];
extern const int16_t VC3_ShiftPitchFilterTable8[VC3_N_INDEX];
extern const int16_t VC3_ShiftPitchFilterTable9[VC3_N_INDEX];
extern const int16_t VC3_ShiftPitchFilterTable10[VC3_N_INDEX];
extern const int16_t VC3_ShiftPitchFilterTable11[VC3_N_INDEX];
extern const int16_t VC3_ShiftPitchFilterTable12[VC3_N_INDEX]; 


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C"{
#endif
	
/*
 * VC3 Callback Function Declaration
 */
void VC3_CB_Init(const SACM_VC3_API_WORKING_RAM *Vc3ApiWorkingRam);
void VC3_CB_StartPlay(const SACM_VC3_API_WORKING_RAM *Vc3ApiWorkingRam);
void VC3_CB_StopPlay(const SACM_VC3_API_WORKING_RAM *Vc3ApiWorkingRam);
void VC3_CB_GetAdc_DmaIsr(const SACM_VC3_API_WORKING_RAM *Vc3ApiWorkingRam, int16_t *AdcInBufAddr, uint16_t DataLen);
void VC3_CB_SendDac_DmaIsr(const SACM_VC3_API_WORKING_RAM *Vc3ApiWorkingRam, int16_t *DacOutBufAddr, uint16_t DataLen);
void VC3_CB_VoiceProcess(const SACM_VC3_API_WORKING_RAM *Vc3ApiWorkingRam, int16_t *SrcBufAddr, int16_t *DstBufAddr, uint8_t Vc3Mode);

	
#ifdef __cplusplus
}
#endif

#endif
