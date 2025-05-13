/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:     		EnvDet_API.h
 *
 * @Version:
 *
 * @Date:     		2019.06.11
 *
 * @Abstract: 		Envelope Detection API declaration
 *
 **************************************************************************************************/
#ifndef _ENVDET_API_H_
#define _ENVDET_API_H_

/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "GPCM3_FM1.h"

/*---------------------------------------------------------------------------------------
 * Definition Area
 *---------------------------------------------------------------------------------------*/

/*
 * EnvDet status definition
 */
#define ENV_DET_RUNNING 													(0x0001)
#define ENV_DET_ATTACK_ACTIVE											(0x0002)
#define ENV_DET_RELEASE_ACTIVE										(0x0004)


typedef struct _ENVDET_WORKING_RAM
{
	uint16_t Env_Flag;

	uint16_t Env_MicEnvelope;
	uint16_t Env_TrackDnSpeed;
	uint16_t Env_AttackCounter;
	uint16_t Env_ReleaseCounter;
	uint16_t Env_AttackLevel;
	uint16_t Env_AttackTime;
	uint16_t Env_ReleaseLevel;
	uint16_t Env_ReleaseTime;
} ENVDET_WORKING_RAM;


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
void EnvDet_Initial(ENVDET_WORKING_RAM *EnvDetWorkingRam);
void EnvDet_Start(void);
void EnvDet_Stop(void);
void EnvDet_SetTrackDnSpeed(uint16_t step);
void EnvDet_SetAttackLevel(uint16_t AttTh);
void EnvDet_SetReleaseLevel(uint16_t RelTh);
void EnvDet_SetAttackTime(uint16_t AttTime);
void EnvDet_SetReleaseTime(uint16_t RelTime);
uint16_t  EnvDet_GetEnvelopeData(void);
uint16_t EnvDet_GetStatus(void);
void EnvDet_IsrService(void);
void EnvDet_UpdateEnvelope(int16_t AdcData);				// trace the envelop data
void EnvDet_CheckEnvelope(void);
void EnvDet_ServiceLoop(int16_t *InPtr, int32_t CNTR);
#endif
