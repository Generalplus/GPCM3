/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *
 * @Version:
 *    V0.0.1
 * @Date:
 *    30th, April 2019
 * @Abstract:
 *
 **************************************************************************************************/

/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "GPCM3_FM1.h"
#include "EnvDet_User.h"

/*---------------------------------------------------------------------------------------
 * Definition Area
 *---------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
 * Gobal Variable Declaration Area
 *---------------------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------------------
 * Callback Function Area
 *---------------------------------------------------------------------------------------*/
/**
 * @brief
 *   Callback function. Implemented by user.
 *   When user call API function EnvDet_Initial, this function is Called by EnvDet Library.
 * @param
 *  *EnvDetWorkingRam [out]: EnvDet library working RAM address.
 * @return
 *   None.
 */
void EnvDet_CB_Init(const ENVDET_WORKING_RAM *EnvDetWorkingRam)
{
}

/**
 * @brief
 *
 * @param
 *  *EnvDetWorkingRam [out]: EnvDet library working RAM address.
 * @return
 *   None.
 */
void EnvDet_CB_Stop(const ENVDET_WORKING_RAM *EnvDetWorkingRam)
{
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   Signed ADC Data
 */
int16_t EnvDet_CB_GetAdc(void)
{
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void EnvDet_DmaIsrService(void)
{
}
