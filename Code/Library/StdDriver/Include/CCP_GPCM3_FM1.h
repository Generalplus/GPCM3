/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   CCP_GPCM3_FM1.h
 * @Version:
 *   V0.9.0
 * @Date:
 *   May 27, 2022
 * @Abstract:
 *
 **************************************************************************************************/
#ifndef _CCP_GPCM3_FM1_H_
#define _CCP_GPCM3_FM1_H_


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "GPCM3_FM1.h"


/*---------------------------------------------------------------------------------------
 * Definition Area
 *---------------------------------------------------------------------------------------*/
 /*
  * CCP0 IO Pin select
	*/
 #define IoSel_IOA3_6                 0x00
 #define IoSel_IOA13_16               0x01

 /*
  * CCP1 IO Pin select
	*/
 #define IoSel_IOA9_12                0x00
 #define IoSel_IOA26_29               0x01

 /*
  * CCP0 PWMIO pin select
	*/
 #define PWM0                         0x00
 #define PWM1                         0x01
 #define PWM2                         0x02
 #define PWM3                         0x03

 /*
  * CCP0 PWM0/1 or PWM2/3 Mode (Complement or Independent)
	*/
 #define PWM01                        0x00
 #define PWM23                        0x01

 /*
  * PWMIO Default Setup ++
	*/
 #define TM0_PLOAD                    0x0008
 #define TM1_PLOAD                    0x0004
 #define TM2_PLOAD                    0x0002
 #define CCP0_TMR_PLOAD               0x0100
 #define CCP_PWM_FullHi               0x0000
 #define CCP_PWM_FullLo               0xFFFF
 #define CCP0_PWM0_DUTY               0x0040
 #define CCP0_PWM1_DUTY               0x0080
 #define CCP0_PWM2_DUTY               0x00C0
 #define CCP0_PWM3_DUTY               0x00F0
 #define CCP0_PWM_DELAY_TIME          0x0000

 #define CCP1_TMR_PLOAD               0x0300
 #define CCP1_PWM0_DUTY               0x0150
 // #define CCP1_TMR_PLOAD               0x0150
 // #define CCP1_PWM0_DUTY               0x0080
 #define CCP1_PWM1_DUTY               0x0060
 #define CCP1_PWM2_DUTY               0x0040
 #define CCP1_PWM3_DUTY               0x0020
 #define CCP1_PWM_DELAY_TIME          0x0000
 #define CCP_PWM_MASTER_DELAY         0x000F
 #define CCP_PWM_SLAVE_DELAY          0x0F00


 /*
  * Compare Default Setup ++
	*/
 #define CCP0_CMPTMR_PLOAD            0x5000
 #define CCP0_CMP0_Data               0x3000
 #define CCP0_CMP1_Data               0x1000
 #define CCP0_CMP2_Data               0x0500
 #define CCP0_CMP3_Data               0x0090

 #define CCP1_CMPTMR_PLOAD            0xA000
 #define CCP1_CMP0_Data               0x9020
 #define CCP1_CMP1_Data               0x5040
 #define CCP1_CMP2_Data               0x1060
 #define CCP1_CMP3_Data               0x0580
 /*
  * Compare Default Setup --
	*/

 /*
  * Capture Default Setup ++
	*/
 #define CAP_TRG_RISING               0x01
 #define CAP_TRG_FALLING              0x02
 #define CAP_TRG_DUAL                 0x03

 #define CCP0_CAPTMR_PLOAD            0xF000
 #define CCP1_CAPTMR_PLOAD            0xA000

 /*
  * Capture Default Setup --
	*/

#define CCP_SETTING_8K               (24000000 / 8000)
#define CCP_SETTING_16K              (24000000 / 16000)
#define CCP_SETTING_32K              (24000000 / 32000)
#define CCP_SETTING_64K              (24000000 / 64000)


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C"{
#endif
void CCP0_Close(void);
void CCP1_Close(void);
void CCP0_Timer_Reload(uint16_t TMR_RELOAD);
void CCP1_Timer_Reload(uint16_t TMR_RELOAD);

void CCP0_PWMIO_Open(uint8_t IOSEL);
void CCP1_PWMIO_Open(uint8_t IOSEL);
void CCP0_PWMIOx_Ctrl(uint32_t PWMx_Enable);
void CCP1_PWMIOx_Ctrl(uint32_t PWMx_Enable);
void CCP0_PWMIOx_SetDuty(uint8_t PWMx,uint32_t Duty);
void CCP1_PWMIOx_SetDuty(uint8_t PWMx,uint32_t Duty);
void CCP0_PWMIOx_Inverse(uint8_t PWMx);
void CCP1_PWMIOx_Inverse(uint8_t PWMx);
void CCP0_PWMxx_ModeSelect(uint8_t PWMxx, uint32_t Mode);
void CCP1_PWMxx_ModeSelect(uint8_t PWMxx, uint32_t Mode);

void CCP0_Capture_Initial(uint8_t IOSEL, uint8_t TRG_MODE);
void CCP1_Capture_Initial(uint8_t IOSEL, uint8_t TRG_MODE);
void CCP0_CAPx_Ctrl(uint32_t CAP_IOx_Enable);
void CCP1_CAPx_Ctrl(uint32_t CAP_IOx_Enable);

void CCP0_Compare_Initial(void);
void CCP1_Compare_Initial(void);
void CCP0_CMPx_Ctrl(uint32_t CMPx_Enable);
void CCP1_CMPx_Ctrl(uint32_t CMPx_Enable);

#ifdef __cplusplus
}
#endif

#endif
