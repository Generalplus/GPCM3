/**************************************************************************************************
 * Copyright(c) 2020 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *    Configuration_Wizard_GPCM1Fx.c
 * @Version:
 *    V1.0.0
 * @Date:
 *    January 21, 2020
 * @Abstract:
 *    - User do not directly modify this configuration_wizard_GPCM1F064A.c to chagne USER_OPTx!!!
 *    - Please modify USER_OPTx from Keil Configuration Wizard. If user do not modify USER_OPTx from
 *  Keil Configuration Wizard, user may modify Non-USER_OPTx to affect IC function!!!
 **************************************************************************************************/


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "Configuration_Wizard_GPCM1Fx.h"


/*---------------------------------------------------------------------------------------
 * User Option(USER_OPTx) Area
 *  - USER DO NOT DIRECTLY MODIFY ANY CODE
 *---------------------------------------------------------------------------------------*/
//*** <<< Use Configuration Wizard in Context Menu >>> ***
// <e> Unlock Security Setting
#define SECURITY_CHECKBOX_UNLOCK    (0)
//   <o.0> Security
//     <0=> Enable
//     <1=> Disable
// </e>

#define USER_OPT0_DEFINED    (0xFFFFFFFF)
#if ((USER_OPT0_DEFINED & 0xFFFFFFFE) != 0xFFFFFFFE)
  #error "USER_OPT0: Invalid";
#endif

#if defined(__CC_ARM)
const uint32_t USER_OPT0  __attribute__((at(0x0F000000))) = USER_OPT0_DEFINED;
#elif defined(__GNUC__)
const uint32_t USER_OPT0  __attribute__((section(".USER_OPT0"))) = USER_OPT0_DEFINED;
#endif

// <e> Unlock VDD33_DSADC Voltage Setting
#define VDD33_DSADC_VOLTAGE_CHECKBOX_UNLOCK    (0)
//   <o.0..1>  VDD33_DSADC Voltage Selection
//     <0=> 2.7V
//		 <1=> 3.0V
//		 <2=> 3.3V
// </e>
#define USER_OPT1_DEFINED_0    (0x1)

// <e> Unlock VDD33_SPI Voltage Setting
#define VDD33_SPI_VOLTAGE_CHECKBOX_UNLOCK    (0)
//   <o.2..3>  VDD33_SPI Voltage Selection
//     <0=> 2.7V
//		 <1=> 3.0V
//		 <2=> 3.3V
//		 <3=> 3.6V
// </e>
#define USER_OPT1_DEFINED_1    (0x4)

#define USER_OPT1_DEFINED    (0xFFFFFDE0 | USER_OPT1_DEFINED_0 | USER_OPT1_DEFINED_1)
#if (((USER_OPT1_DEFINED & 0xFFFFFDE0) != 0xFFFFFDE0) || ((USER_OPT1_DEFINED & 0xFFFFFDE3) == 0xFFFFFDE3))
  #error "USER_OPT1: Invalid";
#endif

#if defined(__CC_ARM)
const uint32_t USER_OPT1  __attribute__((at(0x0F000004))) = USER_OPT1_DEFINED;
#elif  defined(__GNUC__)
const uint32_t USER_OPT1  __attribute__((section(".USER_OPT1"))) = USER_OPT1_DEFINED;
#endif

// <e>  Unlock Internal Flash Protect Setting
#define INTERNAL_FLASH_PROTECT_CHECKBOX_UNLOCK    (0)
//   <o.0>  0x0000 ~ 0x03FF Protect Bit
//     <0=> Protected
//		 <1=> Non-protected
//   <o.1>  0x0400 ~ 0x07FF Protect Bit
//     <0=> Protected
//		 <1=> Non-protected
//   <o.2>  0x0800 ~ 0x0FFF Protect Bit
//     <0=> Protected
//		 <1=> Non-protected
//   <o.3>  0x1000 ~ 0x17FF Protect Bit
//     <0=> Protected
//		 <1=> Non-protected
//   <o.4>  0x1800 ~ 0x1FFF Protect Bit
//     <0=> Protected
//		 <1=> Non-protected
//   <o.5>  0x2000 ~ 0x27FF Protect Bit
//     <0=> Protected
//		 <1=> Non-protected
//   <o.6>  0x2800 ~ 0x37FF Protect Bit
//     <0=> Protected
//		 <1=> Non-protected
//   <o.7>  0x3800 ~ 0x47FF Protect Bit
//     <0=> Protected
//		 <1=> Non-protected
//   <o.8>  0x4800 ~ 0x57FF Protect Bit
//     <0=> Protected
//		 <1=> Non-protected
//   <o.9>  0x5800 ~ 0x77FF Protect Bit
//     <0=> Protected
//		 <1=> Non-protected
//   <o.10> 0x7800 ~ 0x97FF Protect Bit
//     <0=> Protected
//		 <1=> Non-protected
//   <o.11> 0x9800 ~ 0xB7FF Protect Bit
//     <0=> Protected
//		 <1=> Non-protected
//   <o.12> 0xB800 ~ 0xD7FF Protect Bit
//     <0=> Protected
//		 <1=> Non-protected
//   <o.13> 0xD800 ~ 0xF7FF Protect Bit
//     <0=> Protected
//		 <1=> Non-protected
//   <o.14> 0xF800 ~ 0xFBFF Protect Bit
//     <0=> Protected
//		 <1=> Non-protected
// </e>

#define USER_OPT2_DEFINED    (0xFFFF7FFF)
#if ((USER_OPT2_DEFINED & 0xFFFF8000) != 0xFFFF0000)
  #error "USER_OPT2: Invalid";
#endif

#if defined(__CC_ARM)
const uint32_t UOPT2  __attribute__((at(0x0F000008))) = USER_OPT2_DEFINED;
#elif defined(__GNUC__)
const uint32_t UOPT2  __attribute__((section(".UOPT2"))) = USER_OPT2_DEFINED;
#endif

#define USER_OPT3_DEFINED    (0xFFFFFFFF)
#if (USER_OPT3_DEFINED != 0xFFFFFFFF)
  #error "USER_OPT3: Invalid";
#endif

#if defined(__CC_ARM)
const uint32_t UOPT3  __attribute__((at(0x0F00000C))) = USER_OPT3_DEFINED;
#elif  defined(__GNUC__)
const uint32_t UOPT3  __attribute__((section(".UOPT3"))) = USER_OPT3_DEFINED;
#endif

//*** <<< end of configuration section >>> ***
