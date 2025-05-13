/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *    Configuration_Wizard_GPCM1Fx.h
 * @Version:
 *    V1.0.0
 * @Date:
 *    January 21, 2020
 * @Abstract:
 *   USER DO NOT MODIFY ANY CODE. This file is read only.
 **************************************************************************************************/
#ifndef _CONFIGURATION_WIZARD_GPCM1F_H_
#define _CONFIGURATION_WIZARD_GPCM1F_H_


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include <stdint.h>
#if defined(__CC_ARM)
#include "RTE_Components.h"
#endif


/*---------------------------------------------------------------------------------------
 * EMU Chip Option Area.
 *---------------------------------------------------------------------------------------*/
#ifdef _GPCM1F064A
#define EMU_OPT8_DEFINED    (0xFFFFFFFF)
#define EMU_OPT9_DEFINED    (0xFFFFFFFF)
#define EMU_OPT10_DEFINED   (0xFFFFFFFF)
#define EMU_OPT11_DEFINED   (0xFFFFFFFF)
#define EMU_OPT12_DEFINED   (0xFFFFFFFF)

#if (EMU_OPT8_DEFINED != 0xFFFFFFFF)
  #error "EMU8_OPT : Invalid";
#endif

#if (EMU_OPT9_DEFINED != 0xFFFFFFFF)
  #error "EMU9_OPT : Invalid";
#endif

#if (EMU_OPT10_DEFINED != 0xFFFFFFFF)
  #error "EMU10_OPT : Invalid";
#endif

#if (EMU_OPT11_DEFINED != 0xFFFFFFFF)
  #error "EMU1_OPT : Invalid";
#endif

#if (EMU_OPT12_DEFINED != 0xFFFFFFFF)
  #error "EMU12_OPT : Invalid";
#endif
#endif


#ifdef _GPCM1F064A_0001
#define EMU_OPT8_DEFINED    (0xFFFFFFFF)
#define EMU_OPT9_DEFINED    (0xFFFFFFFF)
#define EMU_OPT10_DEFINED   (0xFFFFFFFF)
#define EMU_OPT11_DEFINED   (0xFFFFFFFF)
#define EMU_OPT12_DEFINED   (0xFFFFFFFE)

#if (EMU_OPT8_DEFINED != 0xFFFFFFFF)
  #error "EMU8_OPT : Invalid";
#endif

#if (EMU_OPT9_DEFINED != 0xFFFFFFFF)
  #error "EMU9_OPT : Invalid";
#endif

#if (EMU_OPT10_DEFINED != 0xFFFFFFFF)
  #error "EMU10_OPT : Invalid";
#endif

#if (EMU_OPT11_DEFINED != 0xFFFFFFFF)
  #error "EMU1_OPT : Invalid";
#endif

#if (EMU_OPT12_DEFINED != 0xFFFFFFFE)
  #error "EMU12_OPT : Invalid";
#endif
#endif


#ifdef _GPCM1F064B
#define EMU_OPT8_DEFINED    (0xFFFFFF77)
#define EMU_OPT9_DEFINED    (0xFF3FFFFF)
#define EMU_OPT10_DEFINED   (0xFFFFFFFF)
#define EMU_OPT11_DEFINED   (0xFFFFFFFF)
#define EMU_OPT12_DEFINED   (0xFFFFFFFF)

#if (EMU_OPT8_DEFINED != 0xFFFFFF77)
  #error "EMU8_OPT : Invalid";
#endif

#if (EMU_OPT9_DEFINED != 0xFF3FFFFF)
  #error "EMU9_OPT : Invalid";
#endif

#if (EMU_OPT10_DEFINED != 0xFFFFFFFF)
  #error "EMU10_OPT : Invalid";
#endif

#if (EMU_OPT11_DEFINED != 0xFFFFFFFF)
  #error "EMU1_OPT : Invalid";
#endif

#if (EMU_OPT12_DEFINED != 0xFFFFFFFF)
  #error "EMU12_OPT : Invalid";
#endif
#endif


#ifdef _GPCM1F064B_0001
#define EMU_OPT8_DEFINED    (0xFFFFFF77)
#define EMU_OPT9_DEFINED    (0xFF3FFFFF)
#define EMU_OPT10_DEFINED   (0xFFFFFFFF)
#define EMU_OPT11_DEFINED   (0xFFFFFFFF)
#define EMU_OPT12_DEFINED   (0xFFFFFFFE)

#if (EMU_OPT8_DEFINED != 0xFFFFFF77)
  #error "EMU8_OPT : Invalid";
#endif

#if (EMU_OPT9_DEFINED != 0xFF3FFFFF)
  #error "EMU9_OPT : Invalid";
#endif

#if (EMU_OPT10_DEFINED != 0xFFFFFFFF)
  #error "EMU10_OPT : Invalid";
#endif

#if (EMU_OPT11_DEFINED != 0xFFFFFFFF)
  #error "EMU1_OPT : Invalid";
#endif

#if (EMU_OPT12_DEFINED != 0xFFFFFFFE)
  #error "EMU12_OPT : Invalid";
#endif
#endif

#if defined(__CC_ARM)
const uint32_t UOPT8  __attribute__((at(0x0F000020))) = EMU_OPT8_DEFINED;
const uint32_t UOPT9  __attribute__((at(0x0F000024))) = EMU_OPT9_DEFINED;
const uint32_t UOPT10 __attribute__((at(0x0F000028))) = EMU_OPT10_DEFINED;
const uint32_t UOPT11 __attribute__((at(0x0F00002C))) = EMU_OPT11_DEFINED;
const uint32_t UOPT12 __attribute__((at(0x0F000030))) = EMU_OPT12_DEFINED;
#elif defined(__GNUC__)
const uint32_t UOPT8 __attribute__((section(".UOPT8")))  = EMU_OPT8_DEFINED;
const uint32_t UOPT9 __attribute__((section(".UOPT9")))  = EMU_OPT9_DEFINED;
const uint32_t UOPT10 __attribute__((section(".UOPT10"))) = EMU_OPT10_DEFINED;
const uint32_t UOPT11 __attribute__((section(".UOPT11"))) = EMU_OPT11_DEFINED;
const uint32_t UOPT12 __attribute__((section(".UOPT12"))) = EMU_OPT12_DEFINED;
#endif

#endif
