/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:     
 *   FFT_API.h
 * @Version:  
 *   V0.9.0
 * @Date:     
 *   January 21, 2021
 * @Abstract: 
 *
 **************************************************************************************************/
#ifndef _FFT_API_H_
#define _FFT_API_H_


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------------------
 * User Definition Area
 *---------------------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------------------
 * Definition Area
 *---------------------------------------------------------------------------------------*/ 
 
/*
 * FFT status definition
 */
#define FFT_START_FLAG                         (0x01)
#define	FFT_BUF_EVEN_FLAG                      (0x02)
#define FFT_WORK_FLAG                          (0x04)
#define FFT_CPU_OVERLOAD_FLAG                  (0x10)
#define FFT_ISR_SERVICE_ENABLE_FLAG            (0x80)


/*---------------------------------------------------------------------------------------
 * Structure Definition Area
 *---------------------------------------------------------------------------------------*/ 
typedef struct _FFT_WORKING_RAM
{	
	uint16_t 	FrameSize;
  int16_t *FftPcmBufPtr;
	int16_t *FftProBufPtr;
	int16_t  *FftPcmBuf;
} FFT_WORKING_RAM;


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C"{
#endif 

//void FFT_Initial(FFT_WORKING_RAM *FftWorkingRam);
void FFT_Initial(FFT_WORKING_RAM *FftWorkingRam, int16_t *lFftPcmBuf, int16_t lFrameSize);
void FFT_Start(void);
void FFT_Stop(void);
void FFT_DmaIsrService(void);
//int32_t CSP16_GetResult(void);
void FFT_ServiceLoop(void);
uint32_t FFT_CheckCpuOverload(void);
void FFT_ClearCpuOverloadFlag(void);
uint32_t FFT_GetStatus(void);
void FFT_ClrWorkFlag(void);
	

#ifdef __cplusplus
}
#endif		
	
#endif
