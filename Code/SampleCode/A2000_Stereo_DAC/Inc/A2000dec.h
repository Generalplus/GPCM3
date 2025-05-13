
/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *
 * @Version:
 *
 * @Date:
 *
 * @Abstract:
 *
 **************************************************************************************************/
#ifndef _A2000DEC_H_
#define _A2000DEC_H_


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include <stdint.h>


/*---------------------------------------------------------------------------------------
 * Definition Area
 *---------------------------------------------------------------------------------------*/
#define A20_DEC_Global_Memory   700				// Global Memory of A2000Dec kernel: can not be overwrite after a1800dec_run()
#define A20_DEC_Local_Memory    1280			// Local Memory of A2000Dec kernel:  can be overwrite after a1800dec_run()

#define A20_DEC_HEADER          3
#define A20_DEC_FRAMESIZE       640
#define A20_DEC_BITSTREAM_SIZE  60

#define A20_OK                  0x00000001
#define A20_E_NO_MORE_SRCDATA		0x80040005


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
extern int32_t  a2000dec_run(void *obj, void *obj_local,int16_t *pcm_out, int16_t *bs_buf, uint16_t *EventBit);  //for event-bit process
extern int32_t  a2000dec_init(void *obj);
extern int32_t  a2000dec_parsing(void *obj, int16_t *bs_buf, uint16_t *cwPackageSize);
extern int32_t  a2000dec_GetMemoryBlockSize(void);

extern const int8_t* A20_dec_get_version(void);
extern int32_t  A20_dec_get_bitrate(void *obj);
extern int32_t  A20_dec_get_samplerate(void *obj);
extern int32_t	A20_dec_get_channel(void *obj);
extern int32_t	A20_dec_get_bitspersample(void *obj);

#endif //__A2000DEC_H__
