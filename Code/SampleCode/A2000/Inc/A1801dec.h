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
#ifndef _A1801DEC_H_
#define _A1801DEC_H_


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include <stdint.h>


/*---------------------------------------------------------------------------------------
 * Definition Area
 *---------------------------------------------------------------------------------------*/
#define IN
#define OUT
#define INOUT
 
#define A18_DEC_Global_Memory			(380)				// Global Memory of A1801Dec kernel: can not be overwrite after a1800dec_run()		
#define A18_DEC_Local_Memory			(640)				// Local Memory of A1801Dec kernel:  can be overwrite after a1800dec_run()
#define A18_DEC_FRAMESIZE       	(320)


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
int32_t a1800dec_run(IN void *obj, IN void *obj_local, OUT int16_t *pcm_out, IN int16_t *bs_buf, uint16_t *EventBit); 
int32_t a1800dec_init(IN void *obj);
int32_t a1800dec_parsing(IN void *obj, IN int16_t *bs_buf, OUT uint16_t *cwPackageSize);
int32_t a1800dec_GetMemoryBlockSize(void);
const int8_t* A18_dec_get_version(void);
int32_t A18_dec_get_bitrate(void *obj);
int32_t A18_dec_get_samplerate(void *obj);
int32_t	A18_dec_get_channel(void *obj);
int32_t	A18_dec_get_bitspersample(void *obj);


#endif
