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
#ifndef _DVRIMADEC_H_
#define _DVRIMADEC_H_


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

#define IMA_DEC_Global_Memory			(380)				// Global Memory of DVRIMADec kernel: can not be overwrite after DVRIMAdec_run()
#define IMA_DEC_Local_Memory			(640)				// Local Memory of DVRIMADec kernel:  can be overwrite after DVRIMAdec_run()
#define IMA_DEC_FRAMESIZE       	(80)


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
int32_t DVRIMAdec_run(IN void *obj, IN void *obj_local, OUT int16_t *pcm_out, IN int16_t *bs_buf, uint16_t *EventBit);
int32_t DVRIMAdec_init(IN void *obj);
int32_t DVRIMAdec_parsing(IN void *obj, IN int16_t *bs_buf, OUT uint16_t *cwPackageSize);
int32_t DVRIMAdec_GetMemoryBlockSize(void);
const int8_t* IMA_dec_get_version(void);
int32_t IMA_dec_get_bitrate(void *obj);
int32_t IMA_dec_get_samplerate(void *obj);
int32_t	IMA_dec_get_channel(void *obj);
int32_t	IMA_dec_get_bitspersample(void *obj);


#endif
