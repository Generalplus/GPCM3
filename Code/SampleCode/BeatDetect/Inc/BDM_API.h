
#ifndef _BDM_API_H_
#define _BDM_API_H_

#include "GPCM3_FM1.h"
#include "Mic_BeatDetection.h"

/*---------------------------------------------------------------------------------------
 * Definition Area
 *---------------------------------------------------------------------------------------*/

/*
 * BDM status definition
 */
#define BDM_START_FLAG                         (0x01)
#define	BDM_BUF_EVEN_FLAG                      (0x02)
#define BDM_WORK_FLAG                          (0x04)
#define BDM_CPU_OVERLOAD_FLAG                  (0x10)
#define BDM_ISR_SERVICE_ENABLE_FLAG            (0x80)

#define	BDM_FRAME_SIZE			FrameSize

typedef struct _BDM_WORKING_RAM
{
  int16_t 	*BDMPcmBufPtr;
	int16_t 	*BDMInBufPtr;									//AdcIn_Frame_Addr
	int16_t 	BDMPcmBuffer[2][FrameSize];

	Mic_BeatDetection_Info 	Mic_BeatDetection_Info;
}	BDM_WORKING_RAM;

void BDM_Init(BDM_WORKING_RAM*);
void BDM_Start(void);
void BDM_Stop(void);
void BDM_ServiceLoop(void);
void BDM_DmaIsrService(void);
int16_t BDM_DecodeProcess(int16_t*);
void BDM_DelayOut(int16_t NumFrame);

#endif
