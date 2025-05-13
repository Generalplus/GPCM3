//-------以下參數只對 Sensitive 版本有效----------------------------//
#define SamplingRate		      16000
#define FrameSize				      160
#define BPM_LO_BOUND          50  //80 // BPM Pass Band
#define BPM_HI_BOUND     		  190 //107//160 // BPM Pass Band
#define BPM_RobustWeighting   1   //0.8 遺忘參數
#define BeatItv_weightFactor	0.85
#define BeatItv_miniWeight		0.1
//------------------------------------------------------------------//

#define VAD_SpeechLevel		    8192/3
#define VAD_NoiseBound		    512/3
#define VAD_Debounce		      (SamplingRate/FrameSize)	// 1.0sec



typedef struct BDM_Kernel_RAM
{
   unsigned short 	  gOut_tempo_count;	//	By Panda,2006.03.30
   unsigned short 	  gOut_Pre_tempo;	//	By Panda,2006.03.30
   short   						gRam_rgwX0[96]; // raw input memory
   short   						gRam_rgwY0[128]; // last fft magnitude
   short   						gRam_nY0_sh; // Q of last fft magnitude
   short   						gRam_wItvSteady; // steady beat interval
   short   						gRam_nItvSame;   // same streak
   short   						gRam_nItvDiff;   // diff streak
   short   						gRam_nInterval;  // current interval
   short   						gRam_nOffBeat; // off beat count
   short   						gRam_wLastBeat; // frames to last beat, Q5
   short   						gRam_iBeatAdjust; // adjust flag
   short   						gRam_wOnsetMax;
   short   						gRam_wOnsetRef;
   short   						gRam_wOnsetThr;
   short   						gRam_wOnsetPre;
   short   						gRam_rgwProb[120+1];	// probability
   short   						gRam_rgwOnset[20];
   short   						gRam_rgwPeak[20];
   short   						gRam_nShift;
   short   						gRam_wMax;
   short   						gRam_wTmp;
   short   						gRam_wOnset;
   short   						gRam_wItvRaw;
   short   						gOut_wBtStrength;	// 0-32767
   short   						rgwBuf[256];
   short   						rgwTmp[128];
   short   						PeakOnSet[8];
   short   						IntervalBuf[8];
   short   						PO_Index;
   unsigned short   	INTERVAL;
   short   						adjust_delay;
   int   							NoiseBound;
   int   							SpeechLevel;
   int   							C_Inhibit_CNT;
   short   						MicPeakBuf[32];
   short   						P_IDX;
   short   						Active_Timer;
   short   						ACT_FLAG;
}	BDM_Kernel;




typedef struct Mic_BeatKernel_Status
{
	BDM_Kernel		 tBDM_MEM;
	short gRom_rgwItvInvs[12];		//倍頻表
	short gRom_rgwItvDistri[12];	//倍頻加權
	short gRom_rgwItvBndQ5[5];		//(Beat intervals*32) 臨界點*32
	short gRom_rgwItvWgt[81];		//BPM 偵測Range, 根據不同的喜好,決定BPM偵測範圍
	unsigned short Estimation_BeatOut;
	short          Estimation_BeatInterval_Q5;
	short		   VoiceActivity_Flag;
	short		   adjust_delay;
	short		   BeatMask;
	short          Kernel_BeatMode;
	short          Kernel_State;
	short          tempo;	// frame intervel

	unsigned short	gOut_FFTn[16];
	short			gOut_FFtn_Exp;


}	Mic_BeatDetection_Info;



void F_Mic_BeatDetection_Init(Mic_BeatDetection_Info *Obj);
short F_Mic_BeatDetection_Process(Mic_BeatDetection_Info *Obj, short *PcmBuf);

