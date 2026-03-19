	/* ============================= DPV.h =============================
		 Differential Pulse Voltammetry (DPV) application for AD5940
	================================================================== */

	#ifndef _DPV_H_
	#define _DPV_H_

	#include "ad5940.h"
	#include <stdint.h>
	#include <math.h>

	#define ALIGN_VOLT2LSB     0
	#ifndef DAC12BITVOLT_1LSB
	#define DAC12BITVOLT_1LSB (2200.0f/4095)
	#endif

	#ifndef DAC6BITVOLT_1LSB
	#define DAC6BITVOLT_1LSB (DAC12BITVOLT_1LSB*64)
	#endif

	typedef struct
	{
		/* Common app config */
		BoolFlag  bParaChanged;
		uint32_t  SeqStartAddr;
		uint32_t  MaxSeqLen;
		uint32_t  SeqStartAddrCal;
		uint32_t  MaxSeqLenCal;

		float     LFOSCClkFreq;     /* Hz */
		float     SysClkFreq;       /* Hz */
		float     AdcClkFreq;       /* Hz */
		float     RcalVal;          /* Ohm */
		float     ADCRefVolt;       /* mV */

		/* DPV scan */
		float     Ein_mV;             /* mV, step-up start */
		float     Efinal_mV;          /* mV, end */
		float     StepIncrement_mV;   /* mV per step */

		float     PulseAmplitude_mV;  /* mV pulse height (+) */
		float     WaitTime_s;         /* s baseline hold (tw) */
		float     PulseTime_s;        /* s pulse width (tp) */

		float     Vzero_mV;           /* mV, 6-bit VZERO code target */

		/* ADC sampling near end of each phase */
		float     SampleDelayBase_ms;   /* ms before baseline ADC */
		float     SampleDelayPulse_ms;  /* ms before pulse ADC */

		/* Receive path */
		uint32_t  LPTIARtiaSel;
		float     ExternalRtiaValue;  /* Ohm */
		uint32_t  AdcPgaGain;
		uint8_t   ADCSinc3Osr;

		/* Digital */
		uint32_t  FifoThresh;

		/* Internal state */
		BoolFlag     DPVInited;
		fImpPol_Type RtiaValue;
		SEQInfo_Type InitSeqInfo;
		SEQInfo_Type ADCSeqInfo;

		BoolFlag     bFirstDACSeq;
		SEQInfo_Type DACSeqInfo;

		uint32_t  TotalSteps;         /* number of potential steps */
		uint32_t  UpdatesTotal;       /* total DAC updates (= 2*TotalSteps) */
		uint32_t  UpdateIndex;        /* 0..UpdatesTotal-1 */

		float     CurrE_mV;           /* current step potential (baseline) */
		uint32_t  VzeroCode;          /* 6-bit */
		float     BaseCode12;         /* baseline Vbias code (12-bit) */
		float     PulseCode12;        /* pulse Vbias code (12-bit) */

		BoolFlag  StopRequired;

	} AppDPVCfg_Type;

	#define APPCTRL_START          0
	#define APPCTRL_STOPNOW        1
	#define APPCTRL_STOPSYNC       2
	#define APPCTRL_SHUTDOWN       3

	AD5940Err AppDPVInit(uint32_t *pBuffer, uint32_t BufferSize);
	AD5940Err AppDPVGetCfg(void *pCfg);
	AD5940Err AppDPVISR(void *pBuff, uint32_t *pCount);
	AD5940Err AppDPVCtrl(uint32_t Command, void *pPara);

	#endif
