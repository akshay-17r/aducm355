/******************************************************************************
Copyright (c) 2017-2019 Analog Devices, Inc. All Rights Reserved.
*****************************************************************************/

#ifndef _SQRWAVEVOLTAMMETRY_H_
#define _SQRWAVEVOLTAMMETRY_H_

#include "ad5940.h"
#include <stdint.h>
#include <math.h>

/* Set to 1 to force step sizes to whole DAC LSBs (not required, but sometimes useful). */
#define ALIGN_VOLT2LSB     0
#define DAC12BITVOLT_1LSB  (2200.0f/4095.0f)          /* mV per 12-bit LSB (2.2V span) */
#define DAC6BITVOLT_1LSB   (DAC12BITVOLT_1LSB*64.0f)  /* mV per 6-bit step */

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

  /* SWV sweep description (all mV, referenced as Vbias - Vzero) */
  float     RampStartVolt;      /* mV */
  float     RampPeakVolt;       /* mV */
  float     VzeroStart;         /* mV */
  float     VzeroPeak;          /* mV */
  float     SqrWvAmplitude;     /* mV (peak-to-peak is 2x in this implementation’s toggling) */
  float     SqrWvRampIncrement; /* mV per ramp increment */
  float     Frequency;          /* Hz */

  /* Receive path */
  float     SampleDelay;        /* ms delay from DAC update to ADC sample */
  uint32_t  LPTIARtiaSel;
  float     ExternalRtiaValue;  /* Ohm (only used if LPTIARTIA_OPEN) */
  uint32_t  AdcPgaGain;
  uint8_t   ADCSinc3Osr;

  /* Digital */
  uint32_t  FifoThresh;         /* FIFO threshold in words (KEEP SMALL, e.g. 4..64) */

  /* Internal state */
  BoolFlag     SWVInited;
  fImpPol_Type RtiaValue;
  SEQInfo_Type InitSeqInfo;
  SEQInfo_Type ADCSeqInfo;
  BoolFlag     bFirstDACSeq;
  SEQInfo_Type DACSeqInfo;

  uint32_t  StepNumber;
  uint32_t  CurrStepPos;
  float     DACCodePerStep;
  float     DACCodePerRamp;
  float     CurrRampCode;
  uint32_t  CurrVzeroCode;
  BoolFlag  bSqrWaveHiLevel;
  BoolFlag  StopRequired;

  enum _SWVState { SWV_STATE0 = 0, SWV_STATE1, SWV_STATE2, SWV_STOP } RampState;

} AppSWVCfg_Type;

#define APPCTRL_START          0
#define APPCTRL_STOPNOW        1
#define APPCTRL_STOPSYNC       2
#define APPCTRL_SHUTDOWN       3

AD5940Err AppSWVInit(uint32_t *pBuffer, uint32_t BufferSize);
AD5940Err AppSWVGetCfg(void *pCfg);
AD5940Err AppSWVISR(void *pBuff, uint32_t *pCount);
AD5940Err AppSWVCtrl(uint32_t Command, void *pPara);

#endif
