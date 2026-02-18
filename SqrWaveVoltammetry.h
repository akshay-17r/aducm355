/* ========================= DiffPulseVoltammetry.h ========================= */
#ifndef _DPVTEST_H_
#define _DPVTEST_H_

#include "ad5940.h"
#include <stdio.h>
#include "string.h"
#include "math.h"

/* DAC LSBs (mV) */
#define DAC12BITVOLT_1LSB   (2200.0f/4095.0f)           /* mV */
#define DAC6BITVOLT_1LSB    (DAC12BITVOLT_1LSB*64.0f)   /* mV */

/* App control commands */
#define APPCTRL_START          0
#define APPCTRL_STOPNOW        1
#define APPCTRL_STOPSYNC       2
#define APPCTRL_SHUTDOWN       3

typedef enum
{
  DPV_STATE0 = 0,
  DPV_STATE1,
  DPV_STATE2,
  DPV_STOP
} DPVState_Type;

/* DPV application parameter structure */
typedef struct
{
  /* Common configuration */
  BoolFlag  bParaChanged;
  uint32_t  SeqStartAddr;
  uint32_t  MaxSeqLen;
  uint32_t  SeqStartAddrCal;
  uint32_t  MaxSeqLenCal;

  float     LFOSCClkFreq;
  float     SysClkFreq;
  float     AdcClkFreq;
  float     RcalVal;
  float     ADCRefVolt;

  /* DPV scan definition */
  float     RampStartVolt;          /* Ein (mV) */
  float     RampPeakVolt;           /* Ef  (mV) */
  float     VzeroStart;             /* (mV) */
  float     VzeroPeak;              /* (mV) */
  float     StaircaseInc;           /* (mV) */
  uint32_t  StepNumber;

  /* DPV pulse parameters */
  float     DPV_WaitTime;           /* tw (s) */
  float     DPV_PulseTime;          /* tp (s) */
  float     DPV_PulseHeight;        /* (mV) */

  /* Receive path */
  float     SampleDelay;            /* ms; DAC update -> ADC start */
  uint32_t  LPTIARtiaSel;
  float     ExternalRtiaValue;
  uint32_t  AdcPgaGain;
  uint8_t   ADCSinc3Osr;

  uint32_t  FifoThresh;

  /* Internal state */
  BoolFlag     DPVInited;
  fImpPol_Type RtiaValue;
  SEQInfo_Type InitSeqInfo;
  SEQInfo_Type ADCSeqInfo;

  BoolFlag  bFirstDACSeq;
  SEQInfo_Type DACSeqInfo;

  uint32_t  CurrStepPos;            /* baseline sequence generated steps */
  uint32_t  PulseStepPos;           /* pulse sequence generated steps */
  float     DACCodePerStair;        /* staircase increment in DAC codes */
  float     CurrBaseCode;           /* base DAC code (relative) */
  uint32_t  CurrVzeroCode;

  BoolFlag  StopRequired;
  DPVState_Type RampState;

  /* For dynamic DAC sequence generation */
  BoolFlag  bNextIsSeq0;            /* next update block for seq0 or seq1 */
  BoolFlag  bPulsePhase;            /* 0=baseline, 1=pulse (internal helper) */
} AppDPVCfg_Type;

/* API */
AD5940Err AppDPVInit(uint32_t *pBuffer, uint32_t BufferSize);
AD5940Err AppDPVGetCfg(void *pCfg);
AD5940Err AppDPVISR(void *pBuff, uint32_t *pCount);
AD5940Err AppDPVCtrl(uint32_t Command, void *pPara);

#endif
