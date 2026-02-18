/* ============================= AD5940Main.c ============================== */
#include "SqrWaveVoltammetry.h"

/**
   User could configure following parameters
**/
#define APPBUFF_SIZE 1024
uint32_t AppBuff[APPBUFF_SIZE];
float LFOSCFreq;    /* Measured LFOSC frequency */

/* Calibration models: I(uA) = intercept - slope*log10(C) */
#define CTNI_INTERCEPT_UA   (82.42f)
#define CTNI_SLOPE_UA_DEC   (11.33f)
#define BNP_INTERCEPT_UA    (101.196f)
#define BNP_SLOPE_UA_DEC    (11.95f)

static float CurrentToConc_pgml(float current_uA, float intercept, float slope)
{
  /* Rearranged: C = 10^((intercept - I)/slope) */
  return powf(10.0f, (intercept - current_uA)/slope);
}

static int32_t RampShowResult(float *pData, uint32_t DataCount)
{
  static uint32_t index;
  AppDPVCfg_Type *pCfg;
  AppDPVGetCfg(&pCfg);

  for(uint32_t i=0; i<DataCount; i++)
  {
    float Ebase_mV = pCfg->RampStartVolt + (float)index * pCfg->StaircaseInc;
    if(Ebase_mV > pCfg->RampPeakVolt) Ebase_mV = pCfg->RampPeakVolt;
    float Epulse_mV = Ebase_mV + pCfg->DPV_PulseHeight;

    /* Use calibrated peak differential current magnitude for concentration models */
    float Ipk_uA = fabsf(pData[i]);
    float cTnI_pgml = CurrentToConc_pgml(Ipk_uA, CTNI_INTERCEPT_UA, CTNI_SLOPE_UA_DEC);
    float BNP_pgml  = CurrentToConc_pgml(Ipk_uA, BNP_INTERCEPT_UA, BNP_SLOPE_UA_DEC);

    printf("k:%lu, Ebase(mV): %.1f, Epulse(mV): %.1f, dI(uA): %.6f, |Ipk|(uA): %.6f, cTnI(pg/mL): %.3e, BNP(pg/mL): %.3e\n",
           index++, Ebase_mV, Epulse_mV, pData[i], Ipk_uA, cTnI_pgml, BNP_pgml);
  }
  return 0;
}

static int32_t AD5940PlatformCfg(void)
{
  CLKCfg_Type clk_cfg;
  SEQCfg_Type seq_cfg;
  FIFOCfg_Type fifo_cfg;
  LFOSCMeasure_Type LfoscMeasure;

  AD5940_Initialize();

  clk_cfg.HFOSCEn = bTRUE;
  clk_cfg.HFXTALEn = bFALSE;
  clk_cfg.LFOSCEn = bTRUE;
  clk_cfg.HfOSC32MHzMode = bFALSE;
  clk_cfg.SysClkSrc = SYSCLKSRC_HFOSC;
  clk_cfg.SysClkDiv = SYSCLKDIV_1;
  clk_cfg.ADCCLkSrc = ADCCLKSRC_HFOSC;
  clk_cfg.ADCClkDiv = ADCCLKDIV_1;
  AD5940_CLKCfg(&clk_cfg);

  fifo_cfg.FIFOEn = bTRUE;
  fifo_cfg.FIFOMode = FIFOMODE_FIFO;
  fifo_cfg.FIFOSize = FIFOSIZE_2KB;
  fifo_cfg.FIFOSrc = FIFOSRC_SINC3;
  fifo_cfg.FIFOThresh = 4;
  AD5940_FIFOCfg(&fifo_cfg);

  seq_cfg.SeqMemSize = SEQMEMSIZE_4KB;
  seq_cfg.SeqBreakEn = bFALSE;
  seq_cfg.SeqIgnoreEn = bTRUE;
  seq_cfg.SeqCntCRCClr = bTRUE;
  seq_cfg.SeqEnable = bFALSE;
  seq_cfg.SeqWrTimer = 0;
  AD5940_SEQCfg(&seq_cfg);

  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE);
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  AD5940_INTCCfg(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH|AFEINTSRC_ENDSEQ|AFEINTSRC_CUSTOMINT0, bTRUE);
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);

  LfoscMeasure.CalDuration = 1000.0;
  LfoscMeasure.CalSeqAddr = 0;
  LfoscMeasure.SystemClkFreq = 16000000.0f;
  AD5940_LFOSCMeasure(&LfoscMeasure, &LFOSCFreq);

  printf("LFOSC Freq:%f\n", LFOSCFreq);
  AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);
  return 0;
}

void AD5940DPVStructInit(void)
{
  AppDPVCfg_Type *pCfg;
  AppDPVGetCfg(&pCfg);

  pCfg->SeqStartAddr = 0x10;
  pCfg->MaxSeqLen = 1024 - 0x10;
  pCfg->RcalVal = 200.0f;
  pCfg->ADCRefVolt = 1820.0f;
  pCfg->FifoThresh = 480;
  pCfg->SysClkFreq = 16000000.0f;
  pCfg->LFOSCClkFreq = LFOSCFreq;

  /* Required DPV parameters */
  pCfg->RampStartVolt = 0.0f;       /* Ein = 0 V */
  pCfg->RampPeakVolt  = 600.0f;     /* Ef  = 0.6 V */
  pCfg->VzeroStart    = 1300.0f;
  pCfg->VzeroPeak     = 1300.0f;

  pCfg->DPV_WaitTime    = 0.5f;     /* tw */
  pCfg->DPV_PulseTime   = 0.05f;    /* tp */
  pCfg->DPV_PulseHeight = 50.0f;    /* 50 mV pulse */

  /* Staircase increment (set as needed) */
  pCfg->StaircaseInc = 5.0f;        /* mV per step */

  pCfg->SampleDelay = 10.0f;        /* ms */
  pCfg->LPTIARtiaSel = LPTIARTIA_8K;
}

void AD5940_Main(void)
{
  uint32_t temp;

  AD5940PlatformCfg();
  AD5940DPVStructInit();

  AppDPVInit(AppBuff, APPBUFF_SIZE);
  AppDPVCtrl(APPCTRL_START, 0);

  while(1)
  {
    if(AD5940_GetMCUIntFlag())
    {
      AD5940_ClrMCUIntFlag();
      temp = APPBUFF_SIZE;
      AppDPVISR(AppBuff, &temp);
      RampShowResult((float*)AppBuff, temp);
    }
  }
}
