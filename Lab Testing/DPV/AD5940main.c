/* ========================= ad5940main.c (DPV) =========================
   Platform + App glue for Differential Pulse Voltammetry (DPV)
   - ADDED: prints peak current + BNP/cTnI concentration using calibration curves
====================================================================== */

#include <stdio.h>
#include <string.h>
#include "ad5940.h"
#include "DPV.h"

#define APPBUFF_SIZE 1024
uint32_t AppBuff[APPBUFF_SIZE];
static float LFOSCFreq = 32000.0f;

/* These functions are implemented in DPV.c above */
extern float AppDPV_GetLastPeak_uA(void);
extern float AppDPV_GetLastConc_cTnI_pg_mL(void);
extern float AppDPV_GetLastConc_BNP_pg_mL(void);

static void AD5940_PrintDPV(float *pData, uint32_t n)
{
  for(uint32_t i = 0; i < n; i++)
  {
    printf("index:%lu, %.6f\r\n", (unsigned long)i, pData[i]);
  }
}

static void AD5940_PrintCalibrated(void)
{
  float Ipk = AppDPV_GetLastPeak_uA();
  float cTnI = AppDPV_GetLastConc_cTnI_pg_mL();
  float BNP  = AppDPV_GetLastConc_BNP_pg_mL();

  printf("DPV peak|I| = %.3f uA\r\n", Ipk);
  printf("cTnI (from calibration) = %.3e pg/mL\r\n", cTnI);
  printf("BNP  (from calibration) = %.3e pg/mL\r\n", BNP);
}

static int32_t AD5940PlatformCfg(void)
{
  CLKCfg_Type clk_cfg;
  SEQCfg_Type seq_cfg;
  FIFOCfg_Type fifo_cfg;
  LFOSCMeasure_Type LfoscMeasure;

  AD5940_Initialize();

  memset(&clk_cfg, 0, sizeof(clk_cfg));
  clk_cfg.HFOSCEn = bTRUE;
  clk_cfg.HFXTALEn = bFALSE;
  clk_cfg.LFOSCEn = bTRUE;
  clk_cfg.HfOSC32MHzMode = bFALSE;
  clk_cfg.SysClkSrc = SYSCLKSRC_HFOSC;
  clk_cfg.SysClkDiv = SYSCLKDIV_1;
  clk_cfg.ADCCLkSrc = ADCCLKSRC_HFOSC;
  clk_cfg.ADCClkDiv = ADCCLKDIV_1;
  AD5940_CLKCfg(&clk_cfg);

  memset(&fifo_cfg, 0, sizeof(fifo_cfg));
  fifo_cfg.FIFOEn = bTRUE;
  fifo_cfg.FIFOMode = FIFOMODE_FIFO;
  fifo_cfg.FIFOSize = FIFOSIZE_2KB;
  fifo_cfg.FIFOSrc = FIFOSRC_SINC3;
  fifo_cfg.FIFOThresh = 4;
  AD5940_FIFOCfg(&fifo_cfg);

  memset(&seq_cfg, 0, sizeof(seq_cfg));
  seq_cfg.SeqMemSize = SEQMEMSIZE_4KB;
  seq_cfg.SeqBreakEn = bFALSE;
  seq_cfg.SeqIgnoreEn = bTRUE;
  seq_cfg.SeqCntCRCClr = bTRUE;
  seq_cfg.SeqEnable = bFALSE;
  seq_cfg.SeqWrTimer = 0;
  AD5940_SEQCfg(&seq_cfg);

  /* Enable AFE interrupts to MCU */
  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE);
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);

  /* DPV uses: FIFO thresh, ENDSEQ, and CUSTOMINT0 refill */
  AD5940_INTCCfg(AFEINTC_0,
                 AFEINTSRC_DATAFIFOTHRESH | AFEINTSRC_ENDSEQ | AFEINTSRC_CUSTOMINT0,
                 bTRUE);
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);

  memset(&LfoscMeasure, 0, sizeof(LfoscMeasure));
  LfoscMeasure.CalDuration = 1000.0f;
  LfoscMeasure.CalSeqAddr = 0;
  LfoscMeasure.SystemClkFreq = 16000000.0f;
  AD5940_LFOSCMeasure(&LfoscMeasure, &LFOSCFreq);
  printf("LFOSC Freq:%f\r\n", LFOSCFreq);

  AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);
  return 0;
}

static void AD5940DPV_UserCfg(void)
{
  AppDPVCfg_Type *pCfg;
  AppDPVGetCfg(&pCfg);

  pCfg->bParaChanged = bTRUE;

  pCfg->LFOSCClkFreq = LFOSCFreq;
  pCfg->SysClkFreq   = 16000000.0f;
  pCfg->AdcClkFreq   = 16000000.0f;

  pCfg->SeqStartAddr = 0x10;
  pCfg->MaxSeqLen    = 1024 - 0x10;

  pCfg->RcalVal     = 200.0f;
  pCfg->ADCRefVolt  = 1820.0f;

  pCfg->Ein_mV           = 100.0f;
  pCfg->Efinal_mV        = -100.0f;
  pCfg->StepIncrement_mV = -20.0f;

  pCfg->PulseAmplitude_mV = -100.0f;
  pCfg->WaitTime_s        = 0.50f;
  pCfg->PulseTime_s       = 0.25f;

  pCfg->Vzero_mV = 1600.0f;

  pCfg->SampleDelayBase_ms  = 5.0f;
  pCfg->SampleDelayPulse_ms = 5.0f;

  pCfg->LPTIARtiaSel      = LPTIARTIA_20K;
  pCfg->ExternalRtiaValue = 20000.0f;
  pCfg->AdcPgaGain        = ADCPGA_1P5;
  pCfg->ADCSinc3Osr       = ADCSINC3OSR_4;

  pCfg->FifoThresh        = 4;
}

void AD5940_Main(void)
{
  uint32_t DataCount;

  AD5940PlatformCfg();
  AD5940DPV_UserCfg();

  if(AppDPVInit(AppBuff, APPBUFF_SIZE) != AD5940ERR_OK)
  {
    printf("AppDPVInit failed\r\n");
    while(1);
  }

  if(AppDPVCtrl(APPCTRL_START, 0) != AD5940ERR_OK)
  {
    printf("AppDPVCtrl START failed\r\n");
    while(1);
  }

  while(1)
  {
    if(AD5940_GetMCUIntFlag())
    {
      AD5940_ClrMCUIntFlag();

      DataCount = APPBUFF_SIZE;
      if(AppDPVISR(AppBuff, &DataCount) == AD5940ERR_OK)
      {
        if(DataCount)
        {
//          /* Print Idiff samples */
//          AD5940_PrintDPV((float*)AppBuff, DataCount);

//          /* ADDED: print peak + concentration conversions */
//          AD5940_PrintCalibrated();
        }
      }
    }
  }
}
