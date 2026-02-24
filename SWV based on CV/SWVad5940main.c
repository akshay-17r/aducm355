/******************************************************************************
Copyright (c) 2017-2019 Analog Devices, Inc. All Rights Reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.
By using this software you agree to the terms of the associated
Analog Devices Software License Agreement.
*****************************************************************************/

/*
Description (SWV version of AD5940Main.c):
- Configures AD5940/ADuCM355 AFE platform (clocks, FIFO, sequencer, interrupts)
- Measures LFOSC and passes it to the SWV application
- Initializes and starts the SWV app
- Services AFE interrupts and prints SWV processed results over UART

Notes:
- This file expects your BSP/UART init to already be done in main.c (as in ADI examples).
- This file expects SqrWaveVoltammetry.c/.h to be compiled in the project.
*/

#include <stdio.h>
#include <string.h>
#include "ad5940.h"
#include "SqrWaveVoltammetry.h"

#define APPBUFF_SIZE 1024
uint32_t AppBuff[APPBUFF_SIZE];
static float LFOSCFreq = 32000.0f;

static void AD5940_PrintSWV(float *pData, uint32_t n)
{
  /* pData already contains processed values from AppSWVISR(), typically current in uA */
  for(uint32_t i = 0; i < n; i++)
  {
    printf("index:%lu, %.6f\r\n", (unsigned long)i, pData[i]);
  }
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

  /* SWV uses these interrupts on INTC0 in typical ADI examples */
  AD5940_INTCCfg(AFEINTC_0,
                 AFEINTSRC_DATAFIFOTHRESH | AFEINTSRC_ENDSEQ | AFEINTSRC_CUSTOMINT0,
                 bTRUE);
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);

  /* Measure LFOSC so WUPT timing matches reality */
  memset(&LfoscMeasure, 0, sizeof(LfoscMeasure));
  LfoscMeasure.CalDuration = 1000.0f;
  LfoscMeasure.CalSeqAddr = 0;
  LfoscMeasure.SystemClkFreq = 16000000.0f;
  AD5940_LFOSCMeasure(&LfoscMeasure, &LFOSCFreq);
  printf("LFOSC Freq:%f\r\n", LFOSCFreq);

  AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);
  return 0;
}

static void AD5940SWV_UserCfg(void)
{
  AppSWVCfg_Type *pCfg;
  AppSWVGetCfg(&pCfg);

  pCfg->bParaChanged = bTRUE;

  /* Critical: use measured LFOSC */
  pCfg->LFOSCClkFreq = LFOSCFreq;
  pCfg->SysClkFreq   = 16000000.0f;
  pCfg->AdcClkFreq   = 16000000.0f;

  /* Sequencer SRAM layout.
     Leave room at the beginning (0x10 words) for LFOSC cal and safety.
     MaxSeqLen is in "commands" (32-bit words) for ADI sequence engine. */
  pCfg->SeqStartAddr = 0x10;
  pCfg->MaxSeqLen    = 1024 - 0x10;

  /* Board-dependent constants */
  pCfg->RcalVal     = 200.0f;    /* ADuCM355 eval board RCAL typically 200O */
  pCfg->ADCRefVolt  = 1820.0f;   /* mV: internal reference used by ADI examples */

  /* SWV measurement configuration (adjust to your experiment) */
  pCfg->RampStartVolt      = -100.0f;  /* mV (Vbias-Vzero) */
  pCfg->RampPeakVolt       = +100.0f;  /* mV */
  pCfg->VzeroStart         = 2200.0f;  /* mV */
  pCfg->VzeroPeak          = 400.0f;   /* mV */

  pCfg->SqrWvAmplitude     = 25.0f;    /* mV */
  pCfg->SqrWvRampIncrement = 5.0f;     /* mV */
  pCfg->Frequency          = 25.0f;    /* Hz */

  /* ADC timing relative to DAC update */
  pCfg->SampleDelay        = 1.0f;     /* ms */

  /* TIA and ADC gain */
  pCfg->LPTIARtiaSel       = LPTIARTIA_20K;
  pCfg->ExternalRtiaValue  = 20000.0f;
  pCfg->AdcPgaGain         = ADCPGA_1P5;
  pCfg->ADCSinc3Osr        = ADCSINC3OSR_4;

  /* FIFO interrupt threshold.
     Keep small so you see data quickly over UART. */
  pCfg->FifoThresh         = 4;
}

void AD5940_Main(void)
{
  uint32_t DataCount;

  AD5940PlatformCfg();
  AD5940SWV_UserCfg();

  /* Initialize SWV application */
  if(AppSWVInit(AppBuff, APPBUFF_SIZE) != AD5940ERR_OK)
  {
    printf("AppSWVInit failed\r\n");
    while(1);
  }

  /* Start SWV */
  if(AppSWVCtrl(APPCTRL_START, 0) != AD5940ERR_OK)
  {
    printf("AppSWVCtrl START failed\r\n");
    while(1);
  }

  while(1)
  {
    if(AD5940_GetMCUIntFlag())
    {
      AD5940_ClrMCUIntFlag();

      DataCount = APPBUFF_SIZE;
      if(AppSWVISR(AppBuff, &DataCount) == AD5940ERR_OK)
      {
        if(DataCount)
          AD5940_PrintSWV((float*)AppBuff, DataCount);
      }
    }
  }
}
