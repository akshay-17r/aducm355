/* ============================= DPV.c =============================
   Differential Pulse Voltammetry (DPV) measurement sequences.
   - Two samples per step: baseline and pulse
   - Output: I(pulse) - I(base) (uA)
   - ADDED: peak extraction + calibration-curve concentration conversion
     cTnI: I(uA) = 82.42 - 11.33*log10(C)
     BNP : I(uA) = 101.96 - 11.95*log10(C)
     => C = 10^((A - I)/B)
================================================================== */

#include "DPV.h"
#include <string.h>
#include <math.h>   /* powf() */

#ifndef DPV_CALC_USE_ABS_PEAK
/* 1 = use |I_peak| so sign/polarity doesn’t break the concentration conversion */
#define DPV_CALC_USE_ABS_PEAK  1
#endif

/* ---------------- Default configuration (editable in AD5940Main.c) ---------------- */
static AppDPVCfg_Type AppDPVCfg =
{
  .bParaChanged = bFALSE,
  .SeqStartAddr = 0,
  .MaxSeqLen = 0,
  .SeqStartAddrCal = 0,
  .MaxSeqLenCal = 0,

  .LFOSCClkFreq = 32000.0f,
  .SysClkFreq   = 16000000.0f,
  .AdcClkFreq   = 16000000.0f,

  .RcalVal      = 200.0f,
  .ADCRefVolt   = 1820.0f,

  .Ein_mV            = 0.0f,
  .Efinal_mV         = 600.0f,
  .StepIncrement_mV  = 5.0f,

  .PulseAmplitude_mV = 50.0f,
  .WaitTime_s        = 0.5f,
  .PulseTime_s       = 0.05f,

  .Vzero_mV          = 1100.0f,

  .SampleDelayBase_ms  = 5.0f,
  .SampleDelayPulse_ms = 5.0f,

  .LPTIARtiaSel       = LPTIARTIA_2K,
  .ExternalRtiaValue  = 20000.0f,
  .AdcPgaGain         = ADCPGA_1P5,
  .ADCSinc3Osr        = ADCSINC3OSR_4,

  .FifoThresh         = 4,

  .DPVInited          = bFALSE,
  .bFirstDACSeq       = bTRUE,
  .TotalSteps         = 0,
  .UpdatesTotal       = 0,
  .UpdateIndex        = 0,
  .CurrE_mV           = 0.0f,
  .VzeroCode          = 0,
  .BaseCode12         = 0.0f,
  .PulseCode12        = 0.0f,
  .StopRequired       = bFALSE,
};

static uint32_t clamp_u32(uint32_t x, uint32_t lo, uint32_t hi)
{
  if(x < lo) return lo;
  if(x > hi) return hi;
  return x;
}

static float fabsf_safe(float x) { return (x < 0.0f) ? -x : x; }

/* ===================== ADDED: calibration + peak helpers ===================== */
static float dpv_absf(float x) { return (x < 0.0f) ? -x : x; }

/* cTnI: I = 82.42 - 11.33*log10(C) => C = 10^((82.42 - I)/11.33) */
static float DPV_ConcFromI_cTnI_pg_per_mL(float I_uA)
{
  float x = (82.42f - I_uA) / 11.33f;
  return powf(10.0f, x);
}

/* BNP: I = 101.96 - 11.95*log10(C) => C = 10^((101.96 - I)/11.95) */
static float DPV_ConcFromI_BNP_pg_per_mL(float I_uA)
{
  float x = (101.96f - I_uA) / 11.95f;
  return powf(10.0f, x);
}

/* Extract peak (max magnitude) from Idiff array */
static float DPV_FindPeakAbs_uA(const float *pIdiff, uint32_t n)
{
  float peak = 0.0f;
  for(uint32_t i = 0; i < n; i++)
  {
    float a = dpv_absf(pIdiff[i]);
    if(a > peak) peak = a;
  }
  return peak;
}

/* LAST computed results (read from AD5940Main.c via extern getter below) */
static float gDPV_LastPeak_uA = 0.0f;
static float gDPV_LastConc_cTnI_pg_mL = 0.0f;
static float gDPV_LastConc_BNP_pg_mL  = 0.0f;

/* Optional: expose latest values without changing your header */
float AppDPV_GetLastPeak_uA(void)            { return gDPV_LastPeak_uA; }
float AppDPV_GetLastConc_cTnI_pg_mL(void)    { return gDPV_LastConc_cTnI_pg_mL; }
float AppDPV_GetLastConc_BNP_pg_mL(void)     { return gDPV_LastConc_BNP_pg_mL; }

/* =================== existing code continues (unchanged) =================== */

/* Compute number of steps (inclusive endpoints) */
static uint32_t DPV_ComputeTotalSteps(void)
{
  float span = AppDPVCfg.Efinal_mV - AppDPVCfg.Ein_mV;
  float inc  = AppDPVCfg.StepIncrement_mV;

  if(inc == 0.0f)
    return 1U;

  span = fabsf_safe(span);
  inc  = fabsf_safe(inc);

  uint32_t n = (uint32_t)(span / inc) + 1U;
  if(n < 1U) n = 1U;
  return n;
}

/* ------------------------- Public config getter -------------------------- */
AD5940Err AppDPVGetCfg(void *pCfg)
{
  if(!pCfg) return AD5940ERR_PARA;
  *(AppDPVCfg_Type**)pCfg = &AppDPVCfg;
  return AD5940ERR_OK;
}

/* ------------------------- Sequence generators -------------------------- */
static AD5940Err AppDPVSeqInitGen(void)
{
  AD5940Err error;
  const uint32_t *pSeqCmd;
  uint32_t SeqLen;

  AFERefCfg_Type aferef_cfg;
  LPLoopCfg_Type lploop_cfg;
  DSPCfg_Type dsp_cfg;

  AD5940_SEQGenCtrl(bTRUE);
  AD5940_AFECtrlS(AFECTRL_ALL, bFALSE);

  memset(&aferef_cfg, 0, sizeof(aferef_cfg));
  aferef_cfg.HpBandgapEn = bTRUE;
  aferef_cfg.Hp1V1BuffEn = bTRUE;
  aferef_cfg.Hp1V8BuffEn = bTRUE;

  aferef_cfg.LpBandgapEn = bTRUE;
  aferef_cfg.LpRefBufEn = bTRUE;
  aferef_cfg.LpRefBoostEn = bFALSE;
  AD5940_REFCfgS(&aferef_cfg);

  memset(&lploop_cfg, 0, sizeof(lploop_cfg));

  lploop_cfg.LpAmpCfg.LpAmpSel = LPAMP0;
  lploop_cfg.LpAmpCfg.LpAmpPwrMod = LPAMPPWR_NORM;
  lploop_cfg.LpAmpCfg.LpPaPwrEn = bTRUE;
  lploop_cfg.LpAmpCfg.LpTiaPwrEn = bTRUE;
  lploop_cfg.LpAmpCfg.LpTiaRf = LPTIARF_20K;
  lploop_cfg.LpAmpCfg.LpTiaRload = LPTIARLOAD_10R;
  lploop_cfg.LpAmpCfg.LpTiaRtia = AppDPVCfg.LPTIARtiaSel;

  if(AppDPVCfg.LPTIARtiaSel == LPTIARTIA_OPEN)
    lploop_cfg.LpAmpCfg.LpTiaSW = LPTIASW(2)|LPTIASW(4)|LPTIASW(5)|LPTIASW(9);
  else
    lploop_cfg.LpAmpCfg.LpTiaSW = LPTIASW(2)|LPTIASW(4)|LPTIASW(5);

  lploop_cfg.LpDacCfg.LpdacSel = LPDAC0;
  lploop_cfg.LpDacCfg.LpDacRef = LPDACREF_2P5;
  lploop_cfg.LpDacCfg.LpDacSrc = LPDACSRC_MMR;
  lploop_cfg.LpDacCfg.DataRst  = bFALSE;

  lploop_cfg.LpDacCfg.LpDacSW = LPDACSW_VBIAS2LPPA | LPDACSW_VZERO2LPTIA;

  /* Initial DAC codes: start at Ein */
  {
    uint32_t vzero6 = (uint32_t)((AppDPVCfg.Vzero_mV - 200.0f)/DAC6BITVOLT_1LSB);
    float vbias12f  = (-AppDPVCfg.Ein_mV)/DAC12BITVOLT_1LSB + (float)vzero6*64.0f;

    uint32_t vbias12 = (vbias12f < 0.0f) ? 0U : (uint32_t)vbias12f;
    if(vbias12 > 4095U) vbias12 = 4095U;
    if(vzero6  > 63U)   vzero6  = 63U;

    lploop_cfg.LpDacCfg.DacData6Bit  = vzero6;
    lploop_cfg.LpDacCfg.DacData12Bit = vbias12;

    AppDPVCfg.VzeroCode = vzero6;
  }

  lploop_cfg.LpDacCfg.PowerEn = bTRUE;
  AD5940_LPLoopCfgS(&lploop_cfg);

  AD5940_StructInit(&dsp_cfg, sizeof(dsp_cfg));
  dsp_cfg.ADCBaseCfg.ADCMuxN = ADCMUXN_LPTIA0_N;
  dsp_cfg.ADCBaseCfg.ADCMuxP = ADCMUXP_LPTIA0_P;
  dsp_cfg.ADCBaseCfg.ADCPga  = AppDPVCfg.AdcPgaGain;

  dsp_cfg.ADCFilterCfg.ADCSinc3Osr = AppDPVCfg.ADCSinc3Osr;
  dsp_cfg.ADCFilterCfg.ADCRate = ADCRATE_800KHZ;
  dsp_cfg.ADCFilterCfg.BpSinc3 = bFALSE;
  dsp_cfg.ADCFilterCfg.Sinc2NotchEnable = bTRUE;
  dsp_cfg.ADCFilterCfg.BpNotch = bTRUE;
  dsp_cfg.ADCFilterCfg.ADCSinc2Osr = ADCSINC2OSR_1067;
  dsp_cfg.ADCFilterCfg.ADCAvgNum = ADCAVGNUM_2;
  AD5940_DSPCfgS(&dsp_cfg);

  AD5940_SEQGenInsert(SEQ_STOP());
  AD5940_SEQGenCtrl(bFALSE);

  error = AD5940_SEQGenFetchSeq(&pSeqCmd, &SeqLen);
  if(error != AD5940ERR_OK) return error;
  if(SeqLen >= AppDPVCfg.MaxSeqLen) return AD5940ERR_SEQLEN;

  AD5940_StructInit(&AppDPVCfg.InitSeqInfo, sizeof(AppDPVCfg.InitSeqInfo));
  AppDPVCfg.InitSeqInfo.SeqId = SEQID_3;
  AppDPVCfg.InitSeqInfo.SeqRamAddr = AppDPVCfg.SeqStartAddr;
  AppDPVCfg.InitSeqInfo.pSeqCmd = pSeqCmd;
  AppDPVCfg.InitSeqInfo.SeqLen = SeqLen;
  AppDPVCfg.InitSeqInfo.WriteSRAM = bTRUE;
  AD5940_SEQInfoCfg(&AppDPVCfg.InitSeqInfo);

  return AD5940ERR_OK;
}

static AD5940Err AppDPVSeqADCCtrlGen(void)
{
  AD5940Err error;
  const uint32_t *pSeqCmd;
  uint32_t SeqLen;
  uint32_t WaitClks;
  ClksCalInfo_Type clks_cal;

  memset(&clks_cal, 0, sizeof(clks_cal));
  clks_cal.DataCount = 1;
  clks_cal.DataType = DATATYPE_SINC3;
  clks_cal.ADCSinc3Osr = AppDPVCfg.ADCSinc3Osr;
  clks_cal.ADCSinc2Osr = ADCSINC2OSR_1067;
  clks_cal.ADCAvgNum = ADCAVGNUM_2;
  clks_cal.RatioSys2AdcClk = AppDPVCfg.SysClkFreq / AppDPVCfg.AdcClkFreq;
  AD5940_ClksCalculate(&clks_cal, &WaitClks);

  AD5940_SEQGenCtrl(bTRUE);
  AD5940_AFECtrlS(AFECTRL_ADCPWR, bTRUE);
  AD5940_SEQGenInsert(SEQ_WAIT(16*250));          /* 250us */
  AD5940_AFECtrlS(AFECTRL_ADCCNV, bTRUE);
  AD5940_SEQGenInsert(SEQ_WAIT(WaitClks));
  AD5940_AFECtrlS(AFECTRL_ADCPWR|AFECTRL_ADCCNV, bFALSE);
  AD5940_EnterSleepS();
  error = AD5940_SEQGenFetchSeq(&pSeqCmd, &SeqLen);
  AD5940_SEQGenCtrl(bFALSE);

  if(error != AD5940ERR_OK) return error;
  if((SeqLen + AppDPVCfg.InitSeqInfo.SeqLen) >= AppDPVCfg.MaxSeqLen) return AD5940ERR_SEQLEN;

  AD5940_StructInit(&AppDPVCfg.ADCSeqInfo, sizeof(AppDPVCfg.ADCSeqInfo));
  AppDPVCfg.ADCSeqInfo.SeqId = SEQID_2;
  AppDPVCfg.ADCSeqInfo.SeqRamAddr = AppDPVCfg.InitSeqInfo.SeqRamAddr + AppDPVCfg.InitSeqInfo.SeqLen;
  AppDPVCfg.ADCSeqInfo.pSeqCmd = pSeqCmd;
  AppDPVCfg.ADCSeqInfo.SeqLen = SeqLen;
  AppDPVCfg.ADCSeqInfo.WriteSRAM = bTRUE;
  AD5940_SEQInfoCfg(&AppDPVCfg.ADCSeqInfo);

  return AD5940ERR_OK;
}

/* --------------------- DAC stepping logic (DPV) --------------------- */
static AD5940Err DPV_DacRegUpdate(uint32_t *pDACData)
{
  if(AppDPVCfg.UpdateIndex >= AppDPVCfg.UpdatesTotal)
    return AD5940ERR_OK;

  uint32_t step = AppDPVCfg.UpdateIndex / 2U;
  uint32_t phase = AppDPVCfg.UpdateIndex & 1U; /* 0=baseline, 1=pulse */

  float Ebase = AppDPVCfg.Ein_mV + (float)step * AppDPVCfg.StepIncrement_mV;
  float Euse  = (phase == 0U) ? Ebase : (Ebase - AppDPVCfg.PulseAmplitude_mV);

  uint32_t vzero6 = AppDPVCfg.VzeroCode;
  float vbias12f = (-Euse)/DAC12BITVOLT_1LSB + (float)vzero6*64.0f;

  uint32_t vbias12 = (vbias12f < 0.0f) ? 0U : (uint32_t)vbias12f;
  if(vbias12 > 4095U) vbias12 = 4095U;
  if(vzero6  > 63U)   vzero6  = 63U;

  *pDACData = (vzero6<<12) | (vbias12 & 0x0FFF);

  AppDPVCfg.UpdateIndex++;
//	
//	printf("step=%lu phase=%lu Ebase=%.1f Euse=%.1f VzeroCode=%lu vbias12=%lu\r\n",
//       (unsigned long)step,
//       (unsigned long)phase,
//       Ebase,
//       Euse,
//       (unsigned long)vzero6,
//       (unsigned long)vbias12);
//			 
  return AD5940ERR_OK;
}

/* Ping-pong DAC sequences in SRAM (same pattern as your SWV code) */
static AD5940Err AppDPVSeqDACCtrlGen(void)
{
#define SEQLEN_ONESTEP 4U
#define CURRBLK_BLK0   0U
#define CURRBLK_BLK1   1U

  static BoolFlag  bCmdForSeq0 = bTRUE;
  static uint32_t  DACSeqBlk0Addr, DACSeqBlk1Addr;
  static uint32_t  UpdatesRemaining, UpdatesPerBlock, DACSeqCurrBlk;

  AD5940Err error = AD5940ERR_OK;
  uint32_t BlockStartSRAMAddr;
  uint32_t DACData, SRAMAddr;
  uint32_t i;
  uint32_t UpdatesThisBlock;
  BoolFlag bIsFinalBlk;
  uint32_t SeqCmdBuff[SEQLEN_ONESTEP];

  AppDPVCfg.TotalSteps   = DPV_ComputeTotalSteps();
  AppDPVCfg.UpdatesTotal = 2U * AppDPVCfg.TotalSteps;

  if(AppDPVCfg.FifoThresh < 1U) AppDPVCfg.FifoThresh = 4U;

  if(AppDPVCfg.bFirstDACSeq == bTRUE)
  {
    int32_t DACSeqLenMax;

    AppDPVCfg.UpdateIndex = 0;
    UpdatesRemaining = AppDPVCfg.UpdatesTotal;

    DACSeqLenMax = (int32_t)AppDPVCfg.MaxSeqLen
                 - (int32_t)AppDPVCfg.InitSeqInfo.SeqLen
                 - (int32_t)AppDPVCfg.ADCSeqInfo.SeqLen;

    if(DACSeqLenMax < (int32_t)(SEQLEN_ONESTEP*8U))
      return AD5940ERR_SEQLEN;

    DACSeqLenMax -= (int32_t)(SEQLEN_ONESTEP*2U);
    UpdatesPerBlock = (uint32_t)(DACSeqLenMax/((int32_t)SEQLEN_ONESTEP)/2);
    if(UpdatesPerBlock < 8U) UpdatesPerBlock = 8U;

    DACSeqBlk0Addr = AppDPVCfg.ADCSeqInfo.SeqRamAddr + AppDPVCfg.ADCSeqInfo.SeqLen;
    DACSeqBlk1Addr = DACSeqBlk0Addr + UpdatesPerBlock*SEQLEN_ONESTEP;
    DACSeqCurrBlk  = CURRBLK_BLK0;

    bCmdForSeq0 = bTRUE;
  }

  if(UpdatesRemaining == 0U) return AD5940ERR_OK;

  bIsFinalBlk = (UpdatesRemaining <= UpdatesPerBlock) ? bTRUE : bFALSE;
  UpdatesThisBlock = bIsFinalBlk ? UpdatesRemaining : UpdatesPerBlock;
  UpdatesRemaining -= UpdatesThisBlock;

  BlockStartSRAMAddr = (DACSeqCurrBlk == CURRBLK_BLK0) ? DACSeqBlk0Addr : DACSeqBlk1Addr;
  SRAMAddr = BlockStartSRAMAddr;

  for(i=0; i<UpdatesThisBlock-1U; i++)
  {
    uint32_t CurrAddr = SRAMAddr;
    SRAMAddr += SEQLEN_ONESTEP;

    DPV_DacRegUpdate(&DACData);
    SeqCmdBuff[0] = SEQ_WR(REG_AFE_LPDACDAT0, DACData);
    SeqCmdBuff[1] = SEQ_WAIT(10);
    SeqCmdBuff[2] = SEQ_WR(bCmdForSeq0 ? REG_AFE_SEQ1INFO : REG_AFE_SEQ0INFO,
                           (SRAMAddr<<BITP_AFE_SEQ1INFO_ADDR) | (SEQLEN_ONESTEP<<BITP_AFE_SEQ1INFO_LEN));
    SeqCmdBuff[3] = SEQ_SLP();

    AD5940_SEQCmdWrite(CurrAddr, SeqCmdBuff, SEQLEN_ONESTEP);
    bCmdForSeq0 = bCmdForSeq0 ? bFALSE : bTRUE;
  }

  if(bIsFinalBlk)
  {
    uint32_t CurrAddr = SRAMAddr;
    SRAMAddr += SEQLEN_ONESTEP;

    DPV_DacRegUpdate(&DACData);
    SeqCmdBuff[0] = SEQ_WR(REG_AFE_LPDACDAT0, DACData);
    SeqCmdBuff[1] = SEQ_WAIT(10);
    SeqCmdBuff[2] = SEQ_WR(bCmdForSeq0 ? REG_AFE_SEQ1INFO : REG_AFE_SEQ0INFO,
                           (SRAMAddr<<BITP_AFE_SEQ1INFO_ADDR) | (SEQLEN_ONESTEP<<BITP_AFE_SEQ1INFO_LEN));
    SeqCmdBuff[3] = SEQ_SLP();
    AD5940_SEQCmdWrite(CurrAddr, SeqCmdBuff, SEQLEN_ONESTEP);

    CurrAddr += SEQLEN_ONESTEP;

    SeqCmdBuff[0] = SEQ_NOP();
    SeqCmdBuff[1] = SEQ_NOP();
    SeqCmdBuff[2] = SEQ_NOP();
    SeqCmdBuff[3] = SEQ_STOP();
    AD5940_SEQCmdWrite(CurrAddr, SeqCmdBuff, SEQLEN_ONESTEP);
  }
  else
  {
    uint32_t CurrAddr = SRAMAddr;
    SRAMAddr = (DACSeqCurrBlk == CURRBLK_BLK0) ? DACSeqBlk1Addr : DACSeqBlk0Addr;

    DPV_DacRegUpdate(&DACData);
    SeqCmdBuff[0] = SEQ_WR(REG_AFE_LPDACDAT0, DACData);
    SeqCmdBuff[1] = SEQ_WAIT(10);
    SeqCmdBuff[2] = SEQ_WR(bCmdForSeq0 ? REG_AFE_SEQ1INFO : REG_AFE_SEQ0INFO,
                           (SRAMAddr<<BITP_AFE_SEQ1INFO_ADDR) | (SEQLEN_ONESTEP<<BITP_AFE_SEQ1INFO_LEN));
    SeqCmdBuff[3] = SEQ_INT0();
    AD5940_SEQCmdWrite(CurrAddr, SeqCmdBuff, SEQLEN_ONESTEP);

    bCmdForSeq0 = bCmdForSeq0 ? bFALSE : bTRUE;
  }

  DACSeqCurrBlk = (DACSeqCurrBlk == CURRBLK_BLK0) ? CURRBLK_BLK1 : CURRBLK_BLK0;

  if(AppDPVCfg.bFirstDACSeq)
  {
    AppDPVCfg.bFirstDACSeq = bFALSE;

    if(!bIsFinalBlk)
    {
      error = AppDPVSeqDACCtrlGen();
      if(error != AD5940ERR_OK) return error;
    }

    AppDPVCfg.DACSeqInfo.SeqId = SEQID_0; /* first DAC update */
    AppDPVCfg.DACSeqInfo.SeqLen = SEQLEN_ONESTEP;
    AppDPVCfg.DACSeqInfo.SeqRamAddr = BlockStartSRAMAddr;
    AppDPVCfg.DACSeqInfo.WriteSRAM = bFALSE;
    AD5940_SEQInfoCfg(&AppDPVCfg.DACSeqInfo);
  }

  return AD5940ERR_OK;
}

/* ------------------------------ RTIA Cal -------------------------------- */
static AD5940Err AppDPVRtiaCal(void)
{
  fImpPol_Type RtiaCalValue;
  LPRTIACal_Type lprtia_cal;

  AD5940_StructInit(&lprtia_cal, sizeof(lprtia_cal));
  lprtia_cal.LpAmpSel = LPAMP0;
  lprtia_cal.bPolarResult = bTRUE;
  lprtia_cal.AdcClkFreq = AppDPVCfg.AdcClkFreq;
  lprtia_cal.SysClkFreq = AppDPVCfg.SysClkFreq;
  lprtia_cal.ADCSinc3Osr = ADCSINC3OSR_4;
  lprtia_cal.ADCSinc2Osr = ADCSINC2OSR_22;
  lprtia_cal.DftCfg.DftNum = DFTNUM_2048;
  lprtia_cal.DftCfg.DftSrc = DFTSRC_SINC2NOTCH;
  lprtia_cal.DftCfg.HanWinEn = bTRUE;
  lprtia_cal.fFreq = AppDPVCfg.AdcClkFreq/4.0f/22.0f/2048.0f*3.0f;
  lprtia_cal.fRcal = AppDPVCfg.RcalVal;
  lprtia_cal.LpTiaRtia = AppDPVCfg.LPTIARtiaSel;
  lprtia_cal.LpAmpPwrMod = LPAMPPWR_NORM;
  lprtia_cal.bWithCtia = bFALSE;

  AD5940_LPRtiaCal(&lprtia_cal, &RtiaCalValue);
  AppDPVCfg.RtiaValue = RtiaCalValue;
  return AD5940ERR_OK;
}

/* ----------------------------- App Init --------------------------------- */
AD5940Err AppDPVInit(uint32_t *pBuffer, uint32_t BufferSize)
{
  AD5940Err error;
  FIFOCfg_Type fifo_cfg;
  SEQCfg_Type seq_cfg;

	
	if((AppDPVCfg.Efinal_mV > AppDPVCfg.Ein_mV && AppDPVCfg.StepIncrement_mV <= 0.0f) ||
   (AppDPVCfg.Efinal_mV < AppDPVCfg.Ein_mV && AppDPVCfg.StepIncrement_mV >= 0.0f))
{
  return AD5940ERR_PARA;
}
	
  if(AD5940_WakeUp(10) > 10) return AD5940ERR_WAKEUP;

  seq_cfg.SeqMemSize = SEQMEMSIZE_4KB;
  seq_cfg.SeqBreakEn = bFALSE;
  seq_cfg.SeqIgnoreEn = bFALSE;
  seq_cfg.SeqCntCRCClr = bTRUE;
  seq_cfg.SeqEnable = bFALSE;
  seq_cfg.SeqWrTimer = 0;
  AD5940_SEQCfg(&seq_cfg);

  if((AppDPVCfg.DPVInited == bFALSE) || (AppDPVCfg.bParaChanged == bTRUE))
  {
    if(!pBuffer || BufferSize == 0) return AD5940ERR_PARA;

    if(AppDPVCfg.LPTIARtiaSel == LPTIARTIA_OPEN)
    {
      AppDPVCfg.RtiaValue.Magnitude = AppDPVCfg.ExternalRtiaValue;
      AppDPVCfg.RtiaValue.Phase = 0.0f;
    }
    else
    {
      AppDPVRtiaCal();
    }

    AppDPVCfg.DPVInited = bFALSE;
    AD5940_SEQGenInit(pBuffer, BufferSize);

    error = AppDPVSeqInitGen();
    if(error != AD5940ERR_OK) return error;

    error = AppDPVSeqADCCtrlGen();
    if(error != AD5940ERR_OK) return error;

    AppDPVCfg.bParaChanged = bFALSE;
  }

  AD5940_FIFOCtrlS(FIFOSRC_SINC3, bFALSE);
  memset(&fifo_cfg, 0, sizeof(fifo_cfg));
  fifo_cfg.FIFOEn = bTRUE;
  fifo_cfg.FIFOSrc = FIFOSRC_SINC3;
  fifo_cfg.FIFOThresh = clamp_u32(AppDPVCfg.FifoThresh, 1U, 256U);
  fifo_cfg.FIFOMode = FIFOMODE_FIFO;
  fifo_cfg.FIFOSize = FIFOSIZE_2KB;
  AD5940_FIFOCfg(&fifo_cfg);

  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);

  AppDPVCfg.bFirstDACSeq = bTRUE;
  error = AppDPVSeqDACCtrlGen();
  if(error != AD5940ERR_OK) return error;

  AppDPVCfg.InitSeqInfo.WriteSRAM = bFALSE;
  AD5940_SEQInfoCfg(&AppDPVCfg.InitSeqInfo);

  AD5940_SEQCtrlS(bTRUE);
  AD5940_SEQMmrTrig(AppDPVCfg.InitSeqInfo.SeqId);
  while(AD5940_INTCTestFlag(AFEINTC_1, AFEINTSRC_ENDSEQ) == bFALSE);
  AD5940_INTCClrFlag(AFEINTSRC_ENDSEQ);

  AppDPVCfg.ADCSeqInfo.WriteSRAM = bFALSE;
  AD5940_SEQInfoCfg(&AppDPVCfg.ADCSeqInfo);

  AppDPVCfg.DACSeqInfo.WriteSRAM = bFALSE;
  AD5940_SEQInfoCfg(&AppDPVCfg.DACSeqInfo);

  AD5940_SEQCtrlS(bFALSE);
  AD5940_WriteReg(REG_AFE_SEQCNT, 0);
  AD5940_SEQCtrlS(bTRUE);

  AD5940_ClrMCUIntFlag();
  AD5940_AFEPwrBW(AFEPWR_LP, AFEBW_250KHZ);

  AppDPVCfg.DPVInited = bTRUE;
  AppDPVCfg.StopRequired = bFALSE;

  /* ADDED: reset last results */
  gDPV_LastPeak_uA = 0.0f;
  gDPV_LastConc_cTnI_pg_mL = 0.0f;
  gDPV_LastConc_BNP_pg_mL  = 0.0f;

  return AD5940ERR_OK;
}

/* ---------------------------- App Control ------------------------------- */
AD5940Err AppDPVCtrl(uint32_t Command, void *pPara)
{
  (void)pPara;

  switch(Command)
  {
    case APPCTRL_START:
    {
      WUPTCfg_Type wupt_cfg;
      if(AD5940_WakeUp(10) > 10) return AD5940ERR_WAKEUP;
      if(AppDPVCfg.DPVInited == bFALSE) return AD5940ERR_APPERROR;

      float tw_ms = AppDPVCfg.WaitTime_s  * 1000.0f;
      float tp_ms = AppDPVCfg.PulseTime_s * 1000.0f;

      float sd_base_ms  = AppDPVCfg.SampleDelayBase_ms;
      float sd_pulse_ms = AppDPVCfg.SampleDelayPulse_ms;

      if(sd_base_ms < 0.1f) sd_base_ms = 0.1f;
      if(sd_pulse_ms < 0.1f) sd_pulse_ms = 0.1f;

      if(sd_base_ms > (tw_ms - 0.2f)) sd_base_ms = tw_ms - 0.2f;
      if(sd_pulse_ms > (tp_ms - 0.2f)) sd_pulse_ms = tp_ms - 0.2f;

      uint32_t t_base_adc = (uint32_t)(AppDPVCfg.LFOSCClkFreq * sd_base_ms / 1000.0f);
      uint32_t t_base_dac = (uint32_t)(AppDPVCfg.LFOSCClkFreq * (tw_ms - sd_base_ms) / 1000.0f);

      uint32_t t_pulse_adc = (uint32_t)(AppDPVCfg.LFOSCClkFreq * sd_pulse_ms / 1000.0f);
      uint32_t t_pulse_dac = (uint32_t)(AppDPVCfg.LFOSCClkFreq * (tp_ms - sd_pulse_ms) / 1000.0f);

      if(t_base_adc < 10U) t_base_adc = 10U;
      if(t_base_dac < 10U) t_base_dac = 10U;
      if(t_pulse_adc < 10U) t_pulse_adc = 10U;
      if(t_pulse_dac < 10U) t_pulse_dac = 10U;

      memset(&wupt_cfg, 0, sizeof(wupt_cfg));
      wupt_cfg.WuptEn = bTRUE;
      wupt_cfg.WuptEndSeq = WUPTENDSEQ_D;

      wupt_cfg.WuptOrder[0] = SEQID_0; /* DAC baseline */
      wupt_cfg.WuptOrder[1] = SEQID_2; /* ADC baseline */
      wupt_cfg.WuptOrder[2] = SEQID_1; /* DAC pulse */
      wupt_cfg.WuptOrder[3] = SEQID_2; /* ADC pulse */

      wupt_cfg.SeqxSleepTime[SEQID_2]  = 4;

      wupt_cfg.SeqxSleepTime[SEQID_0]  = 4;
      wupt_cfg.SeqxWakeupTime[SEQID_0] = (t_base_dac > 6U) ? (t_base_dac - 6U) : 4U;

      wupt_cfg.SeqxSleepTime[SEQID_1]  = 4;
      wupt_cfg.SeqxWakeupTime[SEQID_1] = (t_pulse_dac > 6U) ? (t_pulse_dac - 6U) : 4U;

      wupt_cfg.SeqxWakeupTime[SEQID_2] = (t_base_adc > 6U) ? (t_base_adc - 6U) : 4U;

      AD5940_WUPTCfg(&wupt_cfg);
      break;
    }

    case APPCTRL_STOPNOW:
      if(AD5940_WakeUp(10) > 10) return AD5940ERR_WAKEUP;
      AD5940_WUPTCtrl(bFALSE);
      AD5940_WUPTCtrl(bFALSE);
      break;

    case APPCTRL_STOPSYNC:
      AppDPVCfg.StopRequired = bTRUE;
      break;

    case APPCTRL_SHUTDOWN:
      AppDPVCtrl(APPCTRL_STOPNOW, 0);
      AD5940_ShutDownS();
      break;

    default:
      break;
  }

  return AD5940ERR_OK;
}

/* --------------------------- Data processing ---------------------------- */
static int32_t AppDPVRawToCurrent_uA(int32_t * const pData, uint32_t *pDataCount)
{
  uint32_t i, datacount = *pDataCount;
  float *pOut = (float *)pData;

  for(i=0; i<datacount; i++)
  {
    pData[i] &= 0xFFFF;
    float v = AD5940_ADCCode2Volt(pData[i], AppDPVCfg.AdcPgaGain, AppDPVCfg.ADCRefVolt);
    pOut[i] = -v / AppDPVCfg.RtiaValue.Magnitude * 1e3f; /* uA */
  }
  return 0;
}

static uint32_t AppDPVDiffProcess(float *pI, uint32_t nSamples)
{
  uint32_t nPairs = nSamples / 2U;
  for(uint32_t k = 0; k < nPairs; k++)
  {
    float Ibase  = pI[2U*k + 0U];
    float Ipulse = pI[2U*k + 1U];
    pI[k] = (Ipulse - Ibase);
  }
  return nPairs;
}

static int32_t AppDPVRegModify(int32_t * const pData, uint32_t *pDataCount)
{
  (void)pData;
  (void)pDataCount;

  if(AppDPVCfg.StopRequired == bTRUE)
  {
    AD5940_WUPTCtrl(bFALSE);
    return AD5940ERR_OK;
  }
  return AD5940ERR_OK;
}

/* ------------------------------ ISR ------------------------------------- */
AD5940Err AppDPVISR(void *pBuff, uint32_t *pCount)
{
  uint32_t BuffCount = *pCount;
  uint32_t FifoCnt;
  uint32_t IntFlag;

  if(AD5940_WakeUp(10) > 10) return AD5940ERR_WAKEUP;

  AD5940_SleepKeyCtrlS(SLPKEY_LOCK);
  *pCount = 0;

  IntFlag = AD5940_INTCGetFlag(AFEINTC_0);

  if(IntFlag & AFEINTSRC_CUSTOMINT0)
  {
    AD5940_INTCClrFlag(AFEINTSRC_CUSTOMINT0);
    AD5940Err e = AppDPVSeqDACCtrlGen();
    AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);
    if(e != AD5940ERR_OK) return e;
  }

  if(IntFlag & AFEINTSRC_DATAFIFOTHRESH)
  {
    FifoCnt = AD5940_FIFOGetCnt();
    if(FifoCnt > BuffCount) FifoCnt = BuffCount;

    AD5940_FIFORd((uint32_t*)pBuff, FifoCnt);
    AD5940_INTCClrFlag(AFEINTSRC_DATAFIFOTHRESH);

    AppDPVRegModify((int32_t*)pBuff, &FifoCnt);
    AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);

    /* Convert to current (uA) */
    AppDPVRawToCurrent_uA((int32_t*)pBuff, &FifoCnt);

    /* Compute DPV difference per step */
    uint32_t nOut = AppDPVDiffProcess((float*)pBuff, FifoCnt);

    /* ADDED: peak + concentration conversion */
    {
      float peakAbs = DPV_FindPeakAbs_uA((float*)pBuff, nOut);
      float I_for_cal = peakAbs;
#if (DPV_CALC_USE_ABS_PEAK == 0)
      /* if you ever want signed peak, choose max positive instead */
      I_for_cal = peakAbs;
#endif
      gDPV_LastPeak_uA = I_for_cal;
      gDPV_LastConc_cTnI_pg_mL = DPV_ConcFromI_cTnI_pg_per_mL(I_for_cal);
      gDPV_LastConc_BNP_pg_mL  = DPV_ConcFromI_BNP_pg_per_mL(I_for_cal);
    }

    *pCount = nOut;
    return AD5940ERR_OK;
  }

  if(IntFlag & AFEINTSRC_ENDSEQ)
  {
    FifoCnt = AD5940_FIFOGetCnt();
    if(FifoCnt > BuffCount) FifoCnt = BuffCount;

    AD5940_INTCClrFlag(AFEINTSRC_ENDSEQ);
    AD5940_FIFORd((uint32_t*)pBuff, FifoCnt);

    AppDPVRawToCurrent_uA((int32_t*)pBuff, &FifoCnt);
    uint32_t nOut = AppDPVDiffProcess((float*)pBuff, FifoCnt);

    /* ADDED: peak + concentration conversion (final chunk) */
    {
      float peakAbs = DPV_FindPeakAbs_uA((float*)pBuff, nOut);
      float I_for_cal = peakAbs;
      gDPV_LastPeak_uA = I_for_cal;
      gDPV_LastConc_cTnI_pg_mL = DPV_ConcFromI_cTnI_pg_per_mL(I_for_cal);
      gDPV_LastConc_BNP_pg_mL  = DPV_ConcFromI_BNP_pg_per_mL(I_for_cal);
    }

    *pCount = nOut;

    AppDPVCtrl(APPCTRL_STOPNOW, 0);
    AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);
    return AD5940ERR_OK;
  }

  AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);
  return AD5940ERR_OK;
}
