/*!
 *****************************************************************************
 @file:    SqrWaveVoltammetry.c
 @brief:   Square Wave Voltammetry measurement sequences (corrected).
 *****************************************************************************/

#include "SqrWaveVoltammetry.h"
#include <string.h>

/* --------- Default configuration (safe, editable from AD5940Main.c) --------- */
static AppSWVCfg_Type AppSWVCfg =
{
  .bParaChanged = bFALSE,
  .SeqStartAddr = 0,
  .MaxSeqLen = 0,
  .SeqStartAddrCal = 0,
  .MaxSeqLenCal = 0,

  .LFOSCClkFreq = 32000.0f,
  .SysClkFreq   = 16000000.0f,
  .AdcClkFreq   = 16000000.0f,

  .RcalVal      = 200.0f,     /* ADuCM355 EVAL RCAL is typically 200Ω */
  .ADCRefVolt   = 1820.0f,

  /* Sweep: Vbias-Vzero from -100mV to +100mV by 5mV increments */
  .RampStartVolt      = -100.0f,
  .RampPeakVolt       = +100.0f,
  .VzeroStart         = 2200.0f,
  .VzeroPeak          = 400.0f,

  .SqrWvAmplitude     = 25.0f,
  .SqrWvRampIncrement = 5.0f,
  .Frequency          = 25.0f,

  .SampleDelay        = 1.0f,          /* ms */
  .LPTIARtiaSel       = LPTIARTIA_20K,
  .ExternalRtiaValue  = 20000.0f,
  .AdcPgaGain         = ADCPGA_1P5,
  .ADCSinc3Osr        = ADCSINC3OSR_4,

  .FifoThresh         = 4,             /* <-- KEEP SMALL */

  .SWVInited          = bFALSE,
  .bFirstDACSeq       = bTRUE,
  .bSqrWaveHiLevel    = bFALSE,
  .StopRequired       = bFALSE,
  .RampState          = SWV_STATE0,
};

/* ------------------------- Utility clamps/guards ------------------------- */
static uint32_t clamp_u32(uint32_t x, uint32_t lo, uint32_t hi)
{
  if(x < lo) return lo;
  if(x > hi) return hi;
  return x;
}

static float fabsf_safe(float x) { return (x < 0.0f) ? -x : x; }

/* Compute total number of DAC updates:
   For each ramp position we do 2 toggles (low/high). */
static uint32_t SWV_ComputeStepNumber(void)
{
  float span_mV = AppSWVCfg.RampPeakVolt - AppSWVCfg.RampStartVolt;
  float inc_mV  = AppSWVCfg.SqrWvRampIncrement;

  if(inc_mV <= 0.0f) inc_mV = 1.0f;
  span_mV = fabsf_safe(span_mV);

  /* number of ramp positions (inclusive) */
  uint32_t nRamp = (uint32_t)(span_mV / inc_mV) + 1U;
  if(nRamp < 1U) nRamp = 1U;

  /* 2 DAC updates per ramp position (square toggling) */
  return 2U * nRamp;
}

/* ------------------------- Public config getter -------------------------- */
AD5940Err AppSWVGetCfg(void *pCfg)
{
  if(!pCfg) return AD5940ERR_PARA;
  *(AppSWVCfg_Type**)pCfg = &AppSWVCfg;
  return AD5940ERR_OK;
}

/* ------------------------- Sequence generators -------------------------- */
static AD5940Err AppSWVSeqInitGen(void)
{
  AD5940Err error;
  const uint32_t *pSeqCmd;
  uint32_t SeqLen;

  AFERefCfg_Type aferef_cfg;
  LPLoopCfg_Type lploop_cfg;
  DSPCfg_Type dsp_cfg;

  AD5940_SEQGenCtrl(bTRUE);
  AD5940_AFECtrlS(AFECTRL_ALL, bFALSE);

  /* References */
  memset(&aferef_cfg, 0, sizeof(aferef_cfg));
  aferef_cfg.HpBandgapEn = bTRUE;
  aferef_cfg.Hp1V1BuffEn = bTRUE;
  aferef_cfg.Hp1V8BuffEn = bTRUE;
  aferef_cfg.Disc1V1Cap = bFALSE;
  aferef_cfg.Disc1V8Cap = bFALSE;
  aferef_cfg.Hp1V8ThemBuff = bFALSE;
  aferef_cfg.Hp1V8Ilimit = bFALSE;

  aferef_cfg.LpBandgapEn = bTRUE;
  aferef_cfg.LpRefBufEn = bTRUE;
  aferef_cfg.LpRefBoostEn = bFALSE;
  AD5940_REFCfgS(&aferef_cfg);

  /* LP loop + LPDAC */
  memset(&lploop_cfg, 0, sizeof(lploop_cfg));

  lploop_cfg.LpAmpCfg.LpAmpSel = LPAMP0;
  lploop_cfg.LpAmpCfg.LpAmpPwrMod = LPAMPPWR_NORM;
  lploop_cfg.LpAmpCfg.LpPaPwrEn = bTRUE;
  lploop_cfg.LpAmpCfg.LpTiaPwrEn = bTRUE;
  lploop_cfg.LpAmpCfg.LpTiaRf = LPTIARF_20K;
  lploop_cfg.LpAmpCfg.LpTiaRload = LPTIARLOAD_10R;
  lploop_cfg.LpAmpCfg.LpTiaRtia = AppSWVCfg.LPTIARtiaSel;

  if(AppSWVCfg.LPTIARtiaSel == LPTIARTIA_OPEN)
  {
    /* External RTIA: enable switches for external RTIA path */
    lploop_cfg.LpAmpCfg.LpTiaSW = LPTIASW(2)|LPTIASW(4)|LPTIASW(5)|LPTIASW(9);
  }
  else
  {
    /* 3-lead default (CE0/RE0/SE0) */
    lploop_cfg.LpAmpCfg.LpTiaSW = LPTIASW(2)|LPTIASW(4)|LPTIASW(5);
  }

  lploop_cfg.LpDacCfg.LpdacSel = LPDAC0;
  lploop_cfg.LpDacCfg.LpDacRef = LPDACREF_2P5;
  lploop_cfg.LpDacCfg.LpDacSrc = LPDACSRC_MMR;
  lploop_cfg.LpDacCfg.DataRst  = bFALSE;

  /* Route internally (don’t force to pin by default) */
  lploop_cfg.LpDacCfg.LpDacSW = LPDACSW_VBIAS2LPPA | LPDACSW_VZERO2LPTIA;

  /* Initial DAC codes */
  {
    uint32_t vzero6 = (uint32_t)((AppSWVCfg.VzeroStart - 200.0f)/DAC6BITVOLT_1LSB);
    float vbias12f  = (AppSWVCfg.RampStartVolt)/DAC12BITVOLT_1LSB + (float)vzero6*64.0f;

    uint32_t vbias12 = (vbias12f < 0.0f) ? 0U : (uint32_t)vbias12f;
    if(vbias12 > 4095U) vbias12 = 4095U;
    if(vzero6  > 63U)   vzero6  = 63U;

    lploop_cfg.LpDacCfg.DacData6Bit  = vzero6;
    lploop_cfg.LpDacCfg.DacData12Bit = vbias12;
  }

  lploop_cfg.LpDacCfg.PowerEn = bTRUE;
  AD5940_LPLoopCfgS(&lploop_cfg);

  /* DSP/ADC path */
  AD5940_StructInit(&dsp_cfg, sizeof(dsp_cfg));
  dsp_cfg.ADCBaseCfg.ADCMuxN = ADCMUXN_LPTIA0_N;
  dsp_cfg.ADCBaseCfg.ADCMuxP = ADCMUXP_LPTIA0_P;
  dsp_cfg.ADCBaseCfg.ADCPga  = AppSWVCfg.AdcPgaGain;

  dsp_cfg.ADCFilterCfg.ADCSinc3Osr = AppSWVCfg.ADCSinc3Osr;
  dsp_cfg.ADCFilterCfg.ADCRate = ADCRATE_800KHZ; /* 16MHz -> 800kHz path */
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
  if(SeqLen >= AppSWVCfg.MaxSeqLen) return AD5940ERR_SEQLEN;

  AD5940_StructInit(&AppSWVCfg.InitSeqInfo, sizeof(AppSWVCfg.InitSeqInfo));
  AppSWVCfg.InitSeqInfo.SeqId = SEQID_3;
  AppSWVCfg.InitSeqInfo.SeqRamAddr = AppSWVCfg.SeqStartAddr;
  AppSWVCfg.InitSeqInfo.pSeqCmd = pSeqCmd;
  AppSWVCfg.InitSeqInfo.SeqLen = SeqLen;
  AppSWVCfg.InitSeqInfo.WriteSRAM = bTRUE;
  AD5940_SEQInfoCfg(&AppSWVCfg.InitSeqInfo);

  return AD5940ERR_OK;
}

static AD5940Err AppSWVSeqADCCtrlGen(void)
{
  AD5940Err error;
  const uint32_t *pSeqCmd;
  uint32_t SeqLen;
  uint32_t WaitClks;
  ClksCalInfo_Type clks_cal;

  memset(&clks_cal, 0, sizeof(clks_cal));
  clks_cal.DataCount = 1;
  clks_cal.DataType = DATATYPE_SINC3;
  clks_cal.ADCSinc3Osr = AppSWVCfg.ADCSinc3Osr;
  clks_cal.ADCSinc2Osr = ADCSINC2OSR_1067;
  clks_cal.ADCAvgNum = ADCAVGNUM_2;
  clks_cal.RatioSys2AdcClk = AppSWVCfg.SysClkFreq / AppSWVCfg.AdcClkFreq;

  AD5940_ClksCalculate(&clks_cal, &WaitClks);

  AD5940_SEQGenCtrl(bTRUE);
  AD5940_AFECtrlS(AFECTRL_ADCPWR, bTRUE);
  AD5940_SEQGenInsert(SEQ_WAIT(16*250));          /* 250us ref settle */
  AD5940_AFECtrlS(AFECTRL_ADCCNV, bTRUE);
  AD5940_SEQGenInsert(SEQ_WAIT(WaitClks));
  AD5940_AFECtrlS(AFECTRL_ADCPWR|AFECTRL_ADCCNV, bFALSE);
  AD5940_EnterSleepS();
  error = AD5940_SEQGenFetchSeq(&pSeqCmd, &SeqLen);
  AD5940_SEQGenCtrl(bFALSE);

  if(error != AD5940ERR_OK) return error;
  if((SeqLen + AppSWVCfg.InitSeqInfo.SeqLen) >= AppSWVCfg.MaxSeqLen) return AD5940ERR_SEQLEN;

  AD5940_StructInit(&AppSWVCfg.ADCSeqInfo, sizeof(AppSWVCfg.ADCSeqInfo));
  AppSWVCfg.ADCSeqInfo.SeqId = SEQID_2;
  AppSWVCfg.ADCSeqInfo.SeqRamAddr = AppSWVCfg.InitSeqInfo.SeqRamAddr + AppSWVCfg.InitSeqInfo.SeqLen;
  AppSWVCfg.ADCSeqInfo.pSeqCmd = pSeqCmd;
  AppSWVCfg.ADCSeqInfo.SeqLen = SeqLen;
  AppSWVCfg.ADCSeqInfo.WriteSRAM = bTRUE;
  AD5940_SEQInfoCfg(&AppSWVCfg.ADCSeqInfo);

  return AD5940ERR_OK;
}

/* ------------------------- DAC stepping logic --------------------------- */
static AD5940Err SWV_DacRegUpdate(uint32_t *pDACData)
{
  uint32_t VbiasCode, VzeroCode;

  switch(AppSWVCfg.RampState)
  {
    case SWV_STATE0:
      AppSWVCfg.CurrVzeroCode = (uint32_t)((AppSWVCfg.VzeroStart - 200.0f)/DAC6BITVOLT_1LSB);
      AppSWVCfg.RampState = SWV_STATE1;
      break;

    case SWV_STATE1:
      /* optional mid transition, kept for compatibility */
      break;

    case SWV_STATE2:
      break;

    case SWV_STOP:
      break;
  }

  AppSWVCfg.CurrStepPos++;
  if(AppSWVCfg.CurrStepPos >= AppSWVCfg.StepNumber)
    AppSWVCfg.RampState = SWV_STOP;

  /* Toggle square wave and ramp */
  if(AppSWVCfg.bSqrWaveHiLevel)
  {
    AppSWVCfg.CurrRampCode -= (AppSWVCfg.DACCodePerStep - AppSWVCfg.DACCodePerRamp);
    AppSWVCfg.bSqrWaveHiLevel = bFALSE;
  }
  else
  {
    AppSWVCfg.CurrRampCode += AppSWVCfg.DACCodePerStep;
    AppSWVCfg.bSqrWaveHiLevel = bTRUE;
  }

  VzeroCode = AppSWVCfg.CurrVzeroCode;
  VbiasCode = (uint32_t)((float)VzeroCode*64.0f + AppSWVCfg.CurrRampCode);

  if(VbiasCode > 4095U) VbiasCode = 4095U;
  if(VzeroCode > 63U)   VzeroCode = 63U;

  *pDACData = (VzeroCode<<12) | (VbiasCode & 0x0FFF);
  return AD5940ERR_OK;
}

/* Generate/update DAC sequences in SRAM (ping-pong blocks) */
static AD5940Err AppSWVSeqDACCtrlGen(void)
{
  #define SEQLEN_ONESTEP 4U
  #define CURRBLK_BLK0   0U
  #define CURRBLK_BLK1   1U

  static BoolFlag  bCmdForSeq0 = bTRUE;
  static uint32_t  DACSeqBlk0Addr, DACSeqBlk1Addr;
  static uint32_t  StepsRemaining, StepsPerBlock, DACSeqCurrBlk;

  AD5940Err error = AD5940ERR_OK;
  uint32_t BlockStartSRAMAddr;
  uint32_t DACData, SRAMAddr;
  uint32_t i;
  uint32_t StepsThisBlock;
  BoolFlag bIsFinalBlk;
  uint32_t SeqCmdBuff[SEQLEN_ONESTEP];

  /* Compute steps safely */
  AppSWVCfg.StepNumber = SWV_ComputeStepNumber();

  /* DO NOT overwrite FifoThresh with StepNumber.
     User should choose a small threshold (4..64). */
  if(AppSWVCfg.FifoThresh < 1U) AppSWVCfg.FifoThresh = 4U;

  if(AppSWVCfg.bFirstDACSeq == bTRUE)
  {
    int32_t DACSeqLenMax;

    StepsRemaining = AppSWVCfg.StepNumber;

    DACSeqLenMax = (int32_t)AppSWVCfg.MaxSeqLen
                 - (int32_t)AppSWVCfg.InitSeqInfo.SeqLen
                 - (int32_t)AppSWVCfg.ADCSeqInfo.SeqLen;

    if(DACSeqLenMax < (int32_t)(SEQLEN_ONESTEP*8U))
      return AD5940ERR_SEQLEN;

    /* Reserve some space for the block-switching and final stop */
    DACSeqLenMax -= (int32_t)(SEQLEN_ONESTEP*2U);

    StepsPerBlock = (uint32_t)(DACSeqLenMax/((int32_t)SEQLEN_ONESTEP)/2);
    if(StepsPerBlock < 8U) StepsPerBlock = 8U;

    DACSeqBlk0Addr = AppSWVCfg.ADCSeqInfo.SeqRamAddr + AppSWVCfg.ADCSeqInfo.SeqLen;
    DACSeqBlk1Addr = DACSeqBlk0Addr + StepsPerBlock*SEQLEN_ONESTEP;
    DACSeqCurrBlk  = CURRBLK_BLK0;

    /* Analog calculations */
    AppSWVCfg.DACCodePerStep = AppSWVCfg.SqrWvAmplitude / DAC12BITVOLT_1LSB;
    AppSWVCfg.DACCodePerRamp = AppSWVCfg.SqrWvRampIncrement / DAC12BITVOLT_1LSB;

#if ALIGN_VOLT2LSB
    AppSWVCfg.DACCodePerStep = (float)((int32_t)AppSWVCfg.DACCodePerStep);
    AppSWVCfg.DACCodePerRamp = (float)((int32_t)AppSWVCfg.DACCodePerRamp);
#endif

    AppSWVCfg.CurrRampCode = AppSWVCfg.RampStartVolt / DAC12BITVOLT_1LSB;
    AppSWVCfg.RampState    = SWV_STATE0;
    AppSWVCfg.CurrStepPos  = 0;
    AppSWVCfg.bSqrWaveHiLevel = bFALSE;

    bCmdForSeq0 = bTRUE;
  }

  if(StepsRemaining == 0U) return AD5940ERR_OK;

  bIsFinalBlk = (StepsRemaining <= StepsPerBlock) ? bTRUE : bFALSE;
  StepsThisBlock = bIsFinalBlk ? StepsRemaining : StepsPerBlock;
  StepsRemaining -= StepsThisBlock;

  BlockStartSRAMAddr = (DACSeqCurrBlk == CURRBLK_BLK0) ? DACSeqBlk0Addr : DACSeqBlk1Addr;
  SRAMAddr = BlockStartSRAMAddr;

  for(i=0; i<StepsThisBlock-1U; i++)
  {
    uint32_t CurrAddr = SRAMAddr;
    SRAMAddr += SEQLEN_ONESTEP;

    SWV_DacRegUpdate(&DACData);
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

    SWV_DacRegUpdate(&DACData);
    SeqCmdBuff[0] = SEQ_WR(REG_AFE_LPDACDAT0, DACData);
    SeqCmdBuff[1] = SEQ_WAIT(10);
    SeqCmdBuff[2] = SEQ_WR(bCmdForSeq0 ? REG_AFE_SEQ1INFO : REG_AFE_SEQ0INFO,
                           (SRAMAddr<<BITP_AFE_SEQ1INFO_ADDR) | (SEQLEN_ONESTEP<<BITP_AFE_SEQ1INFO_LEN));
    SeqCmdBuff[3] = SEQ_SLP();
    AD5940_SEQCmdWrite(CurrAddr, SeqCmdBuff, SEQLEN_ONESTEP);

    CurrAddr += SEQLEN_ONESTEP;

    /* Stop sequencer */
    SeqCmdBuff[0] = SEQ_NOP();
    SeqCmdBuff[1] = SEQ_NOP();
    SeqCmdBuff[2] = SEQ_NOP();
    SeqCmdBuff[3] = SEQ_STOP();
    AD5940_SEQCmdWrite(CurrAddr, SeqCmdBuff, SEQLEN_ONESTEP);
  }
  else
  {
    /* Jump to next block and request MCU refill using INT0 */
    uint32_t CurrAddr = SRAMAddr;
    SRAMAddr = (DACSeqCurrBlk == CURRBLK_BLK0) ? DACSeqBlk1Addr : DACSeqBlk0Addr;

    SWV_DacRegUpdate(&DACData);
    SeqCmdBuff[0] = SEQ_WR(REG_AFE_LPDACDAT0, DACData);
    SeqCmdBuff[1] = SEQ_WAIT(10);
    SeqCmdBuff[2] = SEQ_WR(bCmdForSeq0 ? REG_AFE_SEQ1INFO : REG_AFE_SEQ0INFO,
                           (SRAMAddr<<BITP_AFE_SEQ1INFO_ADDR) | (SEQLEN_ONESTEP<<BITP_AFE_SEQ1INFO_LEN));
    SeqCmdBuff[3] = SEQ_INT0();
    AD5940_SEQCmdWrite(CurrAddr, SeqCmdBuff, SEQLEN_ONESTEP);

    bCmdForSeq0 = bCmdForSeq0 ? bFALSE : bTRUE;
  }

  DACSeqCurrBlk = (DACSeqCurrBlk == CURRBLK_BLK0) ? CURRBLK_BLK1 : CURRBLK_BLK0;

  if(AppSWVCfg.bFirstDACSeq)
  {
    AppSWVCfg.bFirstDACSeq = bFALSE;

    if(!bIsFinalBlk)
    {
      error = AppSWVSeqDACCtrlGen();
      if(error != AD5940ERR_OK) return error;
    }

    AppSWVCfg.DACSeqInfo.SeqId = SEQID_0;
    AppSWVCfg.DACSeqInfo.SeqLen = SEQLEN_ONESTEP;
    AppSWVCfg.DACSeqInfo.SeqRamAddr = BlockStartSRAMAddr;
    AppSWVCfg.DACSeqInfo.WriteSRAM = bFALSE;
    AD5940_SEQInfoCfg(&AppSWVCfg.DACSeqInfo);
  }

  return AD5940ERR_OK;
}

/* ------------------------------ RTIA Cal -------------------------------- */
static AD5940Err AppSWVRtiaCal(void)
{
  fImpPol_Type RtiaCalValue;
  LPRTIACal_Type lprtia_cal;

  AD5940_StructInit(&lprtia_cal, sizeof(lprtia_cal));
  lprtia_cal.LpAmpSel = LPAMP0;
  lprtia_cal.bPolarResult = bTRUE;
  lprtia_cal.AdcClkFreq = AppSWVCfg.AdcClkFreq;
  lprtia_cal.SysClkFreq = AppSWVCfg.SysClkFreq;
  lprtia_cal.ADCSinc3Osr = ADCSINC3OSR_4;
  lprtia_cal.ADCSinc2Osr = ADCSINC2OSR_22;
  lprtia_cal.DftCfg.DftNum = DFTNUM_2048;
  lprtia_cal.DftCfg.DftSrc = DFTSRC_SINC2NOTCH;
  lprtia_cal.DftCfg.HanWinEn = bTRUE;
  lprtia_cal.fFreq = AppSWVCfg.AdcClkFreq/4.0f/22.0f/2048.0f*3.0f;
  lprtia_cal.fRcal = AppSWVCfg.RcalVal;
  lprtia_cal.LpTiaRtia = AppSWVCfg.LPTIARtiaSel;
  lprtia_cal.LpAmpPwrMod = LPAMPPWR_NORM;
  lprtia_cal.bWithCtia = bFALSE;

  AD5940_LPRtiaCal(&lprtia_cal, &RtiaCalValue);
  AppSWVCfg.RtiaValue = RtiaCalValue;
  return AD5940ERR_OK;
}

/* ----------------------------- App Init --------------------------------- */
AD5940Err AppSWVInit(uint32_t *pBuffer, uint32_t BufferSize)
{
  AD5940Err error;
  FIFOCfg_Type fifo_cfg;
  SEQCfg_Type seq_cfg;

  if(AD5940_WakeUp(10) > 10) return AD5940ERR_WAKEUP;

  /* Sequencer memory: assume platform sets 4kB */
  seq_cfg.SeqMemSize = SEQMEMSIZE_4KB;
  seq_cfg.SeqBreakEn = bFALSE;
  seq_cfg.SeqIgnoreEn = bFALSE;
  seq_cfg.SeqCntCRCClr = bTRUE;
  seq_cfg.SeqEnable = bFALSE;
  seq_cfg.SeqWrTimer = 0;
  AD5940_SEQCfg(&seq_cfg);

  if((AppSWVCfg.SWVInited == bFALSE) || (AppSWVCfg.bParaChanged == bTRUE))
  {
    if(!pBuffer || BufferSize == 0) return AD5940ERR_PARA;

    if(AppSWVCfg.LPTIARtiaSel == LPTIARTIA_OPEN)
    {
      AppSWVCfg.RtiaValue.Magnitude = AppSWVCfg.ExternalRtiaValue;
      AppSWVCfg.RtiaValue.Phase = 0.0f;
    }
    else
    {
      AppSWVRtiaCal();
    }

    AppSWVCfg.SWVInited = bFALSE;
    AD5940_SEQGenInit(pBuffer, BufferSize);

    error = AppSWVSeqInitGen();
    if(error != AD5940ERR_OK) return error;

    error = AppSWVSeqADCCtrlGen();
    if(error != AD5940ERR_OK) return error;

    AppSWVCfg.bParaChanged = bFALSE;
  }

  /* FIFO setup */
  AD5940_FIFOCtrlS(FIFOSRC_SINC3, bFALSE);
  memset(&fifo_cfg, 0, sizeof(fifo_cfg));
  fifo_cfg.FIFOEn = bTRUE;
  fifo_cfg.FIFOSrc = FIFOSRC_SINC3;
  fifo_cfg.FIFOThresh = clamp_u32(AppSWVCfg.FifoThresh, 1U, 256U); /* keep reasonable */
  fifo_cfg.FIFOMode = FIFOMODE_FIFO;
  fifo_cfg.FIFOSize = FIFOSIZE_2KB;
  AD5940_FIFOCfg(&fifo_cfg);

  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);

  /* Generate DAC sequences */
  AppSWVCfg.bFirstDACSeq = bTRUE;
  error = AppSWVSeqDACCtrlGen();
  if(error != AD5940ERR_OK) return error;

  /* Run init seq once */
  AppSWVCfg.InitSeqInfo.WriteSRAM = bFALSE;
  AD5940_SEQInfoCfg(&AppSWVCfg.InitSeqInfo);

  AD5940_SEQCtrlS(bTRUE);
  AD5940_SEQMmrTrig(AppSWVCfg.InitSeqInfo.SeqId);
  while(AD5940_INTCTestFlag(AFEINTC_1, AFEINTSRC_ENDSEQ) == bFALSE);
  AD5940_INTCClrFlag(AFEINTSRC_ENDSEQ);

  /* Configure ADC + DAC seq info */
  AppSWVCfg.ADCSeqInfo.WriteSRAM = bFALSE;
  AD5940_SEQInfoCfg(&AppSWVCfg.ADCSeqInfo);

  AppSWVCfg.DACSeqInfo.WriteSRAM = bFALSE;
  AD5940_SEQInfoCfg(&AppSWVCfg.DACSeqInfo);

  /* Arm sequencer */
  AD5940_SEQCtrlS(bFALSE);
  AD5940_WriteReg(REG_AFE_SEQCNT, 0);
  AD5940_SEQCtrlS(bTRUE);

  AD5940_ClrMCUIntFlag();
  AD5940_AFEPwrBW(AFEPWR_LP, AFEBW_250KHZ);

  AppSWVCfg.SWVInited = bTRUE;
  AppSWVCfg.StopRequired = bFALSE;
  AppSWVCfg.RampState = SWV_STATE0;

  return AD5940ERR_OK;
}

/* ---------------------------- App Control ------------------------------- */
AD5940Err AppSWVCtrl(uint32_t Command, void *pPara)
{
  (void)pPara;

  switch(Command)
  {
    case APPCTRL_START:
    {
      WUPTCfg_Type wupt_cfg;
      if(AD5940_WakeUp(10) > 10) return AD5940ERR_WAKEUP;
      if(AppSWVCfg.SWVInited == bFALSE) return AD5940ERR_APPERROR;
      if(AppSWVCfg.RampState == SWV_STOP) return AD5940ERR_APPERROR;

      /* Compute half-period (ms) */
      float halfPeriod_ms = (AppSWVCfg.Frequency <= 0.1f) ? 5000.0f : (500.0f / AppSWVCfg.Frequency);
      float sd_ms = AppSWVCfg.SampleDelay;
      if(sd_ms < 0.1f) sd_ms = 0.1f;
      if(sd_ms > (halfPeriod_ms - 0.2f)) sd_ms = halfPeriod_ms - 0.2f; /* ensure room */

      uint32_t t_adc = (uint32_t)(AppSWVCfg.LFOSCClkFreq * sd_ms / 1000.0f);
      uint32_t t_dac = (uint32_t)(AppSWVCfg.LFOSCClkFreq * (halfPeriod_ms - sd_ms) / 1000.0f);

      /* Guard underflow of “-4-2” style adjustments */
      if(t_adc < 10U) t_adc = 10U;
      if(t_dac < 10U) t_dac = 10U;

      memset(&wupt_cfg, 0, sizeof(wupt_cfg));
      wupt_cfg.WuptEn = bTRUE;
      wupt_cfg.WuptEndSeq = WUPTENDSEQ_D;
      wupt_cfg.WuptOrder[0] = SEQID_0;
      wupt_cfg.WuptOrder[1] = SEQID_2;
      wupt_cfg.WuptOrder[2] = SEQID_1;
      wupt_cfg.WuptOrder[3] = SEQID_2;

      wupt_cfg.SeqxSleepTime[SEQID_2]  = 4;
      wupt_cfg.SeqxWakeupTime[SEQID_2] = (t_adc > 6U) ? (t_adc - 6U) : 4U;

      wupt_cfg.SeqxSleepTime[SEQID_0]  = 4;
      wupt_cfg.SeqxWakeupTime[SEQID_0] = (t_dac > 6U) ? (t_dac - 6U) : 4U;

      wupt_cfg.SeqxSleepTime[SEQID_1]  = wupt_cfg.SeqxSleepTime[SEQID_0];
      wupt_cfg.SeqxWakeupTime[SEQID_1] = wupt_cfg.SeqxWakeupTime[SEQID_0];

      AD5940_WUPTCfg(&wupt_cfg);
      break;
    }

    case APPCTRL_STOPNOW:
      if(AD5940_WakeUp(10) > 10) return AD5940ERR_WAKEUP;
      AD5940_WUPTCtrl(bFALSE);
      AD5940_WUPTCtrl(bFALSE);
      break;

    case APPCTRL_STOPSYNC:
      AppSWVCfg.StopRequired = bTRUE;
      break;

    case APPCTRL_SHUTDOWN:
      AppSWVCtrl(APPCTRL_STOPNOW, 0);
      AD5940_ShutDownS();
      break;

    default:
      break;
  }

  return AD5940ERR_OK;
}

/* --------------------------- Data processing ---------------------------- */
static int32_t AppSWVDataProcess(int32_t * const pData, uint32_t *pDataCount)
{
  uint32_t i, datacount = *pDataCount;
  float *pOut = (float *)pData;

  for(i=0; i<datacount; i++)
  {
    pData[i] &= 0xFFFF;
    float v = AD5940_ADCCode2Volt(pData[i], AppSWVCfg.AdcPgaGain, AppSWVCfg.ADCRefVolt);
    pOut[i] = -v / AppSWVCfg.RtiaValue.Magnitude * 1e3f; /* uA */
  }
  return 0;
}

static int32_t AppSWVRegModify(int32_t * const pData, uint32_t *pDataCount)
{
  (void)pData;
  (void)pDataCount;

  if(AppSWVCfg.StopRequired == bTRUE)
  {
    AD5940_WUPTCtrl(bFALSE);
    return AD5940ERR_OK;
  }
  return AD5940ERR_OK;
}

/* ------------------------------ ISR ------------------------------------- */
AD5940Err AppSWVISR(void *pBuff, uint32_t *pCount)
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
    AD5940Err e = AppSWVSeqDACCtrlGen();
    AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);
    if(e != AD5940ERR_OK) return e;
  }

  if(IntFlag & AFEINTSRC_DATAFIFOTHRESH)
  {
    FifoCnt = AD5940_FIFOGetCnt();

    if(FifoCnt > BuffCount) FifoCnt = BuffCount; /* simple guard */

    AD5940_FIFORd((uint32_t*)pBuff, FifoCnt);
    AD5940_INTCClrFlag(AFEINTSRC_DATAFIFOTHRESH);

    AppSWVRegModify((int32_t*)pBuff, &FifoCnt);
    AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);

    AppSWVDataProcess((int32_t*)pBuff, &FifoCnt);
    *pCount = FifoCnt;
    return AD5940ERR_OK;
  }

  if(IntFlag & AFEINTSRC_ENDSEQ)
  {
    FifoCnt = AD5940_FIFOGetCnt();
    if(FifoCnt > BuffCount) FifoCnt = BuffCount;

    AD5940_INTCClrFlag(AFEINTSRC_ENDSEQ);
    AD5940_FIFORd((uint32_t*)pBuff, FifoCnt);

    AppSWVDataProcess((int32_t*)pBuff, &FifoCnt);
    *pCount = FifoCnt;

    AppSWVCtrl(APPCTRL_STOPNOW, 0);
    AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);
    return AD5940ERR_OK;
  }

  AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);
  return AD5940ERR_OK;
}
