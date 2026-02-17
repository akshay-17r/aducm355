/* ========================= DiffPulseVoltammetry.c ========================= */
#include "ad5940.h"
#include <stdio.h>
#include "string.h"
#include "math.h"
#include "DiffPulseVoltammetry.h"

/* Default configuration */
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
  .RcalVal      = 10000.0f,
  .ADCRefVolt   = 1820.0f,

  .RampStartVolt = 0.0f,
  .RampPeakVolt  = 600.0f,
  .VzeroStart    = 1300.0f,
  .VzeroPeak     = 1300.0f,
  .StaircaseInc  = 5.0f,
  .StepNumber    = 0,

  .DPV_WaitTime     = 0.5f,
  .DPV_PulseTime    = 0.05f,
  .DPV_PulseHeight  = 50.0f,

  .SampleDelay   = 10.0f,
  .LPTIARtiaSel   = LPTIARTIA_8K,
  .ExternalRtiaValue = 20000.0f,
  .AdcPgaGain     = ADCPGA_1P5,
  .ADCSinc3Osr    = ADCSINC3OSR_4,

  .FifoThresh     = 4,

  .DPVInited      = bFALSE,
  .StopRequired   = bFALSE,
  .RampState      = DPV_STATE0,
  .bFirstDACSeq   = bTRUE,

  .CurrStepPos    = 0,
  .DACCodePerStair = 0.0f,
  .CurrBaseCode   = 0.0f,
  .CurrVzeroCode  = 0,

  .bNextIsSeq0    = bTRUE,
  .bPulsePhase    = bFALSE,
};

AD5940Err AppDPVGetCfg(void *pCfg)
{
  if(pCfg)
  {
    *(AppDPVCfg_Type**)pCfg = &AppDPVCfg;
    return AD5940ERR_OK;
  }
  return AD5940ERR_PARA;
}

/* ---------- Initialization sequence generator ---------- */
static AD5940Err AppDPVSeqInitGen(void)
{
  AD5940Err error = AD5940ERR_OK;
  const uint32_t *pSeqCmd;
  uint32_t SeqLen;

  AFERefCfg_Type aferef_cfg;
  LPLoopCfg_Type lploop_cfg;
  DSPCfg_Type dsp_cfg;

  AD5940_SEQGenCtrl(bTRUE);
  AD5940_AFECtrlS(AFECTRL_ALL, bFALSE);

  aferef_cfg.HpBandgapEn = bTRUE;
  aferef_cfg.Hp1V1BuffEn = bTRUE;
  aferef_cfg.Hp1V8BuffEn = bTRUE;
  aferef_cfg.Disc1V1Cap = bFALSE;
  aferef_cfg.Disc1V8Cap = bFALSE;
  aferef_cfg.Hp1V8ThemBuff = bFALSE;
  aferef_cfg.Hp1V8Ilimit = bFALSE;
  aferef_cfg.Lp1V1BuffEn = bFALSE;
  aferef_cfg.Lp1V8BuffEn = bFALSE;
  aferef_cfg.LpBandgapEn = bTRUE;
  aferef_cfg.LpRefBufEn = bTRUE;
  aferef_cfg.LpRefBoostEn = bFALSE;
  AD5940_REFCfgS(&aferef_cfg);

  /* LP loop config */
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
  lploop_cfg.LpDacCfg.DacData12Bit = 0x800;
  lploop_cfg.LpDacCfg.DacData6Bit = 0;
  lploop_cfg.LpDacCfg.DataRst = bFALSE;
  lploop_cfg.LpDacCfg.LpDacSW = LPDACSW_VBIAS2LPPA|LPDACSW_VZERO2LPTIA;
  lploop_cfg.LpDacCfg.LpDacRef = LPDACREF_2P5;
  lploop_cfg.LpDacCfg.LpDacSrc = LPDACSRC_MMR;

  /* Initialize VZERO and initial VBIAS at Ein */
  lploop_cfg.LpDacCfg.DacData6Bit = (uint32_t)((AppDPVCfg.VzeroStart - 200.0f)/DAC6BITVOLT_1LSB);
  lploop_cfg.LpDacCfg.DacData12Bit = (int32_t)(AppDPVCfg.RampStartVolt/DAC12BITVOLT_1LSB) + lploop_cfg.LpDacCfg.DacData6Bit*64;
  lploop_cfg.LpDacCfg.PowerEn = bTRUE;

  AD5940_LPLoopCfgS(&lploop_cfg);

  /* DSP config (SINC3) */
  dsp_cfg.ADCBaseCfg.ADCMuxP = ADCMUXP_VCE0;
  dsp_cfg.ADCBaseCfg.ADCMuxN = ADCMUXN_VSET0;
  dsp_cfg.ADCBaseCfg.ADCPga = AppDPVCfg.AdcPgaGain;

  dsp_cfg.ADCFilterCfg.ADCSinc3Osr = AppDPVCfg.ADCSinc3Osr;
  dsp_cfg.ADCFilterCfg.ADCSinc2Osr = ADCSINC2OSR_1067;
  dsp_cfg.ADCFilterCfg.ADCAvgNum = ADCAVGNUM_2;
  dsp_cfg.ADCFilterCfg.BpSinc3 = bFALSE;
  dsp_cfg.ADCFilterCfg.BpNotch = bFALSE;
  dsp_cfg.ADCFilterCfg.Sinc2NotchEnable = bTRUE;

  dsp_cfg.DftCfg.DftNum = DFTNUM_16384;
  dsp_cfg.DftCfg.DftSrc = DFTSRC_SINC3;
  dsp_cfg.DftCfg.HanWinEn = bFALSE;

  AD5940_DSPCfgS(&dsp_cfg);

  AD5940_AFECtrlS(AFECTRL_HPREFPWR|AFECTRL_HPTIAPWR|AFECTRL_LPREFPWR|AFECTRL_LPTIAPWR, bTRUE);

  AD5940_EnterSleepS();

  error = AD5940_SEQGenFetchSeq(&pSeqCmd, &SeqLen);
  AD5940_SEQGenCtrl(bFALSE);

  if(error != AD5940ERR_OK) return error;

  AD5940_StructInit(&AppDPVCfg.InitSeqInfo, sizeof(AppDPVCfg.InitSeqInfo));
  if(SeqLen >= AppDPVCfg.MaxSeqLen) return AD5940ERR_SEQLEN;

  AppDPVCfg.InitSeqInfo.SeqId = SEQID_3;
  AppDPVCfg.InitSeqInfo.SeqRamAddr = AppDPVCfg.SeqStartAddr;
  AppDPVCfg.InitSeqInfo.pSeqCmd = pSeqCmd;
  AppDPVCfg.InitSeqInfo.SeqLen = SeqLen;
  AppDPVCfg.InitSeqInfo.WriteSRAM = bTRUE;
  AD5940_SEQInfoCfg(&AppDPVCfg.InitSeqInfo);

  return AD5940ERR_OK;
}

/* ---------- ADC sequence generator (SEQ2) ---------- */
static AD5940Err AppDPVSeqADCCtrlGen(void)
{
  AD5940Err error = AD5940ERR_OK;
  const uint32_t *pSeqCmd;
  uint32_t SeqLen;

  uint32_t WaitClks;
  ClksCalInfo_Type clks_cal;

  clks_cal.DataCount = 1;
  clks_cal.DataType = DATATYPE_SINC3;
  clks_cal.ADCSinc3Osr = AppDPVCfg.ADCSinc3Osr;
  clks_cal.ADCSinc2Osr = ADCSINC2OSR_1067;
  clks_cal.ADCAvgNum = ADCAVGNUM_2;
  clks_cal.RatioSys2AdcClk = AppDPVCfg.SysClkFreq/AppDPVCfg.AdcClkFreq;
  AD5940_ClksCalculate(&clks_cal, &WaitClks);

  AD5940_SEQGenCtrl(bTRUE);
  AD5940_SEQGpioCtrlS(AGPIO_Pin2);
  AD5940_AFECtrlS(AFECTRL_ADCPWR, bTRUE);
  AD5940_SEQGenInsert(SEQ_WAIT(16*250));     /* 250us ref settle */
  AD5940_AFECtrlS(AFECTRL_ADCCNV, bTRUE);
  AD5940_SEQGenInsert(SEQ_WAIT(WaitClks));
  AD5940_AFECtrlS(AFECTRL_ADCPWR|AFECTRL_ADCCNV, bFALSE);
  AD5940_SEQGpioCtrlS(0);
  AD5940_EnterSleepS();

  error = AD5940_SEQGenFetchSeq(&pSeqCmd, &SeqLen);
  AD5940_SEQGenCtrl(bFALSE);
  if(error != AD5940ERR_OK) return error;

  AD5940_StructInit(&AppDPVCfg.ADCSeqInfo, sizeof(AppDPVCfg.ADCSeqInfo));
  if((SeqLen + AppDPVCfg.InitSeqInfo.SeqLen) >= AppDPVCfg.MaxSeqLen) return AD5940ERR_SEQLEN;

  AppDPVCfg.ADCSeqInfo.SeqId = SEQID_2;
  AppDPVCfg.ADCSeqInfo.SeqRamAddr = AppDPVCfg.InitSeqInfo.SeqRamAddr + AppDPVCfg.InitSeqInfo.SeqLen;
  AppDPVCfg.ADCSeqInfo.pSeqCmd = pSeqCmd;
  AppDPVCfg.ADCSeqInfo.SeqLen = SeqLen;
  AppDPVCfg.ADCSeqInfo.WriteSRAM = bTRUE;
  AD5940_SEQInfoCfg(&AppDPVCfg.ADCSeqInfo);

  return AD5940ERR_OK;
}

/* ---------- DPV DAC code computation ---------- */
static AD5940Err DPVDacRegUpdate(uint32_t *pDACData, BoolFlag pulsePhase)
{
  uint32_t VzeroCode, VbiasCode;
  float Ebase_mV;

  switch(AppDPVCfg.RampState)
  {
    case DPV_STATE0:
      AppDPVCfg.CurrVzeroCode = (uint32_t)((AppDPVCfg.VzeroStart - 200.0f)/DAC6BITVOLT_1LSB);
      AppDPVCfg.CurrStepPos = 0;
      AppDPVCfg.RampState = DPV_STATE1;
      break;

    case DPV_STATE1:
      if(AppDPVCfg.CurrStepPos >= AppDPVCfg.StepNumber/2)
      {
        AppDPVCfg.RampState = DPV_STATE2;
        AppDPVCfg.CurrVzeroCode = (uint32_t)((AppDPVCfg.VzeroPeak - 200.0f)/DAC6BITVOLT_1LSB);
      }
      break;

    case DPV_STATE2:
      if(AppDPVCfg.CurrStepPos >= AppDPVCfg.StepNumber)
        AppDPVCfg.RampState = DPV_STOP;
      break;

    case DPV_STOP:
      break;
  }

  if(AppDPVCfg.RampState == DPV_STOP)
    return AD5940ERR_OK;

  /* Staircase position only advances on baseline phase */
  if(pulsePhase == bFALSE)
  {
    Ebase_mV = AppDPVCfg.RampStartVolt + (float)AppDPVCfg.CurrStepPos * AppDPVCfg.StaircaseInc;
    AppDPVCfg.CurrBaseCode = Ebase_mV / DAC12BITVOLT_1LSB;
    AppDPVCfg.CurrStepPos++;
  }

  /* Compute VBIA S code */
  VzeroCode = AppDPVCfg.CurrVzeroCode;

  float pulseCode = (AppDPVCfg.DPV_PulseHeight / DAC12BITVOLT_1LSB);
  float vbias12 = AppDPVCfg.CurrBaseCode + (pulsePhase ? pulseCode : 0.0f);

  VbiasCode = (uint32_t)(VzeroCode*64 + vbias12);

  if(VbiasCode < (VzeroCode*64)) VbiasCode--;
  if(VbiasCode > 4095) VbiasCode = 4095;
  if(VzeroCode > 63)   VzeroCode = 63;

  *pDACData = (VzeroCode<<12) | VbiasCode;
  return AD5940ERR_OK;
}

/* ---------- Generate / update DAC sequences (SEQ0 baseline, SEQ1 pulse) ---------- */
static AD5940Err AppDPVSeqDACCtrlGen(void)
{
  #define SEQLEN_ONESTEP    4L
  #define CURRBLK_BLK0      0
  #define CURRBLK_BLK1      1

  AD5940Err error = AD5940ERR_OK;
  uint32_t BlockStartSRAMAddr;
  uint32_t DACData, SRAMAddr;
  uint32_t i;
  uint32_t StepsThisBlock;
  BoolFlag bIsFinalBlk;
  uint32_t SeqCmdBuff[SEQLEN_ONESTEP];

  static uint32_t CurrBlock = CURRBLK_BLK0;
  static uint32_t Block0Addr = 0;
  static uint32_t Block1Addr = 0;
  static uint32_t StepsPerBlock = 0;

  /* Determine which sequencer we are updating: SEQ0 baseline or SEQ1 pulse */
  BoolFlag genSeq0 = AppDPVCfg.bNextIsSeq0;

  if(AppDPVCfg.bFirstDACSeq)
  {
    AppDPVCfg.bFirstDACSeq = bFALSE;
    CurrBlock = CURRBLK_BLK0;

    /* Leave space for Init + ADC sequences */
    Block0Addr = AppDPVCfg.ADCSeqInfo.SeqRamAddr + AppDPVCfg.ADCSeqInfo.SeqLen;
    Block1Addr = Block0Addr + (AppDPVCfg.MaxSeqLen - (Block0Addr - AppDPVCfg.SeqStartAddr))/2;

    /* 4 commands per step */
    StepsPerBlock = (Block1Addr - Block0Addr)/SEQLEN_ONESTEP;
    if(StepsPerBlock == 0) return AD5940ERR_SEQLEN;
  }

  BlockStartSRAMAddr = (CurrBlock == CURRBLK_BLK0) ? Block0Addr : Block1Addr;
  SRAMAddr = BlockStartSRAMAddr;

  /* How many steps to generate in this block */
  uint32_t stepsLeft = AppDPVCfg.StepNumber - (AppDPVCfg.CurrStepPos);
  StepsThisBlock = (stepsLeft > StepsPerBlock) ? StepsPerBlock : stepsLeft;
  bIsFinalBlk = (stepsLeft <= StepsPerBlock) ? bTRUE : bFALSE;

  for(i=0; i<StepsThisBlock; i++)
  {
    /* For SEQ0 generate baseline, for SEQ1 generate pulse */
    DPVDacRegUpdate(&DACData, genSeq0 ? bFALSE : bTRUE);

    SeqCmdBuff[0] = SEQ_WR(REG_AFE_LPDACDAT0, DACData);
    SeqCmdBuff[1] = SEQ_WAIT(10);

    if(i == (StepsThisBlock-1))
    {
      if(bIsFinalBlk)
      {
        SeqCmdBuff[2] = SEQ_WR(REG_AFE_SEQ0INFO, (BlockStartSRAMAddr<<BITP_AFE_SEQ0INFO_ADDR) | (SEQLEN_ONESTEP<<BITP_AFE_SEQ0INFO_LEN));
        SeqCmdBuff[3] = SEQ_STOP();
      }
      else
      {
        uint32_t NextAddr = (CurrBlock == CURRBLK_BLK0) ? Block1Addr : Block0Addr;

        if(genSeq0)
          SeqCmdBuff[2] = SEQ_WR(REG_AFE_SEQ0INFO, (NextAddr<<BITP_AFE_SEQ0INFO_ADDR) | (SEQLEN_ONESTEP<<BITP_AFE_SEQ0INFO_LEN));
        else
          SeqCmdBuff[2] = SEQ_WR(REG_AFE_SEQ1INFO, (NextAddr<<BITP_AFE_SEQ1INFO_ADDR) | (SEQLEN_ONESTEP<<BITP_AFE_SEQ1INFO_LEN));

        SeqCmdBuff[3] = SEQ_INT0();
      }
    }
    else
    {
      uint32_t NextStepAddr = SRAMAddr + SEQLEN_ONESTEP;
      if(genSeq0)
        SeqCmdBuff[2] = SEQ_WR(REG_AFE_SEQ0INFO, (NextStepAddr<<BITP_AFE_SEQ0INFO_ADDR) | (SEQLEN_ONESTEP<<BITP_AFE_SEQ0INFO_LEN));
      else
        SeqCmdBuff[2] = SEQ_WR(REG_AFE_SEQ1INFO, (NextStepAddr<<BITP_AFE_SEQ1INFO_ADDR) | (SEQLEN_ONESTEP<<BITP_AFE_SEQ1INFO_LEN));

      SeqCmdBuff[3] = SEQ_STOP();
    }

    AD5940_SEQCmdWrite(SRAMAddr, SeqCmdBuff, SEQLEN_ONESTEP);
    SRAMAddr += SEQLEN_ONESTEP;
  }

  /* Toggle which sequence we update next time (SEQ0 then SEQ1 then SEQ0...) */
  AppDPVCfg.bNextIsSeq0 = (AppDPVCfg.bNextIsSeq0 == bTRUE) ? bFALSE : bTRUE;

  /* Switch ping-pong block */
  CurrBlock = (CurrBlock == CURRBLK_BLK0) ? CURRBLK_BLK1 : CURRBLK_BLK0;

  return error;
}

/* ---------- Data process: convert FIFO samples into ΔI pairs ---------- */
static int32_t AppDPVDataProcess(int32_t * const pData, uint32_t *pDataCount)
{
  uint32_t n = *pDataCount;
  float *pOut = (float *)pData;

  /* Convert all samples to current (uA) */
  for(uint32_t i=0; i<n; i++)
  {
    uint32_t code = (uint32_t)pData[i] & 0xffff;
    float v = AD5940_ADCCode2Volt(code, AppDPVCfg.AdcPgaGain, AppDPVCfg.ADCRefVolt);
    pOut[i] = -v/AppDPVCfg.RtiaValue.Magnitude*1e3f;
  }

  /* Collapse into ΔI pairs: (pulse - before) */
  uint32_t pairs = n/2;
  for(uint32_t k=0; k<pairs; k++)
  {
    float I_before = pOut[2*k + 0];
    float I_pulse  = pOut[2*k + 1];
    pOut[k] = (I_pulse - I_before);
  }

  *pDataCount = pairs;
  return 0;
}

/* ---------- Init function ---------- */
AD5940Err AppDPVInit(uint32_t *pBuffer, uint32_t BufferSize)
{
  AD5940Err error = AD5940ERR_OK;

  if(pBuffer == NULL) return AD5940ERR_PARA;
  if(BufferSize < 64) return AD5940ERR_PARA;

  /* Step count from Ein..Ef */
  if(AppDPVCfg.StaircaseInc <= 0.0f) return AD5940ERR_PARA;
  AppDPVCfg.StepNumber = (uint32_t)((AppDPVCfg.RampPeakVolt - AppDPVCfg.RampStartVolt)/AppDPVCfg.StaircaseInc) + 1;

  AppDPVCfg.DPVInited = bFALSE;
  AppDPVCfg.bFirstDACSeq = bTRUE;
  AppDPVCfg.bNextIsSeq0 = bTRUE;
  AppDPVCfg.RampState = DPV_STATE0;
  AppDPVCfg.StopRequired = bFALSE;
  AppDPVCfg.CurrStepPos = 0;

  /* Basic FIFO setup */
  FIFOCfg_Type fifo_cfg;
  fifo_cfg.FIFOEn = bTRUE;
  fifo_cfg.FIFOMode = FIFOMODE_FIFO;
  fifo_cfg.FIFOSize = FIFOSIZE_2KB;
  fifo_cfg.FIFOSrc = FIFOSRC_SINC3;
  fifo_cfg.FIFOThresh = AppDPVCfg.FifoThresh;
  AD5940_FIFOCfg(&fifo_cfg);
  AD5940_FIFOClr();
  AD5940_INTCCfg(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH|AFEINTSRC_ENDSEQ|AFEINTSRC_CUSTOMINT0, bTRUE);
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);

  /* Generate sequences */
  error = AppDPVSeqInitGen();   if(error != AD5940ERR_OK) return error;
  error = AppDPVSeqADCCtrlGen();if(error != AD5940ERR_OK) return error;

  /* Calibrate RTIA quickly using RCAL if desired (keep simple: use nominal) */
  AppDPVCfg.RtiaValue.Magnitude = (AppDPVCfg.LPTIARtiaSel == LPTIARTIA_8K) ? 8000.0f :
                                 (AppDPVCfg.LPTIARtiaSel == LPTIARTIA_20K) ? 20000.0f :
                                 (AppDPVCfg.LPTIARtiaSel == LPTIARTIA_4K) ? 4000.0f : 8000.0f;
  AppDPVCfg.RtiaValue.Phase = 0;

  /* Prime DAC sequences */
  AppDPVSeqDACCtrlGen(); /* SEQ0 */
  AppDPVSeqDACCtrlGen(); /* SEQ1 */

  /* Run init sequence once */
  AD5940_SEQCtrlS(bTRUE);
  AD5940_SEQMmrTrig(AppDPVCfg.InitSeqInfo.SeqId);
  AD5940_SEQCtrlS(bFALSE);

  AppDPVCfg.DPVInited = bTRUE;
  return AD5940ERR_OK;
}

/* ---------- Control ---------- */
AD5940Err AppDPVCtrl(uint32_t Command, void *pPara)
{
  switch (Command)
  {
    case APPCTRL_START:
    {
      WUPTCfg_Type wupt_cfg;

      if(AD5940_WakeUp(10) > 10) return AD5940ERR_WAKEUP;
      if(AppDPVCfg.DPVInited == bFALSE) return AD5940ERR_APPERROR;
      if(AppDPVCfg.RampState == DPV_STOP) return AD5940ERR_APPERROR;

      wupt_cfg.WuptEn = bTRUE;
      wupt_cfg.WuptEndSeq = WUPTENDSEQ_D;

      /* Order: baseline DAC (SEQ0) -> ADC (SEQ2) -> pulse DAC (SEQ1) -> ADC (SEQ2) */
      wupt_cfg.WuptOrder[0] = SEQID_0;
      wupt_cfg.WuptOrder[1] = SEQID_2;
      wupt_cfg.WuptOrder[2] = SEQID_1;
      wupt_cfg.WuptOrder[3] = SEQID_2;

      /* ADC timing (small) */
      wupt_cfg.SeqxSleepTime[SEQID_2] = 4;
      wupt_cfg.SeqxWakeupTime[SEQID_2] = (uint32_t)(AppDPVCfg.LFOSCClkFreq*(AppDPVCfg.SampleDelay/1000.0f)) - 4 - 2;

      /* Baseline wait timing (tw) */
      float t0 = AppDPVCfg.DPV_WaitTime - (AppDPVCfg.SampleDelay/1000.0f);
      if(t0 < 0.001f) t0 = 0.001f;
      wupt_cfg.SeqxSleepTime[SEQID_0] = 4;
      wupt_cfg.SeqxWakeupTime[SEQID_0] = (uint32_t)(AppDPVCfg.LFOSCClkFreq*t0) - 4 - 2;

      /* Pulse width timing (tp) */
      float t1 = AppDPVCfg.DPV_PulseTime - (AppDPVCfg.SampleDelay/1000.0f);
      if(t1 < 0.001f) t1 = 0.001f;
      wupt_cfg.SeqxSleepTime[SEQID_1] = 4;
      wupt_cfg.SeqxWakeupTime[SEQID_1] = (uint32_t)(AppDPVCfg.LFOSCClkFreq*t1) - 4 - 2;

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

/* ---------- ISR ---------- */
AD5940Err AppDPVISR(void *pBuff, uint32_t *pCount)
{
  uint32_t IntFlag;
  if(pBuff == NULL || pCount == NULL) return AD5940ERR_PARA;

  IntFlag = AD5940_INTCGetFlag(AFEINTC_0);
  AD5940_INTCClrFlag(IntFlag);

  if(IntFlag & AFEINTSRC_CUSTOMINT0)
  {
    /* Custom interrupt requests next ping-pong block update */
    AppDPVSeqDACCtrlGen();
  }

  if(IntFlag & AFEINTSRC_DATAFIFOTHRESH)
  {
    uint32_t count = AD5940_FIFOGetCnt();
    if(count > *pCount) count = *pCount;
    AD5940_FIFORd((uint32_t*)pBuff, count);
    *pCount = count;
    AppDPVDataProcess((int32_t*)pBuff, pCount);

    if(AppDPVCfg.StopRequired == bTRUE)
    {
      AppDPVCtrl(APPCTRL_STOPNOW, 0);
      AppDPVCfg.StopRequired = bFALSE;
    }
    return AD5940ERR_OK;
  }

  return AD5940ERR_OK;
}
