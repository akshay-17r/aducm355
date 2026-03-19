/******************************************************************************
Combined AMP + DPV scheduler for ADuCM355
- Replaces separate AD5940Main.c / ad5940main.c files
- Schedules:
    AMP every 5 minutes
    DPV every 10 minutes
- If both are due together:
    run AMP first
    wait a short RTC gap
    then run DPV
- Uses RTC interrupt to wake digital die from hibernate
******************************************************************************/

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

#include "ad5940.h"
#include "AD5940.h"
#include "DPV.h"
#include "Amperometric.h"
#include "RtcLib.h"
#include "PwrLib.h"

/* ========================= Shared buffers / globals ========================= */

#define AMP_APPBUFF_SIZE   1024u
#define DPV_APPBUFF_SIZE   1024u

static uint32_t gAmpAppBuff[AMP_APPBUFF_SIZE];
static uint32_t gDpvAppBuff[DPV_APPBUFF_SIZE];

static float LFOSCFreq = 32000.0f;

/* ========================= User schedule ========================= */

#define RTC_TICKS_PER_SECOND   (32768u)

/* Periods */
#define AMP_PERIOD_S           (30u)   /* 5 min */
#define DPV_PERIOD_S           (60u)   /* 10 min */

/* Short non-blocking gap when both are due */
#define AMP_TO_DPV_GAP_S       (10u)

/* ========================= Scheduler state ========================= */

typedef enum
{
  SCHED_WAIT_EVENT = 0,
  SCHED_WAIT_DPV_GAP
} SchedulerState_t;

static volatile uint8_t gRtcWakeupFlag = 1u;
static SchedulerState_t gSchedState = SCHED_WAIT_EVENT;

static uint32_t gNextAmpAlarm = 0u;
static uint32_t gNextDpvAlarm = 0u;

/* ========================= DPV result helpers ========================= */

/* Implemented in DPV.c */
extern float AppDPV_GetLastPeak_uA(void);


static void DPV_PrintCalibrated(void)
{
		float Ipk  = AppDPV_GetLastPeak_uA();
//  float cTnI = AppDPV_GetLastConc_cTnI_pg_mL();
//  float BNP  = AppDPV_GetLastConc_BNP_pg_mL();

//	printf("Peak Current = %.3f uA\r\n", Ipk);
//  printf("cTnI (from calibration) = %.3e pg/mL\r\n", cTnI);
//  printf("BNP  (from calibration) = %.3e pg/mL\r\n", BNP);
}

/* ========================= Common helpers ========================= */

static uint32_t MinU32(uint32_t a, uint32_t b)
{
  return (a < b) ? a : b;
}

static void EnterShutdownHibernate(void)
{
  /* Wake AFE before shutdown command */
  AD5940_ReadReg(REG_AFE_ADCDAT);
  AD5940_ShutDownS();

  /* Digital die hibernate */
  PwrCfg(ENUM_PMG_PWRMOD_HIBERNATE, MONITOR_VBAT_EN, 0);
}

static void RTCWakeupInit(void)
{
  RtcCfgCR0(BITM_RTC_CR0_CNTEN, 0);                 /* disable RTC while configuring */
  RtcSetPre(RTC1_PRESCALE_1);
  RtcSetCnt(0);
  RtcIntClrSR0(BITM_RTC_SR0_ALMINT);
  RtcCfgCR0(BITM_RTC_CR0_ALMEN | BITM_RTC_CR0_ALMINTEN, 1);
  NVIC_EnableIRQ(RTC1_EVT_IRQn);
  RtcCfgCR0(BITM_RTC_CR0_CNTEN, 1);
}

static uint32_t RTCGetCount32(void)
{
  uint32_t rtc_cnt;
  uint16_t rtc_cnt2;
  RtcGetSnap(&rtc_cnt, &rtc_cnt2);
  (void)rtc_cnt2;
  return rtc_cnt;
}

static void RTCScheduleAlarmAt(uint32_t alarm_ticks)
{
  RtcSetAlarm(alarm_ticks, 0);
  RtcIntClrSR0(BITM_RTC_SR0_ALMINT);
}

/* RTC interrupt */
void RTC1_Int_Handler(void)
{
  if(pADI_RTC1->SR0 & BITM_RTC_SR0_ALMINT)
  {
		printf("RTC WAKEUP\r\n");
    RtcIntClrSR0(BITM_RTC_SR0_ALMINT);
    gRtcWakeupFlag = 1u;
  }
}

/* ========================= Shared platform init ========================= */

static int32_t AD5940PlatformCfg(void)
{
  CLKCfg_Type clk_cfg;
  LFOSCMeasure_Type LfoscMeasure;

  /* Clean reset each measurement cycle */
  AD5940_HWReset();
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

  /* Enable AFE interrupts we need across both apps */
  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE);
  AD5940_INTCCfg(AFEINTC_0,
                 AFEINTSRC_DATAFIFOTHRESH |
                 AFEINTSRC_ENDSEQ |
                 AFEINTSRC_CUSTOMINT0,
                 bTRUE);
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);

  AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);

  memset(&LfoscMeasure, 0, sizeof(LfoscMeasure));
  LfoscMeasure.CalDuration = 1000.0f;
  LfoscMeasure.CalSeqAddr = 0;
  LfoscMeasure.SystemClkFreq = 16000000.0f;
  AD5940_LFOSCMeasure(&LfoscMeasure, &LFOSCFreq);

  printf("LFOSC Freq: %f\r\n", LFOSCFreq);
  return 0;
}

/* ========================= AMP config / run ========================= */

//static int32_t AMPShowResult(float *pData, uint32_t DataCount)
//{
//  AppAMPCfg_Type *pAmpCfg;
//  uint32_t i;

//  (void)pData;
//  AppAMPGetCfg(&pAmpCfg);

//  for(i = 0; i < DataCount; i++)
//  {
//    printf("AMP Index %lu: %.3f uA\r\n",
//           (unsigned long)i,
//           pAmpCfg->SensorCh0.ResultBuffer[i]);
//  }
//  return 0;
//}

static void AMP_UserCfg(void)
{
  AppAMPCfg_Type *pAmpCfg;

  AppAMPGetCfg(&pAmpCfg);

  pAmpCfg->WuptClkFreq = LFOSCFreq;

  /* Sequencer / general */
  pAmpCfg->SeqStartAddr = 0;
  pAmpCfg->MaxSeqLen = 512;
  pAmpCfg->RcalVal = 200.0f;

  pAmpCfg->NumOfData = -1;						/* 5 samples per second, 5 seconds total */
  pAmpCfg->AmpODR = 1.0f;             /* 5 samples per second*/
  pAmpCfg->M355FifoThresh = 1;				/* interrupts every 5 samples*/

  /* Force regeneration after shutdown */
  pAmpCfg->AMPInited = bFALSE;
  pAmpCfg->bParaChanged = bTRUE;
  pAmpCfg->StopRequired = bFALSE;

  /* Sensor config */
  pAmpCfg->SensorCh0.LpTiaRf = LPTIARF_1M;
  pAmpCfg->SensorCh0.LpTiaRl = LPTIARLOAD_10R;
  pAmpCfg->SensorCh0.LptiaRtiaSel = LPTIARTIA_100K;
  pAmpCfg->SensorCh0.Vzero = 1100.0f;
  pAmpCfg->SensorCh0.SensorBias = -100.0f;
}

static void RunAMP(void)
{
  uint32_t temp = 0;
  uint32_t totalCount = 0;
  float sum = 0.0f;
  AppAMPCfg_Type *pAmpCfg;

  printf("Running AMP ...\r\n");

  AppAMPGetCfg(&pAmpCfg);

  AD5940PlatformCfg();
  AMP_UserCfg();

  AppECSnrInit(&pAmpCfg->SensorCh0, M355_CHAN0);

  if(AppAMPInit(gAmpAppBuff, AMP_APPBUFF_SIZE) != AD5940ERR_OK)
  {
    printf("AppAMPInit failed\r\n");
    return;
  }

  AD5940_ClrMCUIntFlag();

  if(AppAMPCtrl(AMPCTRL_START, 0) != AD5940ERR_OK)
  {
    printf("AppAMPCtrl START failed\r\n");
    return;
  }
	
uint32_t t_start = RTCGetCount32();

/* ?? CONFIGURE THESE */
uint32_t total_ticks   = 5 * RTC_TICKS_PER_SECOND;     // total AMP time (5 s)
uint32_t settle_ticks  = (uint32_t)(2.5f * RTC_TICKS_PER_SECOND);  // delay before logging

while((RTCGetCount32() - t_start) < total_ticks)
{
  if(AD5940_GetMCUIntFlag())
  {
    AD5940_ClrMCUIntFlag();

    temp = AMP_APPBUFF_SIZE;
    AppAMPISR(gAmpAppBuff, &temp);

    uint32_t elapsed = RTCGetCount32() - t_start;

    /* ?? ONLY log AFTER 2.5 s */
    if(elapsed >= settle_ticks)
    {
      for(uint32_t i = 0; i < temp; i++)
      {
        float val = pAmpCfg->SensorCh0.ResultBuffer[i];
        sum += val;
        totalCount++;
      }
    }
  }
}

  AppAMPCtrl(AMPCTRL_STOPNOW, 0);

  float avg = - sum / (float)totalCount;

  printf("AMP Avg: %.3f uA\r\n", avg);
}

/* ========================= DPV config / run ========================= */

static void DPV_UserCfg(void)
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

  pCfg->Ein_mV           = 750.0f;
  pCfg->Efinal_mV        = -150.0f;
  pCfg->StepIncrement_mV = -5.0f;

  pCfg->PulseAmplitude_mV = -10.0f;
  pCfg->WaitTime_s        = 0.08f;
  pCfg->PulseTime_s       = 0.02f;

  pCfg->Vzero_mV = 1600.0f;

  pCfg->SampleDelayBase_ms  = 5.0f;
  pCfg->SampleDelayPulse_ms = 5.0f;

  pCfg->LPTIARtiaSel      = LPTIARTIA_20K;
  pCfg->ExternalRtiaValue = 20000.0f;
  pCfg->AdcPgaGain        = ADCPGA_1P5;
  pCfg->ADCSinc3Osr       = ADCSINC3OSR_4;

  pCfg->FifoThresh        = 4;
}

static void RunDPV(void)
{
  uint32_t DataCount;
  uint8_t done = 0u;

  printf("Running DPV...\r\n");

  AD5940PlatformCfg();
  DPV_UserCfg();

  if(AppDPVInit(gDpvAppBuff, DPV_APPBUFF_SIZE) != AD5940ERR_OK)
  {
    printf("AppDPVInit failed\r\n");
    return;
  }

  if(AppDPVCtrl(APPCTRL_START, 0) != AD5940ERR_OK)
  {
    printf("AppDPVCtrl START failed\r\n");
    return;
  }

  while(!done)
  {
    if(AD5940_GetMCUIntFlag())
    {
      uint32_t intsrc;

      AD5940_ClrMCUIntFlag();

      /* Capture flags BEFORE AppDPVISR clears them */
      intsrc = AD5940_INTCGetFlag(AFEINTC_0);

      DataCount = DPV_APPBUFF_SIZE;
      if(AppDPVISR(gDpvAppBuff, &DataCount) == AD5940ERR_OK)
      {
        if(DataCount != 0u)
        {
          DPV_PrintCalibrated();
        }
      }

      if(intsrc & AFEINTSRC_ENDSEQ)
      {
        done = 1u;
      }
    }
  }

  AppDPVCtrl(APPCTRL_STOPNOW, 0);
  printf("DPV Done\r\n");
}

/* ========================= Combined scheduler ========================= */
static void AdvanceAlarmPastNow(uint32_t *pAlarm, uint32_t period_ticks, uint32_t now)
{
  while(*pAlarm <= now)
  {
    *pAlarm += period_ticks;
  }
}

void AD5940_Main(void)
{
  uint32_t now;
  const uint32_t amp_period_ticks = AMP_PERIOD_S * RTC_TICKS_PER_SECOND;
  const uint32_t dpv_period_ticks = DPV_PERIOD_S * RTC_TICKS_PER_SECOND;
  const uint32_t gap_ticks        = AMP_TO_DPV_GAP_S * RTC_TICKS_PER_SECOND;

  RTCWakeupInit();

  now = RTCGetCount32();
  gNextAmpAlarm = now + amp_period_ticks;
  gNextDpvAlarm = now + dpv_period_ticks;

  RTCScheduleAlarmAt(gNextAmpAlarm);

  while(1)
  {
    if(gRtcWakeupFlag)
    {
      uint8_t amp_due;
      uint8_t dpv_due;

      gRtcWakeupFlag = 0u;
      now = RTCGetCount32();

      switch(gSchedState)
      {
        case SCHED_WAIT_EVENT:
        {
          amp_due = (now >= gNextAmpAlarm) ? 1u : 0u;
          dpv_due = (now >= gNextDpvAlarm) ? 1u : 0u;

          if(amp_due && dpv_due)
          {
            printf("AMP due + DPV due -> AMP first\r\n");
            RunAMP();

            /* Re-read time because RunAMP takes seconds */
            now = RTCGetCount32();

            /* Keep absolute cadence, but push alarms ahead if now has already passed them */
            gNextAmpAlarm += amp_period_ticks;
            AdvanceAlarmPastNow(&gNextAmpAlarm, amp_period_ticks, now);

            /* DPV is still due, so schedule short gap from CURRENT time */
            gSchedState = SCHED_WAIT_DPV_GAP;
            RTCScheduleAlarmAt(now + gap_ticks);
          }
          else if(amp_due)
          {
            printf("AMP due\r\n");
            RunAMP();

            now = RTCGetCount32();

            gNextAmpAlarm += amp_period_ticks;
            AdvanceAlarmPastNow(&gNextAmpAlarm, amp_period_ticks, now);
            AdvanceAlarmPastNow(&gNextDpvAlarm, dpv_period_ticks, now);

            RTCScheduleAlarmAt(MinU32(gNextAmpAlarm, gNextDpvAlarm));
          }
          else if(dpv_due)
          {
            printf("DPV due\r\n");
            RunDPV();

            now = RTCGetCount32();

            gNextDpvAlarm += dpv_period_ticks;
            AdvanceAlarmPastNow(&gNextAmpAlarm, amp_period_ticks, now);
            AdvanceAlarmPastNow(&gNextDpvAlarm, dpv_period_ticks, now);

            RTCScheduleAlarmAt(MinU32(gNextAmpAlarm, gNextDpvAlarm));
          }
          else
          {
            AdvanceAlarmPastNow(&gNextAmpAlarm, amp_period_ticks, now);
            AdvanceAlarmPastNow(&gNextDpvAlarm, dpv_period_ticks, now);

            RTCScheduleAlarmAt(MinU32(gNextAmpAlarm, gNextDpvAlarm));
          }
          break;
        }

        case SCHED_WAIT_DPV_GAP:
        {
          printf("Gap elapsed -> DPV\r\n");
          RunDPV();

          now = RTCGetCount32();

          gNextDpvAlarm += dpv_period_ticks;
          AdvanceAlarmPastNow(&gNextAmpAlarm, amp_period_ticks, now);
          AdvanceAlarmPastNow(&gNextDpvAlarm, dpv_period_ticks, now);

          gSchedState = SCHED_WAIT_EVENT;
          RTCScheduleAlarmAt(MinU32(gNextAmpAlarm, gNextDpvAlarm));
          break;
        }

        default:
        {
          now = RTCGetCount32();
          AdvanceAlarmPastNow(&gNextAmpAlarm, amp_period_ticks, now);
          AdvanceAlarmPastNow(&gNextDpvAlarm, dpv_period_ticks, now);

          gSchedState = SCHED_WAIT_EVENT;
          RTCScheduleAlarmAt(MinU32(gNextAmpAlarm, gNextDpvAlarm));
          break;
        }
      }
    }

    EnterShutdownHibernate();
  }
}
