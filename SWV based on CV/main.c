/* ================================== main.c ================================= */
#include "UrtLib.h"
#include "ClkLib.h"
#include "DioLib.h"
#include <stdio.h>
#include "ad5940.h"

static void UartInit(void);
static void ClockInit(void);

int main(void)
{
  void AD5940_Main(void);

  ClockInit();
  UartInit();
  AD5940_MCUResourceInit(0);
  AD5940_Main();
}

static void ClockInit(void)
{
  DigClkSel(DIGCLK_SOURCE_HFOSC);
  ClkDivCfg(1,1);
}

static void UartInit(void)
{
  DioCfgPin(pADI_GPIO0, PIN10|PIN11, 1);
  UrtCfg(pADI_UART0, B230400, (BITM_UART_COMLCR_WLS|3), 0);
  UrtFifoCfg(pADI_UART0, RX_FIFO_1BYTE, BITM_UART_COMFCR_FIFOEN);
  UrtFifoClr(pADI_UART0, BITM_UART_COMFCR_RFCLR|BITM_UART_COMFCR_TFCLR);
}
