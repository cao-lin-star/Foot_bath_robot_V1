#include "log.h"
#include "motor_control.h"
#include "pump_valve.h"
#include "sensor.h"
#include "system_monitor.h"
#include "usart.h"
#include "uv_lamp.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#ifndef LOGGING_TX_BUFFER_LEN
#define LOGGING_TX_BUFFER_LEN       512U
#endif

#ifndef LOGGING_TX_DMA_CHUNK_LEN
#define LOGGING_TX_DMA_CHUNK_LEN    64U
#endif

#ifndef LOGGING_PRINTF_BUFFER_LEN
#define LOGGING_PRINTF_BUFFER_LEN   160U
#endif

#if defined(__CC_ARM)
#pragma import(__use_no_semihosting)
struct __FILE
{
  int handle;
};
FILE __stdout;

void _sys_exit(int x)
{
  (void)x;
}
#endif

static volatile HAL_StatusTypeDef logging_last_status = HAL_OK;
static uint8_t logging_tx_buffer[LOGGING_TX_BUFFER_LEN];
static uint8_t logging_tx_dma_buffer[LOGGING_TX_DMA_CHUNK_LEN];
static volatile uint16_t logging_tx_head;
static volatile uint16_t logging_tx_tail;
static volatile uint8_t logging_dma_busy;

static void Logging_RestoreIrq(uint32_t primask)
{
  if (primask == 0U)
  {
    __enable_irq();
  }
}

static uint16_t Logging_NextIndex(uint16_t index)
{
  index++;
  if (index >= LOGGING_TX_BUFFER_LEN)
  {
    index = 0U;
  }
  return index;
}

static void Logging_StartTxDma(void)
{
  HAL_StatusTypeDef status;
  uint32_t primask;
  uint16_t length;
  uint16_t next_tail;

  length = 0U;
  primask = __get_PRIMASK();
  __disable_irq();
  if ((logging_dma_busy != 0U) || (logging_tx_head == logging_tx_tail))
  {
    Logging_RestoreIrq(primask);
    return;
  }

  logging_dma_busy = 1U;
  next_tail = logging_tx_tail;
  while ((next_tail != logging_tx_head) && (length < LOGGING_TX_DMA_CHUNK_LEN))
  {
    logging_tx_dma_buffer[length] = logging_tx_buffer[next_tail];
    next_tail = Logging_NextIndex(next_tail);
    length++;
  }
  Logging_RestoreIrq(primask);

  if (length == 0U)
  {
    primask = __get_PRIMASK();
    __disable_irq();
    logging_dma_busy = 0U;
    Logging_RestoreIrq(primask);
    return;
  }

  status = HAL_UART_Transmit_DMA(&huart3, logging_tx_dma_buffer, length);
  logging_last_status = status;
  primask = __get_PRIMASK();
  __disable_irq();
  if (status == HAL_OK)
  {
    logging_tx_tail = next_tail;
  }
  else
  {
    logging_dma_busy = 0U;
  }
  Logging_RestoreIrq(primask);
}

static void Logging_WriteBuffer(const uint8_t *data, uint16_t length)
{
  uint16_t next_head;
  uint16_t index;
  uint32_t primask;
  uint8_t overflow;

  if ((data == NULL) || (length == 0U))
  {
    return;
  }

  overflow = 0U;
  primask = __get_PRIMASK();
  __disable_irq();
  for (index = 0U; index < length; index++)
  {
    next_head = Logging_NextIndex(logging_tx_head);
    if (next_head == logging_tx_tail)
    {
      overflow = 1U;
      break;
    }

    logging_tx_buffer[logging_tx_head] = data[index];
    logging_tx_head = next_head;
  }
  logging_last_status = (overflow == 0U) ? HAL_OK : HAL_BUSY;
  Logging_RestoreIrq(primask);

  Logging_StartTxDma();
}

int fputc(int ch, FILE *f)
{
  uint8_t data;

  (void)f;
  data = (uint8_t)ch;
  Logging_WriteBuffer(&data, 1U);
  return ch;
}

void Logging_Init(void)
{
  uint32_t primask;

  primask = __get_PRIMASK();
  __disable_irq();
  logging_tx_head = 0U;
  logging_tx_tail = 0U;
  logging_dma_busy = 0U;
  logging_last_status = HAL_OK;
  Logging_RestoreIrq(primask);

  Logging_Print("LOG UART3 READY\r\n");
}

void Logging_Print(const char *msg)
{
  size_t length;
  uint16_t chunk;

  if (msg != NULL)
  {
    length = strlen(msg);
    while (length > 0U)
    {
      chunk = (length > 0xFFFFU) ? 0xFFFFU : (uint16_t)length;
      Logging_WriteBuffer((const uint8_t *)msg, chunk);
      msg += chunk;
      length -= chunk;
    }
  }
}

void Logging_Printf(const char *fmt, ...)
{
  va_list args;
  char buffer[LOGGING_PRINTF_BUFFER_LEN];
  int length;
  uint8_t truncated;

  if (fmt == NULL)
  {
    return;
  }

  truncated = 0U;
  va_start(args, fmt);
  length = vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  if (length < 0)
  {
    logging_last_status = HAL_ERROR;
    return;
  }
  if ((size_t)length >= sizeof(buffer))
  {
    length = (int)(sizeof(buffer) - 1U);
    truncated = 1U;
  }

  Logging_WriteBuffer((const uint8_t *)buffer, (uint16_t)length);
  if (truncated != 0U)
  {
    logging_last_status = HAL_BUSY;
  }
}

void Logging_TaskProcess(void)
{
  int16_t temp_x10;
  char sign;
  uint16_t battery_dv;

  temp_x10 = Sensor_GetTemperatureCx10();
  sign = '+';
  if (temp_x10 < 0)
  {
    sign = '-';
    temp_x10 = (int16_t)(-temp_x10);
  }

  battery_dv = Sensor_GetBatteryDeciVolt();
  Logging_Printf("T=%c%d.%uC W=%uL BAT=%u.%uV M=%u P=%u UV=%u ERR=%02X/%02X\r\n",
                 sign,
                 temp_x10 / 10,
                 (uint8_t)(temp_x10 % 10),
                 Sensor_GetWaterLevelProtocol(),
                 battery_dv / 10U,
                 battery_dv % 10U,
                 Motor_GetLevel(),
                 (uint8_t)PumpValve_GetMode(),
                 UV_IsOn(),
                 SystemMonitor_GetErrCode1(),
                 SystemMonitor_GetErrCode2());
}

HAL_StatusTypeDef Logging_GetLastStatus(void)
{
  return logging_last_status;
}

void Logging_TxCpltCallback(UART_HandleTypeDef *huart)
{
  uint32_t primask;

  if (huart != &huart3)
  {
    return;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  logging_dma_busy = 0U;
  Logging_RestoreIrq(primask);

  Logging_StartTxDma();
}

void Logging_ErrorCallback(UART_HandleTypeDef *huart)
{
  uint32_t primask;

  if (huart != &huart3)
  {
    return;
  }

  logging_last_status = HAL_ERROR;
  ATOMIC_CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAT);
  ATOMIC_CLEAR_BIT(huart->Instance->CR1, USART_CR1_TCIE);
  if (huart->hdmatx != NULL)
  {
    (void)HAL_DMA_Abort(huart->hdmatx);
  }
  huart->gState = HAL_UART_STATE_READY;

  primask = __get_PRIMASK();
  __disable_irq();
  logging_dma_busy = 0U;
  Logging_RestoreIrq(primask);

  Logging_StartTxDma();
}
