#include "uart_comm.h"
#include "log.h"
#include "motor_control.h"
#include "pump_valve.h"
#include "sensor.h"
#include "system_monitor.h"
#include "temp_control.h"
#include "usart.h"
#include "uv_lamp.h"
#include <string.h>

#ifndef UART_COMM_STATUS_PERIOD_MS
#define UART_COMM_STATUS_PERIOD_MS       1000UL
#endif

#ifndef UART_COMM_BASE_TIMEOUT_MS
#define UART_COMM_BASE_TIMEOUT_MS        3000UL
#endif

#ifndef UART_COMM_FORCE_PROTOCOL_BAUD
#define UART_COMM_FORCE_PROTOCOL_BAUD    1U
#endif

#ifndef UART_COMM_RX_DMA_BUFFER_LEN
#define UART_COMM_RX_DMA_BUFFER_LEN      128U
#endif

#define UART_CMD_IDLE                    0x00U
#define UART_CMD_BUCKET_OFF              0xA0U
#define UART_CMD_BUCKET_STANDBY          0xA1U
#define UART_CMD_BUCKET_TEMP_ON          0xA2U
#define UART_CMD_BUCKET_TEMP_OFF         0xA3U
#define UART_CMD_BUCKET_MOTOR            0xA4U
#define UART_CMD_BUCKET_UV               0xA5U
#define UART_CMD_BUCKET_TIMER            0xA6U
#define UART_CMD_BUCKET_STOP             0xA7U
#define UART_CMD_BUCKET_SELF_CHECK       0xA8U
#define UART_CMD_BUCKET_LOW_POWER        0xA9U
#define UART_CMD_BASE_DRAIN              0xB4U
#define UART_CMD_SYSTEM_RESET            0xC0U

typedef struct
{
  uint8_t buffer[UART_COMM_FRAME_LEN];
  uint8_t index;
} UartParser_t;

typedef struct
{
  uint8_t frame[UART_COMM_FRAME_LEN];
  volatile uint8_t ready;
} UartFrameSlot_t;

typedef struct
{
  UART_HandleTypeDef *huart;
  uint8_t active_frame[UART_COMM_FRAME_LEN];
  uint8_t pending_frame[UART_COMM_FRAME_LEN];
  volatile uint8_t busy;
  volatile uint8_t pending;
} UartTxSlot_t;

static UartParser_t linux_parser;
static UartParser_t base_parser;
static UartFrameSlot_t linux_rx_slot;
static UartFrameSlot_t base_rx_slot;
static uint8_t linux_rx_dma_buffer[UART_COMM_RX_DMA_BUFFER_LEN];
static uint8_t base_rx_dma_buffer[UART_COMM_RX_DMA_BUFFER_LEN];
static uint16_t linux_rx_dma_pos;
static uint16_t base_rx_dma_pos;
static UartTxSlot_t linux_tx_slot;
static UartTxSlot_t base_tx_slot;
static uint8_t base_data[13];
static uint32_t base_last_rx_tick;
static uint32_t last_status_tx_tick;
static uint8_t last_link_mode = UART_COMM_DEFAULT_LINK_MODE;

static void UART_Comm_RestoreIrq(uint32_t primask)
{
  if (primask == 0U)
  {
    __enable_irq();
  }
}

uint8_t UART_Comm_Checksum(const uint8_t *frame)
{
  uint16_t sum;
  uint8_t index;

  sum = 0U;
  if (frame == NULL)
  {
    return 0U;
  }
  for (index = 2U; index <= 28U; index++)
  {
    sum = (uint16_t)(sum + frame[index]);
  }
  return (uint8_t)(sum & 0xFFU);
}

uint8_t UART_Comm_IsFrameValid(const uint8_t *frame)
{
  if (frame == NULL)
  {
    return 0U;
  }
  if ((frame[0] != UART_COMM_HEAD1) || (frame[1] != UART_COMM_HEAD2))
  {
    return 0U;
  }
  return (UART_Comm_Checksum(frame) == frame[29]) ? 1U : 0U;
}

static void UART_Comm_StoreFrame(UartFrameSlot_t *slot, const uint8_t *frame)
{
  if ((slot == NULL) || (frame == NULL))
  {
    return;
  }
  memcpy(slot->frame, frame, UART_COMM_FRAME_LEN);
  slot->ready = 1U;
}

static void UART_Comm_ParserPush(UartParser_t *parser, UartFrameSlot_t *slot, uint8_t byte)
{
  if ((parser == NULL) || (slot == NULL))
  {
    return;
  }

  if (parser->index == 0U)
  {
    if (byte != UART_COMM_HEAD1)
    {
      return;
    }
    parser->buffer[parser->index++] = byte;
    return;
  }

  if (parser->index == 1U)
  {
    if (byte != UART_COMM_HEAD2)
    {
      parser->index = (byte == UART_COMM_HEAD1) ? 1U : 0U;
      parser->buffer[0] = UART_COMM_HEAD1;
      return;
    }
    parser->buffer[parser->index++] = byte;
    return;
  }

  parser->buffer[parser->index++] = byte;
  if (parser->index >= UART_COMM_FRAME_LEN)
  {
    if (UART_Comm_IsFrameValid(parser->buffer) != 0U)
    {
      UART_Comm_StoreFrame(slot, parser->buffer);
    }
    parser->index = 0U;
  }
}

static uint8_t UART_Comm_FetchFrame(UartFrameSlot_t *slot, uint8_t *out_frame)
{
  uint8_t ready;
  uint32_t primask;

  if ((slot == NULL) || (out_frame == NULL))
  {
    return 0U;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  ready = slot->ready;
  if (ready != 0U)
  {
    memcpy(out_frame, slot->frame, UART_COMM_FRAME_LEN);
    slot->ready = 0U;
  }
  UART_Comm_RestoreIrq(primask);

  return ready;
}

static UartTxSlot_t *UART_Comm_GetTxSlot(UART_HandleTypeDef *huart)
{
  if (huart == &huart1)
  {
    return &linux_tx_slot;
  }
  if (huart == &huart2)
  {
    return &base_tx_slot;
  }
  return NULL;
}

static HAL_StatusTypeDef UART_Comm_StartReceiveOne(UART_HandleTypeDef *huart, uint8_t *buffer, uint16_t *old_pos)
{
  HAL_StatusTypeDef status;

  if ((huart == NULL) || (buffer == NULL) || (old_pos == NULL))
  {
    return HAL_ERROR;
  }

  *old_pos = 0U;
  status = HAL_UARTEx_ReceiveToIdle_DMA(huart, buffer, UART_COMM_RX_DMA_BUFFER_LEN);
  if ((status == HAL_OK) && (huart->hdmarx != NULL))
  {
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
  }
  return status;
}

static void UART_Comm_StopReceiveOne(UART_HandleTypeDef *huart)
{
  if (huart == NULL)
  {
    return;
  }

  ATOMIC_CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAR);
  if (huart->hdmarx != NULL)
  {
    (void)HAL_DMA_Abort(huart->hdmarx);
  }
  ATOMIC_CLEAR_BIT(huart->Instance->CR1, (USART_CR1_PEIE | USART_CR1_IDLEIE));
  ATOMIC_CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);
  huart->RxState = HAL_UART_STATE_READY;
  huart->ReceptionType = HAL_UART_RECEPTION_STANDARD;
}

static void UART_Comm_StartReceive(void)
{
  memset(linux_rx_dma_buffer, 0, sizeof(linux_rx_dma_buffer));
  memset(base_rx_dma_buffer, 0, sizeof(base_rx_dma_buffer));
  (void)UART_Comm_StartReceiveOne(&huart1, linux_rx_dma_buffer, &linux_rx_dma_pos);
  (void)UART_Comm_StartReceiveOne(&huart2, base_rx_dma_buffer, &base_rx_dma_pos);
}

static void UART_Comm_ProcessRxRange(UartParser_t *parser, UartFrameSlot_t *slot,
                                     const uint8_t *buffer, uint16_t start, uint16_t end)
{
  uint16_t index;

  for (index = start; index < end; index++)
  {
    UART_Comm_ParserPush(parser, slot, buffer[index]);
  }
}

static void UART_Comm_ProcessDmaRx(UartParser_t *parser, UartFrameSlot_t *slot,
                                   const uint8_t *buffer, uint16_t *old_pos, uint16_t pos)
{
  if ((parser == NULL) || (slot == NULL) || (buffer == NULL) || (old_pos == NULL))
  {
    return;
  }
  if (pos > UART_COMM_RX_DMA_BUFFER_LEN)
  {
    return;
  }

  if (pos == *old_pos)
  {
    return;
  }

  if (pos > *old_pos)
  {
    UART_Comm_ProcessRxRange(parser, slot, buffer, *old_pos, pos);
  }
  else
  {
    UART_Comm_ProcessRxRange(parser, slot, buffer, *old_pos, UART_COMM_RX_DMA_BUFFER_LEN);
    UART_Comm_ProcessRxRange(parser, slot, buffer, 0U, pos);
  }
  *old_pos = pos;
}

static void UART_Comm_TryStartTx(UartTxSlot_t *slot)
{
  HAL_StatusTypeDef status;
  uint32_t primask;
  uint8_t should_start;

  if ((slot == NULL) || (slot->huart == NULL))
  {
    return;
  }

  should_start = 0U;
  primask = __get_PRIMASK();
  __disable_irq();
  if ((slot->busy == 0U) && (slot->pending != 0U))
  {
    memcpy(slot->active_frame, slot->pending_frame, UART_COMM_FRAME_LEN);
    slot->pending = 0U;
    slot->busy = 1U;
    should_start = 1U;
  }
  UART_Comm_RestoreIrq(primask);

  if (should_start != 0U)
  {
    status = HAL_UART_Transmit_DMA(slot->huart, slot->active_frame, UART_COMM_FRAME_LEN);
    if (status != HAL_OK)
    {
      primask = __get_PRIMASK();
      __disable_irq();
      memcpy(slot->pending_frame, slot->active_frame, UART_COMM_FRAME_LEN);
      slot->pending = 1U;
      slot->busy = 0U;
      UART_Comm_RestoreIrq(primask);
    }
  }
}

static void UART_Comm_SendFrame(UART_HandleTypeDef *huart, const uint8_t *frame)
{
  UartTxSlot_t *slot;
  uint32_t primask;

  if ((huart == NULL) || (frame == NULL))
  {
    return;
  }

  slot = UART_Comm_GetTxSlot(huart);
  if (slot == NULL)
  {
    return;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  memcpy(slot->pending_frame, frame, UART_COMM_FRAME_LEN);
  slot->pending = 1U;
  UART_Comm_RestoreIrq(primask);

  UART_Comm_TryStartTx(slot);
}

static void UART_Comm_ForwardToBase(const uint8_t *frame)
{
  if (frame != NULL)
  {
    UART_Comm_SendFrame(&huart2, frame);
  }
}

static void UART_Comm_ProcessBucketCommand(const uint8_t *frame)
{
  uint8_t cmd;

  cmd = frame[3];
  SystemMonitor_SetCommand(cmd);
  SystemMonitor_SetLinkStatus(frame[4]);

  switch (cmd)
  {
    case UART_CMD_IDLE:
      break;

    case UART_CMD_BUCKET_OFF:
      SystemMonitor_StopAllOutputs();
      SystemMonitor_SetBathTimer(0U);
      SystemMonitor_SetMainStatus(BUCKET_STATUS_OFF, 0U);
      break;

    case UART_CMD_BUCKET_STANDBY:
      SystemMonitor_StopAllOutputs();
      SystemMonitor_SetBathTimer(0U);
      SystemMonitor_SetMainStatus(BUCKET_STATUS_STANDBY, 0U);
      break;

    case UART_CMD_BUCKET_TEMP_ON:
      Temp_SetTargetC((float)frame[6]);
      Temp_Enable(1U);
      PumpValve_SetMode(PUMP_VALVE_MODE_CIRCULATION);
      SystemMonitor_SetMainStatus(BUCKET_STATUS_RUNNING, frame[9]);
      break;

    case UART_CMD_BUCKET_TEMP_OFF:
      Temp_Enable(0U);
      if (UV_IsOn() == 0U)
      {
        PumpValve_SetMode(PUMP_VALVE_MODE_OFF);
      }
      SystemMonitor_SetMainStatus(BUCKET_STATUS_RUNNING, frame[9]);
      break;

    case UART_CMD_BUCKET_MOTOR:
      Motor_SetLevel(frame[7]);
      SystemMonitor_SetMainStatus((frame[7] == 0U) ? BUCKET_STATUS_STANDBY : BUCKET_STATUS_RUNNING, frame[9]);
      break;

    case UART_CMD_BUCKET_UV:
      UV_Set(frame[8]);
      if (frame[8] != 0U)
      {
        PumpValve_SetMode(PUMP_VALVE_MODE_CIRCULATION);
      }
      else if (Temp_IsEnabled() == 0U)
      {
        PumpValve_SetMode(PUMP_VALVE_MODE_OFF);
      }
      SystemMonitor_SetMainStatus((frame[8] == 0U) ? BUCKET_STATUS_STANDBY : BUCKET_STATUS_RUNNING, frame[9]);
      break;

    case UART_CMD_BUCKET_TIMER:
      SystemMonitor_SetBathTimer(frame[9]);
      if (frame[6] >= (uint8_t)TEMP_TARGET_MIN_C)
      {
        Temp_SetTargetC((float)frame[6]);
      }
      Motor_SetLevel(frame[7]);
      UV_Set(frame[8]);
      if ((Temp_IsEnabled() != 0U) || (UV_IsOn() != 0U))
      {
        PumpValve_SetMode(PUMP_VALVE_MODE_CIRCULATION);
      }
      SystemMonitor_SetMainStatus(BUCKET_STATUS_RUNNING, SystemMonitor_GetTimerRemainingMin());
      break;

    case UART_CMD_BUCKET_STOP:
      SystemMonitor_StopAllOutputs();
      SystemMonitor_SetBathTimer(0U);
      SystemMonitor_SetMainStatus(BUCKET_STATUS_STANDBY, 0U);
      break;

    case UART_CMD_BUCKET_SELF_CHECK:
      SystemMonitor_ClearErrors();
      SystemMonitor_SetMainStatus(BUCKET_STATUS_SELF_CHECK, 0U);
      break;

    case UART_CMD_BUCKET_LOW_POWER:
      SystemMonitor_StopAllOutputs();
      SystemMonitor_SetMainStatus(BUCKET_STATUS_LOW_POWER, 0U);
      break;

    default:
      break;
  }
}

static void UART_Comm_ProcessSystemCommand(const uint8_t *frame)
{
  SystemMonitor_SetCommand(frame[3]);
  if (frame[3] == UART_CMD_SYSTEM_RESET)
  {
    SystemMonitor_StopAllOutputs();
    SystemMonitor_RequestReset();
  }
}

static void UART_Comm_ProcessLinuxFrame(const uint8_t *frame)
{
  uint8_t cmd;

  if (frame == NULL)
  {
    return;
  }

  last_link_mode = frame[2];
  cmd = frame[3];

  if ((cmd >= 0xA0U) && (cmd <= 0xAFU))
  {
    UART_Comm_ProcessBucketCommand(frame);
  }
  else if ((cmd >= 0xB0U) && (cmd <= 0xBFU))
  {
    SystemMonitor_SetCommand(cmd);
    if (cmd == UART_CMD_BASE_DRAIN)
    {
      PumpValve_SetMode(PUMP_VALVE_MODE_DRAIN);
      SystemMonitor_SetMainStatus(BUCKET_STATUS_RUNNING, 0U);
    }
    UART_Comm_ForwardToBase(frame);
  }
  else if ((cmd >= 0xC0U) && (cmd <= 0xCFU))
  {
    UART_Comm_ProcessSystemCommand(frame);
  }
}

static void UART_Comm_ProcessBaseFrame(const uint8_t *frame)
{
  if (frame == NULL)
  {
    return;
  }

  memcpy(base_data, &frame[16], sizeof(base_data));
  base_last_rx_tick = HAL_GetTick();
  SystemMonitor_SetLinkStatus(1U);
  UART_Comm_SendFrame(&huart1, frame);
}

uint8_t UART_Comm_BuildStatusFrame(uint8_t *frame)
{
  uint16_t battery_dv;
  uint8_t temp_value;
  float temp_c;

  if (frame == NULL)
  {
    return 0U;
  }

  memset(frame, 0, UART_COMM_FRAME_LEN);
  frame[0] = UART_COMM_HEAD1;
  frame[1] = UART_COMM_HEAD2;
  frame[2] = last_link_mode;
  frame[3] = SystemMonitor_GetCommand();
  frame[4] = UART_Comm_IsBaseConnected();
  frame[5] = Sensor_GetWaterLevelProtocol();

  temp_c = Sensor_GetTemperatureC();
  if (Sensor_IsTempSensorOk() != 0U)
  {
    temp_value = (uint8_t)(temp_c + 0.5f);
  }
  else
  {
    temp_value = 0U;
  }
  frame[6] = temp_value;
  frame[7] = Motor_GetLevel();
  frame[8] = UV_IsOn();
  frame[9] = SystemMonitor_GetTimerRemainingMin();

  battery_dv = Sensor_GetBatteryDeciVolt();
  frame[10] = (uint8_t)((battery_dv >> 8) & 0xFFU);
  frame[11] = (uint8_t)(battery_dv & 0xFFU);
  frame[12] = SystemMonitor_GetMainStatus();
  frame[13] = SystemMonitor_GetSubStatus();
  frame[14] = SystemMonitor_GetErrCode1();
  frame[15] = SystemMonitor_GetErrCode2();

  memcpy(&frame[16], base_data, sizeof(base_data));
  frame[29] = UART_Comm_Checksum(frame);
  return UART_COMM_FRAME_LEN;
}

uint8_t UART_Comm_IsBaseConnected(void)
{
  if ((HAL_GetTick() - base_last_rx_tick) < UART_COMM_BASE_TIMEOUT_MS)
  {
    return 1U;
  }
  return SystemMonitor_GetLinkStatus();
}

void UART_ParseFrame(uint8_t *data, uint8_t len)
{
  if ((data == NULL) || (len != UART_COMM_FRAME_LEN))
  {
    return;
  }
  if (UART_Comm_IsFrameValid(data) == 0U)
  {
    return;
  }
  UART_Comm_ProcessLinuxFrame(data);
}

void UART_Comm_Init(void)
{
  memset(&linux_parser, 0, sizeof(linux_parser));
  memset(&base_parser, 0, sizeof(base_parser));
  memset(&linux_rx_slot, 0, sizeof(linux_rx_slot));
  memset(&base_rx_slot, 0, sizeof(base_rx_slot));
  memset(&linux_tx_slot, 0, sizeof(linux_tx_slot));
  memset(&base_tx_slot, 0, sizeof(base_tx_slot));
  memset(base_data, 0, sizeof(base_data));
  linux_tx_slot.huart = &huart1;
  base_tx_slot.huart = &huart2;
  base_last_rx_tick = 0UL;
  last_status_tx_tick = 0UL;
  last_link_mode = UART_COMM_DEFAULT_LINK_MODE;

#if UART_COMM_FORCE_PROTOCOL_BAUD
  huart1.Init.BaudRate = 115200U;
  huart2.Init.BaudRate = 115200U;
  HAL_UART_Init(&huart1);
  HAL_UART_Init(&huart2);
#endif

  UART_Comm_StartReceive();
}

void UART_Comm_TaskProcess(void)
{
  uint8_t frame[UART_COMM_FRAME_LEN];
  uint32_t now;

  if (UART_Comm_FetchFrame(&linux_rx_slot, frame) != 0U)
  {
    UART_Comm_ProcessLinuxFrame(frame);
    UART_Comm_BuildStatusFrame(frame);
    UART_Comm_SendFrame(&huart1, frame);
  }

  if (UART_Comm_FetchFrame(&base_rx_slot, frame) != 0U)
  {
    UART_Comm_ProcessBaseFrame(frame);
  }

  now = HAL_GetTick();
  if ((now - last_status_tx_tick) >= UART_COMM_STATUS_PERIOD_MS)
  {
    last_status_tx_tick = now;
    UART_Comm_BuildStatusFrame(frame);
    UART_Comm_SendFrame(&huart1, frame);
    UART_Comm_SendFrame(&huart2, frame);
  }
}

void UART_Comm_RxEventCallback(UART_HandleTypeDef *huart, uint16_t pos)
{
  if (huart == &huart1)
  {
    UART_Comm_ProcessDmaRx(&linux_parser, &linux_rx_slot,
                           linux_rx_dma_buffer, &linux_rx_dma_pos, pos);
  }
  else if (huart == &huart2)
  {
    UART_Comm_ProcessDmaRx(&base_parser, &base_rx_slot,
                           base_rx_dma_buffer, &base_rx_dma_pos, pos);
  }
}

void UART_Comm_TxCpltCallback(UART_HandleTypeDef *huart)
{
  UartTxSlot_t *slot;
  uint32_t primask;

  slot = UART_Comm_GetTxSlot(huart);
  if (slot == NULL)
  {
    return;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  slot->busy = 0U;
  UART_Comm_RestoreIrq(primask);

  UART_Comm_TryStartTx(slot);
}

void UART_Comm_ErrorCallback(UART_HandleTypeDef *huart)
{
  UartTxSlot_t *slot;
  uint32_t primask;

  if (huart == &huart1)
  {
    UART_Comm_StopReceiveOne(&huart1);
    (void)UART_Comm_StartReceiveOne(&huart1, linux_rx_dma_buffer, &linux_rx_dma_pos);
  }
  else if (huart == &huart2)
  {
    UART_Comm_StopReceiveOne(&huart2);
    (void)UART_Comm_StartReceiveOne(&huart2, base_rx_dma_buffer, &base_rx_dma_pos);
  }

  slot = UART_Comm_GetTxSlot(huart);
  if ((slot != NULL) && (huart->gState == HAL_UART_STATE_READY))
  {
    primask = __get_PRIMASK();
    __disable_irq();
    slot->busy = 0U;
    UART_Comm_RestoreIrq(primask);
    UART_Comm_TryStartTx(slot);
  }
}
