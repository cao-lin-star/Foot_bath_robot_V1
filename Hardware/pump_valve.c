#include "pump_valve.h"
#include "main.h"
#include "sensor.h"

static PumpValveMode_t pump_mode;
static uint8_t pump_fault;
static uint32_t pump_mode_start_tick;

static void PumpValve_Apply(PumpValveMode_t mode)
{
  switch (mode)
  {
    case PUMP_VALVE_MODE_CIRCULATION:
      HAL_GPIO_WritePin(EN_TWV_GPIO_Port, EN_TWV_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(EN_PUMP_GPIO_Port, EN_PUMP_Pin, GPIO_PIN_SET);
      break;

    case PUMP_VALVE_MODE_DRAIN:
      HAL_GPIO_WritePin(EN_TWV_GPIO_Port, EN_TWV_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(EN_PUMP_GPIO_Port, EN_PUMP_Pin, GPIO_PIN_SET);
      break;

    default:
      HAL_GPIO_WritePin(EN_PUMP_GPIO_Port, EN_PUMP_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(EN_TWV_GPIO_Port, EN_TWV_Pin, GPIO_PIN_RESET);
      break;
  }
}

void PumpValve_Init(void)
{
  pump_mode = PUMP_VALVE_MODE_OFF;
  pump_fault = 0U;
  pump_mode_start_tick = HAL_GetTick();
  PumpValve_Apply(PUMP_VALVE_MODE_OFF);
}

void PumpValve_SetMode(PumpValveMode_t mode)
{
  if (mode != pump_mode)
  {
    pump_mode_start_tick = HAL_GetTick();
  }
  pump_mode = mode;
  PumpValve_Apply(mode);
}

PumpValveMode_t PumpValve_GetMode(void)
{
  return pump_mode;
}

void PumpValve_TaskProcess(void)
{
  uint16_t current_ma;

  current_ma = Sensor_GetPumpCurrentMa();
  if ((pump_mode != PUMP_VALVE_MODE_OFF) && (current_ma > PUMP_CURRENT_MAX_MA))
  {
    pump_fault = 1U;
    PumpValve_SetMode(PUMP_VALVE_MODE_OFF);
    return;
  }

  if ((pump_mode == PUMP_VALVE_MODE_DRAIN) &&
      ((HAL_GetTick() - pump_mode_start_tick) > PUMP_VALVE_DRAIN_TIMEOUT_MS) &&
      (Sensor_GetWaterLevelProtocol() > 0U))
  {
    pump_fault = 1U;
    PumpValve_SetMode(PUMP_VALVE_MODE_OFF);
  }
}

uint8_t PumpValve_IsPumpOn(void)
{
  return (pump_mode != PUMP_VALVE_MODE_OFF) ? 1U : 0U;
}

uint8_t PumpValve_HasFault(void)
{
  return pump_fault;
}

void PumpValve_ClearFault(void)
{
  pump_fault = 0U;
}

void Pump_Start(void)
{
  PumpValve_SetMode(PUMP_VALVE_MODE_CIRCULATION);
}

void Pump_Stop(void)
{
  PumpValve_SetMode(PUMP_VALVE_MODE_OFF);
}

void Valve_Set(uint8_t mode)
{
  if (mode != 0U)
  {
    PumpValve_SetMode(PUMP_VALVE_MODE_DRAIN);
  }
  else
  {
    PumpValve_SetMode(PUMP_VALVE_MODE_CIRCULATION);
  }
}
