#include "temp_control.h"
#include "main.h"
#include "sensor.h"

static float temp_target_c = TEMP_TARGET_DEFAULT_C;
static uint8_t temp_enabled;
static uint8_t temp_heating;
static uint8_t temp_fault;
static uint32_t temp_heat_start_tick;

static float Temp_ClampTarget(float target)
{
  if (target < TEMP_TARGET_MIN_C)
  {
    return TEMP_TARGET_MIN_C;
  }
  if (target > TEMP_TARGET_MAX_C)
  {
    return TEMP_TARGET_MAX_C;
  }
  return target;
}

static void Temp_SetHeatOutput(uint8_t on)
{
  GPIO_PinState state;

  state = (on != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET;
  HAL_GPIO_WritePin(EN_HEAT_GPIO_Port, EN_HEAT_Pin, state);
  HAL_GPIO_WritePin(HEAT_GPIO_Port, HEAT_Pin, state);
  temp_heating = (on != 0U) ? 1U : 0U;
}

void Temp_Init(void)
{
  temp_target_c = TEMP_TARGET_DEFAULT_C;
  temp_enabled = 0U;
  temp_fault = 0U;
  temp_heat_start_tick = 0U;
  Temp_SetHeatOutput(0U);
}

void Temp_Set(float target)
{
  Temp_SetTargetC(target);
}

void Temp_SetTargetC(float target)
{
  temp_target_c = Temp_ClampTarget(target);
}

float Temp_GetTargetC(void)
{
  return temp_target_c;
}

float Temp_Read(void)
{
  return Sensor_GetTemperatureC();
}

void Temp_Enable(uint8_t enable)
{
  temp_enabled = (enable != 0U) ? 1U : 0U;
  if (temp_enabled == 0U)
  {
    Temp_SetHeatOutput(0U);
    temp_heat_start_tick = 0U;
  }
}

uint8_t Temp_IsEnabled(void)
{
  return temp_enabled;
}

uint8_t Temp_IsHeating(void)
{
  return temp_heating;
}

uint8_t Temp_HasFault(void)
{
  return temp_fault;
}

void Temp_ClearFault(void)
{
  temp_fault = 0U;
}

void Temp_Control_TaskProcess(void)
{
  float current_temp;
  uint32_t now;

  current_temp = Sensor_GetTemperatureC();
  now = HAL_GetTick();

  if (temp_enabled == 0U)
  {
    Temp_SetHeatOutput(0U);
    return;
  }

  if ((Sensor_IsTempSensorOk() == 0U) ||
      (current_temp >= TEMP_HIGH_CUTOFF_C) ||
      (Sensor_GetWaterLevelProtocol() < SENSOR_WATER_MIN_SAFE_LITERS))
  {
    temp_fault = 1U;
    temp_enabled = 0U;
    Temp_SetHeatOutput(0U);
    return;
  }

  if (current_temp <= (temp_target_c - TEMP_HYSTERESIS_LOW_C))
  {
    if (temp_heating == 0U)
    {
      temp_heat_start_tick = now;
    }
    Temp_SetHeatOutput(1U);
  }
  else if (current_temp >= (temp_target_c + TEMP_HYSTERESIS_HIGH_C))
  {
    Temp_SetHeatOutput(0U);
    temp_heat_start_tick = 0U;
  }

  if ((temp_heating != 0U) &&
      (temp_heat_start_tick != 0U) &&
      ((now - temp_heat_start_tick) > TEMP_HEAT_TIMEOUT_MS) &&
      (current_temp < (temp_target_c - TEMP_HYSTERESIS_LOW_C)))
  {
    temp_fault = 1U;
    temp_enabled = 0U;
    Temp_SetHeatOutput(0U);
  }
}
