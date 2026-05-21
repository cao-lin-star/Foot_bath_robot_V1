#include "system_monitor.h"
#include "motor_control.h"
#include "pump_valve.h"
#include "sensor.h"
#include "temp_control.h"
#include "uv_lamp.h"

#ifndef SYSTEM_BAT_LOW_DV
#define SYSTEM_BAT_LOW_DV            200U
#endif

#ifndef SYSTEM_BAT_HIGH_DV
#define SYSTEM_BAT_HIGH_DV           260U
#endif

static uint8_t system_main_status;
static uint8_t system_sub_status;
static uint8_t system_err1;
static uint8_t system_err2;
static uint8_t system_last_cmd;
static uint8_t system_link_status;
static uint8_t system_reset_requested;
static uint32_t system_reset_tick;
static uint32_t system_timer_deadline_tick;

static uint32_t SystemMonitor_TimerCodeToMs(uint8_t timer_code)
{
  switch (timer_code)
  {
    case 1U:
      return 10UL * 60UL * 1000UL;
    case 2U:
      return 15UL * 60UL * 1000UL;
    case 3U:
      return 20UL * 60UL * 1000UL;
    case 4U:
      return 25UL * 60UL * 1000UL;
    case 5U:
    case 6U:
      return 30UL * 60UL * 1000UL;
    default:
      return 0UL;
  }
}

void SystemMonitor_Init(void)
{
  system_main_status = BUCKET_STATUS_POWER_ON;
  system_sub_status = 0U;
  system_err1 = 0U;
  system_err2 = 0U;
  system_last_cmd = 0xA1U;
  system_link_status = 0U;
  system_reset_requested = 0U;
  system_reset_tick = 0UL;
  system_timer_deadline_tick = 0UL;
}

void SystemMonitor_StopAllOutputs(void)
{
  Temp_Enable(0U);
  Motor_Stop();
  PumpValve_SetMode(PUMP_VALVE_MODE_OFF);
  UV_Off();
}

void SystemMonitor_SetMainStatus(uint8_t main_status, uint8_t sub_status)
{
  system_main_status = main_status;
  system_sub_status = sub_status;
}

uint8_t SystemMonitor_GetMainStatus(void)
{
  return system_main_status;
}

uint8_t SystemMonitor_GetSubStatus(void)
{
  return system_sub_status;
}

uint8_t SystemMonitor_GetErrCode1(void)
{
  return system_err1 & 0x7FU;
}

uint8_t SystemMonitor_GetErrCode2(void)
{
  return system_err2 & 0x7FU;
}

void SystemMonitor_SetCommand(uint8_t cmd)
{
  system_last_cmd = cmd;
}

uint8_t SystemMonitor_GetCommand(void)
{
  return system_last_cmd;
}

void SystemMonitor_SetLinkStatus(uint8_t link_status)
{
  system_link_status = (link_status != 0U) ? 1U : 0U;
}

uint8_t SystemMonitor_GetLinkStatus(void)
{
  return system_link_status;
}

void SystemMonitor_SetBathTimer(uint8_t timer_code)
{
  uint32_t duration_ms;

  duration_ms = SystemMonitor_TimerCodeToMs(timer_code);
  if (duration_ms == 0UL)
  {
    system_timer_deadline_tick = 0UL;
  }
  else
  {
    system_timer_deadline_tick = HAL_GetTick() + duration_ms;
  }
}

uint8_t SystemMonitor_GetTimerRemainingMin(void)
{
  uint32_t now;
  uint32_t remaining_ms;

  if (system_timer_deadline_tick == 0UL)
  {
    return 0U;
  }

  now = HAL_GetTick();
  if ((int32_t)(system_timer_deadline_tick - now) <= 0)
  {
    return 0U;
  }

  remaining_ms = system_timer_deadline_tick - now;
  return (uint8_t)((remaining_ms + 59999UL) / 60000UL);
}

void SystemMonitor_RequestReset(void)
{
  system_reset_requested = 1U;
  system_reset_tick = HAL_GetTick();
}

uint8_t SystemMonitor_IsResetRequested(void)
{
  return system_reset_requested;
}

void SystemMonitor_ClearErrors(void)
{
  system_err1 = 0U;
  system_err2 = 0U;
  Temp_ClearFault();
  Motor_ClearFault();
  PumpValve_ClearFault();
  UV_ClearFault();
}

void SystemMonitor_TaskProcess(void)
{
  uint16_t battery_dv;
  uint8_t water_level;
  uint8_t active_needs_water;

  system_err1 = 0U;
  system_err2 = 0U;

  water_level = Sensor_GetWaterLevelProtocol();
  active_needs_water = ((Temp_IsEnabled() != 0U) ||
                        (PumpValve_GetMode() == PUMP_VALVE_MODE_CIRCULATION) ||
                        (UV_IsOn() != 0U)) ? 1U : 0U;

  if ((active_needs_water != 0U) && (water_level < SENSOR_WATER_MIN_SAFE_LITERS))
  {
    system_err1 |= BUCKET_ERR1_LACK_WATER;
  }
  if (Sensor_IsWaterSensorOk() == 0U)
  {
    system_err1 |= BUCKET_ERR1_WATER_SENSOR;
  }
  if (Sensor_IsTempSensorOk() == 0U)
  {
    system_err1 |= BUCKET_ERR1_TEMP_SENSOR;
  }
  if (Sensor_GetTemperatureC() >= TEMP_HIGH_CUTOFF_C)
  {
    system_err1 |= BUCKET_ERR1_TEMP_ABNORMAL;
  }
  if (Temp_HasFault() != 0U)
  {
    system_err2 |= BUCKET_ERR2_HEAT_MODULE;
  }
  if (PumpValve_HasFault() != 0U)
  {
    system_err2 |= BUCKET_ERR2_PUMP;
  }
  if (Motor_HasFault() != 0U)
  {
    system_err2 |= BUCKET_ERR2_MOTOR;
  }
  if (UV_HasFault() != 0U)
  {
    system_err2 |= BUCKET_ERR2_UV;
  }

  battery_dv = Sensor_GetBatteryDeciVolt();
  if ((battery_dv != 0U) && ((battery_dv < SYSTEM_BAT_LOW_DV) || (battery_dv > SYSTEM_BAT_HIGH_DV)))
  {
    system_err2 |= BUCKET_ERR2_BATTERY;
    if (battery_dv < SYSTEM_BAT_LOW_DV)
    {
      system_main_status = BUCKET_STATUS_LOW_POWER;
      SystemMonitor_StopAllOutputs();
    }
  }

  if ((system_timer_deadline_tick != 0UL) && (SystemMonitor_GetTimerRemainingMin() == 0U))
  {
    system_timer_deadline_tick = 0UL;
    SystemMonitor_StopAllOutputs();
    system_main_status = BUCKET_STATUS_STANDBY;
    system_sub_status = 0U;
  }

  if ((system_reset_requested != 0U) && ((HAL_GetTick() - system_reset_tick) > 200UL))
  {
    NVIC_SystemReset();
  }
}
