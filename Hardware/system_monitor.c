#include "system_monitor.h"
#include "motor_control.h"
#include "pump_valve.h"
#include "sensor.h"
#include "temp_control.h"
#include "uv_lamp.h"

//低电压阈值：20.0V（低于则报警）
#ifndef SYSTEM_BAT_LOW_DV 
#define SYSTEM_BAT_LOW_DV            200U
#endif

//高电压阈值：26.0V（高于则报警）
#ifndef SYSTEM_BAT_HIGH_DV
#define SYSTEM_BAT_HIGH_DV           260U
#endif

static uint8_t system_main_status;    //系统主状态，0=上电 1=待机 2=运行 3=错误
static uint8_t system_sub_status;     //系统子状态，预留给不同主状态下的细分状态使用，具体定义由上层应用决定
static uint8_t system_err1;           //错误码1（水位、传感器、超温）
static uint8_t system_err2;           //错误码2（加热、水泵、电机、UV、电池）
static uint8_t system_last_cmd;       //保存最后一次接收的指令
static uint8_t system_link_status;    //通信连接状态
static uint8_t system_reset_requested;  //系统复位请求标志
static uint32_t system_reset_tick;      //复位请求时间
static uint32_t system_timer_deadline_tick;   //定时关机截止时间

//把指令里的时间编码（1~6）转换成对应的定时毫秒（10/15/20/25/30分钟）
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

//系统监控初始化
void SystemMonitor_Init(void)
{
  system_main_status = BUCKET_STATUS_POWER_ON;  //初始状态：开机
  system_sub_status = 0U;                       //子状态清空
  system_err1 = 0U;                             //错误码1清空
  system_err2 = 0U;                             //错误码2清空
  system_last_cmd = 0xA1U;                      //默认指令
  system_link_status = 0U;                      //通信断开
  system_reset_requested = 0U;                  //无复位请求
  system_reset_tick = 0UL;                      //复位时间清空
  system_timer_deadline_tick = 0UL;             // 定时关机时间清空
}

//关闭加热、电机、水泵、UV灯
void SystemMonitor_StopAllOutputs(void)
{
  Temp_Enable(0U);
  Motor_Stop();
  PumpValve_SetMode(PUMP_VALVE_MODE_OFF);
  UV_Off();
}

//设置系统主状态 + 子状态
void SystemMonitor_SetMainStatus(uint8_t main_status, uint8_t sub_status)
{
  system_main_status = main_status;
  system_sub_status = sub_status;
}

//获取状态接口
uint8_t SystemMonitor_GetMainStatus(void)
{
  return system_main_status;
}

//获取状态接口
uint8_t SystemMonitor_GetSubStatus(void)
{
  return system_sub_status;
}

//获取错误码1接口（最高位屏蔽）
uint8_t SystemMonitor_GetErrCode1(void)
{
  return system_err1 & 0x7FU;
}

//获取错误码2接口（最高位屏蔽）
uint8_t SystemMonitor_GetErrCode2(void)
{
  return system_err2 & 0x7FU;
}

//指令存储接口
void SystemMonitor_SetCommand(uint8_t cmd)
{
  system_last_cmd = cmd;
}

//指令存储接口
uint8_t SystemMonitor_GetCommand(void)
{
  return system_last_cmd;
}

//通信连接状态
void SystemMonitor_SetLinkStatus(uint8_t link_status)
{
  system_link_status = (link_status != 0U) ? 1U : 0U;
}

//通信连接状态
uint8_t SystemMonitor_GetLinkStatus(void)
{
  return system_link_status;
}

//设置定时关机
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

//获取剩余时间（分钟）
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

//请求系统复位
void SystemMonitor_RequestReset(void)
{
  system_reset_requested = 1U;
  system_reset_tick = HAL_GetTick();
}

//查询是否请求复位
uint8_t SystemMonitor_IsResetRequested(void)
{
  return system_reset_requested;
}

//清除所有故障
void SystemMonitor_ClearErrors(void)
{
  system_err1 = 0U;
  system_err2 = 0U;
  Temp_ClearFault();
  Motor_ClearFault();
  PumpValve_ClearFault();
  UV_ClearFault();
}

//系统监控核心任务（周期调用）
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
