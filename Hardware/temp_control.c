#include "temp_control.h"
#include "main.h"
#include "sensor.h"

static float temp_target_c = TEMP_TARGET_DEFAULT_C;   // 目标温度
static uint8_t temp_enabled;  // 温控使能标志 0=禁用 1=启用
static uint8_t temp_heating;  // 加热状态标志 0=未加热 1=正在加热
static uint8_t temp_fault;    // 温控故障标志 0=正常 1=故障
static uint32_t temp_heat_start_tick;   // 加热开始时间，用于加热超时检测

//将目标温度限制在安全范围内
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

//设置加热输出状态
static void Temp_SetHeatOutput(uint8_t on)
{
  GPIO_PinState state;

  state = (on != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET;
  HAL_GPIO_WritePin(EN_HEAT_GPIO_Port, EN_HEAT_Pin, state);
  temp_heating = (on != 0U) ? 1U : 0U;
}

//初始化温控模块
void Temp_Init(void)
{
  temp_target_c = TEMP_TARGET_DEFAULT_C;    //默认目标温度
  temp_enabled = 0U;                  //默认禁用温控  
  temp_fault = 0U;          //默认无故障
  temp_heat_start_tick = 0U;    //加热开始时间清零
  Temp_SetHeatOutput(0U);     //确保加热输出关闭
}

//设置目标温度
void Temp_Set(float target)
{
  Temp_SetTargetC(target);
}

//设置目标温度（摄氏度）
void Temp_SetTargetC(float target)
{
  temp_target_c = Temp_ClampTarget(target);
}

//获取当前设置的目标温度
float Temp_GetTargetC(void)
{
  return temp_target_c;
}

//读取当前温度
float Temp_Read(void)
{
  return Sensor_GetTemperatureC();
}

//使能温控
void Temp_Enable(uint8_t enable)
{
  temp_enabled = (enable != 0U) ? 1U : 0U;
  if (temp_enabled == 0U)
  {
    Temp_SetHeatOutput(0U);
    temp_heat_start_tick = 0U;
  }
}

//检查温控是否使能
uint8_t Temp_IsEnabled(void)
{
  return temp_enabled;
}

//检查是否正在加热
uint8_t Temp_IsHeating(void)
{
  return temp_heating;
}

//检查是否有故障
uint8_t Temp_HasFault(void)
{
  return temp_fault;
}

//清除故障状态
void Temp_ClearFault(void)
{
  temp_fault = 0U;
}

//温控任务处理
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

	/*
  // 3. 【三重安全保护】任意一个触发 → 判定故障，停止加热
  // 条件1：温度传感器故障（短路/断路）
  // 条件2：水温超温（超过安全上限，如50℃）
  // 条件3：水位过低（缺水，防止干烧）
  if ((Sensor_IsTempSensorOk() == 0U) ||
      (current_temp >= TEMP_HIGH_CUTOFF_C) ||
      (Sensor_GetWaterLevelProtocol() < SENSOR_WATER_MIN_SAFE_LITERS))
  {
    temp_fault = 1U;
    temp_enabled = 0U;
    Temp_SetHeatOutput(0U);
    return;
  }*/

  //回差控温逻辑
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

  //加热超时保护：如果持续加热超过设定时间（如15分钟）且温度仍未达到目标温度 - 回差下限 → 判定故障
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
