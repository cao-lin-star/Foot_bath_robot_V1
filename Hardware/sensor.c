// 包含传感器驱动头文件（定义了通道、结构体、宏）
#include "sensor.h"

// 包含STM32 ADC驱动（采集模拟电压用）
#include "adc.h"

// 包含STM32主控核心定义（GPIO、HAL库等）
#include "main.h"

// 数学库，用于NTC温度计算（logf对数函数）
#include <math.h>

// 内存操作库，用于清零数组、结构体
#include <string.h>

// ===================== 配置参数 =====================
// 滤波系数：8次平均滤波（数据更平稳）
#define SENSOR_FILTER_DIV       8U

// NTC热敏电阻 25℃时的标准阻值 10KΩ
#define SENSOR_NTC_R0_OHMS      10000.0f

// NTC热敏电阻B值（计算温度的核心参数）
#define SENSOR_NTC_BETA         3950.0f

// 25℃ 对应的开尔文温度
#define SENSOR_NTC_T0_K         298.15f

// NTC上拉电阻阻值 10KΩ
#define SENSOR_NTC_PULLUP_OHMS  10000.0f

// ===================== 内部全局变量 =====================
// ADC DMA搬运的原始数据（6个通道）
static volatile uint16_t sensor_adc_dma[SENSOR_ADC_CHANNEL_COUNT];

// 滤波累加器，用于平滑ADC数据
static uint32_t sensor_filter_acc[SENSOR_ADC_CHANNEL_COUNT];

// 传感器最终数据快照（温度、水位、电压、电流等）
static SensorSnapshot_t sensor_snapshot;

// 滤波是否就绪标志
static uint8_t sensor_filter_ready;

// ===================== 工具函数：ADC原始值 → 毫伏电压 =====================
static uint16_t Sensor_RawToMv(uint16_t raw)
{
  uint32_t mv;

  // 公式：mv = (原始值 * 参考电压) / 4095
  // +2047 是为了四舍五入
  mv = ((uint32_t)raw * SENSOR_ADC_VREF_MV + 2047U) / 4095U;
  return (uint16_t)mv;
}

// ===================== 工具函数：浮点数限幅 =====================
static float Sensor_ClampFloat(float value, float min_value, float max_value)
{
  // 小于最小值 → 返回最小值
  if (value < min_value)
  {
    return min_value;
  }
  // 大于最大值 → 返回最大值
  if (value > max_value)
  {
    return max_value;
  }
  // 正常范围 → 返回原值
  return value;
}

// ===================== 水位电压 → 实际水量（升） =====================
static float Sensor_CalcWaterLiters(uint16_t mv)
{
  float liters;

  // 电压 ≤ 空水位电压 → 0升
  if (mv <= SENSOR_WATER_EMPTY_MV)
  {
    return 0.0f;
  }
  // 电压 ≥ 满水位电压 → 最大容量
  if (mv >= SENSOR_WATER_FULL_MV)
  {
    return (float)SENSOR_WATER_MAX_LITERS;
  }

  // 线性换算：电压 → 水位
  liters = ((float)(mv - SENSOR_WATER_EMPTY_MV) * (float)SENSOR_WATER_MAX_LITERS) /
           (float)(SENSOR_WATER_FULL_MV - SENSOR_WATER_EMPTY_MV);

  // 限制水位在 0~最大值之间
  return Sensor_ClampFloat(liters, 0.0f, (float)SENSOR_WATER_MAX_LITERS);
}

// ===================== NTC热敏电阻 → 温度（核心） =====================
static float Sensor_CalcNtcTemperature(uint16_t raw)
{
  float voltage_ratio;
  float ntc_ohms;
  float inv_t;
  float temp_k;

  // ADC值异常（短路/断路）→ 返回无效温度
  if ((raw <= 5U) || (raw >= 4090U))
  {
    return SENSOR_TEMP_INVALID_C;
  }

  // 计算电压分压比例
  voltage_ratio = (float)raw / 4095.0f;

  // 根据分压 → 计算NTC电阻值
  ntc_ohms = SENSOR_NTC_PULLUP_OHMS * voltage_ratio / (1.0f - voltage_ratio);

  // 电阻异常 → 无效温度
  if (ntc_ohms <= 1.0f)
  {
    return SENSOR_TEMP_INVALID_C;
  }

  // B值公式计算温度（核心物理公式）
  inv_t = (1.0f / SENSOR_NTC_T0_K) + (logf(ntc_ohms / SENSOR_NTC_R0_OHMS) / SENSOR_NTC_BETA);

  if (inv_t <= 0.0f)
  {
    return SENSOR_TEMP_INVALID_C;
  }

  // 开尔文温度 → 摄氏温度
  temp_k = 1.0f / inv_t;
  return temp_k - 273.15f;
}

// ===================== 计算电池/供电实际电压（0.1V） =====================
static uint16_t Sensor_CalcScaledDecivolt(uint16_t adc_mv, uint16_t divider_x100)
{
  uint32_t input_mv;

  // 根据硬件分压电阻，还原真实电压
  input_mv = ((uint32_t)adc_mv * divider_x100) / 100U;

  // 转成 0.1V 单位（200 = 20.0V）
  return (uint16_t)((input_mv + 50U) / 100U);
}

// ===================== 电流采样电压 → 实际电流（mA） =====================
static uint16_t Sensor_CalcCurrentMa(uint16_t adc_mv)
{
  uint32_t current_ma;

  // 低于零点电压 → 无电流
  if (adc_mv <= SENSOR_CURRENT_ZERO_MV)
  {
    return 0U;
  }

  // 电压 → 电流换算
  current_ma = ((uint32_t)(adc_mv - SENSOR_CURRENT_ZERO_MV) * SENSOR_CURRENT_MA_PER_MV_X10) / 10U;

  // 限制最大值
  if (current_ma > 65535U)
  {
    current_ma = 65535U;
  }

  return (uint16_t)current_ma;
}

// ===================== 传感器初始化（开机必须调用） =====================
void Sensor_Init(void)
{
  // 清空ADC DMA缓冲区
  memset((void *)sensor_adc_dma, 0, sizeof(sensor_adc_dma));

  // 清空滤波累加器
  memset(sensor_filter_acc, 0, sizeof(sensor_filter_acc));

  // 清空数据快照结构体
  memset(&sensor_snapshot, 0, sizeof(sensor_snapshot));

  // 温度初始化为无效值
  sensor_snapshot.temperature_c = SENSOR_TEMP_INVALID_C;

  // 滤波未就绪
  sensor_filter_ready = 0U;

  // 打开NTC温度传感器电源
  HAL_GPIO_WritePin(EN_NTC_GPIO_Port, EN_NTC_Pin, GPIO_PIN_SET);

  // ADC校准
  HAL_ADCEx_Calibration_Start(&hadc1);

  // 启动ADC DMA，自动连续采集6个通道
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)sensor_adc_dma, SENSOR_ADC_CHANNEL_COUNT) != HAL_OK)
  {
    Error_Handler();
  }
}

// ===================== 传感器主任务（定时调用，例如10ms一次） =====================
void Sensor_TaskProcess(void)
{
  uint8_t index;
  uint16_t raw;
  uint16_t mv;
  float temp_c;

  // 遍历6个ADC通道
  for (index = 0U; index < SENSOR_ADC_CHANNEL_COUNT; index++)
  {
    // 从DMA缓冲区读取原始ADC值
    raw = sensor_adc_dma[index];

    // 第一次采集：初始化滤波
    if (sensor_filter_ready == 0U)
    {
      sensor_filter_acc[index] = (uint32_t)raw * SENSOR_FILTER_DIV;
    }
    // 后续：一阶递推滤波（平滑数据）
    else
    {
      sensor_filter_acc[index] = sensor_filter_acc[index] - (sensor_filter_acc[index] / SENSOR_FILTER_DIV) + raw;
    }

    // 计算滤波后的最终值
    sensor_snapshot.raw[index] = (uint16_t)(sensor_filter_acc[index] / SENSOR_FILTER_DIV);

    // 原始值 → 毫伏
    mv = Sensor_RawToMv(sensor_snapshot.raw[index]);
    sensor_snapshot.millivolt[index] = mv;
  }

  // 滤波就绪
  sensor_filter_ready = 1U;

  // ===================== 数据解析 =====================
  // 1. 计算水位（升）
  sensor_snapshot.water_liters = Sensor_CalcWaterLiters(sensor_snapshot.millivolt[SENSOR_ADC_WATER_LEVEL]);

  // 2. 计算电池电压（0.1V）
  sensor_snapshot.battery_decivolt = Sensor_CalcScaledDecivolt(sensor_snapshot.millivolt[SENSOR_ADC_BATTERY_VOLT], SENSOR_BAT_DIVIDER_X100);

  // 3. 计算外部供电电压
  sensor_snapshot.dcin_decivolt = Sensor_CalcScaledDecivolt(sensor_snapshot.millivolt[SENSOR_ADC_DCIN_VOLT], SENSOR_DCIN_DIVIDER_X100);

  // 4. 水泵电流
  sensor_snapshot.pump_current_ma = Sensor_CalcCurrentMa(sensor_snapshot.millivolt[SENSOR_ADC_PUMP_CURRENT]);

  // 5. 电机电流
  sensor_snapshot.motor_current_ma = Sensor_CalcCurrentMa(sensor_snapshot.millivolt[SENSOR_ADC_MOTOR_CURRENT]);

  // 6. 计算NTC温度
  temp_c = Sensor_CalcNtcTemperature(sensor_snapshot.raw[SENSOR_ADC_NTC_TEMP]);
  sensor_snapshot.temperature_c = temp_c;

  // 7. 判断温度传感器是否正常（0~85℃为有效）
  sensor_snapshot.temp_sensor_ok = ((temp_c > 0.0f) && (temp_c < 85.0f)) ? 1U : 0U;

  // 8. 判断水位传感器是否正常
  sensor_snapshot.water_sensor_ok =
      ((sensor_snapshot.raw[SENSOR_ADC_WATER_LEVEL] > 5U) && (sensor_snapshot.raw[SENSOR_ADC_WATER_LEVEL] < 4090U)) ? 1U : 0U;
}

// ===================== 获取全部传感器数据 =====================
void Sensor_GetSnapshot(SensorSnapshot_t *snapshot)
{
  if (snapshot != NULL)
  {
    *snapshot = sensor_snapshot;
  }
}

// ===================== 获取单个通道原始ADC值 =====================
uint16_t Sensor_GetRaw(SensorAdcChannel_t channel)
{
  if ((uint8_t)channel >= SENSOR_ADC_CHANNEL_COUNT)
  {
    return 0U;
  }
  return sensor_snapshot.raw[channel];
}

// ===================== 获取单个通道电压（mV） =====================
uint16_t Sensor_GetMilliVolt(SensorAdcChannel_t channel)
{
  if ((uint8_t)channel >= SENSOR_ADC_CHANNEL_COUNT)
  {
    return 0U;
  }
  return sensor_snapshot.millivolt[channel];
}

// ===================== 获取水位（升） =====================
float Sensor_GetWaterLiters(void)
{
  return sensor_snapshot.water_liters;
}

// ===================== 获取协议用水位（整数升） =====================
uint8_t Sensor_GetWaterLevelProtocol(void)
{
  uint8_t liters;

  liters = (uint8_t)(sensor_snapshot.water_liters + 0.5f); // 四舍五入
  if (liters > SENSOR_WATER_MAX_LITERS)
  {
    liters = SENSOR_WATER_MAX_LITERS;
  }
  return liters;
}

// ===================== 获取温度（℃） =====================
float Sensor_GetTemperatureC(void)
{
  return sensor_snapshot.temperature_c;
}

// ===================== 获取温度×10（方便通信传输） =====================
int16_t Sensor_GetTemperatureCx10(void)
{
  if (sensor_snapshot.temp_sensor_ok == 0U)
  {
    return -1000;
  }
  return (int16_t)(sensor_snapshot.temperature_c * 10.0f);
}

// ===================== 获取电池电压（0.1V） =====================
uint16_t Sensor_GetBatteryDeciVolt(void)
{
  return sensor_snapshot.battery_decivolt;
}

// ===================== 获取供电电压 =====================
uint16_t Sensor_GetDcinDeciVolt(void)
{
  return sensor_snapshot.dcin_decivolt;
}

// ===================== 获取水泵电流（mA） =====================
uint16_t Sensor_GetPumpCurrentMa(void)
{
  return sensor_snapshot.pump_current_ma;
}

// ===================== 获取电机电流（mA） =====================
uint16_t Sensor_GetMotorCurrentMa(void)
{
  return sensor_snapshot.motor_current_ma;
}

// ===================== 水位传感器是否正常 =====================
uint8_t Sensor_IsWaterSensorOk(void)
{
  return sensor_snapshot.water_sensor_ok;
}

// ===================== 温度传感器是否正常 =====================
uint8_t Sensor_IsTempSensorOk(void)
{
  return sensor_snapshot.temp_sensor_ok;
}
