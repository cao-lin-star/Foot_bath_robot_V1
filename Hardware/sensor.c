#include "sensor.h"
#include "adc.h"
#include "tim.h"
#include "main.h"
#include <math.h>
#include <string.h>

#define SENSOR_FILTER_DIV          8U
#define SENSOR_TEMP_FILTER_DIV     16U

#define SENSOR_NTC_R0_OHMS         10000.0f
#define SENSOR_NTC_BETA            3950.0f
#define SENSOR_NTC_T0_K            298.15f
#define SENSOR_NTC_PULLUP_OHMS     100000.0f

static volatile uint16_t sensor_adc_dma[SENSOR_ADC_CHANNEL_COUNT];
static uint32_t sensor_filter_acc[SENSOR_ADC_CHANNEL_COUNT];

static volatile uint32_t sensor_water_pulse_count;
static volatile uint32_t sensor_water_last_pulse_tick;
static uint32_t sensor_water_count_filter_acc;
static uint32_t sensor_water_window_start_tick;
static uint8_t sensor_water_window_started;
static uint8_t sensor_water_filter_ready;

static SensorSnapshot_t sensor_snapshot;
static uint8_t sensor_filter_ready;
static float sensor_temp_filter_c;
static uint8_t sensor_temp_filter_ready;

static void Sensor_RestoreIrq(uint32_t primask)
{
  if (primask == 0U)
  {
    __enable_irq();
  }
}

static uint16_t Sensor_RawToMv(uint16_t raw)
{
  uint32_t mv;

  mv = ((uint32_t)raw * SENSOR_ADC_VREF_MV + 2047U) / 4095U;
  return (uint16_t)mv;
}

static float Sensor_ClampFloat(float value, float min_value, float max_value)
{
  if (value < min_value)
  {
    return min_value;
  }
  if (value > max_value)
  {
    return max_value;
  }
  return value;
}

static float Sensor_CalcWaterLiters(uint32_t count_per_second)
{
  float liters;

  if (count_per_second >= SENSOR_WATER_EMPTY_COUNT)
  {
    return 0.0f;
  }
  if (count_per_second <= SENSOR_WATER_FULL_COUNT)
  {
    return (float)SENSOR_WATER_MAX_LITERS;
  }

  liters = ((float)(SENSOR_WATER_EMPTY_COUNT - count_per_second) *
            (float)SENSOR_WATER_MAX_LITERS) /
           (float)(SENSOR_WATER_EMPTY_COUNT - SENSOR_WATER_FULL_COUNT);

  return Sensor_ClampFloat(liters, 0.0f, (float)SENSOR_WATER_MAX_LITERS);
}

static float Sensor_CalcNtcTemperature(uint16_t raw)
{
  float voltage_ratio;
  float ntc_ohms;
  float inv_t;
  float temp_k;

  if ((raw <= 5U) || (raw >= 4090U))
  {
    return SENSOR_TEMP_INVALID_C;
  }

  voltage_ratio = (float)raw / 4095.0f;
  ntc_ohms = SENSOR_NTC_PULLUP_OHMS * (1.0f - voltage_ratio) / voltage_ratio;
  if (ntc_ohms <= 1.0f)
  {
    return SENSOR_TEMP_INVALID_C;
  }

  inv_t = (1.0f / SENSOR_NTC_T0_K) + (logf(ntc_ohms / SENSOR_NTC_R0_OHMS) / SENSOR_NTC_BETA);
  if (inv_t <= 0.0f)
  {
    return SENSOR_TEMP_INVALID_C;
  }

  temp_k = 1.0f / inv_t;
  return temp_k - 273.15f;
}

static uint16_t Sensor_CalcScaledDecivolt(uint16_t adc_mv, uint16_t divider_x100)
{
  uint32_t input_mv;

  input_mv = ((uint32_t)adc_mv * divider_x100) / 100U;
  return (uint16_t)((input_mv + 50U) / 100U);
}

static uint16_t Sensor_CalcCurrentMa(uint16_t adc_mv)
{
  uint32_t current_ma;

  if (adc_mv <= SENSOR_CURRENT_ZERO_MV)
  {
    return 0U;
  }

  current_ma = ((uint32_t)(adc_mv - SENSOR_CURRENT_ZERO_MV) * SENSOR_CURRENT_MA_PER_MV_X10) / 10U;
  if (current_ma > 65535U)
  {
    current_ma = 65535U;
  }

  return (uint16_t)current_ma;
}

static uint32_t Sensor_TakeWaterPulseSnapshot(void)
{
  uint32_t primask;
  uint32_t pulse_count;

  primask = __get_PRIMASK();
  __disable_irq();
  pulse_count = sensor_water_pulse_count;
  sensor_water_pulse_count = 0UL;
  Sensor_RestoreIrq(primask);

  return pulse_count;
}

static void Sensor_ProcessWaterCount(uint32_t now_tick)
{
  uint32_t elapsed_ms;
  uint32_t pulse_count;
  uint32_t count_per_second;

  if (sensor_water_window_started == 0U)
  {
    (void)Sensor_TakeWaterPulseSnapshot();
    sensor_water_window_start_tick = now_tick;
    sensor_water_window_started = 1U;
    return;
  }

  elapsed_ms = now_tick - sensor_water_window_start_tick;
  if (elapsed_ms < SENSOR_WATER_COUNT_WINDOW_MS)
  {
    if ((sensor_water_last_pulse_tick == 0UL) ||
        ((now_tick - sensor_water_last_pulse_tick) > SENSOR_WATER_COUNT_TIMEOUT_MS))
    {
      sensor_water_filter_ready = 0U;
      sensor_water_count_filter_acc = 0UL;
      sensor_snapshot.water_frequency_hz = 0UL;
      sensor_snapshot.water_liters = 0.0f;
      sensor_snapshot.water_sensor_ok = 0U;
    }
    return;
  }

  pulse_count = Sensor_TakeWaterPulseSnapshot();
  sensor_water_window_start_tick = now_tick;

  count_per_second = (pulse_count * 1000UL + (elapsed_ms / 2UL)) / elapsed_ms;
  if ((sensor_water_last_pulse_tick == 0UL) ||
      ((now_tick - sensor_water_last_pulse_tick) > SENSOR_WATER_COUNT_TIMEOUT_MS) ||
      (count_per_second < SENSOR_WATER_COUNT_MIN_VALID) ||
      (count_per_second > SENSOR_WATER_COUNT_MAX_VALID))
  {
    sensor_water_filter_ready = 0U;
    sensor_water_count_filter_acc = 0UL;
    sensor_snapshot.water_frequency_hz = 0UL;
    sensor_snapshot.water_liters = 0.0f;
    sensor_snapshot.water_sensor_ok = 0U;
    return;
  }

  if (sensor_water_filter_ready == 0U)
  {
    sensor_water_count_filter_acc = count_per_second * SENSOR_FILTER_DIV;
    sensor_water_filter_ready = 1U;
  }
  else
  {
    sensor_water_count_filter_acc = sensor_water_count_filter_acc -
                                    (sensor_water_count_filter_acc / SENSOR_FILTER_DIV) +
                                    count_per_second;
  }

  sensor_snapshot.water_frequency_hz = sensor_water_count_filter_acc / SENSOR_FILTER_DIV;
  sensor_snapshot.water_liters = Sensor_CalcWaterLiters(sensor_snapshot.water_frequency_hz);
  sensor_snapshot.water_sensor_ok = 1U;
}

static void Sensor_ProcessTemperature(float temp_c)
{
  if ((temp_c > 0.0f) && (temp_c < 85.0f))
  {
    if (sensor_temp_filter_ready == 0U)
    {
      sensor_temp_filter_c = temp_c;
      sensor_temp_filter_ready = 1U;
    }
    else
    {
      sensor_temp_filter_c += (temp_c - sensor_temp_filter_c) / (float)SENSOR_TEMP_FILTER_DIV;
    }

    sensor_snapshot.temperature_c = sensor_temp_filter_c;
    sensor_snapshot.temp_sensor_ok = 1U;
  }
  else
  {
    sensor_temp_filter_ready = 0U;
    sensor_temp_filter_c = SENSOR_TEMP_INVALID_C;
    sensor_snapshot.temperature_c = SENSOR_TEMP_INVALID_C;
    sensor_snapshot.temp_sensor_ok = 0U;
  }
}

void Sensor_Init(void)
{
  memset((void *)sensor_adc_dma, 0, sizeof(sensor_adc_dma));
  memset(sensor_filter_acc, 0, sizeof(sensor_filter_acc));
  memset(&sensor_snapshot, 0, sizeof(sensor_snapshot));

  sensor_snapshot.temperature_c = SENSOR_TEMP_INVALID_C;

  sensor_filter_ready = 0U;
  sensor_temp_filter_c = SENSOR_TEMP_INVALID_C;
  sensor_temp_filter_ready = 0U;

  sensor_water_pulse_count = 0UL;
  sensor_water_last_pulse_tick = 0UL;
  sensor_water_count_filter_acc = 0UL;
  sensor_water_window_start_tick = 0UL;
  sensor_water_window_started = 0U;
  sensor_water_filter_ready = 0U;

  HAL_GPIO_WritePin(EN_NTC_GPIO_Port, EN_NTC_Pin, GPIO_PIN_SET);

  HAL_ADCEx_Calibration_Start(&hadc1);

  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)sensor_adc_dma, SENSOR_ADC_CHANNEL_COUNT) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
}

void Sensor_TaskProcess(void)
{
  uint8_t index;
  uint16_t raw;
  uint16_t mv;
  float temp_c;
  uint32_t now_tick;

  for (index = 0U; index < SENSOR_ADC_CHANNEL_COUNT; index++)
  {
    raw = sensor_adc_dma[index];
    if (sensor_filter_ready == 0U)
    {
      sensor_filter_acc[index] = (uint32_t)raw * SENSOR_FILTER_DIV;
    }
    else
    {
      sensor_filter_acc[index] = sensor_filter_acc[index] -
                                 (sensor_filter_acc[index] / SENSOR_FILTER_DIV) +
                                 raw;
    }

    sensor_snapshot.raw[index] = (uint16_t)(sensor_filter_acc[index] / SENSOR_FILTER_DIV);
    mv = Sensor_RawToMv(sensor_snapshot.raw[index]);
    sensor_snapshot.millivolt[index] = mv;
  }

  sensor_filter_ready = 1U;

  now_tick = HAL_GetTick();
  Sensor_ProcessWaterCount(now_tick);

  sensor_snapshot.battery_decivolt = Sensor_CalcScaledDecivolt(sensor_snapshot.millivolt[SENSOR_ADC_BATTERY_VOLT], SENSOR_BAT_DIVIDER_X100);
  sensor_snapshot.dcin_decivolt = Sensor_CalcScaledDecivolt(sensor_snapshot.millivolt[SENSOR_ADC_DCIN_VOLT], SENSOR_DCIN_DIVIDER_X100);
  sensor_snapshot.pump_current_ma = Sensor_CalcCurrentMa(sensor_snapshot.millivolt[SENSOR_ADC_PUMP_CURRENT]);
  sensor_snapshot.motor_current_ma = Sensor_CalcCurrentMa(sensor_snapshot.millivolt[SENSOR_ADC_MOTOR_CURRENT]);

  temp_c = Sensor_CalcNtcTemperature(sensor_snapshot.raw[SENSOR_ADC_NTC_TEMP]);
  Sensor_ProcessTemperature(temp_c);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if ((htim == &htim3) && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1))
  {
    sensor_water_pulse_count++;
    sensor_water_last_pulse_tick = HAL_GetTick();
  }
}

void Sensor_GetSnapshot(SensorSnapshot_t *snapshot)
{
  if (snapshot != NULL)
  {
    *snapshot = sensor_snapshot;
  }
}

uint16_t Sensor_GetRaw(SensorAdcChannel_t channel)
{
  if ((uint8_t)channel >= SENSOR_ADC_CHANNEL_COUNT)
  {
    return 0U;
  }
  return sensor_snapshot.raw[channel];
}

uint16_t Sensor_GetMilliVolt(SensorAdcChannel_t channel)
{
  if ((uint8_t)channel >= SENSOR_ADC_CHANNEL_COUNT)
  {
    return 0U;
  }
  return sensor_snapshot.millivolt[channel];
}

float Sensor_GetWaterLiters(void)
{
  return sensor_snapshot.water_liters;
}

uint8_t Sensor_GetWaterLevelProtocol(void)
{
  uint8_t liters;

  liters = (uint8_t)(sensor_snapshot.water_liters + 0.5f);
  if (liters > SENSOR_WATER_MAX_LITERS)
  {
    liters = SENSOR_WATER_MAX_LITERS;
  }
  return liters;
}

uint32_t Sensor_GetWaterFrequencyHz(void)
{
  return sensor_snapshot.water_frequency_hz;
}

float Sensor_GetTemperatureC(void)
{
  return sensor_snapshot.temperature_c;
}

int16_t Sensor_GetTemperatureCx10(void)
{
  if (sensor_snapshot.temp_sensor_ok == 0U)
  {
    return -1000;
  }
  return (int16_t)(sensor_snapshot.temperature_c * 10.0f);
}

uint16_t Sensor_GetBatteryDeciVolt(void)
{
  return sensor_snapshot.battery_decivolt;
}

uint16_t Sensor_GetDcinDeciVolt(void)
{
  return sensor_snapshot.dcin_decivolt;
}

uint16_t Sensor_GetPumpCurrentMa(void)
{
  return sensor_snapshot.pump_current_ma;
}

uint16_t Sensor_GetMotorCurrentMa(void)
{
  return sensor_snapshot.motor_current_ma;
}

uint8_t Sensor_IsWaterSensorOk(void)
{
  return sensor_snapshot.water_sensor_ok;
}

uint8_t Sensor_IsTempSensorOk(void)
{
  return sensor_snapshot.temp_sensor_ok;
}
