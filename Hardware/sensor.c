#include "sensor.h"
#include "adc.h"
#include "main.h"
#include <math.h>
#include <string.h>

#define SENSOR_FILTER_DIV       8U
#define SENSOR_NTC_R0_OHMS      10000.0f
#define SENSOR_NTC_BETA         3950.0f
#define SENSOR_NTC_T0_K         298.15f
#define SENSOR_NTC_PULLUP_OHMS  10000.0f

static volatile uint16_t sensor_adc_dma[SENSOR_ADC_CHANNEL_COUNT];
static uint32_t sensor_filter_acc[SENSOR_ADC_CHANNEL_COUNT];
static SensorSnapshot_t sensor_snapshot;
static uint8_t sensor_filter_ready;

static const uint32_t sensor_adc_channels[SENSOR_ADC_CHANNEL_COUNT] =
{
  ADC_CHANNEL_0,  /* PA0: W_LEVEL in the schematic table */
  ADC_CHANNEL_4,  /* PA4: AD_DCIN */
  ADC_CHANNEL_5,  /* PA5: AD_BAT */
  ADC_CHANNEL_7,  /* PA7: AD_NTC */
  ADC_CHANNEL_8,  /* PB0: AD_PUMP */
  ADC_CHANNEL_9   /* PB1: AD_MOR */
};

static const uint32_t sensor_adc_ranks[SENSOR_ADC_CHANNEL_COUNT] =
{
  ADC_REGULAR_RANK_1,
  ADC_REGULAR_RANK_2,
  ADC_REGULAR_RANK_3,
  ADC_REGULAR_RANK_4,
  ADC_REGULAR_RANK_5,
  ADC_REGULAR_RANK_6
};

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

static float Sensor_CalcWaterLiters(uint16_t mv)
{
  float liters;

  if (mv <= SENSOR_WATER_EMPTY_MV)
  {
    return 0.0f;
  }
  if (mv >= SENSOR_WATER_FULL_MV)
  {
    return (float)SENSOR_WATER_MAX_LITERS;
  }

  liters = ((float)(mv - SENSOR_WATER_EMPTY_MV) * (float)SENSOR_WATER_MAX_LITERS) /
           (float)(SENSOR_WATER_FULL_MV - SENSOR_WATER_EMPTY_MV);
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
  ntc_ohms = SENSOR_NTC_PULLUP_OHMS * voltage_ratio / (1.0f - voltage_ratio);
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

static void Sensor_ConfigAdcScan(void)
{
  ADC_ChannelConfTypeDef config;
  uint8_t index;

  HAL_ADC_Stop_DMA(&hadc1);

  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = SENSOR_ADC_CHANNEL_COUNT;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&config, 0, sizeof(config));
  config.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  for (index = 0U; index < SENSOR_ADC_CHANNEL_COUNT; index++)
  {
    config.Channel = sensor_adc_channels[index];
    config.Rank = sensor_adc_ranks[index];
    if (HAL_ADC_ConfigChannel(&hadc1, &config) != HAL_OK)
    {
      Error_Handler();
    }
  }
}

void Sensor_Init(void)
{
  memset((void *)sensor_adc_dma, 0, sizeof(sensor_adc_dma));
  memset(sensor_filter_acc, 0, sizeof(sensor_filter_acc));
  memset(&sensor_snapshot, 0, sizeof(sensor_snapshot));
  sensor_snapshot.temperature_c = SENSOR_TEMP_INVALID_C;
  sensor_filter_ready = 0U;

  HAL_GPIO_WritePin(EN_NTC_GPIO_Port, EN_NTC_Pin, GPIO_PIN_SET);
  Sensor_ConfigAdcScan();
  HAL_ADCEx_Calibration_Start(&hadc1);
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)sensor_adc_dma, SENSOR_ADC_CHANNEL_COUNT) != HAL_OK)
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

  for (index = 0U; index < SENSOR_ADC_CHANNEL_COUNT; index++)
  {
    raw = sensor_adc_dma[index];
    if (sensor_filter_ready == 0U)
    {
      sensor_filter_acc[index] = (uint32_t)raw * SENSOR_FILTER_DIV;
    }
    else
    {
      sensor_filter_acc[index] = sensor_filter_acc[index] - (sensor_filter_acc[index] / SENSOR_FILTER_DIV) + raw;
    }

    sensor_snapshot.raw[index] = (uint16_t)(sensor_filter_acc[index] / SENSOR_FILTER_DIV);
    mv = Sensor_RawToMv(sensor_snapshot.raw[index]);
    sensor_snapshot.millivolt[index] = mv;
  }

  sensor_filter_ready = 1U;
  sensor_snapshot.water_liters = Sensor_CalcWaterLiters(sensor_snapshot.millivolt[SENSOR_ADC_WATER_LEVEL]);
  sensor_snapshot.battery_decivolt = Sensor_CalcScaledDecivolt(sensor_snapshot.millivolt[SENSOR_ADC_BATTERY_VOLT],
                                                               SENSOR_BAT_DIVIDER_X100);
  sensor_snapshot.dcin_decivolt = Sensor_CalcScaledDecivolt(sensor_snapshot.millivolt[SENSOR_ADC_DCIN_VOLT],
                                                            SENSOR_DCIN_DIVIDER_X100);
  sensor_snapshot.pump_current_ma = Sensor_CalcCurrentMa(sensor_snapshot.millivolt[SENSOR_ADC_PUMP_CURRENT]);
  sensor_snapshot.motor_current_ma = Sensor_CalcCurrentMa(sensor_snapshot.millivolt[SENSOR_ADC_MOTOR_CURRENT]);

  temp_c = Sensor_CalcNtcTemperature(sensor_snapshot.raw[SENSOR_ADC_NTC_TEMP]);
  sensor_snapshot.temperature_c = temp_c;
  sensor_snapshot.temp_sensor_ok = ((temp_c > 0.0f) && (temp_c < 85.0f)) ? 1U : 0U;
  sensor_snapshot.water_sensor_ok =
      ((sensor_snapshot.raw[SENSOR_ADC_WATER_LEVEL] > 5U) && (sensor_snapshot.raw[SENSOR_ADC_WATER_LEVEL] < 4090U)) ? 1U : 0U;
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
