#ifndef SENSOR_H
#define SENSOR_H

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SENSOR_ADC_CHANNEL_COUNT        5U
#define SENSOR_WATER_MAX_LITERS         12U
#define SENSOR_WATER_MIN_SAFE_LITERS    1U
#define SENSOR_TEMP_INVALID_C           (-100.0f)

#ifndef SENSOR_ADC_VREF_MV
#define SENSOR_ADC_VREF_MV              3300U
#endif

#ifndef SENSOR_WATER_EMPTY_COUNT
#define SENSOR_WATER_EMPTY_COUNT        26700U
#endif

#ifndef SENSOR_WATER_FULL_COUNT
#define SENSOR_WATER_FULL_COUNT         25800U
#endif

#ifndef SENSOR_WATER_COUNT_WINDOW_MS
#define SENSOR_WATER_COUNT_WINDOW_MS    1000U
#endif

#ifndef SENSOR_WATER_COUNT_MIN_VALID
#define SENSOR_WATER_COUNT_MIN_VALID    1000U
#endif

#ifndef SENSOR_WATER_COUNT_MAX_VALID
#define SENSOR_WATER_COUNT_MAX_VALID    40000U
#endif

#ifndef SENSOR_WATER_COUNT_TIMEOUT_MS
#define SENSOR_WATER_COUNT_TIMEOUT_MS   2000U
#endif

#ifndef SENSOR_BAT_DIVIDER_X100
#define SENSOR_BAT_DIVIDER_X100         1100U
#endif

#ifndef SENSOR_DCIN_DIVIDER_X100
#define SENSOR_DCIN_DIVIDER_X100        1100U
#endif

#ifndef SENSOR_CURRENT_ZERO_MV
#define SENSOR_CURRENT_ZERO_MV          0U
#endif

#ifndef SENSOR_CURRENT_MA_PER_MV_X10
#define SENSOR_CURRENT_MA_PER_MV_X10    10U
#endif

typedef enum
{
  SENSOR_ADC_DCIN_VOLT = 0,
  SENSOR_ADC_BATTERY_VOLT,
  SENSOR_ADC_NTC_TEMP,
  SENSOR_ADC_PUMP_CURRENT,
  SENSOR_ADC_MOTOR_CURRENT
} SensorAdcChannel_t;

typedef struct
{
  uint16_t raw[SENSOR_ADC_CHANNEL_COUNT];
  uint16_t millivolt[SENSOR_ADC_CHANNEL_COUNT];
  float water_liters;
  uint32_t water_frequency_hz;
  float temperature_c;
  uint16_t battery_decivolt;
  uint16_t dcin_decivolt;
  uint16_t pump_current_ma;
  uint16_t motor_current_ma;
  uint8_t water_sensor_ok;
  uint8_t temp_sensor_ok;
} SensorSnapshot_t;

void Sensor_Init(void);
void Sensor_TaskProcess(void);
void Sensor_GetSnapshot(SensorSnapshot_t *snapshot);

uint16_t Sensor_GetRaw(SensorAdcChannel_t channel);
uint16_t Sensor_GetMilliVolt(SensorAdcChannel_t channel);
float Sensor_GetWaterLiters(void);
uint8_t Sensor_GetWaterLevelProtocol(void);
uint32_t Sensor_GetWaterFrequencyHz(void);
float Sensor_GetTemperatureC(void);
int16_t Sensor_GetTemperatureCx10(void);
uint16_t Sensor_GetBatteryDeciVolt(void);
uint16_t Sensor_GetDcinDeciVolt(void);
uint16_t Sensor_GetPumpCurrentMa(void);
uint16_t Sensor_GetMotorCurrentMa(void);
uint8_t Sensor_IsWaterSensorOk(void);
uint8_t Sensor_IsTempSensorOk(void);

#ifdef __cplusplus
}
#endif

#endif
