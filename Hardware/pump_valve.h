#ifndef PUMP_VALVE_H
#define PUMP_VALVE_H

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef PUMP_CURRENT_MAX_MA
#define PUMP_CURRENT_MAX_MA          3000U
#endif

#ifndef PUMP_VALVE_DRAIN_TIMEOUT_MS
#define PUMP_VALVE_DRAIN_TIMEOUT_MS  (5UL * 60UL * 1000UL)
#endif

typedef enum
{
  PUMP_VALVE_MODE_OFF = 0,
  PUMP_VALVE_MODE_CIRCULATION = 1,
  PUMP_VALVE_MODE_DRAIN = 2
} PumpValveMode_t;

void PumpValve_Init(void);
void PumpValve_SetMode(PumpValveMode_t mode);
PumpValveMode_t PumpValve_GetMode(void);
void PumpValve_TaskProcess(void);
uint8_t PumpValve_IsPumpOn(void);
uint8_t PumpValve_HasFault(void);
void PumpValve_ClearFault(void);

void Pump_Start(void);
void Pump_Stop(void);
void Valve_Set(uint8_t mode);

#ifdef __cplusplus
}
#endif

#endif
