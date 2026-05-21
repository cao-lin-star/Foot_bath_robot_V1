#ifndef TEMP_CONTROL_H
#define TEMP_CONTROL_H

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef TEMP_TARGET_DEFAULT_C
#define TEMP_TARGET_DEFAULT_C       40.0f
#endif

#ifndef TEMP_TARGET_MIN_C
#define TEMP_TARGET_MIN_C           35.0f
#endif

#ifndef TEMP_TARGET_MAX_C
#define TEMP_TARGET_MAX_C           48.0f
#endif

#ifndef TEMP_HYSTERESIS_LOW_C
#define TEMP_HYSTERESIS_LOW_C       0.5f
#endif

#ifndef TEMP_HYSTERESIS_HIGH_C
#define TEMP_HYSTERESIS_HIGH_C      0.3f
#endif

#ifndef TEMP_HIGH_CUTOFF_C
#define TEMP_HIGH_CUTOFF_C          55.0f
#endif

#ifndef TEMP_HEAT_TIMEOUT_MS
#define TEMP_HEAT_TIMEOUT_MS        (15UL * 60UL * 1000UL)
#endif

void Temp_Init(void);
void Temp_Set(float target);
void Temp_SetTargetC(float target);
float Temp_GetTargetC(void);
float Temp_Read(void);
void Temp_Enable(uint8_t enable);
uint8_t Temp_IsEnabled(void);
uint8_t Temp_IsHeating(void);
uint8_t Temp_HasFault(void);
void Temp_ClearFault(void);
void Temp_Control_TaskProcess(void);

#ifdef __cplusplus
}
#endif

#endif
