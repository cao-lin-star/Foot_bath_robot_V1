#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MOTOR_USE_TIM4_PWM
#define MOTOR_USE_TIM4_PWM           1U
#endif

#ifndef MOTOR_USE_TIM1_PWM
#define MOTOR_USE_TIM1_PWM           0U
#endif

#ifndef MOTOR_AUTO_REVERSE_MS
#define MOTOR_AUTO_REVERSE_MS        30000UL
#endif

#ifndef MOTOR_CURRENT_MAX_MA
#define MOTOR_CURRENT_MAX_MA         3000U
#endif

typedef enum
{
  MOTOR_DIR_FORWARD = 0,
  MOTOR_DIR_REVERSE = 1
} MotorDirection_t;

void Motor_Init(void);
void Motor_Run(uint8_t speed, uint8_t direction);
void Motor_Stop(void);
void Motor_SetLevel(uint8_t level);
void Motor_SetDirection(MotorDirection_t direction);
void Motor_SetAutoReverse(uint8_t enable, uint32_t interval_ms);
uint8_t Motor_GetLevel(void);
uint8_t Motor_GetDutyPercent(void);
MotorDirection_t Motor_GetDirection(void);
uint8_t Motor_IsRunning(void);
uint8_t Motor_HasFault(void);
void Motor_ClearFault(void);
void Motor_TaskProcess(void);

#ifdef __cplusplus
}
#endif

#endif
