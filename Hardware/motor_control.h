#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

#ifdef __cplusplus
extern "C" {
#endif

// 是否使用 TIM4 产生 PWM：0=不使用
#ifndef MOTOR_USE_TIM4_PWM
#define MOTOR_USE_TIM4_PWM           0U
#endif

// 是否使用 TIM1 产生 PWM：1=使用（硬件实际用的是 TIM1）
#ifndef MOTOR_USE_TIM1_PWM
#define MOTOR_USE_TIM1_PWM           1U
#endif

// ==================== 电机保护/运行参数 ====================
// 电机自动反转间隔：30000ms = 30 分钟
// 防止电机长期单方向运转导致卡死/疲劳
#ifndef MOTOR_AUTO_REVERSE_MS
#define MOTOR_AUTO_REVERSE_MS        30000UL
#endif

// 电机最大允许电流：3000mA = 3A
// 超过该电流判定过流故障，停止电机
#ifndef MOTOR_CURRENT_MAX_MA
#define MOTOR_CURRENT_MAX_MA         3000U
#endif

// ==================== 电机方向枚举 ====================
typedef enum
{
  MOTOR_DIR_FORWARD = 0,  // 电机正转（正向按摩）
  MOTOR_DIR_REVERSE = 1   // 电机反转（反向按摩）
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
