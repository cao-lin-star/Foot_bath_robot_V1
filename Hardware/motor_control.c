#include "motor_control.h"
#include "sensor.h"
#include "tim.h"

static uint8_t motor_level;
static uint8_t motor_manual_duty;
static uint8_t motor_running;
static uint8_t motor_fault;
static uint8_t motor_auto_reverse;
static uint32_t motor_reverse_interval_ms = MOTOR_AUTO_REVERSE_MS;
static uint32_t motor_last_reverse_tick;
static MotorDirection_t motor_direction = MOTOR_DIR_FORWARD;

static uint8_t Motor_LevelToDuty(uint8_t level)
{
  switch (level)
  {
    case 1U:
      return 35U;
    case 2U:
      return 60U;
    case 3U:
      return 85U;
    default:
      return 0U;
  }
}

static uint32_t Motor_DutyToPulse(TIM_HandleTypeDef *htim, uint8_t duty_percent)
{
  uint32_t period;
  uint32_t pulse;

  period = htim->Init.Period;
  if (duty_percent > 100U)
  {
    duty_percent = 100U;
  }
  pulse = ((period + 1U) * duty_percent) / 100U;
  if (pulse > period)
  {
    pulse = period;
  }
  return pulse;
}

static void Motor_SetTimerChannel(TIM_HandleTypeDef *htim, uint32_t channel, uint8_t duty_percent)
{
  __HAL_TIM_SET_COMPARE(htim, channel, Motor_DutyToPulse(htim, duty_percent));
}

static void Motor_ApplyPwm(uint8_t duty_percent, MotorDirection_t direction)
{
#if MOTOR_USE_TIM4_PWM
  if (direction == MOTOR_DIR_FORWARD)
  {
    Motor_SetTimerChannel(&htim4, TIM_CHANNEL_1, duty_percent);
    Motor_SetTimerChannel(&htim4, TIM_CHANNEL_2, 0U);
    Motor_SetTimerChannel(&htim4, TIM_CHANNEL_3, duty_percent);
    Motor_SetTimerChannel(&htim4, TIM_CHANNEL_4, 0U);
  }
  else
  {
    Motor_SetTimerChannel(&htim4, TIM_CHANNEL_1, 0U);
    Motor_SetTimerChannel(&htim4, TIM_CHANNEL_2, duty_percent);
    Motor_SetTimerChannel(&htim4, TIM_CHANNEL_3, 0U);
    Motor_SetTimerChannel(&htim4, TIM_CHANNEL_4, duty_percent);
  }
#endif

#if MOTOR_USE_TIM1_PWM
  if (direction == MOTOR_DIR_FORWARD)
  {
    Motor_SetTimerChannel(&htim1, TIM_CHANNEL_1, duty_percent);
    Motor_SetTimerChannel(&htim1, TIM_CHANNEL_4, 0U);
  }
  else
  {
    Motor_SetTimerChannel(&htim1, TIM_CHANNEL_1, 0U);
    Motor_SetTimerChannel(&htim1, TIM_CHANNEL_4, duty_percent);
  }
#endif
}

void Motor_Init(void)
{
#if MOTOR_USE_TIM4_PWM
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
#endif

#if MOTOR_USE_TIM1_PWM
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
#endif

  motor_level = 0U;
  motor_manual_duty = 0U;
  motor_running = 0U;
  motor_fault = 0U;
  motor_auto_reverse = 0U;
  motor_direction = MOTOR_DIR_FORWARD;
  motor_last_reverse_tick = HAL_GetTick();
  Motor_ApplyPwm(0U, motor_direction);
}

void Motor_Run(uint8_t speed, uint8_t direction)
{
  motor_direction = (direction != 0U) ? MOTOR_DIR_REVERSE : MOTOR_DIR_FORWARD;
  if (speed <= 3U)
  {
    Motor_SetLevel(speed);
  }
  else
  {
    if (speed > 100U)
    {
      speed = 100U;
    }
    motor_level = 1U;
    motor_manual_duty = speed;
    motor_running = (speed != 0U) ? 1U : 0U;
  }
}

void Motor_Stop(void)
{
  motor_level = 0U;
  motor_manual_duty = 0U;
  motor_running = 0U;
  Motor_ApplyPwm(0U, motor_direction);
}

void Motor_SetLevel(uint8_t level)
{
  if (level > 3U)
  {
    level = 3U;
  }
  motor_level = level;
  motor_manual_duty = 0U;
  motor_running = (level != 0U) ? 1U : 0U;
  if (motor_running == 0U)
  {
    Motor_ApplyPwm(0U, motor_direction);
  }
}

void Motor_SetDirection(MotorDirection_t direction)
{
  motor_direction = direction;
  motor_last_reverse_tick = HAL_GetTick();
}

void Motor_SetAutoReverse(uint8_t enable, uint32_t interval_ms)
{
  motor_auto_reverse = (enable != 0U) ? 1U : 0U;
  if (interval_ms >= 5000UL)
  {
    motor_reverse_interval_ms = interval_ms;
  }
  motor_last_reverse_tick = HAL_GetTick();
}

uint8_t Motor_GetLevel(void)
{
  return motor_level;
}

uint8_t Motor_GetDutyPercent(void)
{
  if (motor_manual_duty != 0U)
  {
    return motor_manual_duty;
  }
  return Motor_LevelToDuty(motor_level);
}

MotorDirection_t Motor_GetDirection(void)
{
  return motor_direction;
}

uint8_t Motor_IsRunning(void)
{
  return motor_running;
}

uint8_t Motor_HasFault(void)
{
  return motor_fault;
}

void Motor_ClearFault(void)
{
  motor_fault = 0U;
}

void Motor_TaskProcess(void)
{
  uint32_t now;
  uint16_t current_ma;
  uint8_t duty;

  now = HAL_GetTick();
  if ((motor_running != 0U) &&
      (motor_auto_reverse != 0U) &&
      ((now - motor_last_reverse_tick) >= motor_reverse_interval_ms))
  {
    motor_direction = (motor_direction == MOTOR_DIR_FORWARD) ? MOTOR_DIR_REVERSE : MOTOR_DIR_FORWARD;
    motor_last_reverse_tick = now;
  }

  current_ma = Sensor_GetMotorCurrentMa();
  if ((motor_running != 0U) && (current_ma > MOTOR_CURRENT_MAX_MA))
  {
    motor_fault = 1U;
    Motor_Stop();
    return;
  }

  duty = Motor_GetDutyPercent();
  if (motor_running != 0U)
  {
    Motor_ApplyPwm(duty, motor_direction);
  }
  else
  {
    Motor_ApplyPwm(0U, motor_direction);
  }
}
