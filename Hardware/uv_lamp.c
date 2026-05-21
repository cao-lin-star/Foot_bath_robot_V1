#include "uv_lamp.h"
#include "main.h"
#include "sensor.h"

static uint8_t uv_enabled;
static uint8_t uv_fault;

void UV_Init(void)
{
  uv_enabled = 0U;
  uv_fault = 0U;
  HAL_GPIO_WritePin(EN_UV_GPIO_Port, EN_UV_Pin, GPIO_PIN_RESET);
}

void UV_On(void)
{
  uv_enabled = 1U;
  HAL_GPIO_WritePin(EN_UV_GPIO_Port, EN_UV_Pin, GPIO_PIN_SET);
}

void UV_Off(void)
{
  uv_enabled = 0U;
  HAL_GPIO_WritePin(EN_UV_GPIO_Port, EN_UV_Pin, GPIO_PIN_RESET);
}

void UV_Set(uint8_t enable)
{
  if (enable != 0U)
  {
    UV_On();
  }
  else
  {
    UV_Off();
  }
}

uint8_t UV_IsOn(void)
{
  return uv_enabled;
}

uint8_t UV_HasFault(void)
{
  return uv_fault;
}

void UV_ClearFault(void)
{
  uv_fault = 0U;
}

void UV_TaskProcess(void)
{
  if ((uv_enabled != 0U) && (Sensor_GetWaterLevelProtocol() < SENSOR_WATER_MIN_SAFE_LITERS))
  {
    uv_fault = 1U;
    UV_Off();
  }
}
