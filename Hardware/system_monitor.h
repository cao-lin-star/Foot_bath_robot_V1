#ifndef SYSTEM_MONITOR_H
#define SYSTEM_MONITOR_H

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BUCKET_ERR1_LACK_WATER       0x01U
#define BUCKET_ERR1_DRAIN_TIMEOUT    0x02U
#define BUCKET_ERR1_TEMP_ABNORMAL    0x04U
#define BUCKET_ERR1_HEAT_TIMEOUT     0x08U
#define BUCKET_ERR1_TEMP_SENSOR      0x10U
#define BUCKET_ERR1_WATER_SENSOR     0x20U

#define BUCKET_ERR2_HEAT_MODULE      0x01U
#define BUCKET_ERR2_PUMP             0x02U
#define BUCKET_ERR2_VALVE            0x04U
#define BUCKET_ERR2_MOTOR            0x08U
#define BUCKET_ERR2_UV               0x10U
#define BUCKET_ERR2_BATTERY          0x20U

typedef enum
{
  BUCKET_STATUS_POWER_ON = 0,
  BUCKET_STATUS_SELF_CHECK = 1,
  BUCKET_STATUS_OFF = 2,
  BUCKET_STATUS_STANDBY = 3,
  BUCKET_STATUS_RUNNING = 4,
  BUCKET_STATUS_LOW_POWER = 5
} BucketMainStatus_t;

void SystemMonitor_Init(void);
void SystemMonitor_TaskProcess(void);
void SystemMonitor_StopAllOutputs(void);
void SystemMonitor_SetMainStatus(uint8_t main_status, uint8_t sub_status);
uint8_t SystemMonitor_GetMainStatus(void);
uint8_t SystemMonitor_GetSubStatus(void);
uint8_t SystemMonitor_GetErrCode1(void);
uint8_t SystemMonitor_GetErrCode2(void);
void SystemMonitor_SetCommand(uint8_t cmd);
uint8_t SystemMonitor_GetCommand(void);
void SystemMonitor_SetLinkStatus(uint8_t link_status);
uint8_t SystemMonitor_GetLinkStatus(void);
void SystemMonitor_SetBathTimer(uint8_t timer_code);
uint8_t SystemMonitor_GetTimerRemainingMin(void);
void SystemMonitor_RequestReset(void);
uint8_t SystemMonitor_IsResetRequested(void);
void SystemMonitor_ClearErrors(void);

#ifdef __cplusplus
}
#endif

#endif
