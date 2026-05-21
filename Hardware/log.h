#ifndef LOGGING_H
#define LOGGING_H

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

#ifdef __cplusplus
extern "C" {
#endif

void Logging_Init(void);
void Logging_Print(const char *msg);
void Logging_Printf(const char *fmt, ...);
void Logging_TaskProcess(void);
void Logging_TxCpltCallback(UART_HandleTypeDef *huart);
void Logging_ErrorCallback(UART_HandleTypeDef *huart);
HAL_StatusTypeDef Logging_GetLastStatus(void);

#ifdef __cplusplus
}
#endif

#endif
