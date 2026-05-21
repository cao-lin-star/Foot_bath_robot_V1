#ifndef UV_LAMP_H
#define UV_LAMP_H

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

#ifdef __cplusplus
extern "C" {
#endif

void UV_Init(void);
void UV_On(void);
void UV_Off(void);
void UV_Set(uint8_t enable);
uint8_t UV_IsOn(void);
uint8_t UV_HasFault(void);
void UV_ClearFault(void);
void UV_TaskProcess(void);

#ifdef __cplusplus
}
#endif

#endif
