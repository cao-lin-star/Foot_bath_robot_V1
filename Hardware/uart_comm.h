#ifndef UART_COMM_H
#define UART_COMM_H

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

#ifdef __cplusplus
extern "C" {
#endif

#define UART_COMM_FRAME_LEN          30U
#define UART_COMM_HEAD1              0x55U
#define UART_COMM_HEAD2              0xAAU
#define UART_COMM_LINK_MODE_BUCKET   0x01U
#define UART_COMM_LINK_MODE_TRANSIT  0x02U
#define UART_COMM_LINK_MODE_DIRECT   0x03U
#define UART_COMM_DEFAULT_LINK_MODE  UART_COMM_LINK_MODE_BUCKET

typedef enum
{
  UART_COMM_PORT_LINUX = 0,
  UART_COMM_PORT_BASE = 1
} UartCommPort_t;

void UART_Comm_Init(void);
void UART_Comm_TaskProcess(void);
void UART_Comm_RxEventCallback(UART_HandleTypeDef *huart, uint16_t pos);
void UART_Comm_TxCpltCallback(UART_HandleTypeDef *huart);
void UART_Comm_ErrorCallback(UART_HandleTypeDef *huart);
void UART_ParseFrame(uint8_t *data, uint8_t len);
uint8_t UART_Comm_BuildStatusFrame(uint8_t *frame);
uint8_t UART_Comm_IsFrameValid(const uint8_t *frame);
uint8_t UART_Comm_Checksum(const uint8_t *frame);
uint8_t UART_Comm_IsBaseConnected(void);

#ifdef __cplusplus
}
#endif

#endif
