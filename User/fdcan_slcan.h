#ifndef __FDCAN_SLCAN_H
#define __FDCAN_SLCAN_H

#include "main.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/* 配置 */
#define TX_BUF_SIZE 4096      // CAN -> USB 环形缓冲区
#define SLCAN_CMD_MAX_LEN 128 // USB -> CAN 命令缓冲区

/* LED 控制宏 (假设 CubeMX 中已命名为 LED_STATE 和 LED_WORK) */
#define LED_STATE_ON()                                                         \
  HAL_GPIO_WritePin(LED_STATE_GPIO_Port, LED_STATE_Pin, GPIO_PIN_SET)
#define LED_STATE_OFF()                                                        \
  HAL_GPIO_WritePin(LED_STATE_GPIO_Port, LED_STATE_Pin, GPIO_PIN_RESET)
#define LED_STATE_TOGGLE()                                                     \
  HAL_GPIO_TogglePin(LED_STATE_GPIO_Port, LED_STATE_Pin)
#define LED_WORK_TOGGLE() HAL_GPIO_TogglePin(LED_WORK_GPIO_Port, LED_WORK_Pin)
#define LED_WORK_OFF()                                                         \
  HAL_GPIO_WritePin(LED_WORK_GPIO_Port, LED_WORK_Pin, GPIO_PIN_RESET)

/* 错误码 */
typedef enum {
  CAN_OK = 0,
  CAN_ERR_PARAM_INVALID = -1,
  CAN_ERR_HAL_INIT = -2,
  CAN_ERR_HAL_START = -3,
  CAN_ERR_CLOCK = -4
} CAN_Status_t;

/* 缓冲区结构 */
typedef struct {
  uint8_t buffer[TX_BUF_SIZE];
  volatile uint32_t head;
  volatile uint32_t tail;
} USB_TxBuffer_t;

/* 外部调用接口 */
CAN_Status_t CanOpen(void);
void CanClose(void);
CAN_Status_t SLCAN_ProcessCommand(char *cmd, char *response);
void SLCAN_FormatResponse_Fast(FDCAN_RxHeaderTypeDef *RxHeader,
                               uint8_t *RxData);
void Process_USB_TX_Pump(void);
void USB_TxBuf_WriteString(const char *s);

#endif