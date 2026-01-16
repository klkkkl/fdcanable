#ifndef __FDCAN_SLCAN_H
#define __FDCAN_SLCAN_H

#include "main.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 配置参数 ==================== */
#define TX_BUF_SIZE 4096U      /* CAN -> USB 环形缓冲区大小 */
#define SLCAN_CMD_MAX_LEN 150U /* USB -> CAN 命令最大长度 */

/* ==================== LED 控制宏 ==================== */
/* 假设 CubeMX 中已命名为 LED_STATE 和 LED_WORK */
#define LED_STATE_ON()                                                         \
  HAL_GPIO_WritePin(LED_STATE_GPIO_Port, LED_STATE_Pin, GPIO_PIN_SET)
#define LED_STATE_OFF()                                                        \
  HAL_GPIO_WritePin(LED_STATE_GPIO_Port, LED_STATE_Pin, GPIO_PIN_RESET)
#define LED_STATE_TOGGLE()                                                     \
  HAL_GPIO_TogglePin(LED_STATE_GPIO_Port, LED_STATE_Pin)

#define LED_WORK_ON()                                                          \
  HAL_GPIO_WritePin(LED_WORK_GPIO_Port, LED_WORK_Pin, GPIO_PIN_SET)
#define LED_WORK_OFF()                                                         \
  HAL_GPIO_WritePin(LED_WORK_GPIO_Port, LED_WORK_Pin, GPIO_PIN_RESET)
#define LED_WORK_TOGGLE() HAL_GPIO_TogglePin(LED_WORK_GPIO_Port, LED_WORK_Pin)

/* ==================== 错误码定义 ==================== */
typedef enum {
  CAN_OK = 0,                 /* 操作成功 */
  CAN_ERR_PARAM_INVALID = -1, /* 参数无效 */
  CAN_ERR_HAL_INIT = -2,      /* HAL 初始化失败 */
  CAN_ERR_HAL_START = -3,     /* HAL 启动失败 */
  CAN_ERR_CLOCK = -4,         /* 时钟配置错误 */
  CAN_ERR_TX_FULL = -5,       /* 发送缓冲区满 */
  CAN_ERR_NOT_OPEN = -6       /* 通道未打开 */
} CAN_Status_t;

/* ==================== 数据结构 ==================== */

/**
 * @brief USB 发送环形缓冲区
 * @note  head: 写入位置，由生产者（CAN RX）更新
 *        tail: 读取位置，由消费者（USB TX）更新
 */
typedef struct {
  uint8_t buffer[TX_BUF_SIZE];
  volatile uint32_t head;
  volatile uint32_t tail;
} USB_TxBuffer_t;

/* ==================== 外部函数接口 ==================== */

/**
 * @brief 打开 CAN 通道
 * @return CAN_OK 成功，其他值为错误码
 */
CAN_Status_t CanOpen(void);

/**
 * @brief 关闭 CAN 通道
 */
void CanClose(void);

/**
 * @brief 处理 SLCAN 命令
 * @param cmd 输入命令字符串
 * @param response 输出响应字符串
 * @return CAN_OK 成功，其他值为错误码
 */
CAN_Status_t SLCAN_ProcessCommand(char *cmd, char *response);

/**
 * @brief 快速格式化 CAN 接收数据为 SLCAN 响应
 * @param RxHeader CAN 接收头
 * @param RxData 接收数据
 */
void SLCAN_FormatResponse_Fast(FDCAN_RxHeaderTypeDef *RxHeader,
                               uint8_t *RxData);

/**
 * @brief USB 发送数据泵
 * @note 在主循环中调用
 */
void Process_USB_TX_Pump(void);

/**
 * @brief 向 USB 发送缓冲区写入字符串
 * @param s 要发送的字符串
 */
void USB_TxBuf_WriteString(const char *s);

#ifdef __cplusplus
}
#endif

#endif /* __FDCAN_SLCAN_H */