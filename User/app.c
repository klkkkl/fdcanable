#include "fdcan_slcan.h"
#include "main.h"
#include "stm32g4xx_hal.h"
#include "usbd_cdc_if.h"

extern USB_TxBuffer_t g_usb_tx_fifo;
extern volatile uint8_t usb_tx_busy;

/* USB 最大包大小 (Full Speed CDC) */
#define USB_TX_PKT_SIZE 64
#define WORK_LED_HOLD_MS 50U
#define STATE_LED_BLINK_MS 50U

static volatile uint32_t s_work_led_deadline = 0;
static volatile uint32_t s_state_led_deadline = 0;
static volatile uint8_t s_can_online = 0;

static uint32_t USB_TxBuf_GetContiguousLength(uint32_t head, uint32_t tail) {
  if (head > tail) {
    return head - tail;
  }

  return TX_BUF_SIZE - tail;
}

static void App_UpdateLeds(uint32_t current_tick) {
  if ((int32_t)(current_tick - s_work_led_deadline) >= 0) {
    LED_WORK_OFF();
  } else {
    LED_WORK_ON();
  }

  if (!s_can_online) {
    LED_STATE_OFF();
  } else if ((int32_t)(current_tick - s_state_led_deadline) < 0) {
    LED_STATE_OFF();
  } else {
    LED_STATE_ON();
  }
}

void App_NotifyUsbActivity(void) {
  s_work_led_deadline = HAL_GetTick() + WORK_LED_HOLD_MS;
  LED_WORK_ON();
}

void App_NotifyCanRxActivity(void) {
  if (!s_can_online) {
    return;
  }

  s_state_led_deadline = HAL_GetTick() + STATE_LED_BLINK_MS;
  LED_STATE_OFF();
}

void App_SetCanOnline(uint8_t online) {
  s_can_online = online ? 1U : 0U;
  s_state_led_deadline = 0U;

  if (s_can_online) {
    LED_STATE_ON();
  } else {
    LED_STATE_OFF();
  }
}

/**
 * @brief USB TX 数据泵，处理 CAN->USB 数据传输
 * @note  从环形缓冲区读取数据并通过 USB CDC 发送
 *        SPSC 无锁设计：只修改 tail，读取 volatile head
 *        usb_tx_busy 标志防止重入，确保 pkt 缓冲区不被覆盖
 *        tail 在此处更新是安全的（CDC_Transmit_FS 成功后，usb_tx_busy
 *        保护确保下次调用前数据已发送完成）
 */
void Process_USB_TX_Pump(void) {
  /* 检查 USB 发送状态 */
  if (usb_tx_busy) {
    return;
  }

  /* 读取缓冲区指针（使用局部变量避免多次访问 volatile） */
  const uint32_t head = g_usb_tx_fifo.head;
  const uint32_t tail = g_usb_tx_fifo.tail;

  /* 缓冲区为空 */
  if (head == tail) {
    return;
  }

  static uint8_t pkt[USB_TX_PKT_SIZE];
  uint32_t len = USB_TxBuf_GetContiguousLength(head, tail);

  /* 限制单次传输大小 */
  if (len > USB_TX_PKT_SIZE) {
    len = USB_TX_PKT_SIZE;
  }

  /* 复制数据到发送缓冲区 */
  memcpy(pkt, &g_usb_tx_fifo.buffer[tail], len);

  /* 标记发送中并启动传输 */
  usb_tx_busy = 1;
  if (CDC_Transmit_FS(pkt, (uint16_t)len) != USBD_OK) {
    usb_tx_busy = 0; /* 发送失败，重置标志 */
  } else {
    /* 更新尾指针（usb_tx_busy 保护确保安全） */
    g_usb_tx_fifo.tail = (tail + len) & TX_BUF_MASK;
  }
}

/**
 * @brief 应用程序主循环
 */
void AppRun(void) {
  App_SetCanOnline(0);
  LED_WORK_OFF();

  while (1) {
    /* 处理 USB 发送队列 */
    Process_USB_TX_Pump();

    /* Bus-Off 延迟恢复 */
    FDCAN_PollBusOffRecovery();

    /* 获取当前系统时钟 */
    const uint32_t current_tick = HAL_GetTick();
    App_UpdateLeds(current_tick);
  }
}
