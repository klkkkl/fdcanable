#include "fdcan_slcan.h"
#include "main.h"
#include "usbd_cdc_if.h"

extern USB_TxBuffer_t g_usb_tx_fifo;
extern volatile uint8_t usb_tx_busy;

void Process_USB_TX_Pump(void) {
  if (usb_tx_busy)
    return;
  uint32_t h = g_usb_tx_fifo.head, t = g_usb_tx_fifo.tail;
  if (h == t)
    return;

  static uint8_t pkt[64];
  uint32_t len = (h > t) ? (h - t) : (TX_BUF_SIZE - t);
  if (len > 64)
    len = 64;

  memcpy(pkt, &g_usb_tx_fifo.buffer[t], len);
  usb_tx_busy = 1;
  if (CDC_Transmit_FS(pkt, len) != USBD_OK)
    usb_tx_busy = 0;
  else
    g_usb_tx_fifo.tail = (t + len) % TX_BUF_SIZE;
}
void AppRun() {
  uint32_t last_work_led_tick = 0;
  uint32_t last_state_led_tick = 0;
  while (1) {
    Process_USB_TX_Pump();

    // 每 50ms 强制熄灭 WORK 灯，产生清晰闪烁感
    uint32_t current_tick = HAL_GetTick();
    if (current_tick - last_work_led_tick > 50) {
      LED_WORK_OFF();
      last_work_led_tick = current_tick;
    }

    if (current_tick - last_state_led_tick > 500) {
      LED_STATE_TOGGLE();
      last_state_led_tick = current_tick;
    }
  }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo0ITs) {
  FDCAN_RxHeaderTypeDef header;
  uint8_t data[64];
  if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &header, data) == HAL_OK) {
    SLCAN_FormatResponse_Fast(&header, data);
  }
}