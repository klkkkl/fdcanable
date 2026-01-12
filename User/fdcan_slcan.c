#include "fdcan_slcan.h"
#include "usbd_cdc_if.h"

extern FDCAN_HandleTypeDef hfdcan1;
USB_TxBuffer_t g_usb_tx_fifo = {0};
extern volatile uint8_t usb_tx_busy;
static const char hex_table[] = "0123456789ABCDEF";

typedef struct {
  uint32_t nominalBitrate;
  uint32_t dataBitrate;
  uint8_t isDataBitrateSet;
} FDCAN_Config_t;

static FDCAN_Config_t mCfg = {.nominalBitrate = 500000, .isDataBitrateSet = 0};

/* 辅助函数 */
static uint8_t Hex2Int(char c) {
  if (c >= '0' && c <= '9')
    return c - '0';
  if (c >= 'A' && c <= 'F')
    return c - 'A' + 10;
  if (c >= 'a' && c <= 'f')
    return c - 'a' + 10;
  return 0;
}

void USB_TxBuf_WriteString(const char *s) {
  while (*s) {
    uint32_t next = (g_usb_tx_fifo.head + 1) % TX_BUF_SIZE;
    if (next != g_usb_tx_fifo.tail) {
      g_usb_tx_fifo.buffer[g_usb_tx_fifo.head] = *s++;
      g_usb_tx_fifo.head = next;
    } else
      break;
  }
}

/* 快速格式化 CAN 报文为 SLCAN 字符串 (查表法替代 sprintf) */
void SLCAN_FormatResponse_Fast(FDCAN_RxHeaderTypeDef *RxHeader,
                               uint8_t *RxData) {
  char msg[150], *p = msg;
  *p++ = (RxHeader->RxFrameType == FDCAN_REMOTE_FRAME)
             ? ((RxHeader->IdType == FDCAN_EXTENDED_ID) ? 'R' : 'r')
             : ((RxHeader->IdType == FDCAN_EXTENDED_ID) ? 'T' : 't');

  uint32_t id = RxHeader->Identifier;
  int idLen = (RxHeader->IdType == FDCAN_EXTENDED_ID) ? 8 : 3;
  for (int i = idLen - 1; i >= 0; i--)
    p[i] = hex_table[(id >> ((idLen - 1 - i) * 4)) & 0x0F];
  p += idLen;

  static const uint8_t dlc2len[] = {0, 1,  2,  3,  4,  5,  6,  7,
                                    8, 12, 16, 20, 24, 32, 48, 64};
  uint8_t len = dlc2len[RxHeader->DataLength >> 16];
  *p++ = hex_table[len & 0x0F];

  if (RxHeader->RxFrameType == FDCAN_DATA_FRAME) {
    for (int i = 0; i < len; i++) {
      *p++ = hex_table[RxData[i] >> 4];
      *p++ = hex_table[RxData[i] & 0x0F];
    }
  }
  *p++ = '\r';
  *p = '\0';
  USB_TxBuf_WriteString(msg);
  LED_WORK_TOGGLE();
}

/* 自动计算位时间 (TQ) */
static CAN_Status_t Internal_CalculateTiming(uint32_t clk, uint32_t baud,
                                             uint32_t *presc, uint32_t *s1,
                                             uint32_t *s2) {
  for (uint32_t p = 1; p <= 512; p++) {
    if (clk % (baud * p) == 0) {
      uint32_t totalTQ = clk / (baud * p);
      if (totalTQ >= 8 && totalTQ <= 80) {
        *presc = p;
        *s1 = (totalTQ * 800 / 1000) - 1;
        *s2 = totalTQ - 1 - (*s1);
        (*s1)--; // 简化的 80% 采样点计算
        return CAN_OK;
      }
    }
  }
  return CAN_ERR_CLOCK;
}

/* 打开 CAN 通道 */
CAN_Status_t CanOpen(void) {
  if (hfdcan1.State != HAL_FDCAN_STATE_RESET)
    HAL_FDCAN_Stop(&hfdcan1);
  uint32_t p, s1, s2, clk = HAL_RCC_GetPCLK1Freq();
  if (Internal_CalculateTiming(clk, mCfg.nominalBitrate, &p, &s1, &s2) !=
      CAN_OK)
    return CAN_ERR_CLOCK;
  hfdcan1.Init.NominalPrescaler = p;
  hfdcan1.Init.NominalTimeSeg1 = s1;
  hfdcan1.Init.NominalTimeSeg2 = s2;
  if (mCfg.isDataBitrateSet) {
    Internal_CalculateTiming(clk, mCfg.dataBitrate, &p, &s1, &s2);
    hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
    hfdcan1.Init.DataPrescaler = p;
    hfdcan1.Init.DataTimeSeg1 = s1;
    hfdcan1.Init.DataTimeSeg2 = s2;
  } else
    hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;

  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
    return CAN_ERR_HAL_INIT;
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    return CAN_ERR_HAL_START;
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  LED_STATE_ON();
  return CAN_OK;
}

/* 处理 USB 发来的指令 */
CAN_Status_t SLCAN_ProcessCommand(char *cmd, char *response) {
  char type = cmd[0];
  switch (type) {
  case 'S': { // S0-S8
    uint32_t r[] = {10000,  20000,  50000,  100000, 125000,
                    250000, 500000, 800000, 1000000};
    mCfg.nominalBitrate = r[cmd[1] - '0'];
    return CAN_OK;
  }
  case 'Y': { // Y0-Y4 (FD)
    uint32_t r[] = {1000000, 2000000, 4000000, 5000000, 8000000};
    mCfg.dataBitrate = r[cmd[1] - '0'];
    mCfg.isDataBitrateSet = 1;
    return CAN_OK;
  }
  case 'O':
    return CanOpen();
  case 'C':
    HAL_FDCAN_Stop(&hfdcan1);
    LED_STATE_OFF();
    return CAN_OK;
  case 'V':
    strcpy(response, "V1010");
    return CAN_OK;
  case 't':
  case 'T':
  case 'r':
  case 'R': {
    FDCAN_TxHeaderTypeDef th = {0};
    uint8_t isExt = (type == 'T' || type == 'R'),
            isRtr = (type == 'r' || type == 'R');
    int idLen = isExt ? 8 : 3;
    uint32_t id = 0;
    char *ptr = &cmd[1];
    for (int i = 0; i < idLen; i++)
      id = (id << 4) | Hex2Int(*ptr++);
    uint8_t len = Hex2Int(*ptr++);
    uint8_t txData[64] = {0};
    if (!isRtr)
      for (int i = 0; i < len; i++) {
        txData[i] = (Hex2Int(ptr[0]) << 4) | Hex2Int(ptr[1]);
        ptr += 2;
      }
    th.Identifier = id;
    th.IdType = isExt ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
    th.TxFrameType = isRtr ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;
    th.DataLength = (len <= 8) ? len : FDCAN_DLC_BYTES_64; // 此处可精细化映射
    th.FDFormat = mCfg.isDataBitrateSet ? FDCAN_FD_CAN : FDCAN_CLASSIC_CAN;
    th.BitRateSwitch = mCfg.isDataBitrateSet ? FDCAN_BRS_ON : FDCAN_BRS_OFF;
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &th, txData) == HAL_OK) {
      LED_WORK_TOGGLE();
      return CAN_OK;
    }
    return CAN_ERR_HAL_INIT;
  }
  }
  return CAN_ERR_PARAM_INVALID;
}