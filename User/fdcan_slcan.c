#include "fdcan_slcan.h"
#include "usbd_cdc_if.h"

extern FDCAN_HandleTypeDef hfdcan1;
USB_TxBuffer_t g_usb_tx_fifo = {0};
extern volatile uint8_t usb_tx_busy;

static const char hex_table[] = "0123456789ABCDEF";

/* DLC 值到实际字节数的映射 (FDCAN DataLength >> 16 作为索引) */
static const uint8_t dlc2len[] = {0, 1,  2,  3,  4,  5,  6,  7,
                                  8, 12, 16, 20, 24, 32, 48, 64};

/* 实际字节数到 FDCAN DataLength 寄存器值的映射 */
static const uint32_t len2dlc[] = {
    FDCAN_DLC_BYTES_0,  FDCAN_DLC_BYTES_1,  FDCAN_DLC_BYTES_2,
    FDCAN_DLC_BYTES_3,  FDCAN_DLC_BYTES_4,  FDCAN_DLC_BYTES_5,
    FDCAN_DLC_BYTES_6,  FDCAN_DLC_BYTES_7,  FDCAN_DLC_BYTES_8,
    FDCAN_DLC_BYTES_12, FDCAN_DLC_BYTES_12, FDCAN_DLC_BYTES_12,
    FDCAN_DLC_BYTES_12, FDCAN_DLC_BYTES_16, FDCAN_DLC_BYTES_16,
    FDCAN_DLC_BYTES_16, FDCAN_DLC_BYTES_16, FDCAN_DLC_BYTES_20,
    FDCAN_DLC_BYTES_20, FDCAN_DLC_BYTES_20, FDCAN_DLC_BYTES_20,
    FDCAN_DLC_BYTES_24, FDCAN_DLC_BYTES_24, FDCAN_DLC_BYTES_24,
    FDCAN_DLC_BYTES_24, FDCAN_DLC_BYTES_32, FDCAN_DLC_BYTES_32,
    FDCAN_DLC_BYTES_32, FDCAN_DLC_BYTES_32, FDCAN_DLC_BYTES_32,
    FDCAN_DLC_BYTES_32, FDCAN_DLC_BYTES_32, FDCAN_DLC_BYTES_32,
    FDCAN_DLC_BYTES_48, FDCAN_DLC_BYTES_48, FDCAN_DLC_BYTES_48,
    FDCAN_DLC_BYTES_48, FDCAN_DLC_BYTES_48, FDCAN_DLC_BYTES_48,
    FDCAN_DLC_BYTES_48, FDCAN_DLC_BYTES_48, FDCAN_DLC_BYTES_48,
    FDCAN_DLC_BYTES_48, FDCAN_DLC_BYTES_48, FDCAN_DLC_BYTES_48,
    FDCAN_DLC_BYTES_48, FDCAN_DLC_BYTES_48, FDCAN_DLC_BYTES_48,
    FDCAN_DLC_BYTES_48, FDCAN_DLC_BYTES_64, FDCAN_DLC_BYTES_64,
    FDCAN_DLC_BYTES_64, FDCAN_DLC_BYTES_64, FDCAN_DLC_BYTES_64,
    FDCAN_DLC_BYTES_64, FDCAN_DLC_BYTES_64, FDCAN_DLC_BYTES_64,
    FDCAN_DLC_BYTES_64, FDCAN_DLC_BYTES_64, FDCAN_DLC_BYTES_64,
    FDCAN_DLC_BYTES_64, FDCAN_DLC_BYTES_64, FDCAN_DLC_BYTES_64,
    FDCAN_DLC_BYTES_64, FDCAN_DLC_BYTES_64};

typedef struct {
  uint32_t nominalBitrate;
  uint32_t dataBitrate;
  uint8_t isDataBitrateSet;
  uint8_t isOpen;
} FDCAN_Config_t;

static FDCAN_Config_t mCfg = {.nominalBitrate = 500000,
                              .isDataBitrateSet = 1,
                              .dataBitrate = 2000000,
                              .isOpen = 0};

/* 辅助函数：十六进制字符转数值（内联优化） */
static inline uint8_t Hex2Int(char c) {
  /* 利用 ASCII 特性进行快速转换 */
  if ((uint8_t)(c - '0') <= 9u)
    return c - '0';
  if ((uint8_t)(c - 'A') <= 5u)
    return c - 'A' + 10;
  if ((uint8_t)(c - 'a') <= 5u)
    return c - 'a' + 10;
  return 0;
}

/* 获取正确的 FDCAN DataLength 值 */
static inline uint32_t GetFdcanDlc(uint8_t len) {
  return (len <= 64) ? len2dlc[len] : FDCAN_DLC_BYTES_64;
}

/**
 * @brief 向 USB 发送缓冲区写入字符串（中断安全）
 * @param s 要写入的字符串
 * @note  通过禁用中断保证原子性，防止多生产者竞争
 */
void USB_TxBuf_WriteString(const char *s) {
  uint32_t primask = __get_PRIMASK();
  __disable_irq(); /* 进入临界区 */

  while (*s) {
    uint32_t next = (g_usb_tx_fifo.head + 1) % TX_BUF_SIZE;
    if (next != g_usb_tx_fifo.tail) {
      g_usb_tx_fifo.buffer[g_usb_tx_fifo.head] = *s++;
      g_usb_tx_fifo.head = next;
    } else {
      break; /* 缓冲区满 */
    }
  }

  __set_PRIMASK(primask); /* 退出临界区，恢复原中断状态 */
}

/* 快速格式化 CAN 报文为 SLCAN 字符串 (查表法替代 sprintf) */
void SLCAN_FormatResponse_Fast(FDCAN_RxHeaderTypeDef *RxHeader,
                               uint8_t *RxData) {
  char msg[150], *p = msg;
  const uint8_t isExt = (RxHeader->IdType == FDCAN_EXTENDED_ID);
  const uint8_t isRtr = (RxHeader->RxFrameType == FDCAN_REMOTE_FRAME);
  const uint8_t isBrs = (RxHeader->BitRateSwitch == FDCAN_BRS_ON);
  const uint8_t isFdc = (RxHeader->FDFormat == FDCAN_FD_CAN);

  /* 帧类型标识符 */
  if (isFdc) {
    if (isRtr) {
      *p++ = isExt ? 'R' : 'r';
    } else if (isBrs) {
      *p++ = isExt ? 'B' : 'b';
    } else {
      *p++ = isExt ? 'D' : 'd';
    }
  } else {
    *p++ = isExt ? 'T' : 't';
  }

  /* ID 转换（支持 FD 扩展格式） */
  const uint32_t id = RxHeader->Identifier;
  const int idLen = isExt ? 8 : 3;
  for (int i = idLen - 1; i >= 0; i--) {
    p[i] = hex_table[(id >> ((idLen - 1 - i) * 4)) & 0x0F];
  }
  p += idLen;

  /* 数据长度 */
  const uint8_t len = dlc2len[RxHeader->DataLength];

  /* FD CAN 长度可能超过 9，使用两位十六进制 */

  *p++ = hex_table[RxHeader->DataLength];

  /* 数据字段 */
  if (!isRtr) {
    for (uint8_t i = 0; i < len; i++) {
      *p++ = hex_table[RxData[i] >> 4];
      *p++ = hex_table[RxData[i] & 0x0F];
    }
  }
  *p++ = '\r';
  *p = '\0';

  USB_TxBuf_WriteString(msg);
  LED_WORK_TOGGLE();
}

/* 自动计算位时间 (TQ)
 * 目标采样点: 87.5% (CAN FD 推荐)
 * totalTQ = SYNC_SEG(1) + TSEG1 + TSEG2
 * 采样点 = (1 + TSEG1) / totalTQ
 */
static CAN_Status_t Internal_CalculateTiming(uint32_t clk, uint32_t baud,
                                             uint32_t *presc, uint32_t *s1,
                                             uint32_t *s2, uint32_t *sjw) {
  if (baud == 0 || clk == 0)
    return CAN_ERR_PARAM_INVALID;

  /* 遍历预分频值寻找最佳配置 */
  for (uint32_t p = 1; p <= 512; p++) {
    uint32_t baudXp = baud * p;
    if (clk % baudXp != 0)
      continue;

    uint32_t totalTQ = clk / baudXp;
    /* FDCAN 要求: TSEG1 范围 1-256, TSEG2 范围 1-128, totalTQ >= 4 */
    if (totalTQ < 4 || totalTQ > 385)
      continue;

    /* 计算 87.5% 采样点 */
    uint32_t tseg1 =
        (totalTQ * 875 / 1000) - 1; /* (1 + TSEG1) / totalTQ = 0.875 */
    uint32_t tseg2 = totalTQ - 1 - tseg1;

    /* 验证范围 */
    if (tseg1 >= 1 && tseg1 <= 256 && tseg2 >= 1 && tseg2 <= 128) {
      *presc = p;
      *s1 = tseg1;
      *s2 = tseg2;
      *sjw = (tseg2 < 16) ? tseg2 : 16; /* SJW 通常取 min(TSEG2, 16) */
      return CAN_OK;
    }
  }
  return CAN_ERR_CLOCK;
}

/* 打开 CAN 通道 */
CAN_Status_t CanOpen(void) {
  /* 如果已经打开，先关闭 */
  if (mCfg.isOpen || hfdcan1.State != HAL_FDCAN_STATE_RESET) {
    HAL_FDCAN_Stop(&hfdcan1);
    HAL_FDCAN_DeInit(&hfdcan1);
    mCfg.isOpen = 0;
  }

  uint32_t p, s1, s2, sjw;
  const uint32_t clk = HAL_RCC_GetPCLK1Freq();

  /* 计算标称位时序 */
  if (Internal_CalculateTiming(clk, mCfg.nominalBitrate, &p, &s1, &s2, &sjw) !=
      CAN_OK) {
    return CAN_ERR_CLOCK;
  }
  hfdcan1.Init.NominalPrescaler = p;
  hfdcan1.Init.NominalTimeSeg1 = s1;
  hfdcan1.Init.NominalTimeSeg2 = s2;
  hfdcan1.Init.NominalSyncJumpWidth = sjw;

  /* FD CAN 数据位时序配置 */
  if (mCfg.isDataBitrateSet) {
    if (Internal_CalculateTiming(clk, mCfg.dataBitrate, &p, &s1, &s2, &sjw) !=
        CAN_OK) {
      return CAN_ERR_CLOCK;
    }
    hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
    hfdcan1.Init.DataPrescaler = p;
    hfdcan1.Init.DataTimeSeg1 = s1;
    hfdcan1.Init.DataTimeSeg2 = s2;
    hfdcan1.Init.DataSyncJumpWidth = sjw;
  } else {
    hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  }

  /* 配置全局过滤器：接收所有帧 */
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) {
    return CAN_ERR_HAL_INIT;
  }

  /* 配置过滤器：接收所有标准帧和扩展帧 */
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                               FDCAN_ACCEPT_IN_RX_FIFO0, /* 非匹配标准帧 */
                               FDCAN_ACCEPT_IN_RX_FIFO0, /* 非匹配扩展帧 */
                               FDCAN_FILTER_REMOTE,      /* 远程标准帧 */
                               FDCAN_FILTER_REMOTE);     /* 远程扩展帧 */

  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
    return CAN_ERR_HAL_START;
  }

  /* 启用接收中断 */
  HAL_FDCAN_ActivateNotification(
      &hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_BUS_OFF, 0);

  mCfg.isOpen = 1;
  LED_STATE_ON();
  return CAN_OK;
}

/* 关闭 CAN 通道 */
void CanClose(void) {
  if (mCfg.isOpen) {
    HAL_FDCAN_Stop(&hfdcan1);
    mCfg.isOpen = 0;
  }
  LED_STATE_OFF();
}

/* 标准波特率表 (SLCAN S0-S8) */
static const uint32_t std_bitrates[] = {10000,  20000,  50000,  100000, 125000,
                                        250000, 500000, 800000, 1000000};
#define STD_BITRATE_COUNT (sizeof(std_bitrates) / sizeof(std_bitrates[0]))

/* FD 数据波特率表 (Y1-Y5) */
static const uint32_t fd_bitrates[] = {1000000, 2000000, 3000000, 4000000,
                                       5000000};
#define FD_BITRATE_COUNT (sizeof(fd_bitrates) / sizeof(fd_bitrates[0]))

/* 处理 USB 发来的 SLCAN 指令 */
CAN_Status_t SLCAN_ProcessCommand(char *cmd, char *response) {
  if (cmd == NULL || response == NULL || cmd[0] == '\0') {
    return CAN_ERR_PARAM_INVALID;
  }

  const char type = cmd[0];

  switch (type) {
  /* 设置标准波特率 S0-S8 */
  case 'S': {
    if (mCfg.isOpen)
      return CAN_ERR_PARAM_INVALID; /* 通道打开时不允许更改 */
    const uint8_t idx = cmd[1] - '0';
    if (idx >= STD_BITRATE_COUNT)
      return CAN_ERR_PARAM_INVALID;
    mCfg.nominalBitrate = std_bitrates[idx];
    return CAN_OK;
  }

  /* 设置 FD 数据波特率 Y0-Y4 */
  case 'Y': {
    if (mCfg.isOpen)
      return CAN_ERR_PARAM_INVALID;
    const uint8_t idx = cmd[1] - '1';
    if (idx >= FD_BITRATE_COUNT)
      return CAN_ERR_PARAM_INVALID;
    mCfg.dataBitrate = fd_bitrates[idx];
    mCfg.isDataBitrateSet = 1;
    return CAN_OK;
  }

  /* 打开 CAN 通道 */
  case 'O':
    return CanOpen();

  /* 关闭 CAN 通道 */
  case 'C':
    CanClose();
    return CAN_OK;

  /* 查询版本 */
  case 'V':
    strcpy(response, "V1111\r");
    return CAN_OK;

  /* 查询序列号 */
  case 'N':
    strcpy(response, "NFDCAN");
    return CAN_OK;

  /* 查询状态标志 */
  case 'F':
    strcpy(response, "F00\r");
    return CAN_OK;

  /* 发送数据帧/远程帧: t/T/r/R */
  case 't':
  case 'T':
  case 'd':
  case 'D':
  case 'b':
  case 'B':
  case 'r':
  case 'R': {
    if (!mCfg.isOpen)
      return CAN_ERR_HAL_START; /* 通道未打开 */

    FDCAN_TxHeaderTypeDef th = {0};
    const uint8_t isExt =
        (type == 'T' || type == 'D' || type == 'R' || type == 'B');
    const uint8_t isRtr = (type == 'r' || type == 'R');
    const uint8_t isFdc =
        (type == 'D' || type == 'd' || type == 'B' || type == 'b');
    const uint8_t isBrs = (type == 'B' || type == 'b');

    const int idLen = isExt ? 8 : 3;

    /* 检查命令长度 */
    const size_t cmdLen = strlen(cmd);
    if (cmdLen < (size_t)(1 + idLen + 1))
      return CAN_ERR_PARAM_INVALID;

    /* 解析 ID */
    uint32_t id = 0;
    const char *ptr = &cmd[1];
    for (int i = 0; i < idLen; i++) {
      id = (id << 4) | Hex2Int(*ptr++);
    }

    /* 验证 ID 范围 */
    if (isExt && id > 0x1FFFFFFF)
      return CAN_ERR_PARAM_INVALID;
    if (!isExt && id > 0x7FF)
      return CAN_ERR_PARAM_INVALID;

    /* 解析数据长度 */
    uint8_t len = Hex2Int(*ptr++);
    if (len > 64)
      return CAN_ERR_PARAM_INVALID;
    if (!isFdc)
      len = MIN(len, 8); /* Classic CAN 最大 8 字节 */

    /* 解析数据 */
    uint8_t txData[64] = {0};
    if (!isRtr) {
      const size_t expectedLen = 1 + idLen + 1 + len * 2;
      if (cmdLen < expectedLen)
        return CAN_ERR_PARAM_INVALID;
      for (uint8_t i = 0; i < len; i++) {
        txData[i] = (Hex2Int(ptr[0]) << 4) | Hex2Int(ptr[1]);
        ptr += 2;
      }
    }

    /* 配置发送头 */
    th.Identifier = id;
    th.IdType = isExt ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
    th.TxFrameType = isRtr ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;
    th.DataLength = GetFdcanDlc(len);
    th.FDFormat = isFdc ? FDCAN_FD_CAN : FDCAN_CLASSIC_CAN;
    th.BitRateSwitch = isBrs ? FDCAN_BRS_ON : FDCAN_BRS_OFF;
    th.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    th.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    th.MessageMarker = 0;

    /* 发送 */
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &th, txData) == HAL_OK) {
      LED_WORK_TOGGLE();
      return CAN_OK;
    }
    return CAN_ERR_HAL_INIT;
  }

  default:
    break;
  }

  return CAN_ERR_PARAM_INVALID;
}

/**
 * @brief FDCAN RX FIFO0 接收回调
 * @param hfdcan FDCAN 句柄
 * @param RxFifo0ITs 中断标志
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo0ITs) {
  (void)RxFifo0ITs; /* 未使用参数 */

  FDCAN_RxHeaderTypeDef header;
  uint8_t data[64];

  /* 循环读取 FIFO 中所有消息 */
  while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &header, data) ==
         HAL_OK) {
    SLCAN_FormatResponse_Fast(&header, data);
  }
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan,
                                   uint32_t ErrorStatusITs) {
  FDCAN_ProtocolStatusTypeDef protStatus;

  if (ErrorStatusITs & FDCAN_IT_BUS_OFF) {
    HAL_FDCAN_GetProtocolStatus(hfdcan, &protStatus);

    if (protStatus.BusOff == 1) {
      CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
    }
    USB_TxBuf_WriteString("E\r"); /* 发送总线错误状态 */
  }
}