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
  uint8_t nominalIdx; /* 标称波特率索引 (0-8) */
  uint8_t dataIdx;    /* 数据波特率索引 (0-4) */
  uint8_t isDataBitrateSet;
  uint8_t isOpen;
} FDCAN_Config_t;

static FDCAN_Config_t mCfg = {.nominalIdx = 6, /* 默认 S6: 500 kbps */
                              .dataIdx = 1,    /* 默认 Y2: 2 Mbps */
                              .isDataBitrateSet = 1,
                              .isOpen = 0};

/**
 * 预计算的 FDCAN 位时序配置
 * 时钟源: 160 MHz
 * 采样点: 80%
 * 公式: 波特率 = 时钟 / (Prescaler * (1 + TimeSeg1 + TimeSeg2))
 *       采样点 = (1 + TimeSeg1) / (1 + TimeSeg1 + TimeSeg2)
 */
typedef struct {
  uint32_t bitrate;   /* 波特率 (bps) */
  uint16_t prescaler; /* 预分频器 1-512 */
  uint8_t timeSeg1;   /* 时间段1 1-256 */
  uint8_t timeSeg2;   /* 时间段2 1-128 */
  uint8_t sjw;        /* 同步跳转宽度 1-128 */
} FDCAN_TimingConfig_t;

/* 标称波特率时序配置表 (SLCAN S0-S8) - 160 MHz, 80% 采样点 */
static const FDCAN_TimingConfig_t nom_timing_table[] = {
    /* S0: 10 kbps   - 160MHz / (100 * 160) = 10000, SP = 128/160 = 80% */
    {.bitrate = 10000,
     .prescaler = 100,
     .timeSeg1 = 127,
     .timeSeg2 = 32,
     .sjw = 16},
    /* S1: 20 kbps   - 160MHz / (50 * 160) = 20000, SP = 128/160 = 80% */
    {.bitrate = 20000,
     .prescaler = 50,
     .timeSeg1 = 127,
     .timeSeg2 = 32,
     .sjw = 16},
    /* S2: 50 kbps   - 160MHz / (20 * 160) = 50000, SP = 128/160 = 80% */
    {.bitrate = 50000,
     .prescaler = 20,
     .timeSeg1 = 127,
     .timeSeg2 = 32,
     .sjw = 16},
    /* S3: 100 kbps  - 160MHz / (10 * 160) = 100000, SP = 128/160 = 80% */
    {.bitrate = 100000,
     .prescaler = 10,
     .timeSeg1 = 127,
     .timeSeg2 = 32,
     .sjw = 16},
    /* S4: 125 kbps  - 160MHz / (8 * 160) = 125000, SP = 128/160 = 80% */
    {.bitrate = 125000,
     .prescaler = 8,
     .timeSeg1 = 127,
     .timeSeg2 = 32,
     .sjw = 16},
    /* S5: 250 kbps  - 160MHz / (4 * 160) = 250000, SP = 128/160 = 80% */
    {.bitrate = 250000,
     .prescaler = 4,
     .timeSeg1 = 127,
     .timeSeg2 = 32,
     .sjw = 16},
    /* S6: 500 kbps  - 160MHz / (2 * 160) = 500000, SP = 128/160 = 80% */
    {.bitrate = 500000,
     .prescaler = 2,
     .timeSeg1 = 127,
     .timeSeg2 = 32,
     .sjw = 16},
    /* S7: 800 kbps  - 160MHz / (4 * 50) = 800000, SP = 40/50 = 80% */
    {.bitrate = 800000,
     .prescaler = 4,
     .timeSeg1 = 39,
     .timeSeg2 = 10,
     .sjw = 10},
    /* S8: 1000 kbps - 160MHz / (2 * 80) = 1000000, SP = 64/80 = 80% */
    {.bitrate = 1000000,
     .prescaler = 2,
     .timeSeg1 = 63,
     .timeSeg2 = 16,
     .sjw = 16},
};
#define STD_BITRATE_COUNT                                                      \
  (sizeof(nom_timing_table) / sizeof(nom_timing_table[0]))

/* FD 数据波特率时序配置表 (Y1-Y5) - 160 MHz, 80% 采样点 */
static const FDCAN_TimingConfig_t data_timing_table[] = {
    /* Y1: 1 Mbps  - 160MHz / (4 * 40) = 1000000, SP = 32/40 = 80% */
    {.bitrate = 1000000,
     .prescaler = 4,
     .timeSeg1 = 31,
     .timeSeg2 = 8,
     .sjw = 4},
    /* Y2: 2 Mbps  - 160MHz / (2 * 40) = 2000000, SP = 32/40 = 80% */
    {.bitrate = 2000000,
     .prescaler = 2,
     .timeSeg1 = 31,
     .timeSeg2 = 8,
     .sjw = 4},
    /* Y3: 4 Mbps  - 160MHz / (2 * 20) = 4000000, SP = 16/20 = 80% */
    {.bitrate = 4000000,
     .prescaler = 2,
     .timeSeg1 = 15,
     .timeSeg2 = 4,
     .sjw = 2},
    /* Y4: 5 Mbps  - 160MHz / (2 * 16) = 5000000, SP = 13/16 = 81.25% */
    {.bitrate = 5000000,
     .prescaler = 2,
     .timeSeg1 = 12,
     .timeSeg2 = 3,
     .sjw = 2},
    /* Y5: 8 Mbps  - 160MHz / (2 * 10) = 8000000, SP = 8/10 = 80% */
    {.bitrate = 8000000,
     .prescaler = 2,
     .timeSeg1 = 7,
     .timeSeg2 = 2,
     .sjw = 2},
};
#define FD_BITRATE_COUNT                                                       \
  (sizeof(data_timing_table) / sizeof(data_timing_table[0]))

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
/**
 * @brief 向 USB 发送缓冲区写入字符串（在中断中调用）
 * @param s 要写入的字符串
 * @note  SPSC 无锁设计：生产者只修改 head，读取 volatile tail
 *        消费者只修改 tail，读取 volatile head
 *        无需关中断保护，volatile 确保内存可见性
 */
void USB_TxBuf_WriteString(const char *s) {
  while (*s) {
    uint32_t next = (g_usb_tx_fifo.head + 1) % TX_BUF_SIZE;
    if (next != g_usb_tx_fifo.tail) {
      g_usb_tx_fifo.buffer[g_usb_tx_fifo.head] = *s++;
      g_usb_tx_fifo.head = next;
    } else {
      break; /* 缓冲区满 */
    }
  }
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

/* 打开 CAN 通道 */
CAN_Status_t CanOpen(void) {
  /* 如果已经打开，先关闭 */
  if (mCfg.isOpen || hfdcan1.State != HAL_FDCAN_STATE_RESET) {
    HAL_FDCAN_Stop(&hfdcan1);
    HAL_FDCAN_DeInit(&hfdcan1);
    mCfg.isOpen = 0;
  }

  /* 使用预计算的标称位时序配置 */
  const FDCAN_TimingConfig_t *nomTiming = &nom_timing_table[mCfg.nominalIdx];
  hfdcan1.Init.NominalPrescaler = nomTiming->prescaler;
  hfdcan1.Init.NominalTimeSeg1 = nomTiming->timeSeg1;
  hfdcan1.Init.NominalTimeSeg2 = nomTiming->timeSeg2;
  hfdcan1.Init.NominalSyncJumpWidth = nomTiming->sjw;

  /* FD CAN 数据位时序配置 */
  if (mCfg.isDataBitrateSet) {
    const FDCAN_TimingConfig_t *dataTiming = &data_timing_table[mCfg.dataIdx];
    hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
    hfdcan1.Init.DataPrescaler = dataTiming->prescaler;
    hfdcan1.Init.DataTimeSeg1 = dataTiming->timeSeg1;
    hfdcan1.Init.DataTimeSeg2 = dataTiming->timeSeg2;
    hfdcan1.Init.DataSyncJumpWidth = dataTiming->sjw;
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
    mCfg.nominalIdx = idx;
    return CAN_OK;
  }

  /* 设置 FD 数据波特率 Y1-Y5 */
  case 'Y': {
    if (mCfg.isOpen)
      return CAN_ERR_PARAM_INVALID;
    const uint8_t idx = cmd[1] - '1';
    if (idx >= FD_BITRATE_COUNT)
      return CAN_ERR_PARAM_INVALID;
    mCfg.dataIdx = idx;
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

  /* 查询版本 + MCU UID */
  case 'V': {
    char *p = response;
    *p++ = 'V';
    /* 版本号 */
    const char ver[] = "1111";
    for (const char *v = ver; *v;)
      *p++ = *v++;
    /* 追加 96-bit MCU Unique ID (24 hex chars) */
    const uint32_t uid[3] = {*(volatile uint32_t *)(UID_BASE),
                             *(volatile uint32_t *)(UID_BASE + 4U),
                             *(volatile uint32_t *)(UID_BASE + 8U)};
    for (int i = 0; i < 3; i++) {
      for (int s = 28; s >= 0; s -= 4)
        *p++ = hex_table[(uid[i] >> s) & 0x0F];
    }
    *p++ = '\r';
    *p = '\0';
    return CAN_OK;
  }

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
    uint8_t dlc = Hex2Int(*ptr++);
    uint8_t len = dlc2len[dlc];
    if (len > 64)
      return CAN_ERR_PARAM_INVALID;
    if (!isFdc)
      len = MIN(len, 8); /* Classic CAN 最大 8 字节 */

    /* 解析数据 */
    uint8_t txData[64] = {0};
    if (!isRtr) {
      for (uint8_t i = 0; i < len && ptr[0] != '\0'; i++) {
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
      /* 离线后尝试恢复：先进入INIT，延时，重新初始化并打开通道 */
      SET_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
      for (volatile uint32_t i = 0; i < 800000; ++i) {
        __NOP();
      } // 简单延时约10ms@160MHz
      HAL_FDCAN_DeInit(hfdcan);
      CanOpen();
    }
    USB_TxBuf_WriteString("E\r"); /* 发送总线错误状态 */
  }
}

/* 总线离线计数器（由 TIM3 回调递增，CanOpen 成功后清零） */
static volatile uint32_t s_bus_offline_secs = 0;

/**
 * @brief 检测 FDCAN 总线状态，在 TIM3 中断（1 秒）中调用
 * @note  - 总线离线时每秒向上位机发送 "E\r"
 *        - 离线超过 BUS_OFFLINE_THRESHOLD 秒后重启总线
 *        - 总线恢复正常后清零计数器
 */
void FDCAN_CheckBusStatus(void) {
  if (!mCfg.isOpen) {
    return;
  }

  FDCAN_ProtocolStatusTypeDef protStatus;
  HAL_FDCAN_GetProtocolStatus(&hfdcan1, &protStatus);

  if (protStatus.BusOff) {
    s_bus_offline_secs++;
    USB_TxBuf_WriteString("E\r");

    if (s_bus_offline_secs >= BUS_OFFLINE_THRESHOLD) {
      s_bus_offline_secs = 0;
      CanOpen(); /* 重启总线 */
    }
  } else {
    s_bus_offline_secs = 0;
  }
}