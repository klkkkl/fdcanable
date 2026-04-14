#include "fdcan_slcan.h"
#include "usbd_cdc_if.h"

extern FDCAN_HandleTypeDef hfdcan1;
USB_TxBuffer_t g_usb_tx_fifo = {0};
extern volatile uint8_t usb_tx_busy;

#define ARRAY_SIZE(array) (sizeof(array) / sizeof((array)[0]))
#define SLCAN_MAX_DATA_BYTES 64U
#define SLCAN_STD_ID_HEX_LEN 3U
#define SLCAN_EXT_ID_HEX_LEN 8U
#define SLCAN_FRAME_TEXT_MAX_LEN 150U
#define SLCAN_UID_WORD_COUNT 3U
#define BUS_OFF_RECOVERY_DELAY_MS 10U

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
  uint8_t nominalIdx; /* 标称波特率配置索引 */
  uint8_t dataIdx;    /* 数据波特率配置索引 */
  uint8_t isDataBitrateSet;
  uint8_t isSilentMode;
  uint8_t autoRetransmission;
  uint8_t isOpen;
} FDCAN_Config_t;

static FDCAN_Config_t mCfg = {.nominalIdx = 6, /* 默认 S6: 500 kbps */
                              .dataIdx = 1,    /* 默认 Y2: 2 Mbps */
                              .isDataBitrateSet = 1,
                              .isSilentMode = 0,
                              .autoRetransmission = 1,
                              .isOpen = 0};

typedef struct {
  uint8_t idLen;
  uint8_t isExt;
  uint8_t isRtr;
  uint8_t isFdc;
  uint8_t isBrs;
} SLCAN_FrameFormat_t;

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

/* 标称波特率时序配置表 (SLCAN S0-S8, S10) - 160 MHz, 80% 采样点 */
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
    /* S10: 2000 kbps - 160MHz / (2 * 40) = 2000000, SP = 32/40 = 80% */
    {.bitrate = 2000000,
     .prescaler = 2,
     .timeSeg1 = 31,
     .timeSeg2 = 8,
     .sjw = 4},
};
static const uint8_t nom_timing_cmd_values[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 10};

/* FD 数据波特率时序配置表 (Yn 对应 n Mbps，Y8 为 8 Mbps) - 160 MHz */
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
    /* Y3: 3 Mbps 目标值，当前配置约 2.963 Mbps - 160MHz / (2 * 27) */
    {.bitrate = 2962963,
     .prescaler = 2,
     .timeSeg1 = 21,
     .timeSeg2 = 5,
     .sjw = 2},
    /* Y4: 4 Mbps  - 160MHz / (2 * 20) = 4000000, SP = 16/20 = 80% */
    {.bitrate = 4000000,
     .prescaler = 2,
     .timeSeg1 = 15,
     .timeSeg2 = 4,
     .sjw = 2},
    /* Y5: 5 Mbps  - 160MHz / (2 * 16) = 5000000, SP = 13/16 = 81.25% */
    {.bitrate = 5000000,
     .prescaler = 2,
     .timeSeg1 = 12,
     .timeSeg2 = 3,
     .sjw = 2},
    /* Y8: 8 Mbps  - 160MHz / (2 * 10) = 8000000, SP = 8/10 = 80% */
    {.bitrate = 8000000,
     .prescaler = 2,
     .timeSeg1 = 7,
     .timeSeg2 = 2,
     .sjw = 2},
};
static const uint8_t data_timing_cmd_values[] = {1, 2, 3, 4, 5, 8};

/* Bus-Off 恢复状态 */
static volatile uint8_t s_busoff_recovery_pending = 0;
static volatile uint32_t s_busoff_recovery_deadline = 0;
static volatile uint32_t s_bus_offline_secs = 0;

static uint32_t USB_TxBuf_Write(const char *data, uint32_t len);

static uint8_t ParseHexDigit(char c, uint8_t *value) {
  if ((uint8_t)(c - '0') <= 9u) {
    *value = (uint8_t)(c - '0');
    return 1;
  }

  if ((uint8_t)(c - 'A') <= 5u) {
    *value = (uint8_t)(c - 'A' + 10);
    return 1;
  }

  if ((uint8_t)(c - 'a') <= 5u) {
    *value = (uint8_t)(c - 'a' + 10);
    return 1;
  }

  return 0;
}

static uint8_t ParseHexField(const char *src, uint8_t digits, uint32_t *value) {
  uint32_t parsed = 0;

  for (uint8_t i = 0; i < digits; ++i) {
    uint8_t nibble;
    if (!ParseHexDigit(src[i], &nibble)) {
      return 0;
    }
    parsed = (parsed << 4) | nibble;
  }

  *value = parsed;
  return 1;
}

static uint8_t ParseHexBytes(const char *src, uint8_t len, uint8_t *data) {
  for (uint8_t i = 0; i < len; ++i) {
    uint8_t hi;
    uint8_t lo;

    if (!ParseHexDigit(src[0], &hi) || !ParseHexDigit(src[1], &lo)) {
      return 0;
    }

    data[i] = (uint8_t)((hi << 4) | lo);
    src += 2;
  }

  return 1;
}

static uint8_t ParseDecimalUint8(const char *src, uint8_t *value) {
  uint16_t parsed = 0;

  if (src[0] == '\0') {
    return 0;
  }

  for (const char *p = src; *p != '\0'; ++p) {
    if (*p < '0' || *p > '9') {
      return 0;
    }

    parsed = (uint16_t)(parsed * 10U + (uint16_t)(*p - '0'));
    if (parsed > UINT8_MAX) {
      return 0;
    }
  }

  *value = (uint8_t)parsed;
  return 1;
}

/* 获取正确的 FDCAN DataLength 值 */
static inline uint32_t GetFdcanDlc(uint8_t len) {
  return (len <= SLCAN_MAX_DATA_BYTES) ? len2dlc[len] : FDCAN_DLC_BYTES_64;
}

static inline uint8_t GetPayloadLengthFromDlc(uint8_t dlc) {
  return (dlc < ARRAY_SIZE(dlc2len)) ? dlc2len[dlc] : 0U;
}

static int8_t FindTimingIndexByCommand(const uint8_t *cmd_values,
                                       uint8_t count, uint8_t cmd_value) {
  for (uint8_t i = 0; i < count; ++i) {
    if (cmd_values[i] == cmd_value) {
      return (int8_t)i;
    }
  }

  return -1;
}

static uint8_t SLCAN_GetFrameFormat(char type, SLCAN_FrameFormat_t *format) {
  switch (type) {
  case 't':
    *format = (SLCAN_FrameFormat_t){.idLen = SLCAN_STD_ID_HEX_LEN};
    return 1;
  case 'T':
    *format = (SLCAN_FrameFormat_t){.idLen = SLCAN_EXT_ID_HEX_LEN,
                                    .isExt = 1};
    return 1;
  case 'r':
    *format = (SLCAN_FrameFormat_t){.idLen = SLCAN_STD_ID_HEX_LEN,
                                    .isRtr = 1};
    return 1;
  case 'R':
    *format = (SLCAN_FrameFormat_t){.idLen = SLCAN_EXT_ID_HEX_LEN,
                                    .isExt = 1,
                                    .isRtr = 1};
    return 1;
  case 'd':
    *format = (SLCAN_FrameFormat_t){.idLen = SLCAN_STD_ID_HEX_LEN,
                                    .isFdc = 1};
    return 1;
  case 'D':
    *format = (SLCAN_FrameFormat_t){.idLen = SLCAN_EXT_ID_HEX_LEN,
                                    .isExt = 1,
                                    .isFdc = 1};
    return 1;
  case 'b':
    *format = (SLCAN_FrameFormat_t){.idLen = SLCAN_STD_ID_HEX_LEN,
                                    .isFdc = 1,
                                    .isBrs = 1};
    return 1;
  case 'B':
    *format = (SLCAN_FrameFormat_t){.idLen = SLCAN_EXT_ID_HEX_LEN,
                                    .isExt = 1,
                                    .isFdc = 1,
                                    .isBrs = 1};
    return 1;
  default:
    return 0;
  }
}

static void ResetBusRecoveryState(void) {
  s_busoff_recovery_pending = 0;
  s_busoff_recovery_deadline = 0;
  s_bus_offline_secs = 0;
}

static void StopCanController(void) {
  if (hfdcan1.State != HAL_FDCAN_STATE_RESET) {
    HAL_FDCAN_Stop(&hfdcan1);
    HAL_FDCAN_DeInit(&hfdcan1);
  }

  mCfg.isOpen = 0;
}

static void AppendUid(char **response) {
  const uint32_t uid[SLCAN_UID_WORD_COUNT] = {
      *(volatile uint32_t *)(UID_BASE),
      *(volatile uint32_t *)(UID_BASE + 4U),
      *(volatile uint32_t *)(UID_BASE + 8U),
  };

  for (uint8_t i = 0; i < SLCAN_UID_WORD_COUNT; ++i) {
    for (int shift = 28; shift >= 0; shift -= 4) {
      *(*response)++ = hex_table[(uid[i] >> shift) & 0x0F];
    }
  }
}

static void BuildVersionResponse(char *response) {
  char *p = response;
  *p++ = 'V';

  for (const char *v = SLCAN_FW_VERSION; *v != '\0'; ++v) {
    *p++ = *v;
  }

  AppendUid(&p);
  *p++ = '\r';
  *p = '\0';
}

static void BuildSerialResponse(char *response) {
  char *p = response;
  *p++ = 'N';

  for (const char *serial = SLCAN_SERIAL_NAME; *serial != '\0'; ++serial) {
    *p++ = *serial;
  }

  *p++ = '\r';
  *p = '\0';
}

static void AppendHexByte(char **response, uint8_t value) {
  *(*response)++ = hex_table[(value >> 4) & 0x0F];
  *(*response)++ = hex_table[value & 0x0F];
}

static void BuildErrorStatusResponse(char *response) {
  FDCAN_ProtocolStatusTypeDef protocol_status;
  FDCAN_ErrorCountersTypeDef error_counters;
  char *p = response;

  (void)HAL_FDCAN_GetProtocolStatus(&hfdcan1, &protocol_status);
  (void)HAL_FDCAN_GetErrorCounters(&hfdcan1, &error_counters);

  *p++ = 'E';
  AppendHexByte(&p, (uint8_t)error_counters.TxErrorCnt);
  AppendHexByte(&p, (uint8_t)error_counters.RxErrorCnt);
  *p++ = hex_table[protocol_status.LastErrorCode & 0x0F];
  *p++ = '\r';
  *p = '\0';
}

static uint8_t HasNoTrailingArgs(const char *cmd) {
  return (cmd[1] == '\0');
}

static CAN_Status_t HandleStandardBitrateCommand(const char *cmd) {
  uint8_t cmd_value;
  int8_t idx;

  if (mCfg.isOpen) {
    return CAN_ERR_PARAM_INVALID;
  }

  if (!ParseDecimalUint8(&cmd[1], &cmd_value)) {
    return CAN_ERR_PARAM_INVALID;
  }

  idx = FindTimingIndexByCommand(nom_timing_cmd_values,
                                 ARRAY_SIZE(nom_timing_cmd_values), cmd_value);
  if (idx < 0) {
    return CAN_ERR_PARAM_INVALID;
  }

  mCfg.nominalIdx = (uint8_t)idx;
  return CAN_OK;
}

static CAN_Status_t HandleDataBitrateCommand(const char *cmd) {
  uint8_t cmd_value;
  int8_t idx;

  if (mCfg.isOpen) {
    return CAN_ERR_PARAM_INVALID;
  }

  if (!ParseDecimalUint8(&cmd[1], &cmd_value)) {
    return CAN_ERR_PARAM_INVALID;
  }

  idx = FindTimingIndexByCommand(data_timing_cmd_values,
                                 ARRAY_SIZE(data_timing_cmd_values), cmd_value);
  if (idx < 0) {
    return CAN_ERR_PARAM_INVALID;
  }

  mCfg.dataIdx = (uint8_t)idx;
  mCfg.isDataBitrateSet = 1;
  return CAN_OK;
}

static CAN_Status_t HandleModeCommand(const char *cmd) {
  if (mCfg.isOpen || cmd[1] == '\0' || cmd[2] != '\0') {
    return CAN_ERR_PARAM_INVALID;
  }

  switch (cmd[1]) {
  case '0':
    mCfg.isSilentMode = 0;
    return CAN_OK;
  case '1':
    mCfg.isSilentMode = 1;
    return CAN_OK;
  default:
    return CAN_ERR_PARAM_INVALID;
  }
}

static CAN_Status_t HandleAutoRetransmissionCommand(const char *cmd) {
  if (mCfg.isOpen || cmd[1] == '\0' || cmd[2] != '\0') {
    return CAN_ERR_PARAM_INVALID;
  }

  switch (cmd[1]) {
  case '0':
    mCfg.autoRetransmission = 0;
    return CAN_OK;
  case '1':
    mCfg.autoRetransmission = 1;
    return CAN_OK;
  default:
    return CAN_ERR_PARAM_INVALID;
  }
}

static CAN_Status_t HandleTxCommand(const char *cmd, char type) {
  SLCAN_FrameFormat_t format;
  FDCAN_TxHeaderTypeDef tx_header = {0};
  uint8_t tx_data[SLCAN_MAX_DATA_BYTES] = {0};
  uint32_t id;
  uint8_t dlc;
  uint8_t len;
  const size_t cmd_len = strlen(cmd);
  size_t expected_len;

  if (!mCfg.isOpen) {
    return CAN_ERR_HAL_START;
  }

  if (!SLCAN_GetFrameFormat(type, &format)) {
    return CAN_ERR_PARAM_INVALID;
  }

  expected_len = (size_t)(1U + format.idLen + 1U);
  if (cmd_len < expected_len) {
    return CAN_ERR_PARAM_INVALID;
  }

  if (!ParseHexField(&cmd[1], format.idLen, &id)) {
    return CAN_ERR_PARAM_INVALID;
  }

  if ((format.isExt && id > 0x1FFFFFFFU) || (!format.isExt && id > 0x7FFU)) {
    return CAN_ERR_PARAM_INVALID;
  }

  if (!ParseHexDigit(cmd[1 + format.idLen], &dlc)) {
    return CAN_ERR_PARAM_INVALID;
  }

  if (!format.isFdc && dlc > 8U) {
    return CAN_ERR_PARAM_INVALID;
  }

  len = GetPayloadLengthFromDlc(dlc);
  if (len > SLCAN_MAX_DATA_BYTES) {
    return CAN_ERR_PARAM_INVALID;
  }

  expected_len += format.isRtr ? 0U : (size_t)len * 2U;
  if (cmd_len != expected_len) {
    return CAN_ERR_PARAM_INVALID;
  }

  if (!format.isRtr &&
      !ParseHexBytes(&cmd[1 + format.idLen + 1], len, tx_data)) {
    return CAN_ERR_PARAM_INVALID;
  }

  tx_header.Identifier = id;
  tx_header.IdType = format.isExt ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
  tx_header.TxFrameType =
      format.isRtr ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;
  tx_header.DataLength = GetFdcanDlc(len);
  tx_header.FDFormat = format.isFdc ? FDCAN_FD_CAN : FDCAN_CLASSIC_CAN;
  tx_header.BitRateSwitch = format.isBrs ? FDCAN_BRS_ON : FDCAN_BRS_OFF;
  tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  tx_header.MessageMarker = 0;

  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, tx_data) != HAL_OK) {
    return CAN_ERR_HAL_INIT;
  }

  return CAN_OK;
}

/**
 * @brief 向 USB 发送缓冲区批量写入数据
 * @param data 数据指针
 * @param len  数据长度
 * @return 实际写入的字节数
 * @note  SPSC 无锁设计：生产者只修改 head，读取 volatile tail
 *        消费者只修改 tail，读取 volatile head
 *        所有生产者（FDCAN/TIM3/USB 中断）必须在同一优先级，防止抢占导致 head 竞争
 */
static uint32_t USB_TxBuf_Write(const char *data, uint32_t len) {
  const uint32_t head = g_usb_tx_fifo.head;
  const uint32_t tail = g_usb_tx_fifo.tail;

  /* 计算可用空间（保留一个字节区分满/空） */
  uint32_t avail = (tail - head - 1) & TX_BUF_MASK;
  if (len > avail) {
    len = avail;
  }
  if (len == 0)
    return 0;

  /* 从 head 到缓冲区末尾的连续空间 */
  const uint32_t first = TX_BUF_SIZE - head;
  if (len <= first) {
    memcpy(&g_usb_tx_fifo.buffer[head], data, len);
  } else {
    memcpy(&g_usb_tx_fifo.buffer[head], data, first);
    memcpy(&g_usb_tx_fifo.buffer[0], data + first, len - first);
  }

  g_usb_tx_fifo.head = (head + len) & TX_BUF_MASK;
  return len;
}

/**
 * @brief 向 USB 发送缓冲区写入字符串
 * @param s 要写入的字符串
 */
void USB_TxBuf_WriteString(const char *s) {
  USB_TxBuf_Write(s, strlen(s));
}

/* 快速格式化 CAN 报文为 SLCAN 字符串 (查表法替代 sprintf) */
void SLCAN_FormatResponse_Fast(FDCAN_RxHeaderTypeDef *RxHeader,
                               uint8_t *RxData) {
  char msg[SLCAN_FRAME_TEXT_MAX_LEN];
  char *p = msg;
  const uint8_t isExt = (RxHeader->IdType == FDCAN_EXTENDED_ID);
  const uint8_t isRtr = (RxHeader->RxFrameType == FDCAN_REMOTE_FRAME);
  const uint8_t isBrs = (RxHeader->BitRateSwitch == FDCAN_BRS_ON);
  const uint8_t isFdc = (RxHeader->FDFormat == FDCAN_FD_CAN);
  const uint8_t dlc = (uint8_t)RxHeader->DataLength;

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
  const uint8_t len = GetPayloadLengthFromDlc(dlc);

  /* DLC 使用单个十六进制字符编码 */
  *p++ = hex_table[dlc];

  /* 数据字段 */
  if (!isRtr) {
    for (uint8_t i = 0; i < len; i++) {
      *p++ = hex_table[RxData[i] >> 4];
      *p++ = hex_table[RxData[i] & 0x0F];
    }
  }
  *p++ = '\r';

  USB_TxBuf_Write(msg, (uint32_t)(p - msg));
  App_NotifyCanRxActivity();
}

/* 打开 CAN 通道 */
CAN_Status_t CanOpen(void) {
  StopCanController();

  hfdcan1.Init.Mode =
      mCfg.isSilentMode ? FDCAN_MODE_BUS_MONITORING : FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission =
      mCfg.autoRetransmission ? ENABLE : DISABLE;

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
    LED_STATE_OFF();
    return CAN_ERR_HAL_INIT;
  }

  /* 配置过滤器：接收所有标准帧和扩展帧 */
  if (HAL_FDCAN_ConfigGlobalFilter(
          &hfdcan1,
          FDCAN_ACCEPT_IN_RX_FIFO0, /* 非匹配标准帧 */
          FDCAN_ACCEPT_IN_RX_FIFO0, /* 非匹配扩展帧 */
          FDCAN_FILTER_REMOTE,      /* 远程标准帧 */
          FDCAN_FILTER_REMOTE) != HAL_OK) {
    LED_STATE_OFF();
    return CAN_ERR_HAL_INIT;
  }

  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
    LED_STATE_OFF();
    return CAN_ERR_HAL_START;
  }

  /* 启用接收中断 */
  if (HAL_FDCAN_ActivateNotification(
          &hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_BUS_OFF, 0) !=
      HAL_OK) {
    LED_STATE_OFF();
    return CAN_ERR_HAL_START;
  }

  mCfg.isOpen = 1;
  ResetBusRecoveryState();
  App_SetCanOnline(1);
  return CAN_OK;
}

/* 关闭 CAN 通道 */
void CanClose(void) {
  StopCanController();
  ResetBusRecoveryState();
  App_SetCanOnline(0);
}

/* 处理 USB 发来的 SLCAN 指令 */
CAN_Status_t SLCAN_ProcessCommand(char *cmd, char *response) {
  if (cmd == NULL || response == NULL || cmd[0] == '\0') {
    return CAN_ERR_PARAM_INVALID;
  }

  const char type = cmd[0];

  switch (type) {
  /* 设置标准波特率 S0-S8 */
  case 'S':
    return HandleStandardBitrateCommand(cmd);

  /* 设置 FD 数据波特率 Yn，对应 n Mbps */
  case 'Y':
    return HandleDataBitrateCommand(cmd);

  /* 设置工作模式: M0 正常, M1 静默 */
  case 'M':
    return HandleModeCommand(cmd);

  /* 设置自动重发: A0 关闭, A1 开启 */
  case 'A':
    return HandleAutoRetransmissionCommand(cmd);

  /* 打开 CAN 通道 */
  case 'O':
    if (!HasNoTrailingArgs(cmd)) {
      return CAN_ERR_PARAM_INVALID;
    }
    return CanOpen();

  /* 关闭 CAN 通道 */
  case 'C':
    if (!HasNoTrailingArgs(cmd)) {
      return CAN_ERR_PARAM_INVALID;
    }
    CanClose();
    return CAN_OK;

  /* 查询版本 + MCU UID */
  case 'V':
    if (!HasNoTrailingArgs(cmd)) {
      return CAN_ERR_PARAM_INVALID;
    }
    BuildVersionResponse(response);
    return CAN_OK;

  /* 查询序列号 */
  case 'N':
    if (!HasNoTrailingArgs(cmd)) {
      return CAN_ERR_PARAM_INVALID;
    }
    BuildSerialResponse(response);
    return CAN_OK;

  /* 查询状态标志 */
  case 'F':
    if (!HasNoTrailingArgs(cmd)) {
      return CAN_ERR_PARAM_INVALID;
    }
    strcpy(response, "F00\r");
    return CAN_OK;

  /* 查询当前 TEC/REC/LEC */
  case 'E':
    if (!HasNoTrailingArgs(cmd)) {
      return CAN_ERR_PARAM_INVALID;
    }
    BuildErrorStatusResponse(response);
    return CAN_OK;

  /* 发送数据帧/远程帧: t/T/r/R */
  case 't':
  case 'T':
  case 'd':
  case 'D':
  case 'b':
  case 'B':
  case 'r':
  case 'R':
    return HandleTxCommand(cmd, type);

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
      /* 立即进入 INIT 模式停止控制器，延迟恢复交给主循环 */
      SET_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
      s_busoff_recovery_pending = 1;
      s_busoff_recovery_deadline = 0;
      App_SetCanOnline(0);
    }
  }
}

/**
 * @brief 检测 FDCAN 总线状态，在 TIM3 中断（1 秒）中调用
 * @note  - 总线离线超过 BUS_OFFLINE_THRESHOLD 秒后重启总线
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
    App_SetCanOnline(0);

    if (s_bus_offline_secs >= BUS_OFFLINE_THRESHOLD) {
      s_bus_offline_secs = 0;
      CanOpen(); /* 重启总线 */
    }
  } else {
    s_bus_offline_secs = 0;
    App_SetCanOnline(1);
  }
}

/**
 * @brief Bus-Off 延迟恢复，在主循环中轮询调用
 * @note  ISR 中仅设置 INIT 位和标志，恢复延时在主循环中非阻塞完成
 */
void FDCAN_PollBusOffRecovery(void) {
  const uint32_t current_tick = HAL_GetTick();

  if (!s_busoff_recovery_pending) {
    return;
  }

  if (s_busoff_recovery_deadline == 0U) {
    s_busoff_recovery_deadline = current_tick + BUS_OFF_RECOVERY_DELAY_MS;
    return;
  }

  if ((int32_t)(current_tick - s_busoff_recovery_deadline) < 0) {
    return;
  }

  s_busoff_recovery_pending = 0;
  s_busoff_recovery_deadline = 0;
  (void)CanOpen();
}
