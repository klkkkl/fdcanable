#include "fdcan.h"
#include "main.h"

#include "cmsis_os2.h" // <<<---- 重要：RTOS2
#include <stdint.h>
#include <string.h>

#include "common.h"
#include "node_protocol.h"

#define MSG_RX_BUF_SIZE 512
#define MAX_RX_BUF_COUNT 2
#define NODE_SLAVE_ID_FLAG 0b00110000

struct _NodeMessageHead;
typedef struct _NodeMessageHead NodeMessageHead;

static uint8_t rxBufPool[MAX_RX_BUF_COUNT][MSG_RX_BUF_SIZE];

static osMessageQueueId_t mRxBufPoolQueue;
static osMessageQueueId_t mRxBufQueue;

void ProcessProtocolMessage(NodeMessageHead *message);

/*--------------------------
 *  Buffer Pool Alloc/Free
 *-------------------------*/
uint8_t *AllocRxBuf(void) {
  uint8_t *ptr = NULL;
  if (osMessageQueueGet(mRxBufPoolQueue, &ptr, NULL, 0) == osOK) {
    return ptr;
  }
  return NULL;
}

void FreeRxBuf(uint8_t *buf) { osMessageQueuePut(mRxBufPoolQueue, &buf, 0, 0); }

/*--------------------------
 *  FDCAN 帧长度 → DLC
 *-------------------------*/
uint32_t GetDataLengthFlag(uint8_t size) {
  if (size <= 8)
    return size;

  if (size <= 12)
    return FDCAN_DLC_BYTES_12;
  if (size <= 16)
    return FDCAN_DLC_BYTES_16;
  if (size <= 20)
    return FDCAN_DLC_BYTES_20;
  if (size <= 24)
    return FDCAN_DLC_BYTES_24;
  if (size <= 32)
    return FDCAN_DLC_BYTES_32;
  if (size <= 48)
    return FDCAN_DLC_BYTES_48;

  return FDCAN_DLC_BYTES_64;
}

void SendDataToMaster(uint8_t *data, uint8_t len) {
  FDCAN_TxHeaderTypeDef txHeader;

  txHeader.Identifier = 0;
  txHeader.IdType = FDCAN_STANDARD_ID;
  txHeader.TxFrameType = FDCAN_DATA_FRAME;
  txHeader.DataLength = GetDataLengthFlag(len);
  txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  txHeader.BitRateSwitch = FDCAN_BRS_OFF;
  txHeader.FDFormat = FDCAN_FD_CAN;
  txHeader.TxEventFifoControl = FDCAN_STORE_TX_EVENTS;
  txHeader.MessageMarker = 0;

  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, data) != HAL_OK) {
    Error_Handler();
  }
}

static uint8_t IsCanTxFifoQPending() {
  return HAL_FDCAN_IsTxBufferMessagePending(
      &hfdcan1, HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&hfdcan1));
}

/*--------------------------
 *  主解析任务（RTOS2）
 *-------------------------*/
static void ProcessNodeRequestTask(void *argument) {
  (void)argument;

  for (;;) {

    uint8_t *messageHead = NULL;

    if (osMessageQueueGet(mRxBufQueue, &messageHead, NULL, osWaitForever) ==
        osOK) {
      ProcessProtocolMessage((NodeMessageHead *)messageHead);
      FreeRxBuf(messageHead);
    }
  }
}

/*--------------------------
 *  帧拼接逻辑
 *-------------------------*/
void ProtocolProcessMsg(uint8_t *recvData, int len) {
  static int requestBufPos = 0;
  static uint8_t *requestBuf = NULL;
  static uint32_t lastRequestTime = 0;

  if (requestBuf == NULL) {
    requestBuf = AllocRxBuf();
    if (requestBuf == NULL) {
      Error_Handler();
      return;
    }
    requestBufPos = 0;
  }

  uint32_t currentTick = osKernelGetTickCount();
  if (currentTick - lastRequestTime > osKernelGetTickFreq() / 100) { // >10ms
    requestBufPos = 0;
  }
  lastRequestTime = currentTick;

  memcpy(requestBuf + requestBufPos, recvData, len);
  requestBufPos += len;

  if (requestBufPos < sizeof(NodeMessageHead))
    return;

  NodeMessageHead *messageHead = (NodeMessageHead *)requestBuf;
  int messageSize = messageHead->Size;

  if (requestBufPos >= messageSize) {

    // 投递给处理任务
    osMessageQueuePut(mRxBufQueue, &requestBuf, 0, 0);

    requestBuf = NULL;
    requestBufPos = 0;
  }
}

/*--------------------------
 *  FDCAN ISR
 *-------------------------*/
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo0ITs) {
  FDCAN_RxHeaderTypeDef rxHeader;
  static uint8_t rxData[64];

  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0) {
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, rxData) !=
        HAL_OK) {
      Error_Handler();
    }

    ProtocolProcessMsg(rxData, rxHeader.DataLength);
  }
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan,
                                   uint32_t ErrorStatusITs) {}

void HAL_FDCAN_TxEventFifoCallback(FDCAN_HandleTypeDef *hfdcan,
                                   uint32_t TxEventFifoITs) {}

/*--------------------------
 *  清除 TX FIFO
 *-------------------------*/
void NodeCleanPendingRequest() {
  while (IsCanTxFifoQPending()) {
    HAL_FDCAN_AbortTxRequest(&hfdcan1,
                             HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&hfdcan1));
  }
}

void NodeSetSlaveId(int id) {
  HAL_FDCAN_Stop(&hfdcan1);
  FDCAN_FilterTypeDef sFilterConfig;
  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterID1 = NODE_SLAVE_ID_FLAG | id;
  sFilterConfig.FilterID2 = 0x7FF;

  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
    Error_Handler();

  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    Error_Handler();
}
/*--------------------------
 *  模块初始化（RTOS2）
 *-------------------------*/
void NodeInit() {
  /* 创建消息队列 */
  mRxBufPoolQueue =
      osMessageQueueNew(MAX_RX_BUF_COUNT, sizeof(uint8_t *), NULL);
  mRxBufQueue = osMessageQueueNew(MAX_RX_BUF_COUNT, sizeof(uint8_t *), NULL);

  for (int i = 0; i < MAX_RX_BUF_COUNT; i++) {
    uint8_t *ptr = rxBufPool[i];
    osMessageQueuePut(mRxBufPoolQueue, &ptr, 0, 0);
  }

  /* 创建处理任务 */
  const osThreadAttr_t thread_attr = {.name = "ProcessNodeRequest",
                                      .priority = osPriorityAboveNormal,
                                      .stack_size = 512};
  osThreadNew(ProcessNodeRequestTask, NULL, &thread_attr);

  /* 配置 FDCAN 过滤器 */
  FDCAN_FilterTypeDef sFilterConfig;
  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterID1 = 0;
  sFilterConfig.FilterID2 = 0x7FF;

  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
    Error_Handler();

  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
                                   FDCAN_FILTER_REMOTE,
                                   FDCAN_FILTER_REMOTE) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_FDCAN_ActivateNotification(
          &hfdcan1,
          FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_TX_FIFO_EMPTY |
              FDCAN_IT_TIMEOUT_OCCURRED | FDCAN_IT_TX_ABORT_COMPLETE,
          0) != HAL_OK)
    Error_Handler();

  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    Error_Handler();
}
