#include "msg.h"
#include "cmsis_os2.h"
#include "usbd_cdc_if.h"

#define MSG_NUM 10
#define USB_SEDN_CPL_EVENT 0x1

#define MSG_RX_BUF_NUM 10
#define MSG_TX_BUF_NUM 10
#define MSG_RX_MAX_LEN 256
#define MSG_TX_MAX_LEN 256

typedef struct _MsgBuf {
  int len;
  uint8_t *data;
} MsgBuf;

static osMessageQueueId_t mReciveQueue;
static osMessageQueueId_t mSendQueue;

static osEventFlagsId_t mRequestEvent = NULL;

static m_inited = 0;
const osThreadAttr_t mReciveThreadAttr = {
    .name = "mReciveThreadAttr",
    .stack_size = 1524,
    .priority = osPriorityAboveNormal,
};

const osThreadAttr_t mSendThreadAttr = {
    .name = "mSendThreadAttr",
    .stack_size = 1524,
    .priority = osPriorityAboveNormal,
};

osMemoryPoolId_t mRxMemPool;
osMemoryPoolId_t mTxMemPool;

void AppProcessMsg(uint8_t *data, int len);
void ProtocolProcessMsg(uint8_t *recvData, int len);

void ReceiveData(uint8_t *data, int len) {
  if (m_inited) {
    MsgBuf msg;

    msg.data = data;
    msg.len = len;
    if (osOK != osMessageQueuePut(mReciveQueue, &msg, 0, 0)) {
      if (msg.data != NULL) {
        osMemoryPoolFree(mRxMemPool, msg.data);
      }
    }
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
  }
}

void MsgSendData(uint8_t *data, int len) {
  if (m_inited) {
    MsgBuf msg;

    msg.data = data;
    msg.len = len;
    if (osOK != osMessageQueuePut(mSendQueue, &msg, 0, 0)) {
      if (msg.data != NULL) {
        osMemoryPoolFree(mTxMemPool, msg.data);
      }
    }
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
  }
}

void vSenderTask(void *pvParameters) {
  for (;;) {
    MsgBuf msg;
    if (osOK == osMessageQueueGet(mSendQueue, &msg, NULL, osWaitForever)) {
      osEventFlagsClear(mRequestEvent, USB_SEDN_CPL_EVENT);
      uint8_t res = CDC_Transmit_FS(msg.data, msg.len);
      osEventFlagsWait(mRequestEvent, USB_SEDN_CPL_EVENT, osFlagsWaitAny, 100);

      osMemoryPoolFree(mTxMemPool, msg.data);
    }
  }
}

void vReceiverTask(void *pvParameters) {

  for (;;) {
    MsgBuf msg;
    if (osOK == osMessageQueueGet(mReciveQueue, &msg, NULL, osWaitForever)) {

      ProtocolProcessMsg(msg.data, msg.len);
      osMemoryPoolFree(mRxMemPool, msg.data);
    }
  }
}

void MsgSendDataCompleted() {
  uint32_t res = osEventFlagsSet(mRequestEvent, USB_SEDN_CPL_EVENT);
}

uint8_t *MsgAllocRxBuf() { return (uint8_t *)osMemoryPoolAlloc(mRxMemPool, 0); }
uint8_t *MsgAllocTxBuf() { return (uint8_t *)osMemoryPoolAlloc(mTxMemPool, 0); }

void MsgInit(void) {
  mRxMemPool = osMemoryPoolNew(MSG_RX_BUF_NUM, MSG_RX_MAX_LEN, NULL);
  mTxMemPool = osMemoryPoolNew(MSG_TX_BUF_NUM, MSG_TX_MAX_LEN, NULL);

  mReciveQueue = osMessageQueueNew(MSG_NUM, sizeof(MsgBuf), NULL);
  mSendQueue = osMessageQueueNew(MSG_NUM, sizeof(MsgBuf), NULL);

  osThreadNew(vSenderTask, NULL, &mReciveThreadAttr);
  osThreadNew(vReceiverTask, NULL, &mSendThreadAttr);
  mRequestEvent = osEventFlagsNew(NULL);
  m_inited = 1;
}