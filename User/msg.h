#ifndef MSG_H_
#define MSG_H_

#include "main.h"

void MsgSendDataCompleted();
void MsgReceiveData(uint8_t *msg, int len);
void MsgSendData(uint8_t *msg, int len);
uint8_t *MsgAllocRxBuf();
uint8_t *MsgAllocTxBuf();
void MsgInit(void);

#endif // MSG_H_