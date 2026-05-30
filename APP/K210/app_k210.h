#ifndef __APP_K210_H_
#define __APP_K210_H_

#include "AllHeader.h"

void Deal_K210_QR(uint8_t recv_msg);
void Change_state_QR(void);


void Deal_K210_self(uint8_t recv_msg);
void Change_state_self(void);

void Deal_K210_minst(uint8_t recv_msg);
void Change_state_minst(void);

#endif

