#ifndef _RECIVER_H_
#define _RECIVER_H_

#include "zf_common_headfile.h"

#define RECIVER_UART_INDEX 0
#define RECIVER_UART_TX
#define RECIVER_UART_RX
#define RECIVER_UART_BAUDRATE 115200

void reciver_init();
void reciver_callback();

#endif