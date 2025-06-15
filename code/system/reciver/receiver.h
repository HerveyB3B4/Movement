#ifndef _RECIVER_H_
#define _RECIVER_H_

#include "zf_common_headfile.h"

#define RECIVER_UART_INDEX UART_2
#define RECIVER_UART_TX (UART2_RX_P14_3)
#define RECIVER_UART_RX (UART2_TX_P14_2)
#define RECIVER_UART_BAUDRATE 115200

typedef union
{
    float value;
    uint32 bytes;
} float_converter;

void receiver_init();
void receiver_callback();

extern int16 g_received_vel; // 接收的速度数据
extern int16 g_turn_error;   // 接收的转向误差数据

#endif