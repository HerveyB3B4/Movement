#ifndef _RECIVER_H_
#define _RECIVER_H_

#define RECIVER_UART_INDEX
#define RECIVER_UART_TX
#define RECIVER_UART_RX
#define RECIVER_UART_BAUDRATE 115200

void reciver_init(void);
void reciver_callback(void);

#endif