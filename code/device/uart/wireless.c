#include "wireless.h"

void wireless_init() {
    uart_init(UART_1, 115200, UART1_TX_P20_10, UART1_RX_P20_9);
}