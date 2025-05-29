#include "wireless.h"

void wireless_init()
{
    uart_init(UART_1, 115200, UART1_TX_P20_10, UART1_RX_P20_9);
    uart_rx_interrupt(UART_1, 1);
    uart_tx_interrupt(UART_1, 1);
    uart_write_string(UART_1, "wireless_uart_init success\n");
}