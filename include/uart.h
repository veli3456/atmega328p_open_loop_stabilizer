#ifndef UART_H
#define UART_H

void uart_init (void);
void uart_send_str (const char *str);
void uart_send_hex8 (uint8_t val);
void uart_send_dec16 (uint16_t val);


#endif