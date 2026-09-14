#include <avr/io.h>

void uart_init (void) {
    UBRR0 = 16; // UBRR0 set to 16 for 115200 baud @ 16 MHz with U2X0 enable
    UCSR0A |= (1 << U2X0); // Double Speed enabled (sample rate = 8) (required to keep error rate around 2%)
    UCSR0B |= (1 << TXEN0); // Enable transmitter
    UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00); // Set frame format: 8 data bits, 1 stop bit, no parity
}

void uart_send_str (const char *str) {
    while (*str) { // *str evaluates to false when it hits '\0'
        while ( !(UCSR0A & (1<<UDRE0)) ); // If UDRE0 == 1, buffer is empty
        UDR0 = *str++;
    }
}

