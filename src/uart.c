#include <avr/io.h>

#define BAUD_RATE 115200UL
#define SAMPLE_RATE 8UL // Double Speed Mode
#define UBRR_VAL (F_CPU / (SAMPLE_RATE * BAUD_RATE) - 1)

void uart_init (void) {
    UBRR0 = UBRR_VAL; // UBRR0 set to 16 for 115200 baud @ 16 MHz with U2X0 enable
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

void uart_send_hex8 (uint8_t val) {
    static const char hex_digits[] = "0123456789ABCDEF";

    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = hex_digits[(val >> 4) & 0x0F]; // Send high nibble

    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = hex_digits[val & 0x0F];        // Send low nibble
}

void uart_send_dec16 (uint16_t val) {
    static const char base_10[] = "0123456789";
    char str[6] = {0};

    str [0] = base_10[val % 10]; // Let's say 65532, 2 is assigned here
    val /= 10; // 6553
    str [1] = base_10[val % 10]; // We get the 3
    val /= 10; // 655
    str [2] = base_10[val % 10]; // We get the 5
    val /= 10; // 65
    str [3] = base_10[val % 10]; // We get the 5 
    val /= 10; // 6 
    str [4] = base_10[val % 10]; // We get the 6 

    for (int8_t i = 4; i >= 0; i--) { // int8_t ensures it doesn't overflow to 255 like unsigned numbers when it gets less than 0.
        while ( !(UCSR0A & (1<<UDRE0)) ); // If UDRE0 == 1, buffer is empty
        UDR0 = str[i];
    }
}
