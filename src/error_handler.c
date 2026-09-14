#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "uart.h"

#define FATAL_ERROR(code) fatal_error_trap((code), __FILE__, __LINE__)

void fatal_error_trap(uint8_t err_code, const char* file, uint16_t line) {
    cli(); // Disable interfering background tasks

    // MCU-to-PC Diagnostic Output
    uart_send_str("\r\n[FATAL ERROR 0x");
    uart_send_hex8(err_code);
    uart_send_str("] File: ");
    uart_send_str(file);
    uart_send_str(" Line: ");
    uart_send_dec16(line);
    uart_send_str("\r\n");

    DDRB |= (1<<DDB0);

    // Visual Error Handling
    while (1) {
        // Blink LED err_code times (3 blinks = Code 3)
        for (uint8_t i = 0; i < err_code; i++) {
            PORTB |= (1<<PB0);
            _delay_ms(700);
            PORTB &= ~(1<<PB0);
            _delay_ms(700);
        }
        _delay_ms(1500); // Pause between blink patterns
    }
}