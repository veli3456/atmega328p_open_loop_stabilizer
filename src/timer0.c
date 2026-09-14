#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

volatile uint32_t time_var;

void timer0_init (void) {
    TCCR0A = (1<<WGM01); // CTC
    TCCR0B = (1<<CS00) | (1<<CS01); // Prescaler 64, timer freq = 250kHz
    OCR0A = 249; // 1ms
    TIMSK0 = (1<<OCIE0A); // Timer0 compare match a interrupt enable
    sei(); // global interrupt enable
}

ISR (TIMER0_COMPA_vect) {
    time_var++;
}

uint32_t get_time (void) {
    uint32_t temp_var;
    
    // ATOMIC_BLOCK creates a block of code that's guaranteed to be executed atomically
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { // Upon entering the block, the global interrupt flag in SREG is disabled, and re-enabled
        temp_var = time_var;                // upon exiting the block 
        // ATOMIC_RESTORSTATE restores the previous state of SREG
    }
    return temp_var;
}