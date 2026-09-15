#include <avr/io.h>
#include "servo.h"

#define PRESCALER 8UL
#define TIMER1_FREQ (F_CPU / PRESCALER)
#define TIMER1_TICKS_PER_MS (TIMER1_FREQ / 1000UL)

#define SERVO_HERTZ 50UL
#define SERVO_CENTER_MS 1.5f
#define SERVO_LOWEST_ANGLE -90.0f
#define SERVO_HIGHEST_ANGLE 90.0f
#define SERVO_OFFSET_MS(servo_angle) ((servo_angle) / SERVO_HIGHEST_ANGLE) // between -1ms and 1ms
#define SERVO_MS_TO_OCR(servo_angle_ms) ((uint16_t)(((servo_angle_ms) * TIMER1_TICKS_PER_MS) - 1UL))
#define SERVO_PWM_PERIOD_TICKS ((TIMER1_FREQ / SERVO_HERTZ) - 1UL)

// Produces a 50 hz (20ms period) PWM signal for two servos 
void SERVO_INIT (void) {

    // ******* TIMER1 INIT ********
    TCCR1A = (1<<WGM11); // Fast PWM TOP ICR1
    TCCR1A &= ~(1<<WGM10);
    TCCR1B = (1<<WGM12) | (1<<WGM13);

    TCCR1A |= (1<<COM1B1) | (1<<COM1A1); // Clear OC1B/A on Compare Match, Set at Bottom

    TCCR1B |= (1<<CS11); // Prescaler 8, timer freq = 2mHz

    ICR1 = SERVO_PWM_PERIOD_TICKS; // TOP 20ms (50hz)
    OCR1A = SERVO_MS_TO_OCR(SERVO_CENTER_MS); // Compare Match A exactly 1.5ms (0 degrees) by default 2999
    OCR1B = SERVO_MS_TO_OCR(SERVO_CENTER_MS); // Compare Match B exactly 1.5ms (0 degrees) by default
    TCNT1 = 0; // Timer register set to 0 

    DDRB |= (1<<DDB1); // OC1A (Servo1) pin set to O/P
    DDRB |= (1<<DDB2); // OC1B (Servo2) pin set to O/P

}

servo_status_t servo_change_angle (float servo_angle, servo_id_t servo_num) {

    if (servo_angle > SERVO_HIGHEST_ANGLE) { // If the servo angle goes beyond +90
        servo_angle = SERVO_HIGHEST_ANGLE;
    }
    if (servo_angle < SERVO_LOWEST_ANGLE) { // If the angle goes below -90
        servo_angle = SERVO_LOWEST_ANGLE;
    }

    // Calculate the angle ms between 0.5ms and 2.5ms
    float servo_angle_ms = SERVO_CENTER_MS + SERVO_OFFSET_MS(servo_angle); // Servo 1 PWM signal

    if (servo_num  == SERVO_1) {
        OCR1A = SERVO_MS_TO_OCR(servo_angle_ms); // between 1999-3999 corresponding to 1-2ms
    }
    else if (servo_num == SERVO_2) {
        OCR1B = SERVO_MS_TO_OCR(servo_angle_ms); // between 1999-3999 corresponding to 1-2ms
    }
    else {
        return ERR_INVALID_SERVO_ID;
    }

    return SUCCESS_SERVO;
}
