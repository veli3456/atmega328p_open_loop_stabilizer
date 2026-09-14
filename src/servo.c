#include <avr/io.h>
#include "servo.h"
#include "uart.h"
#include <stdlib.h>

#define ONE_MS 2000.0f

#define LOWEST_SERVO_ANGLE -90.0f
#define HIGHEST_SERVO_ANGLE 90.0f


// Produces a 50 hz (20ms period) PWM signal for two servos 
void SERVO_INIT (void) {

    // ******* TIMER1 INIT ********
    TCCR1A = (1<<WGM11); // Fast PWM TOP ICR1
    TCCR1A &= ~(1<<WGM10);
    TCCR1B = (1<<WGM12) | (1<<WGM13);

    TCCR1A |= (1<<COM1B1) | (1<<COM1A1); // Clear OC1B/A on Compare Match, Set at Bottom

    TCCR1B |= (1<<CS11); // Prescaler 8, timer freq = 2mHz

    ICR1 = 39999; // TOP 20ms (50hz)
    OCR1A = 2999; // Compare Match A exactly 1.5ms (90 degrees) by default
    OCR1B = 2999; // Compare Match B exactly 1.5ms (90 degrees) by default
    TCNT1 = 0; // Timer register set to 0 

    DDRB |= (1<<DDB1); // OC1A (Servo1/Roll) pin set to O/P
    DDRB |= (1<<DDB2); // OC1B (Servo2/Pitch) pin set to O/P

}

void servo_change_angle (servo_angle_t *servo_angle) {


    if (servo_angle->pitch > HIGHEST_SERVO_ANGLE) {
        servo_angle->pitch = HIGHEST_SERVO_ANGLE;
    }
    if (servo_angle->pitch < LOWEST_SERVO_ANGLE) { // If the servo pitch drops below 0 degrees
        servo_angle->pitch = LOWEST_SERVO_ANGLE;
    }

    if (servo_angle->roll > HIGHEST_SERVO_ANGLE) {
        servo_angle->roll = HIGHEST_SERVO_ANGLE;
    }
    if (servo_angle->roll < LOWEST_SERVO_ANGLE) {
        servo_angle->roll = LOWEST_SERVO_ANGLE;
    }

    // Converts angle between 0-180 to a number between 1 to 2ms
    float servo_roll_ms = (2 * (servo_angle->roll / 180.0f)) + 1.5f; // Servo 1 PWM signal
    float servo_pitch_ms = (2 * (servo_angle->pitch / 180.0f)) + 1.5f; // Servo 2 PWM signal

    OCR1A = servo_roll_ms * ONE_MS - 1; // between 1999-3999 corresponding to 1-2ms
    OCR1B = servo_pitch_ms * ONE_MS - 1; // between 1999-3999 corresponding to 1-2ms

}
