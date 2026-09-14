#include "twi.h"
#include "mpu6050.h"
#include "timer0.h"
#include "servo.h"
#include "uart.h"
#include "error_handler.h"
#include <stddef.h>

#define SAMPLE_FREQ_HZ 1000UL // 1000 Hz
#define SAMPLING_PERIOD_MS (1000UL / SAMPLE_FREQ_HZ) // 1 ms
#define DELTA_T (1.0f / (float)SAMPLE_FREQ_HZ) // Pre-computed as 0.001f secs by the compiler
#define ELAPSED_TIME(prev_time) (get_time() - (prev_time))

static mpu6050_raw_t raw_data = {0};
static processed_gyro_accel_data_t processed_data = {0};
static fused_angle_t fused_angle = {0};

void system_init (void);
void read_sensor (mpu6050_raw_t *raw_data);
void process_data (const mpu6050_raw_t *raw_data, fused_angle_t *fused_angle, processed_gyro_accel_data_t *processed_data);
void servo_angle_change(const fused_angle_t *fused_angle);

typedef enum {
    SENSOR_READ,
    DATA_PROCESS,
    SERVO_ANGLE_CHANGE
}state_t;

state_t state;


int main (void) {

    system_init();
    state = SENSOR_READ;
    
    while (1) { 

        switch (state) {
        case SENSOR_READ:
            read_sensor(&raw_data);
            break;
        case DATA_PROCESS:
            process_data(&raw_data, &fused_angle, &processed_data);
            break;
        case SERVO_ANGLE_CHANGE:
            servo_angle_change(&fused_angle);
            break;
        }
    }
    return 0;
}


void system_init (void) {

    uart_init(); // required for error handling, might move to the error module, need to be at top because program can enter the error loop
    timer0_init();
    SERVO_INIT();

    twi_result_t status_twi = TWI_INIT(NULL);
    if (status_twi != TWI_INIT_SUCCESSFULL) {
        FATAL_ERROR(status_twi);
    }
    mpu6050_status_t status_mpu6050 = mpu6050_init(NULL);
    if (status_mpu6050 != MPU6050_INIT_SUCCESSFULL) {
        FATAL_ERROR(status_mpu6050);
    }
}

void read_sensor (mpu6050_raw_t *raw_data) {

    static uint32_t prev_time = 0;

    if (ELAPSED_TIME(prev_time) > SAMPLING_PERIOD_MS) {

        mpu6050_status_t status_mpu6050 = mpu6050_read_sensor_data(NULL, raw_data);

        if (status_mpu6050 == MPU6050_SENSOR_DATA_READ_SUCCESSFULL) { // sensor data read fail
            state = DATA_PROCESS;
        }
        else {
            FATAL_ERROR(status_mpu6050); // Handle Error
        }

        prev_time = get_time();
    } 
}

void process_data (const mpu6050_raw_t *raw_data, fused_angle_t *fused_angle, processed_gyro_accel_data_t *processed_data) {

    MPU6050_ReadScaled(raw_data, processed_data);
    mpu6050_compute_fused_angles(fused_angle, processed_data, DELTA_T);

    state = SERVO_ANGLE_CHANGE;
}

void servo_angle_change(const fused_angle_t *fused_angle) {

    servo_status_t status_servo = servo_change_angle(fused_angle->fused_roll, SERVO_1); // converts angles into milliseconds and assign them to OCR1x to change duty cycle

    if (status_servo != SUCCESS_SERVO) {
        FATAL_ERROR(status_servo);
    }

    status_servo = servo_change_angle(fused_angle->fused_pitch, SERVO_2);

    if (status_servo != SUCCESS_SERVO) {
        FATAL_ERROR(status_servo);
    }

    state = SENSOR_READ;
}