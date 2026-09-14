#include "twi.h"
#include "mpu6050.h"
#include "timer0.h"
#include "servo.h"
#include "error_handler.h"
#include <stddef.h>

#define SAMPLE_FREQ_HZ 1000UL // 1000 Hz
#define SAMPLING_PERIOD_MS (1000UL / SAMPLE_FREQ_HZ) // 1 ms
#define DELTA_T (1.0f / (float)SAMPLE_FREQ_HZ) // Pre-computed as 0.001f secs by the compiler
#define ELAPSED_TIME(prev_time) (get_time() - (prev_time))


int main (void) {

    mpu6050_status_t status;

    uint32_t prev_time = 0;

    mpu6050_raw_t raw_data = {0};
    processed_gyro_accel_data_t processed_data = {0};
    fused_angle_t fused_angle = {0};

    if (TWI_INIT(NULL) == TWI_ERROR_INVALID_PARAM) {
        FATAL_ERROR(TWI_ERROR_INVALID_PARAM);
    }
    if (mpu6050_init(NULL) == MPU6050_INIT_FAILURE) {
        FATAL_ERROR(MPU6050_INIT_FAILURE);
    }

    timer0_init();
    SERVO_INIT();
    
    while (1) { 

        if (ELAPSED_TIME(prev_time) > SAMPLING_PERIOD_MS) {

            status = mpu6050_read_sensor_data(NULL, &raw_data);

            if (status == MPU6050_SENSOR_DATA_READ_SUCCESSFULL) { // sensor data read fail
                MPU6050_ReadScaled(&raw_data, &processed_data);
                mpu6050_compute_fused_angles(&fused_angle, &processed_data, DELTA_T);
                servo_change_angle((servo_angle_t*)&fused_angle); // converts angles into milliseconds and assign them to OCR1x to change duty cycle
            }
            prev_time = get_time();
        }   
    }
    return 0;
}



