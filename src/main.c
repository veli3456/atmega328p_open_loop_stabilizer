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

    mpu6050_status_t status_mpu6050 = 0;
    twi_result_t status_twi = 0;
    servo_status_t status_servo = 0;

    uint32_t prev_time = 0;

    mpu6050_raw_t raw_data = {0};
    processed_gyro_accel_data_t processed_data = {0};
    fused_angle_t fused_angle = {0};

    status_twi = TWI_INIT(NULL);
    if (status_twi != TWI_INIT_SUCCESSFULL) {
        FATAL_ERROR(status_twi);
    }
    status_mpu6050 = mpu6050_init(NULL);
    if (status_mpu6050 != MPU6050_INIT_SUCCESSFULL) {
        FATAL_ERROR(status_mpu6050);
    }

    timer0_init();
    SERVO_INIT();
    
    while (1) { 

        if (ELAPSED_TIME(prev_time) > SAMPLING_PERIOD_MS) {

            status_mpu6050 = mpu6050_read_sensor_data(NULL, &raw_data);

            if (status_mpu6050 == MPU6050_SENSOR_DATA_READ_SUCCESSFULL) { // sensor data read fail

                MPU6050_ReadScaled(&raw_data, &processed_data);
                mpu6050_compute_fused_angles(&fused_angle, &processed_data, DELTA_T);

                status_servo = servo_change_angle(fused_angle.fused_roll, SERVO_1); // converts angles into milliseconds and assign them to OCR1x to change duty cycle

                if (status_servo != SUCCESS_SERVO) {
                    FATAL_ERROR(status_servo);
                }

                status_servo = servo_change_angle(fused_angle.fused_pitch, SERVO_2);

                if (status_servo != SUCCESS_SERVO) {
                    FATAL_ERROR(status_servo);
                }
            }

            prev_time = get_time();
        }   
    }
    return 0;
}



