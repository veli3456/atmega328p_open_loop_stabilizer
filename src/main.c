#include "twi.h"
#include "mpu6050.h"
#include <stddef.h>
#include <stdlib.h>
#include <util/delay.h>
#include "timer0.h"
#include "servo.h"
#include <avr/io.h>
#include "twi.h"

#define DT_FREQ_HZ 1000.0f
#define DT_SEC (1.0f / DT_FREQ_HZ) // Pre-computed as 0.01f by the compiler
#define DT_MS (1000 / DT_FREQ_HZ) // 1 ms

uint32_t prev_time;

mpu6050_raw_t raw_data = {0};
processed_gyro_accel_data_t processed_data = {0};
fused_angle_t fused_angle = {0};
servo_angle_t servo_angle = {0};

mpu6050_status_t status;

int main (void) {

    status = TWI_INIT(NULL);
    /*
    if (status == TWI_ERROR_INVALID_PARAM) {
        return 1;
    }
        */

    status = mpu6050_init(NULL);
    if (status == MPU6050_INIT_FAILURE) {
        return 1;
    }

    timer0_init();
    SERVO_INIT();
    
    while (1) { 


        if (get_time() - prev_time > DT_MS) {

            status = mpu6050_read_sensor_data(NULL, &raw_data);

            if (status == MPU6050_SENSOR_DATA_READ_SUCCESSFULL) { // sensor data read fail

                MPU6050_ReadScaled(&raw_data, &processed_data);
                mpu6050_compute_fused_angles(&fused_angle, &processed_data, DT_SEC);
                servo_change_angle((servo_angle_t*)&fused_angle); // converts angles into milliseconds and assign them to OCR1x to change duty cycle
                
            }
            prev_time = get_time();
        }
            
    }

    return 0;
}



