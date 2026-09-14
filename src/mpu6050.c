#include "mpu6050.h"
#include <avr/io.h>
#include <twi.h>
#include <stddef.h>
#include <math.h>

// -------------------------------------------------------------
// MPU6050 Static Calibration Biases (Measured at rest)
// -------------------------------------------------------------
#define MPU6050_GYRO_X_BIAS    3.9165f   // deg/s
#define MPU6050_GYRO_Y_BIAS   -0.6495f   // deg/s
#define MPU6050_GYRO_Z_BIAS    0.35f   // deg/s

#define MPU6050_ACCEL_X_OFFSET -0.0564f // g 
#define MPU6050_ACCEL_Y_OFFSET 0.0027f // g 
#define MPU6050_ACCEL_Z_OFFSET  0.0327f // g

static const mpu6050_cfg_t imu_cfg = {
    .slave_addr = MPU6050_ADDR_AD0_LOW,
    .dlpf_mode = DLPF_MODE_2,
    .accel_fs = ACCEL_FS_2G,
    .gyro_fs = GYRO_FS_250
};

typedef enum {
    PWR_MGMT_1 = 0x6B,
    GYRO_CONFIG = 0x1B,
    ACCEL_CONFIG = 0x1C,
    CONFIG = 0x1A,
    ACCEL_XOUT = 0x3B
} mpu6050_registers_t;


mpu6050_status_t mpu6050_init (const mpu6050_cfg_t *config) {
    if (config == NULL) {
        config = &imu_cfg;
    }

    twi_result_t status;

    status = TWI_WRITE_REGISTER(config->slave_addr, PWR_MGMT_1, 0x01); // Wake up MPU-6050 & select X-axis gyro reference for stability
    if (status != TWI_SUCCESSFULL) {
        return MPU6050_INIT_FAILURE;
    }

    uint8_t gyro_range_val = (uint8_t)(config->gyro_fs << 3);
    status = TWI_WRITE_REGISTER(config->slave_addr, GYRO_CONFIG, gyro_range_val); // Gyro Full Range Set to -+250d/s
    if (status != TWI_SUCCESSFULL) {
        return MPU6050_INIT_FAILURE;
    }

    uint8_t accel_range_val = (uint8_t)(config->accel_fs << 3);
    status = TWI_WRITE_REGISTER(config->slave_addr, ACCEL_CONFIG, accel_range_val); 
    if (status != TWI_SUCCESSFULL) {
        return MPU6050_INIT_FAILURE;
    }

    status = TWI_WRITE_REGISTER(config->slave_addr, CONFIG, config->dlpf_mode); // Set DLPF to level 2
    if (status != TWI_SUCCESSFULL) {
        return MPU6050_INIT_FAILURE;
    }

    return MPU6050_INIT_SUCCESSFULL;
}

mpu6050_status_t mpu6050_read_sensor_data (const mpu6050_cfg_t *config, mpu6050_raw_t *data) {
     if (config == NULL) {
        config = &imu_cfg;
    }
    if (data == NULL) {
        return MPU6050_SENSOR_DATA_READ_FAILURE;
    }
    uint8_t buffer[14];
    twi_result_t status;

    status = TWI_READ_ADJACENT_REGISTER (config->slave_addr, ACCEL_XOUT, 14, buffer);
    if (status != TWI_SUCCESSFULL) {
        return MPU6050_SENSOR_DATA_READ_FAILURE;
    }

    // Unpack Big-Endian sensor data into raw struct fields
    data->accel_x = (int16_t)((uint16_t)buffer[0] << 8 | buffer[1]);
    data->accel_y = (int16_t)((uint16_t)buffer[2] << 8 | buffer[3]);
    data->accel_z = (int16_t)((uint16_t)buffer[4] << 8 | buffer[5]);

    // buffer[6] & buffer[7] (Temperature) ignored

    data->gyro_x = (int16_t)((uint16_t)buffer[8] << 8 | buffer[9]);
    data->gyro_y = (int16_t)((uint16_t)buffer[10] << 8 | buffer[11]);
    data->gyro_z = (int16_t)((uint16_t)buffer[12] << 8 | buffer[13]);

    return MPU6050_SENSOR_DATA_READ_SUCCESSFULL;
}

void MPU6050_ReadScaled (const mpu6050_raw_t *raw_data, processed_gyro_accel_data_t *processed_data) {
    processed_data->accel_x = ((float)raw_data->accel_x / 16384.0f) + MPU6050_ACCEL_X_OFFSET; // accel data between -2.0 and 2.0g
    processed_data->accel_y = ((float)raw_data->accel_y / 16384.0f) + MPU6050_ACCEL_Y_OFFSET;
    processed_data->accel_z = ((float)raw_data->accel_z / 16384.0f) + MPU6050_ACCEL_Z_OFFSET;
    processed_data->gyro_x = ((float)raw_data->gyro_x / 131.0f) + MPU6050_GYRO_X_BIAS; // gyro data between -250.0 and +250.0 degrees/s
    processed_data->gyro_y = ((float)raw_data->gyro_y / 131.0f) + MPU6050_GYRO_Y_BIAS;
    processed_data->gyro_z = ((float)raw_data->gyro_z / 131.0f) + MPU6050_GYRO_Z_BIAS;
} 

void mpu6050_compute_fused_angles(fused_angle_t *fused_angle, processed_gyro_accel_data_t *processed_data, float dt_sec) {

    // square root of the sum of squared y and and squared z produces an angle output range of -90 degrees to 90 degrees.
    // Just accel z as the second parameter to atan2 outputs a range between -180 degrees to 180 degrees
    // Compute accel radian for x
    float roll_rad = atan2f(processed_data->accel_x, processed_data->accel_z);

    // Compute accel radian for y
    float pitch_rad = atan2f (processed_data->accel_y, processed_data->accel_z);

    float roll_deg = roll_rad * (180.0f / (float)M_PI);
    float pitch_deg = pitch_rad * (180.0f / (float)M_PI);

    static float prev_fused_roll = 0;
    static float prev_fused_pitch = 0;

    fused_angle->fused_roll = 0.96f * (prev_fused_roll + processed_data->gyro_x * dt_sec) + 0.04f * roll_deg;
    fused_angle->fused_pitch = 0.96f * (prev_fused_pitch + processed_data->gyro_y * dt_sec) + 0.04f * pitch_deg;

    prev_fused_roll = fused_angle->fused_roll;
    prev_fused_pitch = fused_angle->fused_pitch;
}