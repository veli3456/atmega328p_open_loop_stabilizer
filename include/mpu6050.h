#ifndef MPU_6050
#define MPU_6050

#include <stdint.h>
#include <stdbool.h>

#define MPU6050_ADDR_AD0_LOW  0x68
#define MPU6050_ADDR_AD0_HIGH 0x69

typedef enum {
    GYRO_FS_250  = 0, // (0 << 3) -> 0x00 (±250  dps)
    GYRO_FS_500  = 1, // (1 << 3) -> 0x08 (±500  dps)
    GYRO_FS_1000 = 2, // (2 << 3) -> 0x10 (±1000 dps)
    GYRO_FS_2000 = 3  // (3 << 3) -> 0x18 (±2000 dps)
}mpu6050_gyro_fs_t;

typedef enum {
    ACCEL_FS_2G  = 0, // (0 << 3) -> 0x00 (±2g)
    ACCEL_FS_4G  = 1, // (1 << 3) -> 0x08 (±4g)
    ACCEL_FS_8G  = 2, // (2 << 3) -> 0x10 (±8g)
    ACCEL_FS_16G = 3  // (3 << 3) -> 0x18 (±16g)
}mpu6050_accel_fs_t;

typedef enum __attribute__((packed)) {
    DLPF_MODE_0 = 0, // 260Hz Accel / 256Hz Gyro
    DLPF_MODE_1 = 1, // 184Hz Accel / 188Hz Gyro
    DLPF_MODE_2 = 2, // 94Hz Accel  / 98Hz Gyro
    DLPF_MODE_3 = 3, // 44Hz Accel  / 42Hz Gyro
    DLPF_MODE_4 = 4, // 21Hz Accel  / 20Hz Gyro
    DLPF_MODE_5 = 5, // 10Hz Accel  / 10Hz Gyro
    DLPF_MODE_6 = 6  // 5Hz Accel   / 5Hz Gyro
}mpu6050_dlpf_mode_t;

typedef struct {
    uint8_t slave_addr;     // Target address (0x68 when AD0 is LOW)
    mpu6050_dlpf_mode_t dlpf_mode;
    mpu6050_accel_fs_t accel_fs;
    mpu6050_gyro_fs_t gyro_fs;
} mpu6050_cfg_t;

typedef enum {
    MPU6050_INIT_SUCCESSFULL = 0,
    MPU6050_INIT_FAILURE,
    MPU6050_SENSOR_DATA_READ_FAILURE,
    MPU6050_SENSOR_DATA_READ_SUCCESSFULL
} mpu6050_status_t;

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
}mpu6050_raw_t;

typedef struct {
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
}processed_gyro_accel_data_t;

typedef struct {
    float fused_roll;
    float fused_pitch;
}fused_angle_t;

/**
 * @brief Initializes MPU-6050 power state and configuration registers.
 * 
 * Wakes the sensor (X-axis gyro clock ref), sets gyro range to ±250dps,
 * accel range to ±2g, and enables DLPF mode 2 (~94Hz bandwidth).
 * 
 * @param[in] config Pointer to I2C setup. If NULL, defaults to imu_cfg.
 * 
 * @return mpu6050_status_t Initialization status:
 *          - MPU6050_INIT_SUCCESSFULL: Registers configured successfully.
 *          - MPU6050_INIT_FAILURE: I2C transmission error occurred.
 */
mpu6050_status_t mpu6050_init (const mpu6050_cfg_t *config);

/** 
 * @brief Reads raw 6-axis accelerometer and gyroscope data.
 * 
 * Performs a 14-byte continuous I2C read starting from ACCEL_XOUT and 
 * unpacks big-endian register pairs into 16-bit signed integers.
 * 
 * @param[in]  config Pointer to I2C setup. If NULL, defaults to imu_cfg.
 * @param[out] data   Pointer to destination struct for raw readings (must not be NULL).
 * 
 * @return mpu6050_status_t Sensor read status:
 *          - MPU6050_SENSOR_DATA_READ_SUCCESSFULL: Data read and unpacked successfully.
 *          - MPU6050_SENSOR_DATA_READ_FAILURE: I2C transaction failed.
 */
mpu6050_status_t mpu6050_read_sensor_data (const mpu6050_cfg_t *config, mpu6050_raw_t *data);

void MPU6050_ReadScaled (mpu6050_raw_t *raw_data, processed_gyro_accel_data_t *processed_data);

void mpu6050_compute_fused_angles(fused_angle_t *fused_angle, processed_gyro_accel_data_t *processed_data, float dt_sec);


#endif