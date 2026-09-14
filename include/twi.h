#ifndef TWI_H
#define TWI_H

#include <stdint.h>

// 1. Public Data Types
typedef enum { // enums are better than macros
    TWI_INIT_SUCCESSFULL = 5,
    TWI_ERROR_INVALID_PARAM,
    TWI_ERROR_START_FAILED,
    TWI_SUCCESSFULL,

    // Address
    TWI_ERROR_MT_SLA_ACK_FAILED,
    TWI_ERROR_MR_SLA_ACK_FAILED,
    TWI_ADDR_TRANSMIT_SUCCESSFULL,

    // Data
    TWI_ERROR_MT_DATA_ACK_FAILED,
    TWI_ERROR_MR_DATA_ACK_FAILED,
    TWI_ERROR_MR_DATA_NACK_FAILED,
    TWI_DATA_TRANSMIT_SUCCESSFULL,
    TWI_DATA_RECEIVE_SUCCESSFULL,
    TWI_STOP_SUCCESSFULL
} twi_result_t;

typedef enum {
    TWI_WRITE = 0, // Bit 0 low  -> SLA+W
    TWI_READ  = 1  // Bit 0 high -> SLA+R
} twi_dir_t;

typedef enum {
    TWI_NACK = 0,
    TWI_ACK  = 1
} twi_ack_t;

typedef enum {
    TWI_PULLUPS_DISABLE = 0,
    TWI_PULLUPS_ENABLE = 1
} twi_pullup_mode_t;

typedef struct {
    uint8_t bit_rate;
    uint8_t prescaler;
    twi_pullup_mode_t pullup_mode; // TWI_PULLUPS_ENABLE or TWI_PULLUPS_DISABLE
}twi_config_t;


// 2. Public Function Declarations

/**
 * @brief Initializes the TWI (I2C) peripheral bit rate, prescaler, and GPIO pull-ups.
 *
 * Writes bit rate (TWBR) and prescaler (TWSR) values to peripheral registers, 
 * configures internal pull-ups on PC4/PC5 (SDA/SCL), and enables TWI hardware.
 *
 * @param[in] cfg Pointer to configuration struct. If NULL, defaults to internal fallback configuration.
 *
 * @return twi_result_t Status indicating initialization result:
 *         - TWI_INIT_SUCCESSFULL: Peripheral configured successfully.
 *         - TWI_ERROR_INVALID_PARAM: Register values out of hardware bounds.
 */
twi_result_t TWI_INIT (const twi_config_t *cfg);

twi_result_t TWI_STOP (void);

twi_result_t TWI_RECEIVE_BYTE (uint8_t *byte, twi_ack_t send_ack);

twi_result_t TWI_SEND_BYTE (uint8_t data);

twi_result_t TWI_SEND_ADDR (uint8_t addr, twi_dir_t direction);

twi_result_t TWI_WRITE_REGISTER (uint8_t slave_addr, uint8_t reg_addr, uint8_t data);

twi_result_t TWI_READ_ADJACENT_REGISTER (uint8_t slave_addr, uint8_t reg_addr, uint8_t buffer_size, uint8_t *buffer);

#endif 