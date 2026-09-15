#include <avr/io.h>
#include <stdint.h>
#include <stddef.h>
#include "twi.h"

// Bitrate Register Range
#define MAX 255UL
#define MIN 10UL

typedef enum {
    TWI_STATUS_START = 0x08, // START condition transmitted
    TWI_STATUS_REP_START = 0x10, // Repeated START condition transmitted

    // Master Transmitter (MT) States
    TWI_STATUS_MT_SLA_ACK = 0x18, // SLA+W transmitted, ACK received
    TWI_STATUS_MT_SLA_NACK = 0x20, // SLA+W transmitted, NACK received
    TWI_STATUS_MT_DATA_ACK = 0x28, // Data transmitted, ACK received
    TWI_STATUS_MT_DATA_NACK = 0x30, // Data transmitted, NACK received

    // Master Receiver (MR) States
    TWI_STATUS_MR_SLA_ACK = 0x40, // SLA+R transmitted, ACK received
    TWI_STATUS_MR_SLA_NACK = 0x48, // SLA+R transmitted, NACK received
    TWI_STATUS_MR_DATA_ACK = 0x50, // Data received, ACK returned
    TWI_STATUS_MR_DATA_NACK = 0x58, // Data received, NACK returned

    // Master Receiver/Transmitter Shared States
    TWI_STATUS_ARB_LOST = 0x38 // Arbitration lost in SLA+W/R or data
}twi_hw_status_code_t;

static const twi_config_t twi_config = { // 363 kHz
    .bit_rate = 14,
    .prescaler = 1,
    .pullup_mode = TWI_PULLUPS_ENABLE
};

twi_result_t TWI_INIT (const twi_config_t *cfg) {
    if (cfg == NULL) {
        cfg = &twi_config;
    }

    // Bitrate check (TWBR register range: 10 - 255)
    if (cfg->bit_rate >= MIN && cfg->bit_rate <= MAX) {
        TWBR = cfg->bit_rate;
    }
    else {
        return TWI_ERROR_INVALID_PARAM;
    }

    // Clear prescaler bits (TWPS1:0) once before setting
    TWSR &= ~((1<<TWPS1) | (1<<TWPS0) );

    switch (cfg->prescaler) {
        case 1:
            // Bits already cleared (00)
            break;
        case 4:
            TWSR |= (1<<TWPS0);
            break;
        case 16:
            TWSR |= (1<<TWPS1); 
            break;
        case 64:
            TWSR |= (1<<TWPS1) | (1<<TWPS0); 
            break;
        default:
            return TWI_ERROR_INVALID_PARAM;
    }

    if (cfg->pullup_mode) {
        // Enable internal pull-ups (PC4=SDA, PC5=SCL)
        PORTC |= (1 << PC4) | (1 << PC5); 
    }
    else {
        PORTC &= ~((1 << PC4) | (1 << PC5));
    }

    // Enable TWI peripheral hardware
    TWCR = (1 << TWEN);

    return TWI_INIT_SUCCESSFULL;

}

twi_result_t TWI_SEND_ADDR (uint8_t addr, twi_dir_t direction) {

    // Clear INT flag, Set TWI START Condition Bit
    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);

    // Wait for TWINT flag set.
    while (!(TWCR & (1<<TWINT)) );

    // Read TWI Status Register to see if START has been transmitted
    uint8_t status = TWSR & 0xF8;
    if (status != TWI_STATUS_START && status != TWI_STATUS_REP_START) {
        return TWI_ERROR_START_FAILED;
    }

    // Load SLA+W/R into 
    TWDR = (addr << 1) | direction;
    TWCR = (1<<TWINT) | (1<<TWEN); // Clearing TWINT to start transmission of address

    while (!(TWCR & (1<<TWINT))); // Wait for TWINT set (FAAAAUUULLLLTYYYYY PARTTTTT)

    // Read TWI Status Register to see if addr has been transmitted 
    // and ACK has been received successfully
    if ((TWSR & 0xF8) != TWI_STATUS_MT_SLA_ACK && direction == TWI_WRITE) {
        return TWI_ERROR_MT_SLA_ACK_FAILED;
    }
    else if ((TWSR & 0xF8) != TWI_STATUS_MR_SLA_ACK && direction == TWI_READ) {
        return TWI_ERROR_MR_SLA_ACK_FAILED;
    }

    return TWI_ADDR_TRANSMIT_SUCCESSFULL;
}

twi_result_t TWI_SEND_BYTE (uint8_t data) {
    // Load data into TWDR
    TWDR = data;
    // Clear TWINT and enable TWI to start transmission of data
    TWCR = (1<<TWINT) | (1<<TWEN);

    // Wait for TWINT flag set
    while (!(TWCR & (1<<TWINT)));

    // If data has not been transmitted or ACK not received successfully
    if ((TWSR & 0xF8) != TWI_STATUS_MT_DATA_ACK) {
        return TWI_ERROR_MT_DATA_ACK_FAILED;
    }

    return TWI_DATA_TRANSMIT_SUCCESSFULL;
}

twi_result_t TWI_RECEIVE_BYTE (uint8_t *byte, twi_ack_t send_ack) {

    if (byte == NULL) {
        return TWI_ERROR_INVALID_PARAM;
    }

    if (send_ack) {
        // Clear TWINT to start receiving data and enable ACK bit
        TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
    }
    else {
        // Clear TWINT to start receiving data and send NACK to stop
        TWCR = (1<<TWINT) | (1<<TWEN);
    }


    while (!(TWCR & (1<<TWINT))); // Wait for TWINT set

    uint8_t status = TWSR & 0xF8;

    if (send_ack && status != TWI_STATUS_MR_DATA_ACK) {
        return TWI_ERROR_MR_DATA_ACK_FAILED;
    } 
    else if (!send_ack && status != TWI_STATUS_MR_DATA_NACK) {
        return TWI_ERROR_MR_DATA_NACK_FAILED;
    }

    *byte = TWDR;

    return TWI_DATA_RECEIVE_SUCCESSFULL;

}

twi_result_t TWI_STOP (void) {
    // Transmit STOP condition
    TWCR = ((1<<TWINT)|(1<<TWEN)| (1<<TWSTO)); 

    // Wait until TWSTO is cleared by hardware (STOP condition complete on bus)
    while (TWCR & (1 << TWSTO));

    return TWI_STOP_SUCCESSFULL;
}

twi_result_t TWI_WRITE_REGISTER (uint8_t slave_addr, uint8_t reg_addr, uint8_t data) {
    twi_result_t status;

    status = TWI_SEND_ADDR(slave_addr, TWI_WRITE);
    if (status != TWI_ADDR_TRANSMIT_SUCCESSFULL) {
        TWI_STOP();
        return status;
    }

    status = TWI_SEND_BYTE(reg_addr);
    if (status != TWI_DATA_TRANSMIT_SUCCESSFULL) {
        TWI_STOP();
        return status;
    }

    status = TWI_SEND_BYTE(data);
    if (status != TWI_DATA_TRANSMIT_SUCCESSFULL) {
        TWI_STOP();
        return status;
    }

    TWI_STOP();

    return TWI_SUCCESSFULL;
}

twi_result_t TWI_READ_ADJACENT_REGISTER (uint8_t slave_addr, uint8_t reg_addr, uint8_t buffer_size, uint8_t *buffer) {
    twi_result_t status;

    status = TWI_SEND_ADDR(slave_addr, TWI_WRITE);
    if (status != TWI_ADDR_TRANSMIT_SUCCESSFULL) {
        TWI_STOP();
        return status;
    }

    status = TWI_SEND_BYTE (reg_addr);
    if (status != TWI_DATA_TRANSMIT_SUCCESSFULL) {
        TWI_STOP();
        return status;
    }

    status = TWI_SEND_ADDR(slave_addr, TWI_READ);
    if (status != TWI_ADDR_TRANSMIT_SUCCESSFULL) {
        TWI_STOP();
        return status;
    }

    for (uint8_t i = 0; i < buffer_size; i++) {

        if (i == buffer_size - 1) {
            status = TWI_RECEIVE_BYTE(&buffer[i], TWI_NACK);
        }
        else {
            status = TWI_RECEIVE_BYTE(&buffer[i], TWI_ACK);
        }

        if (status != TWI_DATA_RECEIVE_SUCCESSFULL) {
            TWI_STOP();
            return status;
        }
    }

    TWI_STOP();

    return TWI_SUCCESSFULL;
}