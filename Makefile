# Target microcontroller and clock frequency
MCU          = atmega328p
F_CPU        = 16000000UL

# Compiler tools
CC           = avr-gcc
OBJCOPY      = avr-objcopy
AVRDUDE      = avrdude

# Programmer settings (arduino, usbasp, or wiring)
PROGRAMMER   = arduino
# Query the system for active ACM or USB ports
PORT ?= $(shell ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null | head -n 1)
BAUD         = 115200

# Directory paths
SRC_DIR      = src
INC_DIR      = include
BUILD_DIR    = build

# Source, Object, and Target definitions
SRCS         = $(wildcard $(SRC_DIR)/*.c)
OBJS         = $(patsubst $(SRC_DIR)/%.c, $(BUILD_DIR)/%.o, $(SRCS))
TARGET       = $(BUILD_DIR)/main

# Compiler flags: Optimization level -Os is required for accurate _delay_ms() timing
CFLAGS       = -mmcu=$(MCU) -DF_CPU=$(F_CPU) -I$(INC_DIR) -Os -Wall -Wextra

.PHONY: all clean flash

# Default target
all: $(BUILD_DIR) $(TARGET).hex

# Create build directory
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Compile C files to Object files
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

# Link Object files into ELF binary
$(TARGET).elf: $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) -o $@

# Extract HEX file from ELF binary
$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@

# Flash HEX file to ATmega328P
flash: $(TARGET).hex
	$(AVRDUDE) -c $(PROGRAMMER) -p $(MCU) -P $(PORT) -b $(BAUD) -U flash:w:$<:i

# Clean build artifacts
clean:
	rm -rf $(BUILD_DIR)