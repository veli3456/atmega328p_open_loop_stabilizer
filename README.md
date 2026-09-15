# ATmega328P Open-Loop Stabilizer

A bare-metal embedded C firmware project implementing an open-loop stabilization system using an ATmega328P microcontroller and an MPU6050 6-axis accelerometer/gyroscope sensor. It processes real-time motion data from the sensor mounted on the base platform to drive two servos, keeping their armatures at a configured angle relative to the ground regardless of platform movement.

## Features
- **Error Handler Module:** Captures and reports runtime errors by transmitting formatted error code along with the original file name and line number. Also features a diagnostic LED that blinks out the error code, providing a fast, visual debugging method. 
- **Fast Alternative to atan2f:** atan2_fast executes 32% faster than standard atan2f, as verified via logic analyzer testing.
- **Automated Makefile Build System:** Automated compilation and flashing via `avrdude` with dynamic serial port detection.

## Hardware Requirements
- **Linux Environment:** Required for building and flashing the project using the Linux-compatible Makefile.
- ATmega328P Microcontroller (running at 16 MHz)
- MPU6050 Accelerometer & Gyroscope module
- USB-to-Serial programmer (or Arduino Uno used as ISP/serial bridge)

## Pin Configuration
- **MPU6050 (SDA):** PC4 (SDA) (I2C Data Line)
- **MPU6050 (SDL):** PC5 (SCL) (I2C Clock Line)
- **Servo1 PWM Signal (Roll):** PB1 (OC1A)
- **Servo2 PWM Signal (Pitch):** PB2 (OC1B)
- **Diagnostic LED:** PB0 (Error Blink Output)