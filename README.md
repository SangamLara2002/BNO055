# BNO055 Sensor Interface Project

## Overview
This project implements an interface for the Bosch BNO055 9-axis absolute orientation sensor, providing accurate orientation, acceleration, and magnetometer data.

## Features
- Full 9-axis orientation sensing
- Multiple operation modes support
- Calibration status monitoring
- Raw and processed sensor data access
- Temperature sensing capability

## Hardware Requirements
- BNO055 sensor module
- Compatible microcontroller (Arduino/ESP32/etc)
- I2C interface connections
- 3.3V power supply

## Pin Connections
| BNO055 Pin | Microcontroller Pin |
|------------|-------------------|
| VIN        | 3.3V             |
| GND        | GND              |
| SDA        | I2C SDA          |
| SCL        | I2C SCL          |

## Software Dependencies
- I2C communication
- ESP IDE or compatible platform

## Installation
1. Connect the BNO055 sensor according to the pin connection table
2. Install required libraries
3. Upload the code to your microcontroller

## Usage
```cpp
// Initialize the sensor
bno055_init();

// Read sensor data
void read_bno055_euler_angles(void);
void read_bno055_acceleration(void);
void read_bno055_magnetometer(void);
void read_bno055_gyroscope(void);
void read_bno055_temperature(void);//NOT implemented
// Calibration status
void read_bno055_calibration_status(void);
// Sensor modes
