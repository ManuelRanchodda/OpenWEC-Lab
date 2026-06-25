# Inertial and Electrical Measurement PCB Firmware

This directory contains the firmware developed for the **Inertial and Electrical Measurement PCB** of the OpenWEC-Lab platform.

## Description

This firmware performs the synchronized acquisition of inertial, mechanical, and electrical variables during laboratory-scale wave energy converter experiments.

## Hardware

- ESP32 Development Board
- MPU9250 IMU
- Two INA219 Current/Voltage Sensors
- Two Incremental Encoders
- TCA9548A I2C Multiplexer

## Functions

- Roll, Pitch and Yaw estimation
- Encoder angle acquisition
- Voltage, current and power measurement
- Sensor synchronization
- Real-time serial logging

## Output

The firmware streams synchronized measurements of inertial, mechanical, and electrical variables through the serial interface for experimental analysis.
