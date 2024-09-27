# FreeRTOS CAN Bus ESP32

This project implements a real-time vehicle sensor network using two ESP32 microcontrollers running FreeRTOS. The ESP32s communicate via CAN bus, simulating real-time data transmission in an automotive system.

## Features
- Real-time data acquisition from multiple sensors:
  - **Ultrasonic sensor (HC-SR04)**: Simulates proximity detection (e.g., parking sensor).
  - **Temperature & humidity sensor (DHT11)**: Simulates cabin environmental monitoring.
  - **Photoresistor**: Acts as an ignition button to start or stop the vehicle system.
- CAN bus communication between two ESP32s.
- FreeRTOS task scheduling and prioritization.

## Hardware Used
- **ESP32** x 2
- **MCP2515 CAN Bus modules** x 2
- **Sensors**:
  - Ultrasonic sensor (HC-SR04)
  - Temperature & humidity sensor (DHT11)
  - Photoresistor (5528)
  - LED

## Folder Structure
- **/src**: Contains the main code for FreeRTOS tasks and CAN bus communication.
- **/hardware**: Wiring diagrams and schematics.
- **/docs**: Documentation on system architecture and task breakdown.

## Getting Started
To replicate this project, ensure you have the necessary hardware components and follow the wiring diagrams in the `/hardware/` folder. Use PlatformIO or Arduino IDE for ESP32 programming.
