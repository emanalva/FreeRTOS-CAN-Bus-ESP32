# 🚗 FreeRTOS CAN Bus Autonomous Vehicle System

This project simulates a real-time sensor network for an autonomous vehicle using two ESP32 microcontrollers running FreeRTOS. The ESP32s communicate via CAN bus, mimicking real-time data transmission between different vehicle components.

## 🚀 Features
- Real-time data acquisition from multiple sensors:
  - **Ultrasonic sensor (HC-SR04)**: Simulates proximity detection (e.g., parking sensor).
  - **Temperature & humidity sensor (DHT11)**: Simulates cabin environmental monitoring.
  - **Photoresistor (5528)**: Simulates automatic headlight control based on ambient light conditions.
  - **Touch sensor (TTP223)**: Acts as an ignition button to start or stop the vehicle system.
- CAN bus communication between two ESP32s.
- FreeRTOS task scheduling and prioritization.

## 🛠️ Hardware Used
- **ESP32** x 2
- **MCP2515 CAN Bus modules** x 2
- **Sensors**:
  - Ultrasonic sensor (HC-SR04)
  - Temperature & humidity sensor (DHT11)
  - Photoresistor (5528)
  - Touch sensor (TTP223)
  - LED

## 📂 Folder Structure
- **/src**: Contains the main code for FreeRTOS tasks and CAN bus communication in `.ino` files.
- **/hardware**: Wiring diagrams and schematics.
- **/docs**: Documentation on system architecture and task breakdown.

## 🚧 Future Expansion
This project is designed to support the integration of additional ESP32s. A future extension will incorporate a **third ESP32** to control a **stepper motor** and a **servo motor**, mimicking moving mechanical parts in an autonomous vehicle system. This addition will further enhance the simulation of real-time mechanical actions in an autonomous vehicle using CAN bus communication.

## 🏁 Getting Started
To replicate this project, ensure you have the necessary hardware components and follow the wiring diagrams in the `/hardware/` folder. Use PlatformIO or Arduino IDE for ESP32 programming.

© 2024 Emanuel Alvarez