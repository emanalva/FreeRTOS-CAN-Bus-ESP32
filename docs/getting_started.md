# Getting Started

## Prerequisites

### Hardware:
- **2x ESP32** microcontrollers
- **2x MCP2515 CAN Bus Modules**
- **1x DHT11 Temperature Sensor**
- **1x Ultrasonic Sensor (HC-SR04)**
- **1x Photoresistor**
- **1x LED**
- **1x Touch Sensor**
- **1x 16x2 LCD Display (I2C or parallel)**

### Software:
- Arduino IDE with ESP32 and CAN libraries installed.

## Steps:

1. **Wiring**:
   - Follow the wiring diagrams in the `/hardware` folder for the correct connections between sensors, CAN bus modules, and the ESP32s.

2. **Flashing the Code**:
   - Upload `setup_hub.ino` to the ESP32 that acts as the Sensor Hub.
   - Upload `controller_hub.ino` to the ESP32 that acts as the Controller Hub.

3. **Testing**:
   - Power both ESP32s and observe the LCD for real-time sensor data.
   - Touch the sensor to toggle the system on/off.
   - Check the Serial Monitor to verify sensor data and CAN communication.