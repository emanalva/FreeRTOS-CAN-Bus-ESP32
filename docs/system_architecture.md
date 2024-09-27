# System Architecture

## Overview
This project simulates a basic autonomous vehicle system using two ESP32 microcontrollers running FreeRTOS, communicating over CAN bus. One ESP32 acts as a sensor hub, collecting data from various sensors, while the other acts as a controller hub, managing system state and displaying data.

## Roles of Each ESP32

- **ESP32 #1 (Controller Hub)**:
    - Controls system state (on/off) via a touch sensor.
    - Displays received sensor data on a 16x2 LCD screen.
    - Sends commands to the Sensor Hub via CAN bus to start or stop data collection.

- **ESP32 #2 (Sensor Hub)**: 
    - Collects data from the ultrasonic sensor, DHT11 temperature sensor, and photoresistor.
    - Sends sensor data (Auto Lights, Temperature, Distance) to the Controller Hub via CAN bus.

## CAN Bus Communication
- The two ESP32s communicate using CAN bus, allowing real-time data transfer between the hubs.
- CAN ID **0x100** is used for sensor data transfer from the Sensor Hub to the Controller Hub.
- CAN ID **0x200** is used for sending system on/off commands from the Controller Hub to the Sensor Hub.

## FreeRTOS Task Scheduling
- Both ESP32s use FreeRTOS to manage tasks such as reading sensor data, handling system state changes, and updating the LCD display. Each task is scheduled to run at a specific interval to ensure smooth operation.