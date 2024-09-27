# Task Breakdown

## Overview
Each ESP32 runs multiple tasks concurrently, managed by FreeRTOS. Here's a breakdown of the tasks and their responsibilities.

### Controller Hub (ESP32 #1)

1. **Touch Task**:
   - Monitors the touch sensor and toggles the system state (on/off).
   - Sends CAN bus commands to the Sensor Hub based on system state.

2. **CAN Task**:
   - Sends commands to the Sensor Hub (on/off) via CAN bus.
   - Receives sensor data (Auto Lights, Temperature, Distance) from the Sensor Hub via CAN bus.

3. **LCD Task**:
   - Updates the 16x2 LCD display with sensor data when the system is on.
   - Displays "System Off" when the system is off.

### Sensor Hub (ESP32 #2)

1. **Photo Task**:
   - Reads the photoresistor value and controls the LED (simulating auto headlights).
   - Sends Auto Lights status (On/Off) to the Controller Hub.

2. **DHT Task**:
   - Reads temperature data from the DHT11 sensor.
   - Sends temperature data (in Fahrenheit) to the Controller Hub.

3. **Ultrasonic Task**:
   - Reads the distance from the ultrasonic sensor.
   - Sends distance data (in centimeters) to the Controller Hub.

4. **CAN Task**:
   - Sends sensor data (Auto Lights, Temperature, Distance) to the Controller Hub every second.
