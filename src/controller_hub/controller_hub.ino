#include <SPI.h>
#include <mcp2515.h>
#include <LiquidCrystal.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// CAN Defines
#define CAN_ACK_ID 0x037  // CAN ID for acknowledgment
#define CAN_SENSOR_ID 0x100 // CAN ID for sensor data
#define CAN_CMD_ID 0x036    // CAN ID for on/off command

MCP2515 mcp2515(5);  // CS pin is GPIO 5

// Pin definitions for the LCD (using digital pin numbers)
const int rs = 2, en = 4, d4 = 26, d5 = 25, d6 = 33, d7 = 32;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// Pin definition for the touch sensor (digital input)
const int TOUCH_PIN = 13;

// Debounce delay for touch sensor
const int DEBOUNCE_DELAY = 500;

// System state variable
volatile bool systemOn = false;
volatile int autoLights = 0;
volatile int temperature = 0;
volatile int distance = 0;

void setup() {
  Serial.begin(115200);
  SPI.begin();

  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  // Initialize LCD
  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("System Off");

  // Initialize the touch sensor pin
  pinMode(TOUCH_PIN, INPUT);

  // Create the CAN send task
  xTaskCreate(sendTask, "CAN Send Task", 4096, NULL, 2, NULL);
  xTaskCreate(touchTask, "Touch Task", 1024, NULL, 1, NULL);
  xTaskCreate(lcdTask, "LCD Task", 1024, NULL, 1, NULL);
  xTaskCreate(receiveTask, "CAN Receive Task", 4096, NULL, 1, NULL);
}

// Task to update the LCD display based on the system state and sensor data
void lcdTask(void *pvParameters) {
  while (1) {
    lcd.clear();
    lcd.setCursor(0, 0);
    
    if (systemOn) {
      lcd.print("Auto Lights: ");
      lcd.print(autoLights ? "On" : "Off");

      lcd.setCursor(0, 1);
      lcd.print("Temp");
      lcd.print(temperature);
      lcd.print("F Dist");
      lcd.print(distance);
      lcd.print("cm");
    } else {
      lcd.print("System Off");
    }
    
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

// Task to monitor the touch sensor and change the system state
void touchTask(void *pvParameters) {
  bool lastTouchState = LOW;
  while (1) {
    int touchValue = digitalRead(TOUCH_PIN);

    if (touchValue == HIGH && lastTouchState == LOW) {
      systemOn = !systemOn;
      lastTouchState = HIGH;
      vTaskDelay(DEBOUNCE_DELAY / portTICK_PERIOD_MS);
      Serial.print("System State: ");
      Serial.println(systemOn ? "ON" : "OFF");
    } else if (touchValue == LOW) {
      lastTouchState = LOW;
    }

    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

// Task to send the system state (on/off) to the sensor hub
void sendTask(void *parameter) {
  struct can_frame canMsg;
  while (1) {
    canMsg.can_id  = CAN_CMD_ID;  // CAN ID for on/off command
    canMsg.can_dlc = 1;  // Data length code: 1 byte
    canMsg.data[0] = systemOn ? 0x01 : 0x00;  // 0x01 for ON, 0x00 for OFF

    if (mcp2515.sendMessage(&canMsg) == MCP2515::ERROR_OK) {
      Serial.println("System State Sent via CAN!");
    } else {
      Serial.println("Failed to Send System State via CAN!");
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Send system state every second
  }
}

// Task to receive sensor data from the sensor hub
void receiveTask(void *parameter) {
  struct can_frame canMsg;
  while (1) {
    if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK && canMsg.can_id == CAN_SENSOR_ID) {
      // Parse the received data
      autoLights = canMsg.data[0];  // Auto Lights ON/OFF
      temperature = canMsg.data[1]; // Temperature (whole number)
      distance = canMsg.data[2];    // Distance in cm

      Serial.println("Sensor Data Received:");
      Serial.print("Auto Lights: ");
      Serial.println(autoLights ? "ON" : "OFF");
      Serial.print("Temperature: ");
      Serial.println(temperature);
      Serial.print("Distance: ");
      Serial.println(distance);
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Check for CAN messages every second
  }
}

void loop() {
  // Empty - tasks handle everything
}