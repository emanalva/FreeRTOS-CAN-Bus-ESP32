#include <LiquidCrystal.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <CAN.h>

// Pin definitions for the LCD (using digital pin numbers)
const int rs = 2, en = 4, d4 = 5, d5 = 18, d6 = 19, d7 = 21;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// Pin definition for the touch sensor (digital input)
const int TOUCH_PIN = 13;
const int canRxPin = 16;  // CAN RX pin
const int canTxPin = 17;  // CAN TX pin

// FreeRTOS task handles
TaskHandle_t touchTaskHandle, lcdTaskHandle, canTaskHandle;

// System state variable
volatile bool systemOn = false;
volatile int autoLights = 0;
volatile int temperature = 0;
volatile int distance = 0;

// Debounce delay for touch sensor
const int DEBOUNCE_DELAY = 500;  // 500ms debounce to prevent rapid toggling

// Task to monitor the touch sensor and change the system state
void touchTask(void *pvParameters) {
  bool lastTouchState = LOW;  // Track the last state of the touch sensor
  while (1) {
    // Read the touch sensor value (digital read for touch sensor)
    int touchValue = digitalRead(TOUCH_PIN);

    // If touch sensor is HIGH and was previously LOW, toggle system state
    if (touchValue == HIGH && lastTouchState == LOW) {
      systemOn = !systemOn;  // Toggle system state
      lastTouchState = HIGH;  // Update last touch state
      vTaskDelay(DEBOUNCE_DELAY / portTICK_PERIOD_MS);  // Debounce delay
    }
    // If touch sensor is LOW, update the last touch state
    else if (touchValue == LOW) {
      lastTouchState = LOW;
    }

    vTaskDelay(50 / portTICK_PERIOD_MS);  // Small delay between checks
  }
}

// Task to handle CAN bus communication
void canTask(void *pvParameters) {
  while (1) {
    // Send system on/off command to the sensor hub
    CAN.beginPacket(0x200);  // CAN ID 0x200
    CAN.write(systemOn ? 0x01 : 0x00);  // Command to turn system on/off
    CAN.endPacket();

    if (systemOn) {
      // Check for incoming sensor data
      int packetSize = CAN.parsePacket();
      if (packetSize) {
        autoLights = CAN.read();  // Auto Lights status
        temperature = CAN.read();  // Temperature in F
        distance = CAN.read();  // Distance in cm
      }
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Poll for CAN messages every second
  }
}

// Task to update the LCD display based on the system state and incoming data
void lcdTask(void *pvParameters) {
  while (1) {
    lcd.clear();
    lcd.setCursor(0, 0);  // First row, first column

    if (systemOn) {
      lcd.print("Auto Lights: ");
      lcd.print(autoLights ? "On" : "Off");

      lcd.setCursor(0, 1);  // Second row
      lcd.print("Temp: ");
      lcd.print(temperature);
      lcd.print(" F");

      lcd.print(" | Dist: ");
      lcd.print(distance);
      lcd.print(" cm");
    } else {
      lcd.print("System Off");
    }

    vTaskDelay(500 / portTICK_PERIOD_MS);  // Update every 500ms
  }
}

void setup() {
  // Start serial communication for debugging with 115200 baud rate
  Serial.begin(115200);

  // Initialize the LCD
  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("System Off");  // Initially display "System Off"

  // Initialize the touch sensor pin
  pinMode(TOUCH_PIN, INPUT);

  // Initialize CAN bus communication
  CAN.setPins(canTxPin, canRxPin);
  if (!CAN.begin(500E3)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }

  // Create FreeRTOS tasks for touch sensor, CAN, and LCD
  xTaskCreate(touchTask, "Touch Task", 1024, NULL, 1, &touchTaskHandle);
  xTaskCreate(canTask, "CAN Task", 1024, NULL, 1, &canTaskHandle);
  xTaskCreate(lcdTask, "LCD Task", 1024, NULL, 1, &lcdTaskHandle);
}

void loop() {
  // No actions needed in the main loop, FreeRTOS manages the tasks
}
