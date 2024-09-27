#include <LiquidCrystal.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Pin definitions for the LCD (using digital pin numbers)
const int rs = 16, en = 17, d4 = 5, d5 = 18, d6 = 19, d7 = 21;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// Pin definition for the touch sensor (digital input)
const int TOUCH_PIN = 13;

// FreeRTOS task handles
TaskHandle_t touchTaskHandle, lcdTaskHandle;

// System state variable
volatile bool systemOn = false;

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

// Task to update the LCD display based on the system state
void lcdTask(void *pvParameters) {
  while (1) {
    lcd.clear();
    lcd.setCursor(0, 0);  // First row, first column
    if (systemOn) {
      lcd.print("System On");
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

  // Create FreeRTOS tasks for touch sensor and LCD
  xTaskCreate(touchTask, "Touch Task", 1024, NULL, 1, &touchTaskHandle);
  xTaskCreate(lcdTask, "LCD Task", 1024, NULL, 1, &lcdTaskHandle);
}

void loop() {
  // No actions needed in the main loop, FreeRTOS manages the tasks
}
