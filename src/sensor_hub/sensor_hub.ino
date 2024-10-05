#include <SPI.h>
#include <mcp2515.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <DHT.h>

// CAN Defines
#define CAN_ACK_ID 0x037   // CAN ID for acknowledgment
#define CAN_CMD_ID 0x036   // CAN ID for on/off command
#define CAN_SENSOR_ID 0x100  // CAN ID for sensor data
#define INT_PIN 13          // Interrupt pin connected to MCP2515

MCP2515 mcp2515(5);       // CS pin is GPIO 5
SemaphoreHandle_t xCANInterruptSemaphore;  // Semaphore to handle CAN interrupts

// Sensor Pin Definitions
const int ledPin = 22;    // Digital pin to toggle LED
const int photoPin = 34;  // Analog pin for photoresistor
const int dhtPin = 25;    // DHT11 data pin
const int trigPin = 26;   // HC-SR04 Trig pin
const int echoPin = 14;   // HC-SR04 Echo pin

// DHT11 sensor type
#define DHTTYPE DHT11
DHT dht(dhtPin, DHTTYPE);

// Sensor Variables
float temperature = 0.0;
int photoValue = 0;
int distance = 0;
volatile bool systemOn = false;  // System state received from the controller hub

void IRAM_ATTR onCANInterrupt() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(xCANInterruptSemaphore, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken) {
    portYIELD_FROM_ISR();
  }
}

void setup() {
  Serial.begin(115200);
  SPI.begin();

  // Initialize CAN bus
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  // Create binary semaphore
  xCANInterruptSemaphore = xSemaphoreCreateBinary();

  // Attach the interrupt to the INT_PIN
  pinMode(INT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(INT_PIN), onCANInterrupt, FALLING);

  // Initialize LED pin as output
  pinMode(ledPin, OUTPUT);

  // Initialize DHT11 sensor
  dht.begin();

  // Initialize Ultrasonic sensor pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Create FreeRTOS tasks
  xTaskCreate(receiveTask, "CAN Receive Task", 4096, NULL, 1, NULL);
  xTaskCreate(photoTask, "Photo Task", 2048, NULL, 1, NULL);
  xTaskCreate(dhtTask, "DHT Task", 2048, NULL, 1, NULL);
  xTaskCreate(ultrasonicTask, "Ultrasonic Task", 2048, NULL, 1, NULL);
  xTaskCreate(sendSensorDataTask, "CAN Send Sensor Data Task", 4096, NULL, 2, NULL);
}

// Task to handle CAN communication and receive system on/off command
void receiveTask(void *parameter) {
  struct can_frame canMsg;
  while (1) {
    // Wait for interrupt signal
    if (xSemaphoreTake(xCANInterruptSemaphore, portMAX_DELAY) == pdTRUE) {
      if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
        if (canMsg.can_id == CAN_CMD_ID) {  // Check if the message is an on/off command
          systemOn = (canMsg.data[0] == 0x01);  // Turn system on or off based on received command
          Serial.println(systemOn ? "System ON" : "System OFF");
        }

        // Send acknowledgment
        // canMsg.can_id = CAN_ACK_ID;  // Use ACK ID
        // canMsg.can_dlc = 0;          // No data needed for ACK
        // mcp2515.sendMessage(&canMsg);
        // Serial.println("ACK sent");
      }
    }
  }
}

// Task to read the photoresistor value and control the LED
void photoTask(void *pvParameters) {
  while (1) {
    if (systemOn) {
      photoValue = analogRead(photoPin);

      Serial.print("Photoresistor Value: ");
      Serial.println(photoValue);

      // If the light level is below 250, turn on the LED (Headlights ON)
      if (photoValue < 250) {
        digitalWrite(ledPin, HIGH);  // Turn on LED
      } else {
        digitalWrite(ledPin, LOW);   // Turn off LED
      }

      vTaskDelay(1000 / portTICK_PERIOD_MS);  // Delay for 1 second
    }
  }
}

// Task to read DHT11 sensor (Temperature only)
void dhtTask(void *pvParameters) {
  while (1) {
    if (systemOn) {
      temperature = dht.readTemperature(true);  // Read temperature in Fahrenheit

      if (!isnan(temperature)) {
        Serial.print("Temperature: ");
        Serial.print(temperature);
        Serial.println(" °F");
      } else {
        Serial.println("Failed to read from DHT sensor!");
      }

      vTaskDelay(2000 / portTICK_PERIOD_MS);  // Delay for 2 seconds
    }
  }
}

// Task to read Ultrasonic sensor (HC-SR04)
void ultrasonicTask(void *pvParameters) {
  while (1) {
    if (systemOn) {
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);

      long duration = pulseIn(echoPin, HIGH);
      distance = duration * 0.034 / 2;  // Calculate distance in cm

      Serial.print("Distance: ");
      Serial.print(distance);
      Serial.println(" cm");

      vTaskDelay(1000 / portTICK_PERIOD_MS);  // Delay for 1 second
    }
  }
}

// Task to send sensor data via CAN bus
void sendSensorDataTask(void *pvParameters) {
  struct can_frame canMsg;
  
  while (1) {
    if (systemOn) {
      // Prepare CAN message with sensor data
      canMsg.can_id = CAN_SENSOR_ID;
      canMsg.can_dlc = 3;  // 3 bytes: auto lights, temperature, and distance
      canMsg.data[0] = (photoValue < 250) ? 0x01 : 0x00;  // Auto Lights ON/OFF
      canMsg.data[1] = (int)temperature;            // Temperature (whole number)
      canMsg.data[2] = distance;                    // Distance in cm

      Serial.print("Auto Lights: ");
      Serial.println(canMsg.data[0]);

      Serial.print("Temperature: ");
      Serial.println(canMsg.data[1]);

      Serial.print("Distance: ");
      Serial.println(canMsg.data[2]);
      
      // Send sensor data to controller hub
      if (mcp2515.sendMessage(&canMsg) == MCP2515::ERROR_OK) {
        Serial.println("Sensor Data Sent via CAN");
      } else {
        Serial.println("Failed to Send Sensor Data via CAN");
      }

      vTaskDelay(1000 / portTICK_PERIOD_MS);  // Send data every second
    }
  }
}

void loop() {
  // Empty - tasks handle everything
}
