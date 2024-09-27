#include <DHT.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Pin Definitions
const int photoPin = 34;  // Analog pin for photoresistor
const int ledPin = 23;    // LED pin to indicate headlights
const int dhtPin = 25;    // DHT11 data pin
const int trigPin = 26;   // HC-SR04 Trig pin
const int echoPin = 27;   // HC-SR04 Echo pin

// DHT11 sensor type
#define DHTTYPE DHT11
DHT dht(dhtPin, DHTTYPE);

// Variables
volatile int photoValue = 0;
volatile float temperature = 0.0;
volatile float humidity = 0.0;
volatile int distance = 0;

// FreeRTOS Task Handles
TaskHandle_t photoTaskHandle, dhtTaskHandle, ultrasonicTaskHandle;

// Task to read Photoresistor and control the LED (Headlights)
void photoTask(void *pvParameters) {
  while (1) {
    // Read Photoresistor value
    photoValue = analogRead(photoPin);

    // If photoresistor value is below a threshold, turn on the LED (headlights)
    if (photoValue < 250) {  // Adjust threshold as necessary
      digitalWrite(ledPin, HIGH);  // Turn on LED (headlights on)
    } else {
      digitalWrite(ledPin, LOW);  // Turn off LED (headlights off)
    }

    // Print the Photoresistor value and Headlights state
    Serial.print("Photoresistor Value: ");
    Serial.print(photoValue);
    Serial.print(" | Headlights: ");
    Serial.print(photoValue < 250 ? "ON" : "OFF");
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Delay for 1 second
  }
}

// Task to read DHT11 sensor (Temperature & Humidity)
void dhtTask(void *pvParameters) {
  while (1) {
    temperature = dht.readTemperature();  // Read temperature in Celsius
    humidity = dht.readHumidity();        // Read humidity

    // Convert temperature to Fahrenheit
    float temperatureF = (temperature * 9.0 / 5.0) + 32.0;

    // Print the Temperature and Humidity
    if (!isnan(temperature) && !isnan(humidity)) {
      Serial.print(" | Temperature: ");
      Serial.print(temperatureF);
      Serial.print(" *F");

      Serial.print(" | Humidity: ");
      Serial.print(humidity);
      Serial.print(" %");
    } else {
      Serial.print(" | Failed to read from DHT sensor!");
    }

    vTaskDelay(2000 / portTICK_PERIOD_MS);  // Delay for 2 seconds
  }
}

// Task to read Ultrasonic sensor (HC-SR04)
void ultrasonicTask(void *pvParameters) {
  while (1) {
    // Trigger the ultrasonic sensor to send a pulse
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Read the echo pin, and calculate the distance
    long duration = pulseIn(echoPin, HIGH);
    distance = duration * 0.034 / 2;  // Speed of sound wave divided by 2 (one-way distance)

    // Print the Distance
    Serial.print(" | Distance: ");
    Serial.print(distance);
    Serial.println(" cm");

    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Delay for 1 second
  }
}

void setup() {
  // Start serial communication for debugging
  Serial.begin(115200);

  // Initialize DHT11 sensor
  dht.begin();

  // Initialize LED pin
  pinMode(ledPin, OUTPUT);

  // Initialize Ultrasonic sensor pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Create FreeRTOS tasks for each sensor
xTaskCreate(photoTask, "Photo Task", 2048, NULL, 1, &photoTaskHandle);
xTaskCreate(dhtTask, "DHT Task", 2048, NULL, 1, &dhtTaskHandle);
xTaskCreate(ultrasonicTask, "Ultrasonic Task", 2048, NULL, 1, &ultrasonicTaskHandle);
}

void loop() {
  // No actions needed in the loop, FreeRTOS manages the tasks
}
