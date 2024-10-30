#define BLYNK_TEMPLATE_ID "TMPL3qoi8CqXG" // Blynk firmware configuration
#define BLYNK_TEMPLATE_NAME "Air Quality Monitor" // Blynk firmware configuration
#define BLYNK_AUTH_TOKEN "up1H-bvqYaqqkWAYLGnsJMfPgpBlivua" // Blynk firmware configuration
#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include "DHT.h"

// Pin definitions for sensors, LEDs, and buzzer
const int mq2Pin = 33;
const int mq7Pin = 34;
const int mq135Pin = 26;
const int alarmLedPin = 25;
const int safeLedPin = 12;
const int buzzerPin = 13; 

// DHT11 sensor pin
#define DHTPIN 32
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// Ultrasonic sensor pins
const int trigPin = 14;
const int echoPin = 27;

BlynkTimer timer;

// Calibration data for each sensor
const float MQ2_Ro = 10.0;
const float MQ7_Ro = 10.0;

// Define safety thresholds for each sensor
const float safetyThresholdMQ2 = 5;
const float safetyThresholdMQ7 = 5;
float safetyThresholdMQ135 = 200.0;

float mq2GasConcentration;
float mq7GasConcentration;
float temperature;
float humidity;
float distance;

char auth[] = BLYNK_AUTH_TOKEN; 

// Function to calculate distance using ultrasonic sensor
float getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  return (duration * 0.034) / 2; // Convert to distance in cm
}

// Function to simulate MQ135 values between 50 and 100 during alarm situation
float getSimulatedMQ135Value() {
  return random(50, 101);  // Returns a random value between 50 and 100
}

void sendSensorData() {
  int mq2Value = analogRead(mq2Pin);
  int mq7Value = analogRead(mq7Pin);

  float mq2Rs = MQ2_Ro * (1023.0 - mq2Value) / mq2Value;
  float mq7Rs = MQ7_Ro * (1023.0 - mq7Value) / mq7Value;

  mq2GasConcentration = pow(10, ((log10(mq2Rs / MQ2_Ro) - 0.207) / -0.68)); 
  mq7GasConcentration = pow(10, ((log10(mq7Rs / MQ7_Ro) - 0.207) / -0.68)); 

  float mq135GasConcentration = 0;  // Initialize MQ135 concentration

  // Read DHT11 sensor values
  temperature = dht.readTemperature();
  humidity = dht.readHumidity();

  // Read ultrasonic sensor distance
  distance = getDistance();

  // Print gas concentrations to Serial
  Serial.print("MQ2 Gas Concentration: ");
  Serial.println(mq2GasConcentration);
  Serial.print("MQ7 Gas Concentration: ");
  Serial.println(mq7GasConcentration);

  // Update MQ135 gas concentration if alarm is triggered
  if (mq2GasConcentration > safetyThresholdMQ2 || mq7GasConcentration > safetyThresholdMQ7) {
    mq135GasConcentration = getSimulatedMQ135Value();  // Simulate value between 50 and 100
  } else {
    mq135GasConcentration = 30;  // Default value for non-alarm situation
  }

  Serial.print("MQ135 Gas Concentration : ");
  Serial.println(mq135GasConcentration);

  Serial.print("Temperature: ");
  Serial.println(temperature);
  Serial.print("Humidity: ");
  Serial.println(humidity);

  Serial.print("Distance: ");
  Serial.println(distance);

  // Send gas sensor data to Blynk
  Blynk.virtualWrite(V0, mq2GasConcentration);
  Blynk.virtualWrite(V1, mq7GasConcentration);
  Blynk.virtualWrite(V2, mq135GasConcentration);  // Send the adjusted MQ135 value

  // Send DHT11 data to Blynk
  Blynk.virtualWrite(V5, temperature);
  Blynk.virtualWrite(V6, humidity);

  // Send ultrasonic sensor data to Blynk
  Blynk.virtualWrite(V3, distance);
}

void setup() {
  Serial.begin(115200);
  Blynk.begin(auth, "OnePlus Nord", "viraj1234");

  pinMode(alarmLedPin, OUTPUT);
  pinMode(safeLedPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);

  dht.begin(); 

  pinMode(trigPin, OUTPUT); 
  pinMode(echoPin, INPUT);

  timer.setInterval(1000L, sendSensorData);

  digitalWrite(alarmLedPin, LOW);
  digitalWrite(safeLedPin, HIGH);
  digitalWrite(buzzerPin, LOW);
}

void loop() {
  Blynk.run();
  timer.run();

  if (mq2GasConcentration > safetyThresholdMQ2 || mq7GasConcentration > safetyThresholdMQ7) {
    digitalWrite(alarmLedPin, HIGH);   // Turn on alarm LED
    digitalWrite(safeLedPin, LOW);     // Turn off safe LED
    digitalWrite(buzzerPin, HIGH);     // Turn on buzzer
  } else {
    digitalWrite(alarmLedPin, LOW);    // Turn off alarm LED
    digitalWrite(safeLedPin, HIGH);    // Turn on safe LED
    digitalWrite(buzzerPin, LOW);      // Turn off buzzer
  }
}
