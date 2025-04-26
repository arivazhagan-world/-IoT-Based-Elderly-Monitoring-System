#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <DHT.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

// --- DHT setup ---
#define DHTPIN 7
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// --- Rain Sensor Pin ---
#define RAIN_ANALOG_PIN A3

// --- Bluetooth Serial ---
#define BT_TX 11
#define BT_RX 10
SoftwareSerial bluetooth(BT_RX, BT_TX);

// --- ADXL345 setup ---
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(123);

// --- LCD setup ---
LiquidCrystal_I2C lcd(0x27, 16, 2);

// --- Step Counter Variables ---
int stepCount = 0;
bool stepDetected = false;
unsigned long lastStepTime = 0;

// --- Speed Calculation ---
unsigned long speedTimer = 0;
int stepsAtLastCheck = 0;
float speed = 0.0;
const float strideLength = 0.7;

// --- DHT Cache ---
float temp = NAN, humidity = NAN;
unsigned long lastDHTRead = 0;

// --- Fall Detection ---
bool fallDetected = false;
unsigned long lastFallTime = 0;
const unsigned long fallCooldown = 3000;
const unsigned long postStepBuffer = 100;

// --- Fall Detection (Improved) ---
bool inFreeFall = false;
unsigned long freeFallStart = 0;
const float freeFallThreshold = 5.0;      // More sensitive
const float impactThreshold = 9.8;        // Roughly 1g
const unsigned long maxFallWindow = 1500; // Slightly more forgiving
unsigned long lastMovementTime = 0;
const unsigned long inactivityAfterFall = 2000;

void setup() {
  Serial.begin(9600);
  bluetooth.begin(9600);
  dht.begin();

  if (!accel.begin()) {
    sendData("ADXL345 not detected.");
    while (1);
  }
  accel.setRange(ADXL345_RANGE_2_G);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("EnviroTrack");
  lcd.setCursor(0, 1);
  lcd.print("Shield");
  delay(2000);
  lcd.clear();

  sendData("System Initialized.");
}

void loop() {
  readSensors();
  calculateSpeed();
  handleBluetoothCommands();
  delay(1000);
}

void readSensors() {
  if (millis() - lastDHTRead > 2000) {
    temp = dht.readTemperature();
    humidity = dht.readHumidity();
    lastDHTRead = millis();
  }

  int rainAnalog = analogRead(RAIN_ANALOG_PIN);
  bool rainDetected = rainAnalog < 550;

  // Accelerometer reading
  sensors_event_t event;
  accel.getEvent(&event);

  float accMag = sqrt(
    event.acceleration.x * event.acceleration.x +
    event.acceleration.y * event.acceleration.y +
    event.acceleration.z * event.acceleration.z
  );

  // Debug: Uncomment if needed
  // Serial.println("AccMag: " + String(accMag, 2));

  // --- Step Detection ---
  float stepHigh = 11.2;
  float stepLow = 9.6;

  if (accMag > stepHigh && !stepDetected && millis() - lastStepTime > 300) {
    stepCount++;
    stepDetected = true;
    lastStepTime = millis();
  } else if (accMag < stepLow && stepDetected) {
    stepDetected = false;
  }

  // --- Improved Fall Detection ---
  if (!inFreeFall && accMag < freeFallThreshold) {
    inFreeFall = true;
    freeFallStart = millis();
  }

  if (inFreeFall && accMag > impactThreshold &&
      (millis() - freeFallStart <= maxFallWindow) &&
      (millis() - lastStepTime > postStepBuffer) &&
      (millis() - lastFallTime > fallCooldown)) {

    inFreeFall = false;
    fallDetected = true;
    lastFallTime = millis();
    lastMovementTime = millis();
    bluetooth.println("ALERT_FALL");
  }

  // Reset inFreeFall if fall not completed
  if (inFreeFall && millis() - freeFallStart > maxFallWindow) {
    inFreeFall = false;
  }

  // Update last movement time
  if (accMag > 1.5) {
    lastMovementTime = millis();
  }

  // Optional: Confirm fall if no movement after impact
  if (fallDetected && millis() - lastMovementTime > inactivityAfterFall) {
    bluetooth.println("FALL_CONFIRMED");
  }

  // Reset fall flag after cooldown
  if (millis() - lastFallTime > fallCooldown) {
    fallDetected = false;
  }

  // --- LCD Display ---
  lcd.setCursor(0, 0);
  lcd.print("S:"); lcd.print(stepCount);
  lcd.print(" V:"); lcd.print(speed, 1); lcd.print("m ");

  lcd.setCursor(0, 1);
  lcd.print(fallDetected ? "FALL " : "SAFE ");
  if (rainDetected) lcd.print("RAIN");
  else lcd.print("    ");

  // --- Bluetooth/Serial Output ---
  String output = "Temp: " + String(temp, 1) + " C, ";
  output += "Humidity: " + String(humidity, 1) + "%, ";
  output += "Steps: " + String(stepCount) + ", ";
  output += "Speed: " + String(speed, 2) + " m/s, ";
  output += "Fall: " + String(fallDetected ? "YES" : "NO") + ", ";
  output += "Rain: " + String(rainDetected ? "YES" : "NO") + ", ";
  output += "Rain(A): " + String(rainAnalog);
  sendData(output);

  if (rainDetected) bluetooth.println("ALERT_RAIN");
}

void calculateSpeed() {
  unsigned long currentTime = millis();
  if (currentTime - speedTimer >= 5000) {
    int stepsTaken = stepCount - stepsAtLastCheck;
    speed = (stepsTaken * strideLength) / 5.0;
    stepsAtLastCheck = stepCount;
    speedTimer = currentTime;

    bluetooth.println("Speed: " + String(speed, 2) + " m/s");
  }
}

void handleBluetoothCommands() {
  if (bluetooth.available()) {
    String command = bluetooth.readStringUntil('\n');
    command.trim();
    command.toUpperCase();

    if (command == "RESET") {
      stepCount = 0;
      sendData("Step counter reset.");
    } else if (command == "STATUS") {
      readSensors();
    } else {
      sendData("Unknown command: " + command);
    }
  }
}

void sendData(String message) {
  Serial.println(message);
  bluetooth.println(message);
}