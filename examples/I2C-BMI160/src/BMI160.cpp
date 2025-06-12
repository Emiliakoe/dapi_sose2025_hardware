#include <Arduino.h>
#include <Wire.h>
#include <DFRobot_BMI160.h>

//Beschleunigung = die Änderung der Geschwindigkeit eines Objekts pro Zeiteinheit
// Gyroskop = misst die Winkelgeschwindigkeit eines Objekts


// I2C Pins für LILYGO T-A7670E
#define SDA_PIN 21
#define SCL_PIN 22
#define I2C_ADDR 0x69  // Adresse wenn SDO auf VCC

DFRobot_BMI160 bmi160;

void initSensor() {
  Serial.println("Initialisiere BMI160...");
  
  if (bmi160.softReset() != BMI160_OK) {
    Serial.println("BMI160 Reset fehlgeschlagen!");
    return;
  }

  if (bmi160.I2cInit(I2C_ADDR) != BMI160_OK) {
    Serial.println("BMI160 nicht gefunden!");
    return;
  }

  Serial.println("BMI160 erfolgreich initialisiert.");
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("HALLOOOO!");
  delay(1000);
  Serial.println("HALLOOOO 222!");

  Wire.begin(SDA_PIN, SCL_PIN);
  initSensor();
}

void loop() {
  // *******************
  // Eingabe prüfen
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'r' || c == 'R') {
      Serial.println("==> Reinitialisiere Sensor per Serial-Kommando...");
      initSensor();
    }
  }
  
  int16_t sensorData[6] = {0};  // gyro: 0–2, accel: 3–5

  if (bmi160.getAccelGyroData(sensorData) == BMI160_OK) {
    Serial.print("Beschleunigung [mg]: ");
    Serial.print(sensorData[3]); Serial.print(", ");
    Serial.print(sensorData[4]); Serial.print(", ");
    Serial.println(sensorData[5]);

    Serial.print("Gyroskop [°/s]: ");
    Serial.print(sensorData[0]); Serial.print(", ");
    Serial.print(sensorData[1]); Serial.print(", ");
    Serial.println(sensorData[2]);
    
  } else {
    Serial.println("Sensor-Daten konnten nicht gelesen werden.");
  }

  delay(1000);
}

