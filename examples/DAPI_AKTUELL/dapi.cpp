#include "utilities.h"
#include <Arduino.h>
#include <Wire.h>
#include <DFRobot_BMI160.h>
#include <TinyGsmClient.h>
#include <PubSubClient.h>

// === Modem-Definition und Debug ===
#define TINY_GSM_MODEM_SIM7600
#define TINY_GSM_RX_BUFFER 1024
#define SerialMon Serial
#define TINY_GSM_DEBUG SerialMon
#define DUMP_AT_COMMANDS

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

TinyGsmClient gsmClient(modem);
PubSubClient mqtt(gsmClient);

// === Zugangsdaten ===
const char SIM_PIN[] = "9180";
const char apn[]  = "internet.telekom";
const char user[] = "016097693296";
const char pass[] = "DapiProjekt2025";
const char* broker = "test.mosquitto.org";
const int   port   = 1883;
const char* topic  = "dapi2025/jEGrvfPcYMMuuMgMVCZeOhaSTz03/test";

// === Sensor ===
#define SDA_PIN 21
#define SCL_PIN 22
#define I2C_ADDR 0x69
DFRobot_BMI160 bmi160;

// === Batterie-Pin ===
#define BOARD_BAT_ADC_PIN 35

// === GPS-Variablen ===
float lastLat = 0.0, lastLon = 0.0;
unsigned long lastGPSMillis = 0;

// === Hilfsfunktionen ===
float computeDistance(float lat1, float lon1, float lat2, float lon2) {
  const float R = 6371000.0;
  float dLat = radians(lat2 - lat1);
  float dLon = radians(lon2 - lon1);
  float a = sin(dLat / 2) * sin(dLat / 2) +
            cos(radians(lat1)) * cos(radians(lat2)) *
            sin(dLon / 2) * sin(dLon / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return R * c;
}

float readBatteryVoltage() {
  int adcValue = analogRead(BOARD_BAT_ADC_PIN);
  return (adcValue / 4095.0) * 3.3 * 2; // Spannungsteiler x2
}

int voltageToPercent(float voltage) {
  float percent = (voltage - 3.0) / (4.2 - 3.0) * 100.0;
  return constrain(percent, 0, 100);
}

void initSensor() {
  if (bmi160.softReset() != BMI160_OK || bmi160.I2cInit(I2C_ADDR) != BMI160_OK) {
    SerialMon.println("BMI160 nicht initialisiert.");
    while (true);
  }
  SerialMon.println("BMI160 bereit.");
}

float vecLength(float x, float y, float z) {
  return sqrt(x * x + y * y + z * z);
}

// === Setup ===
void setup() {
  SerialMon.begin(115200);
  delay(1000);
  analogReadResolution(12);
  Wire.begin(SDA_PIN, SCL_PIN);
  initSensor();

#ifdef BOARD_POWERON_PIN
  pinMode(BOARD_POWERON_PIN, OUTPUT);
  digitalWrite(BOARD_POWERON_PIN, HIGH);
#endif

#ifdef MODEM_RESET_PIN
  pinMode(MODEM_RESET_PIN, OUTPUT);
  digitalWrite(MODEM_RESET_PIN, !MODEM_RESET_LEVEL);
  delay(100);
  digitalWrite(MODEM_RESET_PIN, MODEM_RESET_LEVEL);
  delay(2600);
  digitalWrite(MODEM_RESET_PIN, !MODEM_RESET_LEVEL);
#endif

  pinMode(BOARD_PWRKEY_PIN, OUTPUT);
  digitalWrite(BOARD_PWRKEY_PIN, LOW);
  delay(100);
  digitalWrite(BOARD_PWRKEY_PIN, HIGH);
  delay(1000);
  digitalWrite(BOARD_PWRKEY_PIN, LOW);

  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);
  delay(3000);

  SerialMon.println("Modem starten...");
  if (!modem.testAT()) {
    SerialMon.println("Fehler: Modem antwortet nicht!");
    while (true);
  }

  if (modem.getSimStatus() != SIM_READY) {
    SerialMon.println("SIM entsperren...");
    if (!modem.simUnlock(SIM_PIN)) {
      SerialMon.println("Fehler: SIM-PIN falsch oder fehlgeschlagen.");
      while (true);
    }
  }

  SerialMon.println("Warte auf Netzwerk...");
  if (!modem.waitForNetwork()) {
    SerialMon.println("Fehler: Kein GSM-Netz.");
    while (true);
  }

  SerialMon.println("Verbinde mit GPRS...");
  if (!modem.gprsConnect(apn, user, pass)) {
    SerialMon.println("Fehler: GPRS fehlgeschlagen.");
    while (true);
  }

  SerialMon.println("Aktiviere GPS...");
  while (!modem.enableGPS(MODEM_GPS_ENABLE_GPIO, MODEM_GPS_ENABLE_LEVEL)) {
    SerialMon.print(".");
    delay(500);
  }
  modem.setGPSBaud(115200);
  SerialMon.println("\nGPS aktiviert.");

  mqtt.setServer(broker, port);
  SerialMon.println("Verbinde mit MQTT...");
  while (!mqtt.connect("LilyGO-GPS-Client")) {
    SerialMon.print(".");
    delay(1000);
  }
  SerialMon.println("\nMQTT verbunden.");
}

// === Loop ===
void loop() {
  if (!modem.isNetworkConnected()) {
    SerialMon.println("Netzwerk verloren. Reconnect...");
    modem.waitForNetwork();
  }

  if (!modem.isGprsConnected()) {
    SerialMon.println("GPRS verloren. Reconnect...");
    modem.gprsDisconnect();
    delay(1000);
    modem.gprsConnect(apn, user, pass);
  }

  if (!mqtt.connected()) {
    SerialMon.println("MQTT-Verbindung verloren. Erneuter Verbindungsversuch...");
    while (!mqtt.connect("LilyGO-GPS-Client")) {
      SerialMon.print(".");
      delay(1000);
    }
    SerialMon.println("\nMQTT wieder verbunden.");
  }

  mqtt.loop();

  float lat = 0, lon = 0, speed_kmph = 0.0;
  uint8_t fixMode = 0;

  if (modem.getGPS(&fixMode, &lat, &lon)) {
    SerialMon.printf("GPS: %.6f, %.6f\n", lat, lon);

    unsigned long now = millis();
    if (lastLat != 0.0 && lastLon != 0.0) {
      float dist = computeDistance(lat, lon, lastLat, lastLon);
      float tH = (now - lastGPSMillis) / 3600000.0;
      if (tH > 0.0) {
        speed_kmph = (dist / 1000.0) / tH;
        if (speed_kmph > 100.0) speed_kmph = 0.0;
      }
    }
    lastLat = lat;
    lastLon = lon;
    lastGPSMillis = now;

    int16_t data[6];
    float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
    String key_wort = "unbekannt";

    if (bmi160.getAccelGyroData(data) == BMI160_OK) {
      ax = data[3] * 0.00981;
      ay = data[4] * 0.00981;
      az = data[5] * 0.00981;
      gx = data[0];
      gy = data[1];
      gz = data[2];

      float gyroMag = vecLength(gx, gy, gz);

      if (speed_kmph < 2.0) key_wort = "ruhig";
      else if (gyroMag >= 200.0) key_wort = "schuetteln";
      else if (speed_kmph >= 10.0) key_wort = "rennen";
      else if (speed_kmph >= 2.0) key_wort = "gehen";
    }

    float voltage = readBatteryVoltage();
    int battery_percent = voltageToPercent(voltage);

    char payload[256];
    snprintf(payload, sizeof(payload),
      "{\"latitude\": %.6f, \"longitude\": %.6f, \"speed_kmph\": %.2f, "
      "\"accel\": {\"x\": %.2f, \"y\": %.2f, \"z\": %.2f}, "
      "\"gyro\": {\"x\": %.2f, \"y\": %.2f, \"z\": %.2f}, "
      "\"aktion\": \"%s\", \"akku\": %d}",
      lat, lon, speed_kmph,
      ax, ay, az,
      gx, gy, gz,
      key_wort.c_str(), battery_percent);

    if (mqtt.publish(topic, payload)) {
      SerialMon.println("MQTT-Daten gesendet:");
      SerialMon.println(payload);
    } else {
      SerialMon.println("MQTT-Senden fehlgeschlagen. Reconnect...");
      mqtt.disconnect();
      delay(1000);
      while (!mqtt.connect("LilyGO-GPS-Client")) {
        SerialMon.print(".");
        delay(1000);
      }
      SerialMon.println("\nMQTT wieder verbunden.");
    }

  } else {
    SerialMon.println("Kein GPS-Fix verfügbar.");
  }

  delay(5000); // Zyklus alle 5 Sekunden
}
