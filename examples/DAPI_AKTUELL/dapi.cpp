#include "utilities.h"
#include <Arduino.h>
#include <Wire.h>
#include <DFRobot_BMI160.h>
#include <TinyGsmClient.h>
#include <PubSubClient.h>

// Definition des verwendeten Modems und Puffers
#define TINY_GSM_MODEM_SIM7600
#define TINY_GSM_RX_BUFFER 1024
#define SerialMon Serial
#define TINY_GSM_DEBUG SerialMon
#define DUMP_AT_COMMANDS

// Aktiviert Debugging für AT-Kommandos, um Modem-Kommunikation zu überwachen
#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

// Initialisierung des GSM-Clients und MQTT-Clients
TinyGsmClient gsmClient(modem);
PubSubClient mqtt(gsmClient);

// Zugangsdaten für SIM, APN und MQTT-Broker
const char SIM_PIN[] = "9180";
const char apn[]  = "internet.telekom";
const char user[] = "016097693296";
const char pass[] = "DapiProjekt2025";
const char* broker = "test.mosquitto.org";
const int   port   = 1883;
const char* topic  = "dapi2025/jEGrvfPcYMMuuMgMVCZeOhaSTz03/test";

// Definition der I2C-Pins und Adresse für den Bewegungssensor
#define SDA_PIN 21
#define SCL_PIN 22
#define I2C_ADDR 0x69
DFRobot_BMI160 bmi160;

// Pin für die Akku-Spannungsmessung
#define BOARD_BAT_ADC_PIN 35

// Variablen für GPS-Position und Zeit
float lastLat = 0.0, lastLon = 0.0;
unsigned long lastGPSMillis = 0;

// Berechnet die Distanz zwischen zwei GPS-Koordinaten (Haversine-Formel)
float computeDistance(float lat1, float lon1, float lat2, float lon2) {
  const float R = 6371000.0;
  float dLat = radians(lat2 - lat1);
  float dLon = radians(lon2 - lon1);
  float a = sin(dLat/2) * sin(dLat/2) + cos(radians(lat1)) * cos(radians(lat2)) * sin(dLon/2) * sin(dLon/2);
  float c = 2 * atan2(sqrt(a), sqrt(1-a));
  return R * c;
}

// Liest die aktuelle Akku-Spannung aus
float readBatteryVoltage() {
  int adcValue = analogRead(BOARD_BAT_ADC_PIN);
  return (adcValue / 4095.0) * 3.3 * 2;
}

// Wandelt die Spannung in einen Prozentwert um (für Li-Ion-Akkus)
int voltageToPercent(float voltage) {
  float percent = (voltage - 3.0) / (4.2 - 3.0) * 100.0;
  return constrain(percent, 0, 100);
}

// Initialisiert den Bewegungssensor (BMI160)
void initSensor() {
  if (bmi160.softReset() != BMI160_OK || bmi160.I2cInit(I2C_ADDR) != BMI160_OK) {
    SerialMon.println("BMI160 nicht initialisiert.");
    while (true);
  }
  SerialMon.println("BMI160 bereit.");
}

// Berechnet die Länge eines Vektors (z.B. für Beschleunigung oder Gyro)
float vecLength(float x, float y, float z) {
  return sqrt(x*x + y*y + z*z);
}

// Setup-Funktion: Initialisiert alle Komponenten und stellt Verbindungen her
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

  // Startet das Modem
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

  // Entsperrt die SIM-Karte falls nötig
  if (modem.getSimStatus() != SIM_READY) {
    SerialMon.println("SIM entsperren...");
    if (!modem.simUnlock(SIM_PIN)) {
      SerialMon.println("Fehler: SIM-PIN falsch oder fehlgeschlagen.");
      while (true);
    }
  }

  // Wartet auf das Mobilfunknetz
  SerialMon.println("Warte auf Netzwerk...");
  if (!modem.waitForNetwork()) {
    SerialMon.println("Fehler: Kein GSM-Netz.");
    while (true);
  }

  // Verbindet mit dem Internet (GPRS)
  SerialMon.println("Verbinde mit GPRS...");
  if (!modem.gprsConnect(apn, user, pass)) {
    SerialMon.println("Fehler: GPRS fehlgeschlagen.");
    while (true);
  }

  // Aktiviert GPS
  SerialMon.println("Aktiviere GPS...");
  while (!modem.enableGPS(MODEM_GPS_ENABLE_GPIO, MODEM_GPS_ENABLE_LEVEL)) {
    SerialMon.print(".");
    delay(500);
  }
  modem.setGPSBaud(115200);
  SerialMon.println("\nGPS aktiviert.");

  // Verbindet mit dem MQTT-Broker
  mqtt.setServer(broker, port);
  SerialMon.println("Verbinde mit MQTT...");
  while (!mqtt.connect("LilyGO-GPS-Client")) {
    SerialMon.print(".");
    delay(1000);
  }
  SerialMon.println("\nMQTT verbunden.");
}

// Hauptschleife: Liest Sensoren aus, bereitet Daten auf und sendet sie per MQTT
void loop() {
  // Prüft, ob MQTT noch verbunden ist, sonst reconnect
  if (!mqtt.connected()) {
    SerialMon.println("MQTT-Verbindung verloren. Erneuter Verbindungsversuch...");
    while (!mqtt.connect("LilyGO-GPS-Client")) {
      SerialMon.print(".");
      delay(1000);
    }
    SerialMon.println("\nMQTT wieder verbunden.");
  }

  mqtt.loop();

  float lat = 0, lon = 0;
  float speed_kmph = 0.0;
  uint8_t fixMode = 0;

  // Liest aktuelle GPS-Daten aus
  if (modem.getGPS(&fixMode, &lat, &lon)) {
    SerialMon.print("Latitude: ");
    SerialMon.print(lat, 6);
    SerialMon.print(", Longitude: ");
    SerialMon.println(lon, 6);

    unsigned long now = millis();
    // Berechnet Geschwindigkeit, falls vorherige Position vorhanden
    if (lastLat != 0.0 && lastLon != 0.0) {
      float distance = computeDistance(lat, lon, lastLat, lastLon);
      float timeHours = (now - lastGPSMillis) / 3600000.0;
      if (timeHours > 0.0) {
        speed_kmph = (distance / 1000.0) / timeHours;
        if (speed_kmph > 100.0) speed_kmph = 0.0; // unrealistische Werte filtern
      }
    }
    lastLat = lat;
    lastLon = lon;
    lastGPSMillis = now;

    // Liest Beschleunigungs- und Gyroskopdaten aus
    int16_t sensorData[6];
    float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
    String key_wort = "unbekannt";

    if (bmi160.getAccelGyroData(sensorData) == BMI160_OK) {
      ax = sensorData[3] * 0.00981;
      ay = sensorData[4] * 0.00981;
      az = sensorData[5] * 0.00981;
      gx = sensorData[0];
      gy = sensorData[1];
      gz = sensorData[2];

      float acc_total = vecLength(ax, ay, az);
      float gyro_total = vecLength(gx, gy, gz);

      /* // Einfache Aktivitätserkennung anhand der Sensorwerte
      if (abs(acc_total - 9.81) < 2.5 && gyro_total < 20.0)
        key_wort = "ruhig";
      else if (abs(gz) > 100.0 || abs(gy) > 100.0)
        key_wort = "schuetteln";
      else if (acc_total > 15.0 && gyro_total > 50.0)
        key_wort = "rennen";
      else if (acc_total > 11.0 || gyro_total > 10.0)
        key_wort = "gehen";
      else
        key_wort = "ruhig"; */

        if (speed_kmph < 2.0){
        key_wort = "ruhig";
      } else if (vecLength(gx, gy, gz) >= 200.0) {
        key_wort = "schuetteln";
      } else if (speed_kmph >= 10.0) {
        key_wort = "rennen";
      } else if (speed_kmph >= 2.0 && speed_kmph < 10.0) {
        key_wort = "gehen";
      } else {
        key_wort = "unbekannt";
      }
    }
    
      


    // Liest Akkuspannung und berechnet Prozentwert
    float voltage = readBatteryVoltage();
    int battery_percent = voltageToPercent(voltage);

    // Baut JSON-Payload für MQTT
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

    // Sendet die Daten per MQTT
    if (mqtt.publish(topic, payload)) {
      SerialMon.println("MQTT-Daten gesendet:");
      SerialMon.println(payload);
    } else {
      SerialMon.println("Fehler beim Senden. Versuche MQTT-Reconnect...");
      mqtt.disconnect();
      delay(1000);
      while (!mqtt.connect("LilyGO-GPS-Client")) {
        SerialMon.print(".");
        delay(1000);
      }
      SerialMon.println("\nMQTT wieder verbunden.");
    }

  } else {
    SerialMon.println("Kein GPS-Fix aktuell...");
  }

  delay(1000); // Wartet 5 Sekunden bis zum nächsten Sendezyklus
}