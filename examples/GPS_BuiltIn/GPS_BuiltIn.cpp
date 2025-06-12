#include "utilities.h"
#include <Arduino.h>
#include <TinyGsmClient.h>
#include <PubSubClient.h>

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

// === SIM-Zugang (PIN falls nötig) ===
const char SIM_PIN[] = "9180";  // <<== Hier deine echte PIN eintragen

// === Congstar APN-Daten ===
const char apn[]  = "internet.telekom";
const char user[] = "016097693296";
const char pass[] = "DapiProjekt2025";

// === MQTT-Broker ===
const char* broker = "test.mosquitto.org"; // "broker.mqttdashboard.com";
const int   port   = 1883;
const char* topic  = "dapi2025/jEGrvfPcYMMuuMgMVCZeOhaSTz03/test";

void setup()
{
  SerialMon.begin(115200);
  delay(1000);

  // Einschalten & Reset
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

  // SIM entsperren
  if (modem.getSimStatus() != SIM_READY) {
    SerialMon.println("SIM entsperren...");
    if (!modem.simUnlock(SIM_PIN)) {
      SerialMon.println("Fehler: SIM-PIN falsch oder fehlgeschlagen.");
      while (true);
    }
  }

  // Netz verbinden
  SerialMon.println("Warte auf Netzwerk...");
  if (!modem.waitForNetwork()) {
    SerialMon.println("Fehler: Kein GSM-Netz.");
    while (true);
  }

  // Internet verbinden
  SerialMon.println("Verbinde mit GPRS...");
  if (!modem.gprsConnect(apn, user, pass)) {
    SerialMon.println("Fehler: GPRS fehlgeschlagen.");
    while (true);
  }

  // GPS aktivieren
  SerialMon.println("Aktiviere GPS...");
  while (!modem.enableGPS(MODEM_GPS_ENABLE_GPIO, MODEM_GPS_ENABLE_LEVEL)) {
    SerialMon.print(".");
    delay(500);
  }
  modem.setGPSBaud(115200);
  SerialMon.println("\nGPS aktiviert.");

  // MQTT verbinden
  mqtt.setServer(broker, port);
  SerialMon.println("Verbinde mit MQTT...");
  while (!mqtt.connect("LilyGO-GPS-Client")) {
    SerialMon.print(".");
    delay(1000);
  }
  SerialMon.println("\nMQTT verbunden.");
}

void loop()
{
  // === Verbindung prüfen ===
  if (!mqtt.connected()) {
    SerialMon.println("MQTT-Verbindung verloren. Erneuter Verbindungsversuch...");
    while (!mqtt.connect("LilyGO-GPS-Client")) {
      SerialMon.print(".");
      delay(1000);
    }
    SerialMon.println("\nMQTT wieder verbunden.");
  }

  // === Verbindung aktiv halten ===
  mqtt.loop();  // Wichtig: vor dem publish aufrufen!

  // === GPS-Daten holen ===
  float lat = 0;
  float lon = 0;
  uint8_t fixMode = 0;

  if (modem.getGPS(&fixMode, &lat, &lon)) {
    SerialMon.print("Latitude: ");
    SerialMon.print(lat, 6);
    SerialMon.print(", Longitude: ");
    SerialMon.println(lon, 6);

    // === MQTT-Nachricht senden ===
    char payload[100];
    snprintf(payload, sizeof(payload), "{\"latitude\": %.6f, \"longitude\": %.6f}", lat, lon);

    if (mqtt.publish(topic, payload)) {
      SerialMon.println("MQTT-Daten gesendet.");
    } else {
      SerialMon.println("Fehler beim Senden. Versuche MQTT-Reconnect...");

      // Verbindung resetten & neu aufbauen
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

  delay(10000);  // 10 Sekunden warten bis zum nächsten Senden
}

#ifndef TINY_GSM_FORK_LIBRARY
#error "Bitte Bibliotheken aus dem LilyGO lib-Ordner installieren!"
#endif
