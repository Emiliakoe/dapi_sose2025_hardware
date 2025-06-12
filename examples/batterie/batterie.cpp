#include <Arduino.h>

#define BOARD_BAT_ADC_PIN 35  // Batteriemessung

void setup() {
  Serial.begin(115200);
  delay(1000); // Warten auf seriellen Monitor
  analogReadResolution(12); // 12 Bit ADC (0 - 4095)
}

float readBatteryVoltage() {
  int adcValue = analogRead(BOARD_BAT_ADC_PIN);
  // Bei TTGO T-A7670E ist i.d.R. ein 100k/100k Spannungsteiler vorhanden -> *2
  float voltage = (adcValue / 4095.0) * 3.3 * 2;
  return voltage;
}

int voltageToPercent(float voltage) {
  float percent = (voltage - 3.0) / (4.2 - 3.0) * 100.0;
  return constrain(percent, 0, 100);
}

void loop() {
  float voltage = readBatteryVoltage();
  int percent = voltageToPercent(voltage);

  Serial.print("Batteriespannung: ");
  Serial.print(voltage, 2);
  Serial.print(" V | Ladung: ");
  Serial.print(percent);
  Serial.println(" %");

  delay(5000);
}
