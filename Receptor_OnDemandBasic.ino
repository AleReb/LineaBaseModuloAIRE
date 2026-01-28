/*
 * Project: Receptor_OnDemand_Basic
 * Description: Receptor bloqueante básico que solicita datos y los imprime.
 * Autor: Alejandro Rebolledo (arebolledo@udd.cl)
 * Licencia: CC BY-NC 4.0
 */

#include <Arduino.h>

/* ===================== PIN DEFINITIONS ===================== */
static const int SENSOR_POWER_PIN = 4;

// UART link from emitter -> receiver (ESP32)
static const int RX_PIN = 39; // RX (input-only pin is OK here)
static const int TX_PIN = 33; // TX (must be output-capable)
static const uint32_t LINK_BAUD =
    9600; // Recommended. Change to 9600 if needed.

/* ===================== UART INSTANCE ===================== */
HardwareSerial pmsSerial(1);

void setup() {
  Serial.begin(115200);

  // Power on sensors (receiver side)
  pinMode(SENSOR_POWER_PIN, OUTPUT);
  digitalWrite(SENSOR_POWER_PIN, HIGH);

  pmsSerial.begin(LINK_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);

  delay(2000);
  Serial.println("=== RECEPTOR BÁSICO INICIADO ===");
}

void solicitarYLeer() {
  Serial.println(">> Enviando SCAN...");
  pmsSerial.println("SCAN");

  bool finDeTrama = false;
  unsigned long timeout = millis();

  // Bucle bloqueante
  while (!finDeTrama && (millis() - timeout < 5000)) {
    if (pmsSerial.available()) {
      String data = pmsSerial.readStringUntil('\n');
      data.trim();

      if (data.length() > 0) {
        Serial.println("[DATO]: " + data);

        if (data == "END") {
          Serial.println(">> Trama finalizada correctamente.");
          finDeTrama = true;
        }
      }
      // Reset timeout with each line received
      timeout = millis();
    }
  }

  if (!finDeTrama) {
    Serial.println(">> ERROR: Timeout esperando END.");
  }
}

void loop() {
  // Pedir datos cada 5 segundos
  solicitarYLeer();
  delay(5000);
}
