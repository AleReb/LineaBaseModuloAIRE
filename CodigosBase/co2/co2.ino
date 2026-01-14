#include <Arduino.h>
#include "MHZ19.h"

/* ===== MH-Z19 CO2 via HW UART2 ===== */
static const int MHZ19_RX_PIN = 4;     // ESP32 RX  <- Sensor TX
static const int MHZ19_TX_PIN = 3;     // ESP32 TX  -> Sensor RX
static const uint32_t MHZ19_BAUD = 9600;

HardwareSerial co2Serial(2);   // UART2
MHZ19 co2Sensor;

static bool co2_ok = false;

double co2Ppm = 0;
double co2Raw = 0;
double co2Custom = 0;

static uint32_t lastReadMs = 0;

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println();
  Serial.println("=== MH-Z19 CO2 Reader (UART2 RX=3 TX=4 @9600) ===");

  co2Serial.begin(MHZ19_BAUD, SERIAL_8N1, MHZ19_RX_PIN, MHZ19_TX_PIN);
  co2Sensor.begin(co2Serial);

  // Recommended: disable auto-calibration unless you really want ABC
  co2Sensor.autoCalibration(false);

  co2_ok = true;
  Serial.println("[INIT] CO2 serial started, MH-Z19 initialized");
}

static void readCO2() {
  co2Ppm = co2Sensor.getCO2();

  if (co2Sensor.errorCode == RESULT_OK) {
    co2Raw = co2Sensor.getCO2Raw();
    co2Custom = -0.674 * co2Raw + 36442;

    Serial.print("CO2: ");
    Serial.print(co2Ppm);
    Serial.print(" ppm | RAW: ");
    Serial.print(co2Raw, 2);
    Serial.print(" | CUSTOM: ");
    Serial.print(co2Custom, 2);
    Serial.println(" ppm");

  } else {
    Serial.print("[ERR] MH-Z19 errorCode: ");
    Serial.println(co2Sensor.errorCode);
  }
}

void loop() {
  // Read every 2 seconds (safe for MH-Z19)
  if (millis() - lastReadMs >= 2000) {
    lastReadMs = millis();
    if (co2_ok) readCO2();
  }

  delay(2);
}
