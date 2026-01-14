#include "MHZ19.h"
#include <Adafruit_ADS1X15.h>
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>

/*
 * Project: LineaBaseModuloAIRE
 * Description: Unified firmware for Air Quality Module (ESP32-S3).
 * Autor: Alejandro Rebolledo (arebolledo@udd.cl)
 * Licencia: CC BY-NC 4.0
 * Descargo de Responsabilidad: Código ofrecido 'tal cual', sin garantías.
 */

/* ===================== CONFIGURACIÓN DE PINES ===================== */
// I2C Buses (ADS1115)
#define I2C0_SDA 8
#define I2C0_SCL 9
#define I2C1_SDA 10
#define I2C1_SCL 11

// UART0 (Salida duplicada)
#define UART0_RX 44
#define UART0_TX 43

// UART1 (PMS5003)
#define PMS_RX_PIN 1
#define PMS_TX_PIN 2

// UART2 (MH-Z19 CO2)
#define MHZ19_RX_PIN 4 // ESP32 RX <- Sensor TX
#define MHZ19_TX_PIN 3 // ESP32 TX -> Sensor RX

// SoftwareSerial (VOC Winsen)
#define VOC_RX_PIN 6 // ESP32 RX <- Sensor TX
#define VOC_TX_PIN 5 // ESP32 TX -> Sensor RX

/* ===================== OBJETOS Y VARIABLES ===================== */
TwoWire I2C_0 = TwoWire(0);
TwoWire I2C_1 = TwoWire(1);

// ADS1115
const uint8_t NUM_BUSES = 2;
const uint8_t NUM_ADS = 4;
const uint8_t ADS_ADDRESSES[NUM_ADS] = {0x48, 0x49, 0x4A, 0x4B};
Adafruit_ADS1115 ads[NUM_BUSES][NUM_ADS];
bool ads_ok[NUM_BUSES][NUM_ADS] = {false};
TwoWire *buses[NUM_BUSES] = {&I2C_0, &I2C_1};
const char *BUS_NAMES[NUM_BUSES] = {"I2C_0", "I2C_1"};

// PMS5003 (UART1)
HardwareSerial pmsSerial(1);
unsigned int PMS1 = 0, PMS2_5 = 0, PMS10 = 0;
unsigned int TPS = 0, HDS = 0;
float FMHDSB = 0.0f;
uint8_t pmsBuffer[32];
uint8_t pmsIdx = 0;

// MH-Z19 (UART2)
HardwareSerial co2Serial(2);
MHZ19 co2Sensor;
double co2Ppm = 0, co2Raw = 0, co2Custom = 0;
bool co2_ok = false;

// Winsen VOC (SoftwareSerial)
SoftwareSerial vocSerial(VOC_RX_PIN, VOC_TX_PIN);
uint16_t tvoc_ugm3 = 0;
uint8_t vocPacket[9];
uint8_t vocIdx = 0;

unsigned long lastPrintMs = 0;

/* ===================== FUNCIONES DE AYUDA ===================== */
void dualPrint(String msg) {
  Serial.print(msg);
  Serial0.print(msg);
}

void dualPrintln(String msg = "") {
  Serial.println(msg);
  Serial0.println(msg);
}

// Checksum PMS5003
uint16_t pmsChecksum(const uint8_t *buf, int len) {
  uint16_t sum = 0;
  for (int i = 0; i < len; i++)
    sum += buf[i];
  return sum;
}

// Checksum VOC Winsen
uint8_t vocChecksum(const uint8_t *buf) {
  uint8_t sum = 0;
  for (int i = 1; i < 8; i++)
    sum += buf[i];
  return (uint8_t)((~sum) + 1);
}

/* ===================== INICIALIZACIÓN ===================== */
void initSensors() {
  // I2C
  I2C_0.begin(I2C0_SDA, I2C0_SCL, 400000);
  I2C_1.begin(I2C1_SDA, I2C1_SCL, 400000);

  // ADS1115
  for (uint8_t b = 0; b < NUM_BUSES; b++) {
    for (uint8_t i = 0; i < NUM_ADS; i++) {
      buses[b]->beginTransmission(ADS_ADDRESSES[i]);
      if (buses[b]->endTransmission() == 0) {
        ads_ok[b][i] = ads[b][i].begin(ADS_ADDRESSES[i], buses[b]);
      }
    }
  }

  // UARTs
  pmsSerial.begin(9600, SERIAL_8N1, PMS_RX_PIN, PMS_TX_PIN);
  co2Serial.begin(9600, SERIAL_8N1, MHZ19_RX_PIN, MHZ19_TX_PIN);
  co2Sensor.begin(co2Serial);
  co2Sensor.autoCalibration(false);
  co2_ok = true;

  vocSerial.begin(9600);
}

/* ===================== CICLO PRINCIPAL ===================== */
void setup() {
  // USB Serial
  Serial.begin(115200);
  // Hardware Serial 0 (Pines 43/44)
  Serial0.begin(9600, SERIAL_8N1, UART0_RX, UART0_TX);

  delay(500);
  dualPrintln("\n=== MODULO AIRE: SISTEMA UNIFICADO ===");

  initSensors();
}

void loop() {
  // 1. Lectura PMS5003
  while (pmsSerial.available() > 0) {
    uint8_t b = pmsSerial.read();
    if (pmsIdx == 0 && b != 0x42)
      continue;
    if (pmsIdx == 1 && b != 0x4D) {
      pmsIdx = 0;
      continue;
    }
    pmsBuffer[pmsIdx++] = b;
    if (pmsIdx >= 32) {
      pmsIdx = 0;
      uint16_t CR1 = ((uint16_t)pmsBuffer[30] << 8) | pmsBuffer[31];
      if (CR1 == pmsChecksum(pmsBuffer, 30)) {
        PMS1 = ((uint16_t)pmsBuffer[10] << 8) | pmsBuffer[11];
        PMS2_5 = ((uint16_t)pmsBuffer[12] << 8) | pmsBuffer[13];
        PMS10 = ((uint16_t)pmsBuffer[14] << 8) | pmsBuffer[15];
        FMHDSB =
            (float)(((uint16_t)pmsBuffer[22] << 8) | pmsBuffer[23]) / 1000.0f;
        TPS = ((uint16_t)pmsBuffer[24] << 8) | pmsBuffer[25];
        HDS = ((uint16_t)pmsBuffer[26] << 8) | pmsBuffer[27];
      }
    }
  }

  // 2. Lectura VOC Winsen
  while (vocSerial.available() > 0) {
    uint8_t b = vocSerial.read();
    if (vocIdx == 0 && b != 0xFF)
      continue;
    vocPacket[vocIdx++] = b;
    if (vocIdx >= 9) {
      if (vocChecksum(vocPacket) == vocPacket[8]) {
        tvoc_ugm3 = ((uint16_t)vocPacket[4] << 8) | vocPacket[5];
      }
      vocIdx = 0;
    }
  }

  // 3. Impresión Periódica (Cada 2 segundos)
  if (millis() - lastPrintMs >= 2000) {
    lastPrintMs = millis();

    dualPrintln("\n---------------- SCAN DATA ----------------");

    // ADS
    for (uint8_t b = 0; b < NUM_BUSES; b++) {
      for (uint8_t i = 0; i < NUM_ADS; i++) {
        if (ads_ok[b][i]) {
          String adsMsg = "ADS," + String(BUS_NAMES[b]) + ",0x" +
                          String(ADS_ADDRESSES[i], HEX);
          for (int ch = 0; ch < 4; ch++) {
            adsMsg += ",V" + String(ch) + ":" +
                      String(ads[b][i].computeVolts(
                                 ads[b][i].readADC_SingleEnded(ch)),
                             3);
          }
          dualPrintln(adsMsg);
        }
      }
    }

    // PMS
    dualPrintln("PMS: PM1.0=" + String(PMS1) + ", PM2.5=" + String(PMS2_5) +
                ", PM10=" + String(PMS10) + " ug/m3");
    dualPrintln("PMS: T=" + String(TPS / 10) + "." + String(TPS % 10) +
                "C, RH=" + String(HDS / 10) + "." + String(HDS % 10) +
                "%, HCHO=" + String(FMHDSB, 3) + " mg/m3");

    // CO2
    if (co2_ok) {
      co2Ppm = co2Sensor.getCO2();
      if (co2Sensor.errorCode == RESULT_OK) {
        co2Raw = co2Sensor.getCO2Raw();
        dualPrintln("CO2: " + String(co2Ppm, 0) +
                    " ppm | RAW: " + String(co2Raw, 0));
      } else {
        dualPrintln("CO2: [ERR " + String(co2Sensor.errorCode) + "]");
      }
    }

    // VOC
    dualPrintln("VOC: " + String(tvoc_ugm3) + " ug/m3");

    dualPrintln("-------------------------------------------");
  }

  // 4. Puente: Serial0 -> Serial (USB)
  while (Serial0.available() > 0) {
    Serial.write(Serial0.read());
  }

  /*/ Opcional: Serial (USB) -> Serial0 (Para comandos remotos)
  while (Serial.available() > 0) {
    Serial0.write(Serial.read());
  }
*/

  delay(10);
}