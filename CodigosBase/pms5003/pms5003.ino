/*!
 * Air Quality Monitor (PM 2.5, HCHO, Temperature & Humidity)
 * UART output: 32-byte frames @9600
 * Modified by Alejandro Rebolledo to include formaldehyde (HCHO) field.
 */

#include <Arduino.h>

/* ===== HW UART1 (instead of SoftwareSerial) ===== */
static const int PMS_RX_PIN = 1;   // ESP32 RX  <- Sensor TX
static const int PMS_TX_PIN = 2;   // ESP32 TX  -> Sensor RX
static const uint32_t PMS_BAUD = 9600;

HardwareSerial pms5(1);  // UART1

/* ===== Data ===== */
unsigned int PMS1 = 0, PMS2_5 = 0, PMS10 = 0;
unsigned int TPS = 0, HDS = 0;
float FMHDSB = 0.0f;

static uint8_t frame[32];
static uint8_t idx = 0;

static unsigned long lastPrintMs = 0;
static const unsigned long PRINT_INTERVAL_MS = 3000;

static char tempStr[16];

/* ===== Helpers ===== */
static uint16_t checksum16(const uint8_t *buf, int len) {
  uint16_t sum = 0;
  for (int i = 0; i < len; i++) sum += buf[i];
  return sum;
}

static void parseFrame32(const uint8_t *buf) {
  // Basic header check (most of these modules start with 0x42 0x4D)
  if (buf[0] != 0x42 || buf[1] != 0x4D) {
    return;
  }

  // Checksum: last 2 bytes = sum of bytes [0..29]
  uint16_t CR1 = ((uint16_t)buf[30] << 8) | buf[31];
  uint16_t CR2 = checksum16(buf, 30);

  if (CR1 != CR2) {
    PMS1 = PMS2_5 = PMS10 = 0;
    TPS = HDS = 0;
    FMHDSB = 0.0f;
    return;
  }

  // PM values (based on your original offsets)
  PMS1   = ((uint16_t)buf[10] << 8) | buf[11];
  PMS2_5 = ((uint16_t)buf[12] << 8) | buf[13];
  PMS10  = ((uint16_t)buf[14] << 8) | buf[15];

  // Formaldehyde (HCHO) at [22..23]
  uint16_t hcho_raw = ((uint16_t)buf[22] << 8) | buf[23];
  FMHDSB = (float)hcho_raw / 1000.0f; // mg/m3

  // Temp / Humidity at [24..27]
  TPS = ((uint16_t)buf[24] << 8) | buf[25];
  HDS = ((uint16_t)buf[26] << 8) | buf[27];
}

static void pollPMS() {
  while (pms5.available() > 0) {
    uint8_t b = (uint8_t)pms5.read();

    // Sync to header: wait for 0x42 then 0x4D
    if (idx == 0) {
      if (b != 0x42) continue;
      frame[idx++] = b;
      continue;
    }
    if (idx == 1) {
      if (b != 0x4D) {
        idx = 0;
        continue;
      }
      frame[idx++] = b;
      continue;
    }

    frame[idx++] = b;

    if (idx >= 32) {
      idx = 0;
      parseFrame32(frame);
    }
  }
}

static void printData() {
  Serial.println("-----------------------pms--------------------------");

  Serial.print("Temp : ");
  snprintf(tempStr, sizeof(tempStr), "%d%d.%d", TPS / 100, (TPS / 10) % 10, TPS % 10);
  Serial.print(tempStr);
  Serial.println(" C");

  Serial.print("RH   : ");
  snprintf(tempStr, sizeof(tempStr), "%d%d.%d", HDS / 100, (HDS / 10) % 10, HDS % 10);
  Serial.print(tempStr);
  Serial.println(" %");

  Serial.print("PM1.0: ");
  Serial.print(PMS1);
  Serial.println(" ug/m3");

  Serial.print("PM2.5: ");
  Serial.print(PMS2_5);
  Serial.println(" ug/m3");

  Serial.print("PM 10: ");
  Serial.print(PMS10);
  Serial.println(" ug/m3");

  Serial.print("Formaldehyde: ");
  Serial.print(FMHDSB, 3);
  Serial.println(" mg/m3");
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println();
  Serial.println("=== Air Quality Monitor (UART1 RX=1 TX=2 @9600) ===");

  pms5.begin(PMS_BAUD, SERIAL_8N1, PMS_RX_PIN, PMS_TX_PIN);
}

void loop() {
  pollPMS();

  if (millis() - lastPrintMs >= PRINT_INTERVAL_MS) {
    lastPrintMs = millis();
    printData();
  }

  delay(2);
}
