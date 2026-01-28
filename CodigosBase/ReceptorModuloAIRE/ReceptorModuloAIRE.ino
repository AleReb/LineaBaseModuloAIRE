/*
 * Project: ReceptorModuloAIRE
 * Description: Receiver firmware to process RAW lines received over UART from
 *              the emitter module. Robust, non-blocking, with optional checksum framing.
 * Author: Alejandro Rebolledo (arebolledo@udd.cl)
 * License: CC BY-NC 4.0
 */

#include <Arduino.h>

/* ===================== PIN DEFINITIONS ===================== */
static const int SENSOR_POWER_PIN = 4;

// UART link from emitter -> receiver (ESP32)
static const int LINK_RX_PIN = 39;   // RX (input-only pin is OK here)
static const int LINK_TX_PIN = 33;   // TX (must be output-capable)
static const uint32_t LINK_BAUD = 9600; // Recommended. Change to 9600 if needed.

/* ===================== UART INSTANCE ===================== */
// UART2 on ESP32: HardwareSerial(2)
HardwareSerial linkSerial(2);

/* ===================== GLOBAL FLAGS / STATUS ===================== */
static bool link_configured = false;
static bool link_active = false;      // becomes true once data is received
static bool power_ok = false;

/* ===================== TIMING ===================== */
static uint32_t lastStatusMs = 0;
static uint32_t lastRxMs = 0;
static uint32_t lastReinitTryMs = 0;
static uint32_t lastPingMs = 0;

static const uint32_t STATUS_PERIOD_MS = 3000;
static const uint32_t RX_TIMEOUT_MS = 8000;
static const uint32_t REINIT_PERIOD_MS = 5000;
static const uint32_t PING_PERIOD_MS = 5000;

/* ===================== LINE BUFFER ===================== */
static char lineBuf[320];
static size_t lineLen = 0;

/* ===================== ADS1115 CONVERSION ===================== */
/*
  For ADS1115 with GAIN_FOUR (+/- 1.024V):
  LSB size = 0.00003125 V (31.25 uV) per count.
  If your emitter uses a different gain, update this constant.
*/
static const float ADS_LSB_VOLTS = 0.00003125f;

/* ===================== HELPERS ===================== */
static uint8_t xorChecksum(const char *s) {
  uint8_t cs = 0;
  while (*s) cs ^= (uint8_t)(*s++);
  return cs;
}

static int hexToByte2(const char *p) {
  // expects at least 2 hex chars, returns -1 on error
  auto hexVal = [](char c) -> int {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
    if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
    return -1;
  };
  int hi = hexVal(p[0]);
  int lo = hexVal(p[1]);
  if (hi < 0 || lo < 0) return -1;
  return (hi << 4) | lo;
}

static void sendFramed(const char *bodyNoDollarNoStar) {
  // Send $BODY*CS\n (NMEA-like). Works even if emitter ignores it.
  uint8_t cs = xorChecksum(bodyNoDollarNoStar);
  linkSerial.print('$');
  linkSerial.print(bodyNoDollarNoStar);
  linkSerial.print('*');
  char hex[3];
  snprintf(hex, sizeof(hex), "%02X", cs);
  linkSerial.print(hex);
  linkSerial.print('\n');
}

static void tryInitLink() {
  // Keep initialization non-blocking and retryable.
  // begin() does not return success/failure, so we track "configured" and "activity".
  linkSerial.end();

#if defined(ARDUINO_ARCH_ESP32)
  linkSerial.setRxBufferSize(2048);
  linkSerial.setTxBufferSize(512);
#endif

  linkSerial.begin(LINK_BAUD, SERIAL_8N1, LINK_RX_PIN, LINK_TX_PIN);
  link_configured = true;

  Serial.print("[LINK] UART2 configured. RX=");
  Serial.print(LINK_RX_PIN);
  Serial.print(" TX=");
  Serial.print(LINK_TX_PIN);
  Serial.print(" BAUD=");
  Serial.println(LINK_BAUD);
}

static void printStatus() {
  Serial.print("[STATUS] power=");
  Serial.print(power_ok ? "OK" : "OFF");
  Serial.print(" link_cfg=");
  Serial.print(link_configured ? "YES" : "NO");
  Serial.print(" link_active=");
  Serial.print(link_active ? "YES" : "NO");
  Serial.print(" last_rx_ms_ago=");
  Serial.println((uint32_t)(millis() - lastRxMs));
}

/* ===================== PARSERS ===================== */
static void handleAdsCsv(const char *payload /* no checksum framing */) {
  // Expected examples:
  // ADS,I2C_0,0x48,V0:13296,V1:4,V2:9830,V3:5
  // ADS,I2C_1,0x49,V0:8045,V1:-3,V2:7729,V3:-3

  // Work on a modifiable copy for strtok
  char tmp[320];
  strncpy(tmp, payload, sizeof(tmp) - 1);
  tmp[sizeof(tmp) - 1] = '\0';

  char *save = nullptr;
  char *tok = strtok_r(tmp, ",", &save);
  if (!tok || strcmp(tok, "ADS") != 0) return;

  char *bus = strtok_r(nullptr, ",", &save);
  char *addr = strtok_r(nullptr, ",", &save);

  Serial.println();
  Serial.println("[ADC DATA RECEIVED]");
  if (bus) {
    Serial.print("Bus: ");
    Serial.print(bus);
    Serial.print(" | ");
  }
  if (addr) {
    Serial.print("Addr: ");
    Serial.println(addr);
  } else {
    Serial.println("Addr: (unknown)");
  }

  // Remaining tokens are channels Vx:raw
  while ((tok = strtok_r(nullptr, ",", &save)) != nullptr) {
    char *colon = strchr(tok, ':');
    if (!colon) continue;
    *colon = '\0';
    const char *chName = tok;
    const char *rawTxt = colon + 1;

    long raw = strtol(rawTxt, nullptr, 10);
    float v = (float)raw * ADS_LSB_VOLTS;

    Serial.print("  ");
    Serial.print(chName);
    Serial.print(" -> RAW: ");
    Serial.print(raw);
    Serial.print(" | Est. Volts: ");
    Serial.print(v, 4);
    Serial.println(" V");
  }
}

static void dispatchLine(const char *lineNoCrlf) {
  if (!lineNoCrlf || lineNoCrlf[0] == '\0') return;

  // Backward compatible:
  // 1) If framed: $BODY*CS
  // 2) Else: plain CSV lines (current emitter)

  char body[320];
  const char *payload = lineNoCrlf;

  bool framedOk = false;
  if (payload[0] == '$') {
    const char *star = strchr(payload, '*');
    if (star && (star - payload) > 1) {
      size_t bodyLen = (size_t)(star - (payload + 1));
      if (bodyLen < sizeof(body)) {
        memcpy(body, payload + 1, bodyLen);
        body[bodyLen] = '\0';

        // checksum: 2 hex chars after '*'
        if (strlen(star + 1) >= 2) {
          int rxCs = hexToByte2(star + 1);
          if (rxCs >= 0) {
            uint8_t calc = xorChecksum(body);
            if ((uint8_t)rxCs == calc) {
              framedOk = true;
              payload = body; // validated body
            } else {
              Serial.println("[WARN] Bad checksum frame dropped.");
              return;
            }
          } else {
            Serial.println("[WARN] Invalid checksum format dropped.");
            return;
          }
        }
      }
    }
  }

  // Classification (fixed: else-if chain)
  if (strncmp(payload, "ADS", 3) == 0) {
    handleAdsCsv(payload);
    return;
  } else if (strncmp(payload, "PMS", 3) == 0) {
    Serial.print("[PMS] ");
    Serial.println(payload);
    return;
  } else if (strncmp(payload, "CO2", 3) == 0) {
    Serial.print("[CO2] ");
    Serial.println(payload);
    return;
  } else if (strncmp(payload, "VOC", 3) == 0) {
    Serial.print("[VOC] ");
    Serial.println(payload);
    return;
  } else {
    Serial.print("[OTHER] ");
    Serial.println(payload);
    if (framedOk) {
      // Example: you can add custom actions for framed messages here.
    }
  }
}

/* ===================== UART RX (NON-BLOCKING) ===================== */
static void pollLinkUart() {
  while (linkSerial.available() > 0) {
    char c = (char)linkSerial.read();
    if (c == '\r') continue;

    lastRxMs = millis();
    link_active = true;

    if (c == '\n') {
      lineBuf[lineLen] = '\0';
      // Trim leading/trailing spaces
      // (simple trim without using String)
      char *s = lineBuf;
      while (*s == ' ' || *s == '\t') s++;
      // rtrim
      char *e = s + strlen(s);
      while (e > s && (e[-1] == ' ' || e[-1] == '\t')) e--;
      *e = '\0';

      if (*s) dispatchLine(s);

      lineLen = 0;
    } else {
      if (lineLen < (sizeof(lineBuf) - 1)) {
        lineBuf[lineLen++] = c;
      } else {
        // Overflow: drop line
        lineLen = 0;
        Serial.println("[WARN] RX line overflow, line dropped.");
      }
    }
  }
}

/* ===================== USB -> LINK COMMAND BRIDGE ===================== */
static void pollUsbToLink() {
  // Allows you to type commands in Serial Monitor and forward them to emitter.
  // Sends exactly what you type, line-based.
  static char usbBuf[180];
  static size_t usbLen = 0;

  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      usbBuf[usbLen] = '\0';
      if (usbLen > 0) {
        // If you type: PING -> send as framed message
        if (strcmp(usbBuf, "PING") == 0) {
          sendFramed("PING,receiver");
          Serial.println("[TX] Sent framed PING.");
        } else {
          // Raw line passthrough
          linkSerial.print(usbBuf);
          linkSerial.print('\n');
          Serial.print("[TX] Sent raw: ");
          Serial.println(usbBuf);
        }
      }
      usbLen = 0;
    } else {
      if (usbLen < sizeof(usbBuf) - 1) usbBuf[usbLen++] = c;
      else usbLen = 0;
    }
  }
}

/* ===================== SETUP / LOOP ===================== */
void setup() {
  Serial.begin(115200);
  delay(200);

  // Power on sensors (receiver side)
  pinMode(SENSOR_POWER_PIN, OUTPUT);
  digitalWrite(SENSOR_POWER_PIN, HIGH);
  power_ok = true;

  Serial.println("\n=== AIR MODULE RECEIVER STARTED ===");

  // Initialize link (do not abort if something is wrong; keep retrying in loop)
  tryInitLink();
  lastRxMs = millis();
}

void loop() {
  const uint32_t now = millis();

  // Poll UART link
  if (link_configured) {
    pollLinkUart();
  }

  // Optional: USB serial -> link commands
  pollUsbToLink();

  // Periodic ping (harmless if emitter ignores it)
  if (link_configured && (now - lastPingMs >= PING_PERIOD_MS)) {
    lastPingMs = now;
    sendFramed("PING,receiver");
  }

  // Status output
  if (now - lastStatusMs >= STATUS_PERIOD_MS) {
    lastStatusMs = now;
    printStatus();
  }

  // If no RX activity for a while, try re-init the UART link (non-blocking retry)
  if (link_configured && (now - lastRxMs >= RX_TIMEOUT_MS)) {
    Serial.println("[WARN] No data received recently. Will try to re-init link.");
    if (now - lastReinitTryMs >= REINIT_PERIOD_MS) {
      lastReinitTryMs = now;
      tryInitLink();
      // keep link_active false until data arrives again
      link_active = false;
      lastRxMs = now;
    }
  }

  // No blocking delay needed; keep loop responsive
  delay(1);
}
