/*
 * Project: Receptor_OnDemand_Pro
 * Description: Receptor bloqueante con almacenamiento multi-dirección para ADS.
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
HardwareSerial pmsSerial(2);

/* ===================== VARIABLES GLOBALES DE ALMACENAMIENTO
 * ===================== */
// PMS5003
float pmsTempEXT = 0, pmsHumEXT = 0, pmsHchoEXT = 0;
uint32_t pm1EXT = 0, pm25EXT = 0, pm10EXT = 0;

// CO2 MH-Z19
double co2PpmEXT = 0, co2RawEXT = 0, co2CustomEXT = 0;

// VOC Winsen
uint16_t tvocEXT = 0;

/*
  ADS1115: [Direccion][Canal]
  Direcciones mapeadas: 0=0x48, 1=0x49, 2=0x4A, 3=0x4B
*/
int16_t ads0_vEXT[4][4] = {{0}}; // I2C_0
int16_t ads1_vEXT[4][4] = {{0}}; // I2C_1

/* ===================== FUNCIONES DE AYUDA ===================== */
float extraerValor(String cad, String clave) {
  int p = cad.indexOf(clave + ":");
  if (p == -1)
    return 0;
  int inicio = p + clave.length() + 1;
  int fin = cad.indexOf(",", inicio);
  if (fin == -1)
    fin = cad.length();
  return cad.substring(inicio, fin).toFloat();
}

int getAddrIndex(String addrHex) {
  if (addrHex.equalsIgnoreCase("0x48"))
    return 0;
  if (addrHex.equalsIgnoreCase("0x49"))
    return 1;
  if (addrHex.equalsIgnoreCase("0x4A"))
    return 2;
  if (addrHex.equalsIgnoreCase("0x4B"))
    return 3;
  return -1;
}

void procesarLinea(String linea) {
  int pComa = linea.indexOf(',');
  if (pComa == -1)
    return;

  String tipo = linea.substring(0, pComa);
  String payload = linea.substring(pComa + 1);

  if (tipo == "PMS") {
    pm1EXT = (uint32_t)extraerValor(payload, "PM1.0");
    pm25EXT = (uint32_t)extraerValor(payload, "PM2.5");
    pm10EXT = (uint32_t)extraerValor(payload, "PM10");
    pmsTempEXT = extraerValor(payload, "T");
    pmsHumEXT = extraerValor(payload, "RH");
    pmsHchoEXT = extraerValor(payload, "HCHO");
  } else if (tipo == "CO2") {
    co2PpmEXT = extraerValor(payload, "PPM");
    co2RawEXT = extraerValor(payload, "RAW");
  } else if (tipo == "VOC") {
    tvocEXT = (uint16_t)extraerValor(payload, "TVOC");
  } else if (tipo == "ADS") {
    // Formato: ADS,BUS,ADDR,V0:raw,V1:raw...
    int s2 = payload.indexOf(',');
    String bus = payload.substring(0, s2);
    String remaining = payload.substring(s2 + 1);

    int s3 = remaining.indexOf(',');
    String addr = remaining.substring(0, s3);
    String dataAds = remaining.substring(s3 + 1);

    int addrIdx = getAddrIndex(addr);
    if (addrIdx != -1) {
      int16_t (*ptrArr)[4] = (bus == "I2C_0") ? ads0_vEXT : ads1_vEXT;
      ptrArr[addrIdx][0] = (int16_t)extraerValor(dataAds, "V0");
      ptrArr[addrIdx][1] = (int16_t)extraerValor(dataAds, "V1");
      ptrArr[addrIdx][2] = (int16_t)extraerValor(dataAds, "V2");
      ptrArr[addrIdx][3] = (int16_t)extraerValor(dataAds, "V3");
    }
  }
}

void solicitarYLeer() {
  Serial.println("\n--- Iniciando ESCANEO On-Demand ---");
  pmsSerial.println("SCAN");

  bool finDeTrama = false;
  unsigned long timeout = millis();

  while (!finDeTrama && (millis() - timeout < 5000)) {
    if (pmsSerial.available()) {
      String data = pmsSerial.readStringUntil('\n');
      data.trim();
      if (data == "END") {
        finDeTrama = true;
      } else if (data.length() > 0) {
        procesarLinea(data);
      }
      timeout = millis();
    }
  }

  if (finDeTrama) {
    Serial.println(">>> Datos sincronizados.");

    // 1. PMS
    Serial.printf("PMS Temp: %.1f C  Hum: %.1f %%  PM1.0: %u ug/m3  PM2.5: %u "
                  "ug/m3  PM10: %u ug/m3\n",
                  pmsTempEXT, pmsHumEXT, pm1EXT, pm25EXT, pm10EXT);

    // 2. CO2 (Concatenación y cálculo Custom)
    co2CustomEXT = -0.674 * co2RawEXT + 36442;
    String outputCO2 = "CO2_Internal: " + String(co2PpmEXT, 0) +
                       " ppm  CO2_Raw: " + String(co2RawEXT, 2) +
                       "  CO2_Custom: " + String(co2CustomEXT, 2) + " ppm";
    Serial.println(outputCO2);

    // 3. VOC (Impresión simple)
    Serial.println("VOC TVOC: " + String(tvocEXT) + " ug/m3");

    // 4. ADS Resumen
    const char *addrs[] = {"0x48", "0x49", "0x4A", "0x4B"};
    for (int b = 0; b < 2; b++) {
      Serial.print(b == 0 ? "ADS I2C_0: " : "ADS I2C_1: ");
      for (int i = 0; i < 4; i++) {
        int16_t *chs = (b == 0) ? ads0_vEXT[i] : ads1_vEXT[i];
        // Solo mostramos si hay algún valor distinto de cero (opcional, aquí
        // mostramos todo)
        Serial.print("[" + String(addrs[i]) + ":");
        for (int c = 0; c < 4; c++)
          Serial.print(String(chs[c]) + (c < 3 ? "," : ""));
        Serial.print("] ");
      }
      Serial.println();
    }
    Serial.println("------------------------------------------");
  }
}

void setup() {
  Serial.begin(115200);

  // Power on sensors (receiver side)
  pinMode(SENSOR_POWER_PIN, OUTPUT);
  digitalWrite(SENSOR_POWER_PIN, HIGH);

  pmsSerial.begin(LINK_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);
  delay(2000);
  Serial.println("=== RECEPTOR PRO V2 (SOPORTE MULTI-ADS) ===");
}

void loop() {
  solicitarYLeer();
  delay(10000);
}
