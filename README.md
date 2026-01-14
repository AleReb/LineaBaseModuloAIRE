# Línea Base Módulo AIRE - ESP32-S3

Firmware unificado para el monitoreo de calidad de aire utilizando un ESP32-S3. Este proyecto integra múltiples sensores y proporciona una salida de datos duplicada para monitoreo local y remoto.

## Sensores Integrados
- **ADS1115 (x8)**: Lectura de 8 canales ADC a través de dos buses I2C independientes.
- **PMS5003**: Sensor de material particulado (PM1.0, PM2.5, PM10), temperatura, humedad y formaldehído.
- **MH-Z19**: Sensor de CO2 por infrarrojos (NDIR).
- **Winsen VOC**: Sensor de compuestos orgánicos volátiles.

## Configuración de Hardware (Pines)
- **I2C_0 (ADS1115)**: SDA=8, SCL=9
- **I2C_1 (ADS1115)**: SDA=10, SCL=11
- **UART0 (Salida Duplicada)**: RX=44, TX=43 (Serial0)
- **UART1 (PMS5003)**: RX=1, TX=2
- **UART2 (MH-Z19)**: RX=4, TX=3
- **SoftSerial (Winsen VOC)**: RX=5, TX=6

## Salida de Datos
Los datos se imprimen cada segundo en formato legible tanto en el puerto **USB Serial** como en el puerto **Serial0 (Pines 43/44)**.

## Descargo de Responsabilidad (Disclaimer)
Este código se ofrece 'tal cual', sin garantías de ningún tipo, expresas o implícitas. El autor no se hace responsable de ningún daño o pérdida derivada del uso de este software. El usuario lo utiliza bajo su propio riesgo.

## Licencia
Este proyecto está bajo la licencia [Creative Commons Attribution-NonCommercial 4.0 International (CC BY-NC 4.0)](LICENSE).

---
**Autor**: Alejandro Rebolledo (arebolledo@udd.cl)
