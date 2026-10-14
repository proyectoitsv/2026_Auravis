/*
  Test de circuito - XIAO ESP32C6 + ADS1115 + MPU6050 + Módulo SD
  ------------------------------------------------------------------
  - ADS1115 y MPU6050: conectados por I2C (SDA = D4, SCL = D5)
  - Módulo SD: conectado por SPI (SCK = D8, MISO = D9, MOSI = D10, CS = D2)

  El código detecta cada dispositivo por separado y sigue funcionando
  aunque falte alguno (por ejemplo, la SD sin tarjeta insertada).

  Librerías necesarias (instalar desde el Library Manager de Arduino IDE):
    - Adafruit ADS1X15
    - Adafruit MPU6050
    - Adafruit Unified Sensor (dependencia del MPU6050)
    - SD (viene incluida en el core de ESP32)
*/

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ---- Pines I2C (XIAO ESP32C6) ----
#define SDA_PIN 22   // D4
#define SCL_PIN 23   // D5

// ---- Pines SPI para el módulo SD ----
#define SD_CS_PIN 2  // D2 (podés cambiarlo por cualquier pin libre)
// SCK = D8 (GPIO19), MISO = D9 (GPIO20), MOSI = D10 (GPIO18) -> son los SPI por defecto del XIAO C6

Adafruit_ADS1115 ads;
Adafruit_MPU6050 mpu;

bool adsOK = false;
bool mpuOK = false;
bool sdOK  = false;

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("\n=== Test de circuito XIAO ESP32C6 ===");

  // ---- Iniciar I2C ----
  Wire.begin(SDA_PIN, SCL_PIN);

  // ---- Escaneo I2C rápido (útil para confirmar direcciones) ----
  Serial.println("\nEscaneando bus I2C...");
  byte count = 0;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  Dispositivo encontrado en 0x%02X\n", addr);
      count++;
    }
  }
  if (count == 0) Serial.println("  No se encontró ningún dispositivo I2C.");

  // ---- ADS1115 ----
  Serial.println("\nInicializando ADS1115...");
  if (ads.begin()) {
    adsOK = true;
    Serial.println("  ADS1115 OK");
  } else {
    Serial.println("  ADS1115 NO detectado (revisar cableado/dirección)");
  }

  // ---- MPU6050 ----
  Serial.println("\nInicializando MPU6050...");
  if (mpu.begin()) {
    mpuOK = true;
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("  MPU6050 OK");
  } else {
    Serial.println("  MPU6050 NO detectado (revisar cableado/dirección)");
  }

  // ---- Módulo SD ----
  Serial.println("\nInicializando módulo SD...");
  SPI.begin(19, 20, 18, SD_CS_PIN); // SCK, MISO, MOSI, CS
  if (SD.begin(SD_CS_PIN)) {
    sdOK = true;
    Serial.println("  Tarjeta SD detectada correctamente");
  } else {
    Serial.println("  No se detectó tarjeta SD (esperado si todavía no la conectaste/insertaste)");
  }

  Serial.println("\n=== Resumen ===");
  Serial.printf("ADS1115: %s\n", adsOK ? "OK" : "FALLA");
  Serial.printf("MPU6050: %s\n", mpuOK ? "OK" : "FALLA");
  Serial.printf("Modulo SD: %s\n", sdOK ? "OK" : "SIN TARJETA/NO CONECTADO");
  Serial.println("================\n");
}

void loop() {
  Serial.println("---- Lecturas ----");

  if (adsOK) {
    int16_t a0 = ads.readADC_SingleEnded(0);
    float voltage = ads.computeVolts(a0);
    Serial.printf("ADS1115 A0: %d (%.3f V)\n", a0, voltage);
  }

  if (mpuOK) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    Serial.printf("Accel (m/s^2): X=%.2f Y=%.2f Z=%.2f\n",
                  a.acceleration.x, a.acceleration.y, a.acceleration.z);
    Serial.printf("Gyro (rad/s):  X=%.2f Y=%.2f Z=%.2f\n",
                  g.gyro.x, g.gyro.y, g.gyro.z);
    Serial.printf("Temp: %.2f C\n", temp.temperature);
  }

  if (sdOK) {
    File testFile = SD.open("/test.txt", FILE_APPEND);
    if (testFile) {
      testFile.printf("Lectura en millis=%lu\n", millis());
      testFile.close();
      Serial.println("Dato escrito en /test.txt");
    } else {
      Serial.println("Error al abrir /test.txt en la SD");
    }
  }

  Serial.println();
  delay(1000);
}
