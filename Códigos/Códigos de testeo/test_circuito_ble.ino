/*
  TEST DE CIRCUITO + TRANSMISION BLE - XIAO ESP32-C6

  Hace los mismos tests de arranque que test_circuito.ino (I2C, MPU6050, ADS1115,
  SD, leads-off) y despues transmite los datos por BLE a la app del celular.

  El ESP32 es el servidor BLE; el celular se conecta y recibe notificaciones.
    Nombre   : XIAO-ECG
    Servicio : 5b1c0001-7a3e-4d8f-9c21-3f6a8e2b4d10

  Caracteristica ECG (notify) 5b1c0002-..., 20 bytes, 25 paquetes/s:
    [0-1]  numero de paquete (uint16, sube de a 1; sirve para detectar perdidas)
    [2]    flags: bit0 = LO+, bit1 = LO-   (1 = electrodo desconectado)
    [3]    periodo de muestreo en ms
    [4-19] 4 muestras de (A0, A1); cada valor es un int16 crudo del ADS1115

  Caracteristica IMU (notify) 5b1c0003-..., 14 bytes, 10 paquetes/s:
    7 x int16: ax, ay, az, temp, gx, gy, gz (crudos del MPU6050)

  Caracteristica BATERIA (notify) 5b1c0004-..., 4 bytes, 1 paquete cada 2 s:
    [0-1] milivolts de la bateria (uint16)
    [2]   porcentaje aproximado 0-100 (255 = desconocido)
    [3]   bit0 = bateria detectada

  BATERIA - hardware: la XIAO ESP32-C6 NO trae divisor para medirla. Hay que soldar
  dos resistencias iguales (ej. 2 x 200k) en serie entre BAT+ y BAT-, y el punto
  medio al pin D0 (GPIO0). El firmware multiplica la lectura por 2.
  Con el USB conectado y la bateria cargando, la tension marca ~4.2 V aunque la
  bateria no este llena, asi que el porcentaje solo vale con la placa sin USB.

  Todo en little-endian.
  Conversion: volts = crudo * 0.000125 (GAIN_ONE), g = crudo / 16384,
              grados/s = crudo / 131, temperatura C = crudo / 340 + 36.53

  Librerias necesarias:
    - Adafruit ADS1X15 (instala Adafruit BusIO si lo pide)
    - BLE, SD, SPI y Wire vienen con el core de ESP32

  Placa: XIAO_ESP32C6 | Monitor serie: 115200
*/

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_ADS1X15.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ---------- Pines (numero GPIO) ----------
// D1=GPIO1, D2=GPIO2, D3=GPIO21, D4=GPIO22, D5=GPIO23, D6=GPIO16, D9=GPIO20, D10=GPIO18
#define PIN_SDA   20   // D9
#define PIN_SCL   18   // D10
#define PIN_MISO  1    // D1
#define PIN_CLK   2    // D2
#define PIN_MOSI  21   // D3
#define PIN_CS    22   // D4
#define PIN_LO_N  23   // D5  (LO-)
#define PIN_LO_P  16   // D6  (LO+)
#define PIN_BAT   0    // D0  (punto medio del divisor de la bateria)

#define BAT_DIV_FACTOR  2.0f   // 2 resistencias iguales -> la tension se divide por 2
#define BAT_PERIOD_MS   2000

// ---------- Direcciones I2C ----------
#define MPU_ADDR_1  0x68   // AD0 a GND
#define MPU_ADDR_2  0x69   // AD0 a VCC
#define ADS_ADDR    0x48   // ADDR a GND

// ---------- BLE ----------
#define BLE_NAME       "XIAO-ECG"
#define SERVICE_UUID   "5b1c0001-7a3e-4d8f-9c21-3f6a8e2b4d10"
#define ECG_CHAR_UUID  "5b1c0002-7a3e-4d8f-9c21-3f6a8e2b4d10"
#define IMU_CHAR_UUID  "5b1c0003-7a3e-4d8f-9c21-3f6a8e2b4d10"
#define BAT_CHAR_UUID  "5b1c0004-7a3e-4d8f-9c21-3f6a8e2b4d10"

// ---------- Muestreo ----------
#define SAMPLE_HZ        100    // muestras por segundo de A0 y A1
#define SAMPLES_PER_PKT  4      // muestras por paquete ECG (4 -> 25 paquetes/s)
#define IMU_PERIOD_MS    100    // MPU6050 a 10 Hz
#define ADS_LSB_V        0.000125f  // volts por cuenta con GAIN_ONE (+-4.096 V)

const uint32_t SAMPLE_PERIOD_US = 1000000UL / SAMPLE_HZ;

Adafruit_ADS1115 ads;
uint8_t mpuAddr = 0;
bool okSD = false, okMPU = false, okADS = false;
bool adsIniciado = false;

BLEServer *pServer = nullptr;
BLECharacteristic *pEcgChar = nullptr;
BLECharacteristic *pImuChar = nullptr;
BLECharacteristic *pBatChar = nullptr;
volatile bool deviceConnected = false;
bool oldDeviceConnected = false;

uint16_t seqPkt = 0;
int16_t bufA0[SAMPLES_PER_PKT];
int16_t bufA1[SAMPLES_PER_PKT];
uint8_t nBuf = 0;

uint32_t nextSampleUs = 0;
uint32_t lastImuMs = 0;
uint32_t lastPrintMs = 0;
uint32_t muestrasSeg = 0, paquetesSeg = 0, atrasos = 0;
float ultV0 = 0, ultV1 = 0, ultG = 0;

float batV = 0;
int batPct = -1;            // -1 = desconocido
bool batPresente = false;
uint32_t lastBatMs = 0;

// ---------- Utilidades ----------
void titulo(const char *t) {
  Serial.println();
  Serial.println("==============================");
  Serial.println(t);
  Serial.println("==============================");
}

void resultado(const char *nombre, bool ok) {
  Serial.print("[");
  Serial.print(ok ? " OK " : "FALLA");
  Serial.print("] ");
  Serial.println(nombre);
}

bool i2cPresente(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

static inline void putI16(uint8_t *p, int16_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)(((uint16_t)v) >> 8);
}

// ---------- Escaner I2C ----------
void escanerI2C() {
  titulo("1) ESCANER I2C");
  int n = 0;
  for (uint8_t a = 1; a < 127; a++) {
    if (i2cPresente(a)) {
      Serial.printf("  Dispositivo en 0x%02X", a);
      if (a == 0x68 || a == 0x69) Serial.print("  (posible MPU6050)");
      if (a >= 0x48 && a <= 0x4B) Serial.print("  (posible ADS1115)");
      Serial.println();
      n++;
    }
  }
  if (n == 0) {
    Serial.println("  No se encontro nada. Revisar SDA(D9), SCL(D10), 3V3, GND y pull-ups.");
  }
}

// ---------- MPU6050 ----------
bool mpuEscribir(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool mpuLeer(uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)mpuAddr, (int)len) != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

void testMPU() {
  titulo("2) MPU6050");
  if (i2cPresente(MPU_ADDR_1)) mpuAddr = MPU_ADDR_1;
  else if (i2cPresente(MPU_ADDR_2)) mpuAddr = MPU_ADDR_2;
  else {
    Serial.println("  MPU6050 no responde en 0x68 ni 0x69.");
    resultado("MPU6050", false);
    return;
  }
  Serial.printf("  Direccion: 0x%02X\n", mpuAddr);

  uint8_t who = 0;
  if (!mpuLeer(0x75, &who, 1)) {
    Serial.println("  No se pudo leer WHO_AM_I.");
    resultado("MPU6050", false);
    return;
  }
  Serial.printf("  WHO_AM_I = 0x%02X (esperado 0x68; algunos clones dan 0x70/0x72)\n", who);

  mpuEscribir(0x6B, 0x00);  // salir de sleep
  delay(100);

  uint8_t d[14];
  if (!mpuLeer(0x3B, d, 14)) {
    Serial.println("  No se pudieron leer los datos.");
    resultado("MPU6050", false);
    return;
  }
  int16_t ax = (d[0] << 8) | d[1];
  int16_t ay = (d[2] << 8) | d[3];
  int16_t az = (d[4] << 8) | d[5];
  int16_t t  = (d[6] << 8) | d[7];
  float g = sqrt((float)ax * ax + (float)ay * ay + (float)az * az) / 16384.0;  // rango +-2g
  Serial.printf("  Aceleracion total = %.2f g (quieto deberia dar ~1.00)\n", g);
  Serial.printf("  Temperatura = %.1f C\n", t / 340.0 + 36.53);

  okMPU = (g > 0.7 && g < 1.3);
  if (!okMPU) Serial.println("  Datos fuera de lo esperado (todo en 0 o -1 suele indicar mal contacto).");
  resultado("MPU6050", okMPU);
}

// ---------- ADS1115 ----------
void testADS() {
  titulo("3) ADS1115");
  if (!ads.begin(ADS_ADDR, &Wire)) {
    Serial.println("  ADS1115 no responde en 0x48. Revisar pin ADDR (a GND = 0x48).");
    resultado("ADS1115", false);
    return;
  }
  adsIniciado = true;
  ads.setGain(GAIN_ONE);              // +-4.096 V (1 bit = 0.125 mV)
  ads.setDataRate(RATE_ADS1115_128SPS);

  int16_t r0 = ads.readADC_SingleEnded(0);
  int16_t r1 = ads.readADC_SingleEnded(1);
  float v0 = ads.computeVolts(r0);
  float v1 = ads.computeVolts(r1);

  Serial.printf("  A0 (operacional) = %d  ->  %.3f V  (esperado 1 a 3 V)\n", r0, v0);
  Serial.printf("  A1 (AD8232)      = %d  ->  %.3f V  (esperado ~1.5 V +- ruido)\n", r1, v1);

  bool a0ok = (v0 > 0.8 && v0 < 3.2);
  bool a1ok = (v1 > 0.1 && v1 < 3.2);
  if (!a0ok) Serial.println("  A0 fuera de rango: revisar el operacional y su conexion al ADS.");
  if (!a1ok) Serial.println("  A1 fuera de rango (0 V o saturado): revisar OUTPUT del AD8232 y su alimentacion.");

  okADS = a0ok && a1ok;
  resultado("ADS1115 A0", a0ok);
  resultado("ADS1115 A1", a1ok);
}

// ---------- Tarjeta SD ----------
void testSD() {
  titulo("4) TARJETA SD");
  SPI.begin(PIN_CLK, PIN_MISO, PIN_MOSI, PIN_CS);

  if (!SD.begin(PIN_CS, SPI, 4000000)) {
    Serial.println("  No se pudo montar la SD. Revisar: tarjeta insertada, formato FAT32,");
    Serial.println("  conexiones MISO/MOSI/CLK/CS, y alimentacion del modulo (muchos piden 5 V).");
    resultado("SD", false);
    return;
  }

  uint8_t tipo = SD.cardType();
  Serial.print("  Tipo: ");
  if (tipo == CARD_MMC) Serial.println("MMC");
  else if (tipo == CARD_SD) Serial.println("SDSC");
  else if (tipo == CARD_SDHC) Serial.println("SDHC");
  else Serial.println("desconocido");
  Serial.printf("  Tamano: %llu MB\n", SD.cardSize() / (1024ULL * 1024ULL));

  File f = SD.open("/test_esp32.txt", FILE_WRITE);
  if (!f) {
    Serial.println("  No se pudo crear el archivo de prueba.");
    resultado("SD", false);
    return;
  }
  f.println("Hola desde el XIAO ESP32-C6");
  f.close();

  f = SD.open("/test_esp32.txt");
  if (!f) {
    Serial.println("  No se pudo reabrir el archivo.");
    resultado("SD", false);
    return;
  }
  String contenido = f.readStringUntil('\n');
  f.close();
  Serial.print("  Leido: ");
  Serial.println(contenido);

  okSD = contenido.startsWith("Hola desde el XIAO");
  SD.remove("/test_esp32.txt");
  resultado("SD (escritura/lectura)", okSD);
}

// ---------- Leads-off AD8232 ----------
void leerLeadsOff() {
  int lop = digitalRead(PIN_LO_P);
  int lon = digitalRead(PIN_LO_N);
  Serial.printf("  LO+ (D6) = %d | LO- (D5) = %d  -> %s\n", lop, lon,
                (lop || lon) ? "electrodo DESCONECTADO" : "electrodos conectados");
}

// ---------- Bateria ----------
float leerBateriaV() {
  uint32_t acc = 0;
  for (int i = 0; i < 16; i++) acc += analogReadMilliVolts(PIN_BAT);
  return (acc / 16.0f) * BAT_DIV_FACTOR / 1000.0f;
}

// Curva aproximada de una LiPo de 3.7 V en reposo
int porcentajeLiPo(float v) {
  static const float tv[] = {3.27, 3.61, 3.69, 3.71, 3.73, 3.75, 3.77, 3.79, 3.80, 3.82, 3.84,
                             3.85, 3.87, 3.91, 3.95, 3.98, 4.02, 4.08, 4.11, 4.15, 4.20};
  static const int tp[]  = {0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50,
                            55, 60, 65, 70, 75, 80, 85, 90, 95, 100};
  const int n = sizeof(tv) / sizeof(tv[0]);
  if (v <= tv[0]) return 0;
  if (v >= tv[n - 1]) return 100;
  for (int i = 1; i < n; i++) {
    if (v <= tv[i]) {
      float f = (v - tv[i - 1]) / (tv[i] - tv[i - 1]);
      return (int)(tp[i - 1] + f * (tp[i] - tp[i - 1]) + 0.5f);
    }
  }
  return 100;
}

void actualizarBateria() {
  batV = leerBateriaV();
  batPresente = (batV >= 2.5f && batV <= 4.5f);  // fuera de rango = sin bateria o sin divisor
  batPct = batPresente ? porcentajeLiPo(batV) : -1;
}

// ---------- BLE ----------
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *s) {
    deviceConnected = true;
  }
  void onDisconnect(BLEServer *s) {
    deviceConnected = false;
  }
};

void iniciarBLE() {
  titulo("7) BLE");
  BLEDevice::init(BLE_NAME);

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pEcgChar = pService->createCharacteristic(ECG_CHAR_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pImuChar = pService->createCharacteristic(IMU_CHAR_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pBatChar = pService->createCharacteristic(BAT_CHAR_UUID, BLECharacteristic::PROPERTY_NOTIFY);

#if defined(CONFIG_BLUEDROID_ENABLED)
  // Con NimBLE el descriptor 0x2902 se agrega solo; con Bluedroid hay que ponerlo a mano.
  pEcgChar->addDescriptor(new BLE2902());
  pImuChar->addDescriptor(new BLE2902());
  pBatChar->addDescriptor(new BLE2902());
#endif

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();

  Serial.printf("  Publicitando como \"%s\". Abre la app y toca Conectar.\n", BLE_NAME);
}

void enviarECG() {
  if (!deviceConnected) return;

  uint8_t pkt[4 + 4 * SAMPLES_PER_PKT];
  pkt[0] = (uint8_t)(seqPkt & 0xFF);
  pkt[1] = (uint8_t)(seqPkt >> 8);
  seqPkt++;
  pkt[2] = (digitalRead(PIN_LO_P) ? 1 : 0) | (digitalRead(PIN_LO_N) ? 2 : 0);
  pkt[3] = (uint8_t)(1000 / SAMPLE_HZ);
  for (int i = 0; i < SAMPLES_PER_PKT; i++) {
    putI16(pkt + 4 + 4 * i, bufA0[i]);
    putI16(pkt + 6 + 4 * i, bufA1[i]);
  }
  pEcgChar->setValue(pkt, sizeof(pkt));
  pEcgChar->notify();
  paquetesSeg++;
}

void muestrear() {
  int16_t a0 = 0, a1 = 0;
  if (adsIniciado) {  // si el ADS no respondio al arranque, no se lo consulta (se colgaria esperando)
    a0 = ads.readADC_SingleEnded(0);
    a1 = ads.readADC_SingleEnded(1);
  }
  ultV0 = a0 * ADS_LSB_V;
  ultV1 = a1 * ADS_LSB_V;
  muestrasSeg++;

  bufA0[nBuf] = a0;
  bufA1[nBuf] = a1;
  nBuf++;
  if (nBuf >= SAMPLES_PER_PKT) {
    enviarECG();
    nBuf = 0;
  }
}

void procesarIMU() {
  if (!mpuAddr) return;
  uint8_t d[14];
  if (!mpuLeer(0x3B, d, 14)) return;

  int16_t v[7];  // ax, ay, az, temp, gx, gy, gz
  for (int i = 0; i < 7; i++) v[i] = (int16_t)((d[2 * i] << 8) | d[2 * i + 1]);
  ultG = sqrt((float)v[0] * v[0] + (float)v[1] * v[1] + (float)v[2] * v[2]) / 16384.0;

  if (deviceConnected) {
    uint8_t pkt[14];
    for (int i = 0; i < 7; i++) putI16(pkt + 2 * i, v[i]);
    pImuChar->setValue(pkt, sizeof(pkt));
    pImuChar->notify();
  }
}

void enviarBateria() {
  if (!deviceConnected) return;
  uint16_t mv = (uint16_t)(batV * 1000.0f + 0.5f);
  uint8_t pkt[4];
  pkt[0] = (uint8_t)(mv & 0xFF);
  pkt[1] = (uint8_t)(mv >> 8);
  pkt[2] = (uint8_t)(batPct < 0 ? 255 : batPct);
  pkt[3] = batPresente ? 1 : 0;
  pBatChar->setValue(pkt, sizeof(pkt));
  pBatChar->notify();
}

void imprimirEstado() {
  Serial.printf("[%s] %lu muestras/s | A0=%.3f V | A1=%.3f V | LO+=%d LO-=%d | %.2f g | pkts/s=%lu | atrasos=%lu | bat=%.2f V\n",
                deviceConnected ? "BLE conectado" : "BLE esperando",
                (unsigned long)muestrasSeg, ultV0, ultV1,
                digitalRead(PIN_LO_P), digitalRead(PIN_LO_N), ultG,
                (unsigned long)paquetesSeg, (unsigned long)atrasos, batV);
  if (muestrasSeg < SAMPLE_HZ * 0.9) {
    Serial.println("  Aviso: no llega a la frecuencia pedida. Baja SAMPLE_HZ o revisa el cableado I2C.");
  }
  muestrasSeg = 0;
  paquetesSeg = 0;
}

void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 4000) delay(10);
  delay(500);

  Serial.println();
  Serial.println("*** TEST DE CIRCUITO + BLE - XIAO ESP32-C6 ***");

  pinMode(PIN_LO_P, INPUT);
  pinMode(PIN_LO_N, INPUT);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);

  escanerI2C();
  testMPU();
  testADS();
  testSD();

  titulo("5) AD8232 LEADS-OFF");
  leerLeadsOff();
  Serial.println("  Con electrodos desconectados deberian dar 1; conectados al cuerpo, 0.");

  titulo("6) BATERIA");
  pinMode(PIN_BAT, INPUT);
  actualizarBateria();
  if (batPresente) {
    Serial.printf("  Bateria: %.2f V (~%d %%)\n", batV, batPct);
    Serial.println("  Con el USB conectado y cargando marca cerca de 4.2 V; el nivel real se ve sin USB.");
  } else {
    Serial.printf("  Sin lectura valida (%.2f V). Revisar que la bateria este conectada y el divisor:\n", batV);
    Serial.println("  2 resistencias iguales en serie entre BAT+ y BAT-, punto medio a D0.");
  }

  titulo("RESUMEN");
  resultado("MPU6050", okMPU);
  resultado("ADS1115", okADS);
  resultado("SD", okSD);
  resultado("Bateria", batPresente);

  // Para transmitir: I2C a 400 kHz y ADS1115 a 860 SPS (si no, no alcanza para 100 muestras/s)
  Wire.setClock(400000);
  if (adsIniciado) ads.setDataRate(RATE_ADS1115_860SPS);

  iniciarBLE();

  nextSampleUs = micros();
  lastImuMs = millis();
  lastPrintMs = millis();
}

void loop() {
  // Muestreo con horario fijo (no acumula error)
  uint32_t nowUs = micros();
  if ((int32_t)(nowUs - nextSampleUs) >= 0) {
    nextSampleUs += SAMPLE_PERIOD_US;
    if ((int32_t)(nowUs - nextSampleUs) >= 0) {  // se atraso mas de un periodo
      nextSampleUs = nowUs + SAMPLE_PERIOD_US;
      atrasos++;
    }
    muestrear();
  }

  uint32_t nowMs = millis();
  if (nowMs - lastImuMs >= IMU_PERIOD_MS) {
    lastImuMs = nowMs;
    procesarIMU();
  }
  if (nowMs - lastBatMs >= BAT_PERIOD_MS) {
    lastBatMs = nowMs;
    actualizarBateria();
    enviarBateria();
  }
  if (nowMs - lastPrintMs >= 1000) {
    lastPrintMs = nowMs;
    imprimirEstado();
  }

  // Conexion / desconexion del celular
  if (!deviceConnected && oldDeviceConnected) {
    delay(300);                   // darle tiempo al stack antes de volver a publicitar
    pServer->startAdvertising();
    Serial.println("BLE: celular desconectado, publicitando de nuevo");
    oldDeviceConnected = deviceConnected;
  }
  if (deviceConnected && !oldDeviceConnected) {
    Serial.println("BLE: celular conectado");
    nBuf = 0;
    lastBatMs = millis() - BAT_PERIOD_MS + 800;  // primer dato de bateria enseguida
    oldDeviceConnected = deviceConnected;
  }

  delay(1);
}
