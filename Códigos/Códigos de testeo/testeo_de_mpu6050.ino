#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Usamos los GPIOs reales del XIAO ESP32-C6
const int PIN_SDA = 22; // Pin físico D4
const int PIN_SCL = 23; // Pin físico D5

Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("\n=== INICIANDO DIAGNÓSTICO DE HARDWARE I2C ===");

  // Activamos las resistencias pull-up internas como ayuda adicional
  pinMode(PIN_SDA, INPUT_PULLUP);
  pinMode(PIN_SCL, INPUT_PULLUP);

  // Reiniciar bus y forzar los pines D4 y D5
  Wire.end();
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000); // 100 kHz para máxima estabilidad
  delay(250);

  Serial.println("Escaneando bus I2C en D4 (GPIO 22) y D5 (GPIO 23)...");
  bool dispositivoEncontrado = false;
  
  for (byte i = 1; i < 127; i++) {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0) {
      Serial.print("-> ¡Hardware detectado en la dirección: 0x");
      Serial.println(i, HEX);
      dispositivoEncontrado = true;
    }
  }

  if (!dispositivoEncontrado) {
    Serial.println("\n[FALLO] No se detecta el MPU6050.");
    Serial.println("Si ya tienes puestas las resistencias externas y esto falla, el problema no es eléctrico.");
    while(1) delay(10); // Detener ejecución
  }

  Serial.println("\nIntentando inicializar la librería MPU6050...");
  if (!mpu.begin(0x68, &Wire)) {
    Serial.println("[ERROR] El dispositivo I2C respondió, pero falló la librería del MPU6050.");
    while(1) delay(10);
  }

  Serial.println("\n=== ¡ÉXITO! MPU6050 CONFIGURADO Y LEYENDO EN D4/D5 ===");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void loop() {
  sensors_event_t a, g, temp;
  
  if (mpu.getEvent(&a, &g, &temp)) {
    Serial.print("Aceleración Z: "); 
    Serial.print(a.acceleration.z, 2); 
    Serial.println(" m/s^2");
  } else {
    Serial.println("Fallo de lectura continua.");
  }
  
  delay(500);
}