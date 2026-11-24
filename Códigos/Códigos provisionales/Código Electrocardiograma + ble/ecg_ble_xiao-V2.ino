/*
 * ============================================================================
 *  ecg_ble_xiao.ino — ECG (SparkFun AD8232) + MPU6050 + SD + A0 por BLE
 *  Wearable pre-ictal · XIAO ESP32-C6 + ADS1115
 *  Version 3.2
 *   3.2: la FC daba de mas. Causa reproducida en simulacion: con una onda T
 *        grande (comun en el brazo, donde el QRS es chico) la T se contaba
 *        como latido, la mediana de los R-R se enganchaba en la FC falsa
 *        (100 lpm reales -> 163 informados) y el diagnostico igual decia
 *        "latidos regulares". Cambios:
 *        - regla de la onda T por PENDIENTE (como Pan-Tompkins): un pico que
 *          llega antes del 60 % del R-R mediano (minimo 360 ms) necesita al
 *          menos la mitad de la pendiente tipica del QRS; la T es mas lenta;
 *        - nivel de ruido = percentil 10 (antes 25) y factor 7 (antes 5):
 *          umbral mas exigente sin perder latidos a FC alta.
 *        No se volvio al detector de la v2.2: en la misma simulacion deja de
 *        detectar por completo por encima de ~85-90 lpm (su nivel de ruido
 *        incluye la energia del QRS), justo el rango de una crisis.
 *        Verificado en simulacion (55-180 lpm; QRS de 30-90 uV; T grande;
 *        EMG; movimiento): error de FC <= 1,3 lpm en todos los casos.
 *   3.1: detector de latidos rehecho (la FC salia mal). Fallas de la 3.0,
 *        reproducidas simulando este mismo codigo con ECG real (MIT-BIH) y
 *        sintetico:
 *        - con FC de ~140 lpm o mas dejaba de detectar: el nivel de ruido era
 *          el promedio de la senal integrada, que incluye la energia del QRS,
 *          y la condicion "8 veces el ruido" ya no se cumplia;
 *        - si los 2 s de aprendizaje no tenian QRS (electrodos recien puestos,
 *          salida en el riel), durante ~20 s marcaba latidos sobre el ruido cada
 *          250 ms e informaba 160-210 lpm con R-R "regulares".
 *        Ahora: nivel de ruido = percentil 25 de los ultimos 2 s; picos de la
 *        senal integrada clasificados con umbral de Pan-Tompkins, nivel de senal
 *        = mediana de los ultimos 8 latidos, regla de la onda T, reemplazo de una
 *        onda P tomada por latido, sin deteccion con la salida en el riel, notch
 *        de 50 Hz en la entrada del detector. Los R-R se miden entre las R (antes,
 *        entre los picos de la senal integrada, ~70 ms despues y con mas
 *        dispersion). Si el loop se atrasa mas de 48 ms, el contador de muestras
 *        avanza igual (antes los R-R se acortaban). El byte [3] del paquete de
 *        ECG ahora indica donde esta la R (ver BLE).
 *
 *  BASE: test_ad8232_biceps.ino v2.2 (el sketch que ya funciona). El
 *  procesamiento del ECG (muestreo a 250 Hz sobre AIN1, filtros HP 0,5 Hz +
 *  notch 50 Hz + LP 40 Hz, R-R, FC, RMSSD, amplitud del QRS, diagnostico,
 *  comandos por serie y perfiles de grafico) esta tomado de ese sketch; el
 *  detector de latidos se rehizo en la 3.1. Se agregaron:
 *    - transmision por BLE a la app del celular;
 *    - MPU6050 (I2C) y la salida del operacional en A0 del ADS1115;
 *    - test del circuito (I2C, MPU6050, ADS1115, SD, leads-off), al arrancar
 *      y con el comando 't'.
 *  Diferencias con el sketch base (todas minimas):
 *    - Pines de leads-off segun el cableado real (ver abajo).
 *    - ADS1115 en modo single-shot, no continuo: cada muestra de ECG dispara una
 *      conversion de AIN1 y la espera (~1,2 ms), y A0 (EDA) se lee igual. Con un
 *      solo conversor, pasar del modo continuo a otro canal no es confiable: el
 *      chip termina la conversion en curso y se apaga sin iniciar otra, asi que la
 *      "lectura de A0" devolvia el ultimo dato de AIN1 (ECG y EDA mostraban lo
 *      mismo). El single-shot es el mecanismo del firmware anterior, que si
 *      leia bien A0 y A1.
 *    - Cuando falta tiempo para la proxima muestra se cede la CPU con
 *      delay(1), para que el stack BLE trabaje (el base hacia espera activa).
 *    - Serial.setTxTimeoutMs(0): con el USB sin monitor serie abierto no se
 *      frena el muestreo.
 *
 *  CONEXIONES (numeracion XIAO ESP32-C6 -> GPIO)
 *    I2C (ADS1115 0x48, MPU6050 0x68/0x69): SDA = D9 (GPIO20), SCL = D10 (GPIO18)
 *    ADS1115: AIN1 = OUTPUT del AD8232 (ECG) | AIN0 = EDA (operacional, 1 a 3 V)
 *    Leads-off: LO+ = D1 (GPIO1), LO- = D0 (GPIO0)   (cableado real; el sketch
 *               base traia LO+ = D0 y LO- = D1, al reves)
 *    SD (SPI): CS = D2 (GPIO2), MOSI = D3 (GPIO21), CLK = D4 (GPIO22),
 *              MISO = D5 (GPIO23)
 *    Bateria: sin medicion por ahora.
 *
 *  BLE: el celular se conecta a "XIAO-ECG". Todo en little-endian.
 *    Servicio 5b1c0001-7a3e-4d8f-9c21-3f6a8e2b4d10
 *    ECG      ...0002 (notify, 20 bytes, 31,25 paquetes/s)
 *       [0-1]  numero de paquete (uint16)
 *       [2]    bit0 = LO+ (LA) en alto, bit1 = LO- (RA) en alto (en el paquete)
 *       [3]    0 = sin latido; k = 1..255: en este paquete se confirmo un latido
 *              y su R esta k-1 muestras antes de la ULTIMA muestra del paquete
 *              (el detector confirma el latido 100-250 ms despues de la R)
 *       [4-19] 8 muestras de ECG filtrado, int16 en decimas de mV (salida del
 *              modulo, 250 muestras/s)
 *    AUX      ...0003 (notify, 17 bytes, 10 por segundo)
 *       [0-13]  ax, ay, az, temp, gx, gy, gz (int16 crudos del MPU6050)
 *       [14-15] A0 del ADS1115 (int16 crudo, 0,125 mV por cuenta)
 *       [16]    bit0 = MPU6050 ok, bit1 = ADS1115 ok
 *    METRICAS ...0004 (notify, 14 bytes, 2 por segundo)
 *       [0-1]  FC en lpm (0 = sin dato)       [2] cantidad de R-R validos
 *       [3]    CV de los R-R en %             [4-5] RMSSD en ms
 *       [6-7]  QRS p-p referido a la entrada, en uV (0 = sin dato)
 *       [8]    codigo de diagnostico (ver codigoDiagnostico)
 *       [9]    bit0 = banda angosta, bit1 = notch activo
 *       [10-11] nivel DC medio de la salida del AD8232, en mV
 *       [12-13] atrasos de muestreo acumulados (veces que una muestra salio
 *               mas de 4 ms tarde; si crece seguido, la FC no es confiable)
 *  Conversion: g = crudo/16384 | grados/s = crudo/131 | temp = crudo/340+36,53
 *
 *  REQUISITOS
 *   - Core arduino-esp32 3.x, placa "XIAO_ESP32C6", USB CDC On Boot: Enabled.
 *   - Libreria Adafruit ADS1X15 (v2.x). BLE, SD, SPI y Wire vienen con el core.
 *
 *  COMANDOS POR SERIE (una letra, desde el Serial Monitor o la caja de texto
 *  del Serial Plotter)
 *   f  ECG filtrado en mV                                  [por defecto]
 *   c  crudo en V absolutos (nivel DC y saturacion)
 *   a  ambos: "solo pasa-altos" (con 50 Hz) y filtrado, en mV
 *   d  diagnostico en texto cada 2 s (usar el Serial Monitor)
 *   i  imprimir una vez las estadisticas de la ultima ventana
 *   w  banda angosta 7-24 Hz <-> banda ancha 0,5-40 Hz
 *   n  notch de 50 Hz on/off
 *   p  invertir polaridad del grafico
 *   l  lineas de escala fija on/off
 *   +  duplicar la escala fija      -  dividir la escala fija por 2
 *   b  marcadores de latido detectado on/off
 *   r  reiniciar filtros y detector
 *   s  escaneo I2C
 *   t  test del circuito (I2C, MPU6050, ADS1115, SD, leads-off). Usar el
 *      Serial Monitor: interrumpe el muestreo unos segundos.
 *   1/2/3  perfil de salida (Serial Plotter IDE 2 / IDE 1.8 / Teleplot)
 *   h  ayuda
 *
 *  Si se usa el Serial Plotter: poner TEST_AL_ARRANQUE en 0 (el test imprime
 *  texto con numeros que el plotter interpreta como series) y usar 't' a mano.
 * ============================================================================
 */

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_ADS1X15.h>
#include <math.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ----------------------------- PINES ----------------------------------------
#define PIN_SDA        20   // D9
#define PIN_SCL        18   // D10
// Leads-off del modulo (salidas digitales del AD8232, nivel 3,3 V). En ALTO, ese
// electrodo no tiene contacto DC. LO+ corresponde a LA (+IN), LO- a RA (-IN).
// Cableado real confirmado: LO+ en D1, LO- en D0. -1 = sin cablear.
#define PIN_LO_MAS     1    // D1 (GPIO1) -> LO+ del modulo
#define PIN_LO_MENOS   0    // D0 (GPIO0) -> LO- del modulo
// SDN es activo en BAJO (datasheet + esquematico v10, con pull-up de 10 kohm
// en el modulo). Cableado real: SDN en D6 (GPIO16). El codigo lo fuerza a ALTO
// al arrancar y lo mantiene asi: el AD8232 queda siempre encendido.
#define PIN_SDN        16   // D6 (GPIO16) -> SDN del modulo
// Lector de tarjeta SD (SPI)
#define PIN_SD_CS      2    // D2 (GPIO2)
#define PIN_SD_MOSI    21   // D3 (GPIO21)
#define PIN_SD_CLK     22   // D4 (GPIO22)
#define PIN_SD_MISO    23   // D5 (GPIO23)

// ----------------------------- ADS1115 --------------------------------------
#define ADS_DIRECCION  0x48
#define ADS_MUX_ECG    ADS1X15_REG_CONFIG_MUX_SINGLE_1   // AIN1 = OUTPUT AD8232
#define ADS_MUX_AUX    ADS1X15_REG_CONFIG_MUX_SINGLE_0   // AIN0 = operacional
#define VCC_V          3.3f

// ----------------------------- MPU6050 --------------------------------------
#define MPU_ADDR_1     0x68   // AD0 a GND
#define MPU_ADDR_2     0x69   // AD0 a VCC

// ----------------------------- BLE ------------------------------------------
#define BLE_NOMBRE       "XIAO-ECG"
#define SERVICE_UUID     "5b1c0001-7a3e-4d8f-9c21-3f6a8e2b4d10"
#define ECG_CHAR_UUID    "5b1c0002-7a3e-4d8f-9c21-3f6a8e2b4d10"
#define AUX_CHAR_UUID    "5b1c0003-7a3e-4d8f-9c21-3f6a8e2b4d10"
#define MET_CHAR_UUID    "5b1c0004-7a3e-4d8f-9c21-3f6a8e2b4d10"
#define BLE_MUESTRAS_PKT 8             // 250 / 8 = 31,25 paquetes/s
#define AUX_PERIODO_N    25            // cada 25 muestras = 10 Hz (MPU6050 y A0)
#define MET_PERIODO_N    125           // cada 125 muestras = 2 Hz

// ----------------------------- ARRANQUE -------------------------------------
// LEER_EDA 1 = ADS1115 en single-shot: ECG (A1) y EDA (A0) se leen por turnos.
// LEER_EDA 0 = ADS1115 en modo continuo SOLO sobre A1, exactamente como el sketch
//              test_ad8232_biceps.ino (la EDA queda sin medir). Sirve para descartar
//              que la forma de leer el ADS afecte al ECG.
#define LEER_EDA         1
#define TEST_AL_ARRANQUE 1             // 1 = hacer el test del circuito al encender

// ----------------------------- MUESTREO -------------------------------------
#define FS_HZ          250
#define PERIODO_US     (1000000UL / FS_HZ)               // 4000 us
#define ATRASO_MAX_US  (12UL * PERIODO_US)               // 48 ms
#define VENTANA_EST_S  2
#define N_VENTANA      (FS_HZ * VENTANA_EST_S)

// ----------------------------- FILTROS DE VISUALIZACION ---------------------
#define ANCHA_HP_HZ    0.5
#define ANCHA_LP_HZ    40.0
#define ANGOSTA_HP_HZ  7.0
#define ANGOSTA_LP_HZ  24.0
#define NOTCH_F0_HZ    50.0
#define NOTCH_Q        8.0
#define Q_BUTTER       0.70710678

// ----------------------------- DETECTOR DE LATIDOS --------------------------
// Pan-Tompkins: notch 50 Hz + pasa-banda 5-20 Hz, cuadrado e integracion de
// 80 ms. Se buscan los PICOS de la senal integrada y cada pico se clasifica
// como latido o ruido. Es independiente de la polaridad y del filtro de
// visualizacion elegido. (v3.1: detector rehecho, ver el historial arriba.)
#define DET_HP_HZ        5.0
#define DET_LP_HZ        20.0
#define DET_N_INT        20          // 80 ms de integracion
#define DET_REFRACT_N    63          // ~250 ms -> FC maxima ~240 lpm
#define DET_PICO_MAX_N   38          // 150 ms maximo para cerrar un pico
#define DET_N_QRS        40          // 160 ms hacia atras para medir el QRS y ubicar la R
#define DET_T_N          90          // 360 ms: ventana de la regla de la onda T
#define DET_N_SPK        8           // nivel de senal = mediana de los ultimos 8 latidos
// Regla de la onda T (v3.2). Un pico que llega antes del DET_T_FRAC del R-R
// mediano (y como minimo DET_T_N) despues del latido anterior se acepta solo
// si su pendiente maxima (en la banda 5-20 Hz) supera DET_T_PEND veces la
// pendiente tipica de los QRS recientes: la onda T es mas lenta que el QRS
// aunque tenga mucha energia (Pan-Tompkins usa el mismo criterio).
#define DET_T_FRAC       0.60f
#define DET_T_PEND       0.50
// Nivel de ruido = percentil 10 de la senal integrada en los ultimos 2 s
// (64 valores, uno cada 8 muestras). Es el tramo mas quieto entre latidos: ni
// el QRS ni una onda T grande lo contaminan, aun a 160-200 lpm (v3.2; con el
// percentil 25, una T grande a FC alta lo subia y se perdian latidos).
#define DET_DIEZ_RUIDO   8
#define DET_N_RUIDO      64
#define RR_MIN_MS        270.0f
#define RR_MAX_MS        2000.0f
#define RR_TOL           0.30f       // +-30 % de la mediana: si no, irregular
#define N_RR             16
#define RR_EDAD_MAX_N    (10 * FS_HZ) // solo R-R de los ultimos 10 s
// Energia minima (mV^2, banda 5-20 Hz integrada) para aceptar un latido.
// Con entradas en corto (ruido del AD8232 de 2,6-3,4 mV RMS, medido en banco)
// la senal integrada promedia ~4 mV^2 y no pasa de ~28 mV^2 en 30 min de
// simulacion; un QRS de brazo de 30 uV p-p da ~60-100 y uno de 70 uV, ~500.
// 35 mV^2 equivale al criterio de la v2.2 (8 veces el ruido medio) medido en
// banco, pero sin depender del contenido de la senal.
#define DET_Y_MIN        35.0
// Ademas, el pico debe superar DET_K_RUIDO veces el nivel de ruido (rechaza
// ruido de 50 Hz, EMG o movimiento). v3.2: 7 (la 3.1 usaba 5, la v2.2 usaba 8
// sobre la media). Con 8 se perdian latidos a 160 lpm con una T grande.
#define DET_K_RUIDO      7.0
// Salida pegada a un riel durante 80 ms seguidos (sin contacto, artefacto):
// no se detecta durante el riel ni 0,5 s despues. Una R que recorta dura menos.
#define DET_RIEL_BAJO_MV 120.0
#define DET_RIEL_ALTO_MV (VCC_V * 1000.0 - 120.0)
#define DET_RIEL_N       20
#define DET_INHIBIR_N    (FS_HZ / 2)
// RMS minimo esperable en la salida filtrada con el AD8232 amplificando
// (su ruido propio solo ya da ~1,5-2,5 mV RMS). Por debajo, la etapa de
// entrada esta saturada o bloqueada y lo que se ve es el ruido del ADC.
#define RUIDO_MIN_MV     0.5f

// ----------------------------- SALIDA ---------------------------------------
#define PERFIL_INICIAL     1         // 1 = IDE 2.x, 2 = IDE 1.8.x, 3 = Teleplot
#define DIEZ_IDE2          12        // 250/12 = 20,8 puntos/s -> 50 pts = 2,4 s
#define DIEZ_IDE1          2         // 125 puntos/s -> 500 pts = 4 s
#define ESCALA_INICIAL_MV  60.0f     // lineas fijas en +-60 mV
#define SERIAL_BAUD        115200    // irrelevante con USB nativo

// Ganancia del modulo SparkFun: 100 (IA) x 11 (op-amp, R9/R8 = 1M/100k).
#define GANANCIA_MODULO    1100.0f

// ============================================================================
//  FILTRO BIQUAD (forma directa II transpuesta, coeficientes RBJ)
//  double: el ESP32-C6 no tiene FPU, asi que float y double cuestan parecido,
//  y el pasa-altos de 0,5 Hz tiene polos muy cerca de 1.
// ============================================================================
struct Biquad {
  double b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0;
  double z1 = 0, z2 = 0;

  double procesar(double x) {
    double y = b0 * x + z1;
    z1 = b1 * x - a1 * y + z2;
    z2 = b2 * x - a2 * y;
    return y;
  }
  void reset() { z1 = z2 = 0; }

  // Estado de regimen permanente para una entrada constante x (evita el
  // transitorio de arranque del pasa-altos). Devuelve la salida.
  double estadoEstable(double x) {
    double y = x * (b0 + b1 + b2) / (1.0 + a1 + a2);
    z1 = y - b0 * x;
    z2 = b2 * x - a2 * y;
    return y;
  }
};

enum TipoFiltro { PASA_BAJOS, PASA_ALTOS, RECHAZA_BANDA };


// ============================================================================
//  ESTADO GLOBAL
// ============================================================================
Adafruit_ADS1115 ads;
bool adsOk = false;
uint16_t adsFallasSeguidas = 0;   // lecturas del ADS que no terminaron, seguidas
uint32_t erroresADS = 0;          // lecturas del ADS que no terminaron, en total
int16_t  ultimaCuentaECG = 0;     // ultima muestra valida de AIN1

Biquad fHP, fNotch, fLP;
bool notchActivo   = true;
bool bandaAngosta  = false;
bool filtrosCebados = false;

enum Modo { MODO_FILTRADO, MODO_CRUDO, MODO_AMBOS, MODO_DIAG };
Modo  modo       = MODO_FILTRADO;
int   perfil     = PERFIL_INICIAL;
float polaridad  = 1.0f;
bool  limites    = true;
float escalaMv   = ESCALA_INICIAL_MV;
bool  marcadores = false;

uint32_t proximoUs = 0;
uint32_t nMuestra = 0;
uint32_t atrasos = 0;              // veces que una muestra salio mas de 4 ms tarde
bool     enAtraso = false;
uint32_t ultimoReintento = 0;

// Acumuladores del diezmado (perfiles 1 y 2)
float  acF = 0, acH = 0;
double acV = 0;
bool   acLat = false, acLo = false;
int    acN = 0;

// Estadisticas por ventana de 2 s
struct Estadisticas {
  uint32_t n;
  float crudoMin, crudoMax;  double crudoSuma;   // V
  float filtMin, filtMax;    double filtCuad;    // mV
  double hpCuad;                                 // mV^2
  uint32_t loCuenta;        // muestras con LO+ o LO- en alto
  uint32_t loMasCuenta;     // LO+ (LA)
  uint32_t loMenosCuenta;   // LO- (RA)
  uint32_t nLat;            // latidos detectados en la ventana
  double   qrsPPSuma;       // suma de amplitudes p-p del QRS (mV)
  uint32_t nIrreg;          // R-R descartados por irregulares
};
Estadisticas est, ultimaEst;
bool hayEst = false;

// Detector de latidos
struct Detector {
  Biquad notch, hp, lp;
  double cuad[DET_N_INT];
  int    idx;
  double suma;
  uint32_t n;                 // muestras desde el reinicio
  float  yHist[DET_N_RUIDO];  // senal integrada, 1 de cada DET_DIEZ_RUIDO muestras
  int    iHist, nHist;
  double ruido;               // percentil 10 de yHist
  float  spkHist[DET_N_SPK];  // pico integrado de los ultimos latidos
  int    iSpk, nSpk;
  double spk;                 // nivel de senal (mediana de spkHist)
  bool   subiendo;            // siguiendo un pico de la senal integrada
  double pkVal, yPrev;
  uint32_t pkN;
  uint32_t nRiel;             // muestras seguidas en un riel
  uint32_t inhibir;           // muestras que faltan sin detectar
  float  polaridad;           // >0: la deflexion mayor del QRS es positiva
  bool   hayUltimo;
  uint32_t ultimoN;           // muestra de la R del ultimo latido
  uint32_t ultimoPkN;         // muestra del pico integrado del ultimo latido
  double ultimoPico;          // valor de ese pico (regla de la onda T)
  float  slpHist[DET_N_SPK];  // pendiente maxima de los ultimos latidos
  double slpRef;              // mediana de slpHist
  float  dBuf[128];           // |pendiente| de la senal 5-20 Hz, ultimas 128 muestras
  double bPrev;
  uint32_t nRef;              // ultimo latido
  float  rrCrudo[N_RR];       // ms
  bool   rrOk[N_RR];
  uint32_t rrN[N_RR];         // muestra en la que termina cada R-R
  int    iRR, nRR;
};
Detector det;
// Copia de lo que cambia registrarLatido(), para poder deshacer el ultimo
// latido si dentro del periodo refractario aparece el QRS verdadero.
struct CopiaLatido {
  bool valida;
  float spkHist[DET_N_SPK]; int iSpk, nSpk; double spk;
  float slpHist[DET_N_SPK]; double slpRef;
  float polaridad;
  bool hayUltimo; uint32_t ultimoN, ultimoPkN, nRef; double ultimoPico;
  float rrCrudo; bool rrOk; uint32_t rrN; int iRR, nRR;   // casillero que se piso
  uint32_t nIrreg; float qrsPP;
};
CopiaLatido copiaLat;
float bufFilt[128];           // ultimas 128 muestras filtradas (para el QRS)
float fcInstantanea = 0;      // para Teleplot
bool  fcNueva = false;

// Resultados del test del circuito
bool okMPU = false, okSD = false, okADS = false;

// MPU6050
uint8_t mpuAddr = 0;
int16_t imuRaw[7];            // ax, ay, az, temp, gx, gy, gz
bool    imuValido = false;
int16_t a0Counts = 0;         // ultima lectura de A0 (0,125 mV por cuenta)

// BLE
BLEServer *pServer = nullptr;
BLECharacteristic *pEcgChar = nullptr;
BLECharacteristic *pAuxChar = nullptr;
BLECharacteristic *pMetChar = nullptr;
volatile bool deviceConnected = false;
bool oldDeviceConnected = false;
bool reanunciar = false;
uint32_t reanunciarMs = 0;
uint16_t seqPkt = 0;
int16_t  bleBuf[BLE_MUESTRAS_PKT];
uint8_t  bleN = 0, bleLo = 0, bleBeat = 0;
uint32_t bleBeatN = 0;        // muestra de la R del latido confirmado en este paquete

// ============================================================================
//  DISEÑO DE FILTROS
//  IMPORTANTE: ninguna funcion puede definirse antes de los structs/enums.
//  El IDE de Arduino inserta prototipos automaticos antes de la primera
//  funcion del archivo; si un tipo se define despues, no compila.
// ============================================================================
void disenarBiquad(Biquad &f, TipoFiltro tipo, double fc, double q, double fs) {
  double w0 = 2.0 * M_PI * fc / fs;
  double cw = cos(w0);
  double alpha = sin(w0) / (2.0 * q);
  double a0 = 1.0 + alpha;
  double b0, b1, b2;
  switch (tipo) {
    case PASA_BAJOS: b0 = (1.0 - cw) / 2.0; b1 = 1.0 - cw;     b2 = b0;  break;
    case PASA_ALTOS: b0 = (1.0 + cw) / 2.0; b1 = -(1.0 + cw);  b2 = b0;  break;
    default:         b0 = 1.0;              b1 = -2.0 * cw;    b2 = 1.0; break;
  }
  f.b0 = b0 / a0;  f.b1 = b1 / a0;  f.b2 = b2 / a0;
  f.a1 = (-2.0 * cw) / a0;
  f.a2 = (1.0 - alpha) / a0;
  f.reset();
}

bool hayPinesLO() { return (PIN_LO_MAS >= 0) || (PIN_LO_MENOS >= 0); }

void resetEst(Estadisticas &e) {
  e.n = 0;
  e.crudoMin = 1e9f;  e.crudoMax = -1e9f;  e.crudoSuma = 0;
  e.filtMin  = 1e9f;  e.filtMax  = -1e9f;  e.filtCuad  = 0;
  e.hpCuad = 0;
  e.loCuenta = 0; e.loMasCuenta = 0; e.loMenosCuenta = 0;
  e.nLat = 0; e.qrsPPSuma = 0; e.nIrreg = 0;
}

void resetAcumuladores() {
  acF = 0; acH = 0; acV = 0; acLat = false; acLo = false; acN = 0;
}

// Mensajes de estado: solo cuando no rompen el grafico. El Serial Plotter de
// Arduino interpreta numeros sueltos como series nuevas; Teleplot ignora las
// lineas que no empiezan con '>'.
bool textoPermitido() { return (modo == MODO_DIAG) || (perfil == 3); }

// ============================================================================
//  ADS1115
// ============================================================================
bool iniciarADS() {
  if (!ads.begin(ADS_DIRECCION, &Wire)) return false;
  // +-4,096 V -> 125 uV/LSB. Con ~77 mV p-p de QRS esperado en la salida
  // quedan ~600 cuentas de senal: resolucion de sobra.
  ads.setGain(GAIN_ONE);
  ads.setDataRate(RATE_ADS1115_860SPS);
#if !LEER_EDA
  ads.startADCReading(ADS_MUX_ECG, /*continuous=*/true);   // igual que test_ad8232_biceps
  delay(5);
#endif
  return true;
}

// Una lectura single-shot de un canal (AIN1 = ECG, AIN0 = EDA). Siempre se
// inicia con el chip en reposo (la conversion anterior termino y el chip se
// apago): es el unico caso en que escribir OS=1 inicia una conversion. A 860 SPS
// la conversion dura 1,16 ms (+-10 %): se espera 1 ms y despues se consulta el
// bit OS. Devuelve false si no termino en 4 ms.
bool adsLeer(uint16_t mux, int16_t &cuentas) {
  ads.startADCReading(mux, /*continuous=*/false);
  uint32_t t0 = micros();
  while ((uint32_t)(micros() - t0) < 1000) { ; }
  while (!ads.conversionComplete()) {
    if ((uint32_t)(micros() - t0) > 4000) return false;
  }
  cuentas = ads.getLastConversionResults();
  return true;
}

// EDA (A0). Si la lectura falla se conserva el valor anterior.
int16_t leerA0Counts() {
  int16_t c = 0;
  if (adsLeer(ADS_MUX_AUX, c)) a0Counts = c; else erroresADS++;
  return a0Counts;
}

// Devuelve una mascara: bit 0 = LO+ (LA) en alto, bit 1 = LO- (RA) en alto.
uint8_t leerLeadsOff() {
  uint8_t m = 0;
  if (PIN_LO_MAS   >= 0 && digitalRead(PIN_LO_MAS)   == HIGH) m |= 0x01;
  if (PIN_LO_MENOS >= 0 && digitalRead(PIN_LO_MENOS) == HIGH) m |= 0x02;
  return m;
}

// ============================================================================
//  FILTROS
// ============================================================================
void configurarFiltros() {
  double hp = bandaAngosta ? ANGOSTA_HP_HZ : ANCHA_HP_HZ;
  double lp = bandaAngosta ? ANGOSTA_LP_HZ : ANCHA_LP_HZ;
  disenarBiquad(fHP,    PASA_ALTOS,    hp,          Q_BUTTER, FS_HZ);
  disenarBiquad(fNotch, RECHAZA_BANDA, NOTCH_F0_HZ, NOTCH_Q,  FS_HZ);
  disenarBiquad(fLP,    PASA_BAJOS,    lp,          Q_BUTTER, FS_HZ);
  disenarBiquad(det.notch, RECHAZA_BANDA, NOTCH_F0_HZ, NOTCH_Q, FS_HZ);
  disenarBiquad(det.hp, PASA_ALTOS,    DET_HP_HZ,   Q_BUTTER, FS_HZ);
  disenarBiquad(det.lp, PASA_BAJOS,    DET_LP_HZ,   Q_BUTTER, FS_HZ);
  filtrosCebados = false;   // se ceban con la proxima muestra
}

void cebarFiltros(double x0_mV) {
  double y = fHP.estadoEstable(x0_mV);
  y = fNotch.estadoEstable(y);
  fLP.estadoEstable(y);
  double d = det.notch.estadoEstable(x0_mV);
  d = det.hp.estadoEstable(d);
  det.lp.estadoEstable(d);
}

// ============================================================================
//  DETECTOR DE LATIDOS
// ============================================================================
void resetDetector() {
  for (int i = 0; i < DET_N_INT; i++) det.cuad[i] = 0;
  det.idx = 0;  det.suma = 0;  det.n = 0;
  for (int i = 0; i < DET_N_RUIDO; i++) det.yHist[i] = 0;
  det.iHist = 0;  det.nHist = 0;  det.ruido = 0;
  for (int i = 0; i < DET_N_SPK; i++) det.spkHist[i] = 0;
  det.iSpk = 0;  det.nSpk = 0;  det.spk = 0;
  det.subiendo = false;  det.pkVal = 0;  det.yPrev = 0;  det.pkN = 0;
  det.nRiel = 0;  det.inhibir = 0;  det.polaridad = 0;
  det.hayUltimo = false;  det.ultimoN = 0;  det.ultimoPkN = 0;  det.ultimoPico = 0;
  det.nRef = 0;
  for (int i = 0; i < DET_N_SPK; i++) det.slpHist[i] = 0;
  for (int i = 0; i < 128; i++) det.dBuf[i] = 0;
  det.slpRef = 0;  det.bPrev = 0;
  copiaLat.valida = false;
  det.iRR = 0;  det.nRR = 0;
  for (int i = 0; i < N_RR; i++) { det.rrCrudo[i] = 0; det.rrOk[i] = false; det.rrN[i] = 0; }
}

// k-esimo menor valor de v[0..n-1] (k desde 0). Quickselect sobre una copia.
float kEsimo(const float *v, int n, int k) {
  float t[DET_N_RUIDO > DET_N_SPK ? DET_N_RUIDO : DET_N_SPK];
  for (int i = 0; i < n; i++) t[i] = v[i];
  int lo = 0, hi = n - 1;
  while (lo < hi) {
    float piv = t[(lo + hi) / 2];
    int i = lo, j = hi;
    while (i <= j) {
      while (t[i] < piv) i++;
      while (t[j] > piv) j--;
      if (i <= j) { float s = t[i]; t[i] = t[j]; t[j] = s; i++; j--; }
    }
    if (k <= j) hi = j;
    else if (k >= i) lo = i;
    else break;
  }
  return t[k];
}

// Ordenamiento por insercion (N <= 16)
void ordenar(float *v, int n) {
  for (int i = 1; i < n; i++) {
    float x = v[i]; int j = i - 1;
    while (j >= 0 && v[j] > x) { v[j + 1] = v[j]; j--; }
    v[j + 1] = x;
  }
}

// Mediana de los R-R recientes dentro del rango fisiologico (validos o no:
// asi se adapta si la FC cambia de golpe).
float medianaRRCrudos() {
  float tmp[N_RR]; int k = 0;
  for (int i = 0; i < det.nRR; i++) {
    float r = det.rrCrudo[i];
    if (r >= RR_MIN_MS && r <= RR_MAX_MS) tmp[k++] = r;
  }
  if (k == 0) return 0;
  ordenar(tmp, k);
  return (k & 1) ? tmp[k / 2] : 0.5f * (tmp[k / 2 - 1] + tmp[k / 2]);
}

void agregarRR(float rr, uint32_t nFin) {
  bool ok = (rr >= RR_MIN_MS && rr <= RR_MAX_MS);
  if (ok) {
    float med = medianaRRCrudos();
    if (det.nRR >= 3 && med > 0 && fabsf(rr - med) > RR_TOL * med) {
      ok = false;
      est.nIrreg++;
    }
  }
  det.rrCrudo[det.iRR] = rr;
  det.rrOk[det.iRR] = ok;
  det.rrN[det.iRR] = nFin;
  det.iRR = (det.iRR + 1) % N_RR;
  if (det.nRR < N_RR) det.nRR++;
  if (ok) { fcInstantanea = 60000.0f / rr; fcNueva = true; }
}

// FC (mediana), coeficiente de variacion y RMSSD sobre los R-R validos de los
// ultimos 10 s (los mas viejos se ignoran: no se reportan latidos "viejos").
// Devuelve la cantidad de R-R validos usados.
int metricasRR(float &fc, float &cv, float &rmssd, int &nPares) {
  float val[N_RR]; int k = 0;
  double suma = 0, suma2 = 0, sumDif2 = 0;
  nPares = 0;
  float prev = 0; bool prevOk = false;
  int inicio = (det.nRR < N_RR) ? 0 : det.iRR;       // el mas antiguo
  for (int j = 0; j < det.nRR; j++) {
    int i = (inicio + j) % N_RR;
    bool reciente = (nMuestra - det.rrN[i]) <= RR_EDAD_MAX_N;
    if (det.rrOk[i] && reciente) {
      float r = det.rrCrudo[i];
      val[k++] = r;  suma += r;  suma2 += (double)r * r;
      if (prevOk) { double d = r - prev; sumDif2 += d * d; nPares++; }
      prev = r;  prevOk = true;
    } else {
      prevOk = false;                                 // corta la secuencia
    }
  }
  fc = 0; cv = 0; rmssd = 0;
  if (k == 0) return 0;
  ordenar(val, k);
  float med = (k & 1) ? val[k / 2] : 0.5f * (val[k / 2 - 1] + val[k / 2]);
  fc = 60000.0f / med;
  double media = suma / k;
  double var = suma2 / k - media * media;
  cv = (var > 0) ? (float)(sqrt(var) / media) : 0;
  rmssd = nPares ? (float)sqrt(sumDif2 / nPares) : 0;
  return k;
}

void registrarLatido(double pk, uint32_t pkN, double slp) {
  CopiaLatido &c = copiaLat;
  c.valida = true;
  for (int i = 0; i < DET_N_SPK; i++) { c.spkHist[i] = det.spkHist[i]; c.slpHist[i] = det.slpHist[i]; }
  c.slpRef = det.slpRef;
  c.iSpk = det.iSpk;  c.nSpk = det.nSpk;  c.spk = det.spk;  c.polaridad = det.polaridad;
  c.hayUltimo = det.hayUltimo;  c.ultimoN = det.ultimoN;  c.ultimoPkN = det.ultimoPkN;
  c.nRef = det.nRef;  c.ultimoPico = det.ultimoPico;
  c.rrCrudo = det.rrCrudo[det.iRR];  c.rrOk = det.rrOk[det.iRR];  c.rrN = det.rrN[det.iRR];
  c.iRR = det.iRR;  c.nRR = det.nRR;
  c.nIrreg = est.nIrreg;

  // Nivel de senal: mediana de los ultimos DET_N_SPK latidos. Un artefacto
  // aislado (miles de veces la energia de un QRS) no la mueve, y si la
  // amplitud real cambia, la mediana la sigue en 4-5 latidos.
  det.spkHist[det.iSpk] = (float)pk;
  det.iSpk = (det.iSpk + 1) % DET_N_SPK;
  if (det.nSpk < DET_N_SPK) det.nSpk++;
  det.spk = (det.nSpk & 1) ? kEsimo(det.spkHist, det.nSpk, det.nSpk / 2)
          : 0.5 * (kEsimo(det.spkHist, det.nSpk, det.nSpk / 2 - 1) +
                   kEsimo(det.spkHist, det.nSpk, det.nSpk / 2));
  det.ultimoPico = pk;
  // Pendiente tipica del QRS: mediana de los ultimos latidos (usa el mismo
  // casillero que spkHist; iSpk ya avanzo).
  det.slpHist[(det.iSpk + DET_N_SPK - 1) % DET_N_SPK] = (float)slp;
  det.slpRef = (det.nSpk & 1) ? kEsimo(det.slpHist, det.nSpk, det.nSpk / 2)
             : 0.5 * (kEsimo(det.slpHist, det.nSpk, det.nSpk / 2 - 1) +
                      kEsimo(det.slpHist, det.nSpk, det.nSpk / 2));

  // Ventana de 160 ms que termina en el pico de la senal integrada (que llega
  // ~50-80 ms despues de la R): amplitud p-p del QRS en la senal mostrada y
  // ubicacion de la R = deflexion mayor, con la polaridad que domina en los
  // latidos recientes (asi no salta entre R y S de un latido a otro).
  float mx = -1e9f, mn = 1e9f, suma = 0;
  uint32_t nMx = pkN, nMn = pkN;
  for (int k = 0; k <= DET_N_QRS; k++) {
    uint32_t i = pkN - k;
    float a = bufFilt[i & 127];
    suma += a;
    if (a > mx) { mx = a; nMx = i; }
    if (a < mn) { mn = a; nMn = i; }
  }
  float media = suma / (DET_N_QRS + 1);
  det.polaridad = 0.8f * det.polaridad + 0.2f * ((mx - media >= media - mn) ? 1.0f : -1.0f);
  uint32_t rN = (det.polaridad >= 0) ? nMx : nMn;

  est.nLat++;
  est.qrsPPSuma += (mx - mn);
  c.qrsPP = mx - mn;

  if (det.hayUltimo) agregarRR((float)(rN - det.ultimoN) * 1000.0f / FS_HZ, rN);
  det.hayUltimo = true;
  det.ultimoN = rN;
  det.ultimoPkN = pkN;
  det.nRef = nMuestra;
}

// Deshace el ultimo latido registrado (nivel de senal, R-R y estadisticas).
void deshacerLatido() {
  CopiaLatido &c = copiaLat;
  if (!c.valida) return;
  c.valida = false;
  for (int i = 0; i < DET_N_SPK; i++) { det.spkHist[i] = c.spkHist[i]; det.slpHist[i] = c.slpHist[i]; }
  det.slpRef = c.slpRef;
  det.iSpk = c.iSpk;  det.nSpk = c.nSpk;  det.spk = c.spk;  det.polaridad = c.polaridad;
  det.hayUltimo = c.hayUltimo;  det.ultimoN = c.ultimoN;  det.ultimoPkN = c.ultimoPkN;
  det.nRef = c.nRef;  det.ultimoPico = c.ultimoPico;
  det.rrCrudo[c.iRR] = c.rrCrudo;  det.rrOk[c.iRR] = c.rrOk;  det.rrN[c.iRR] = c.rrN;
  det.iRR = c.iRR;  det.nRR = c.nRR;
  if (est.nIrreg > c.nIrreg) est.nIrreg = c.nIrreg;   // (si la ventana no cambio)
  if (est.nLat > 0) {
    est.nLat--;
    est.qrsPPSuma -= c.qrsPP;
    if (est.qrsPPSuma < 0) est.qrsPPSuma = 0;
  }
}

// Clasifica un pico de la senal integrada. Devuelve true si es un latido.
bool evaluarPico(double pk, uint32_t pkN) {
  // Pendiente maxima del candidato: 160 ms que terminan en su pico integrado
  double slp = 0;
  for (int k = 0; k <= DET_N_QRS; k++) {
    float d = det.dBuf[(pkN - k) & 127];
    if (d > slp) slp = d;
  }
  // Por encima del ruido, en terminos absolutos y relativos
  if (pk < DET_Y_MIN || pk < DET_K_RUIDO * det.ruido) return false;
  // Periodo refractario desde el pico del latido anterior. Excepcion: un pico
  // de mas del doble de energia es el QRS verdadero y el "latido" anterior era
  // una onda P o ruido (pasa al arrancar, cuando el nivel de senal todavia no
  // se conoce): se reemplaza. Sin esto, la P quedaba como latido y el QRS caia
  // siempre en el refractario.
  if (det.hayUltimo && (pkN - det.ultimoPkN) <= DET_REFRACT_N) {
    if (pk > 2.0 * det.ultimoPico && copiaLat.valida) {
      deshacerLatido();
      registrarLatido(pk, pkN, slp);
      return true;
    }
    return false;
  }
  // Umbral adaptativo de Pan-Tompkins (si ya hay latidos)
  if (det.nSpk > 0 && pk < det.ruido + 0.25 * (det.spk - det.ruido)) return false;
  // Onda T: candidato temprano (antes del 60 % del R-R mediano, minimo 360 ms)
  // con menos de la mitad de la energia del latido anterior o con menos de la
  // mitad de la pendiente tipica del QRS.
  if (det.hayUltimo) {
    uint32_t dist = pkN - det.ultimoPkN;
    float med = medianaRRCrudos();
    uint32_t ventT = DET_T_N;
    if (med > 0) {
      uint32_t v = (uint32_t)(DET_T_FRAC * med * FS_HZ / 1000.0f);
      if (v > ventT) ventT = v;
    }
    if (dist < ventT) {
      if (pk < 0.5 * det.ultimoPico) return false;
      if (det.nSpk >= 3 && slp < DET_T_PEND * det.slpRef) return false;
    }
  }
  registrarLatido(pk, pkN, slp);
  return true;
}

// Procesa una muestra. Devuelve true cuando se confirma un latido (unos
// 100-200 ms despues de la R); la R queda en det.ultimoN.
bool procesarDetector(double x_mV, float filt) {
  bufFilt[nMuestra & 127] = filt;

  double b = det.lp.procesar(det.hp.procesar(det.notch.procesar(x_mV)));
  det.dBuf[nMuestra & 127] = (float)fabs(b - det.bPrev);
  det.bPrev = b;
  double c = b * b;
  det.suma += c - det.cuad[det.idx];
  det.cuad[det.idx] = c;
  det.idx = (det.idx + 1) % DET_N_INT;
  if (det.suma < 0) det.suma = 0;
  double y = det.suma / DET_N_INT;
  det.n++;

  // Salida en un riel: no detectar durante el riel ni 0,5 s despues (el borde
  // al salir del riel tiene muchisima energia en la banda del QRS).
  bool riel = (x_mV < DET_RIEL_BAJO_MV) || (x_mV > DET_RIEL_ALTO_MV);
  det.nRiel = riel ? det.nRiel + 1 : 0;
  if (det.nRiel >= DET_RIEL_N) det.inhibir = DET_INHIBIR_N;
  if (det.inhibir > 0) {
    det.inhibir--;
    det.subiendo = false;
    det.yPrev = y;
    return false;
  }

  // Nivel de ruido: percentil 10 de los ultimos 2 s
  if ((det.n % DET_DIEZ_RUIDO) == 0) {
    det.yHist[det.iHist] = (float)y;
    det.iHist = (det.iHist + 1) % DET_N_RUIDO;
    if (det.nHist < DET_N_RUIDO) det.nHist++;
    det.ruido = kEsimo(det.yHist, det.nHist, det.nHist / 10);
  }
  if (det.nHist < DET_N_RUIDO) { det.yPrev = y; return false; }   // 2 s de aprendizaje

  // Picos de la senal integrada: se sigue el maximo y el pico se cierra cuando
  // la senal cae a la mitad o pasan 150 ms sin superarlo.
  bool latido = false;
  if (!det.subiendo) {
    if (y > det.yPrev) { det.subiendo = true; det.pkVal = y; det.pkN = nMuestra; }
  } else if (y >= det.pkVal) {
    det.pkVal = y; det.pkN = nMuestra;
  } else if (y < 0.5 * det.pkVal || (nMuestra - det.pkN) > DET_PICO_MAX_N) {
    latido = evaluarPico(det.pkVal, det.pkN);
    det.subiendo = false;
  }

  // Sin latidos durante 2 s: el nivel de senal cae a la mitad por segundo,
  // para recuperar la deteccion si la amplitud bajo.
  if (det.nSpk > 0 && (nMuestra - det.nRef) > 2 * FS_HZ) {
    det.spk *= 0.99723;
    for (int i = 0; i < det.nSpk; i++) det.spkHist[i] *= 0.99723f;
  }
  det.yPrev = y;
  return latido;
}

// ============================================================================
//  SALIDA AL GRAFICADOR
// ============================================================================
void salida(float v, double hp, double filt, bool lo, bool marca) {
  if (modo == MODO_DIAG) return;
  float f = polaridad * (float)filt;
  float h = polaridad * (float)hp;

  // ---- Perfil 3: Teleplot, todas las muestras con tiempo derivado del
  //      contador (el eje X refleja el muestreo real aunque el USB agrupe).
  if (perfil == 3) {
    unsigned long t = (unsigned long)(((uint64_t)nMuestra * 1000ULL) / FS_HZ);
    switch (modo) {
      case MODO_CRUDO:    Serial.printf(">ecg_V:%lu:%.4f\n", t, v); break;
      case MODO_FILTRADO: Serial.printf(">ecg_mV:%lu:%.3f\n", t, f); break;
      case MODO_AMBOS:    Serial.printf(">solo_HP_mV:%lu:%.3f\n>ecg_mV:%lu:%.3f\n",
                                        t, h, t, f); break;
      default: break;
    }
    if (marcadores && marca) Serial.printf(">latido:%lu:1\n", t);
    if (fcNueva) { Serial.printf(">fc_lpm:%lu:%.1f\n", t, fcInstantanea); fcNueva = false; }
    if (hayPinesLO()) Serial.printf(">leads_off:%lu:%d\n", t, lo ? 1 : 0);
    return;
  }
  fcNueva = false;

  // ---- Perfiles 1 y 2: Serial Plotter de Arduino, con diezmado que conserva
  //      el valor de mayor modulo de cada bloque (los QRS no desaparecen).
  int diez = (perfil == 1) ? DIEZ_IDE2 : DIEZ_IDE1;
  if (acN == 0 || fabsf(f) > fabsf(acF)) acF = f;
  if (acN == 0 || fabsf(h) > fabsf(acH)) acH = h;
  acV += v;
  acLat |= marca;
  acLo  |= lo;
  acN++;
  if (acN < diez) return;

  char linea[160];
  int p = 0;
  switch (modo) {
    case MODO_CRUDO:
      p += snprintf(linea + p, sizeof(linea) - p, "ecg_V:%.4f", acV / acN);
      if (hayPinesLO())
        p += snprintf(linea + p, sizeof(linea) - p, ",leads_off:%.1f", acLo ? VCC_V : 0.0f);
      break;
    case MODO_FILTRADO:
      p += snprintf(linea + p, sizeof(linea) - p, "ecg_mV:%.2f", acF);
      break;
    case MODO_AMBOS:
      p += snprintf(linea + p, sizeof(linea) - p, "solo_HP_mV:%.2f,ecg_mV:%.2f", acH, acF);
      break;
    default: break;
  }
  if (modo != MODO_CRUDO) {
    if (limites)
      p += snprintf(linea + p, sizeof(linea) - p, ",lim_sup:%.0f,lim_inf:%.0f",
                    escalaMv, -escalaMv);
    if (marcadores)
      p += snprintf(linea + p, sizeof(linea) - p, ",latido:%.0f",
                    acLat ? 0.8f * escalaMv : 0.0f);
    if (hayPinesLO())
      p += snprintf(linea + p, sizeof(linea) - p, ",leads_off:%.0f",
                    acLo ? 0.9f * escalaMv : 0.0f);
  }
  Serial.println(linea);
  resetAcumuladores();
}

// ============================================================================
//  ESTADISTICAS Y DIAGNOSTICO (umbrales orientativos para banco)
// ============================================================================
void imprimirEstadisticas(const Estadisticas &e) {
  if (e.n == 0) return;
  float media      = (float)(e.crudoSuma / e.n);
  float ppCrudo_mV = (e.crudoMax - e.crudoMin) * 1000.0f;
  float ppFilt_mV  = e.filtMax - e.filtMin;
  float rmsFilt    = sqrtf((float)(e.filtCuad / e.n));
  float rmsHp      = sqrtf((float)(e.hpCuad / e.n));
  float difCuad    = rmsHp * rmsHp - rmsFilt * rmsFilt;
  float fueraBanda = sqrtf(difCuad > 0 ? difCuad : 0);
  const char *banda = bandaAngosta ? "7-24 Hz" : "0,5-40 Hz";

  Serial.printf("\n# --- Ventana de %d s (%lu muestras) ---\n",
                VENTANA_EST_S, (unsigned long)e.n);
  Serial.printf("# Salida AD8232: media %.3f V | min %.3f V | max %.3f V | p-p %.1f mV\n",
                media, e.crudoMin, e.crudoMax, ppCrudo_mV);
  Serial.printf("# Filtrado %s%s: p-p %.1f mV | RMS %.2f mV\n",
                banda, notchActivo ? " + notch" : "", ppFilt_mV, rmsFilt);
  Serial.printf("# RMS fuera de banda (50 Hz y resto, estimado): %.2f mV\n", fueraBanda);
  float loMasPct   = 100.0f * e.loMasCuenta / e.n;
  float loMenosPct = 100.0f * e.loMenosCuenta / e.n;
  if (hayPinesLO())
    Serial.printf("# Leads-off: LO+ (LA) %.0f %% | LO- (RA) %.0f %% del tiempo\n",
                  loMasPct, loMenosPct);

  // --- Latidos ---
  float fc, cv, rmssd; int nPares;
  int nOk = metricasRR(fc, cv, rmssd, nPares);
  Serial.printf("# Latidos en la ventana: %lu | R-R irregulares descartados: %lu\n",
                (unsigned long)e.nLat, (unsigned long)e.nIrreg);
  float qrs = (e.nLat > 0) ? (float)(e.qrsPPSuma / e.nLat) : 0;
  if (e.nLat > 0) {
    Serial.printf("# QRS p-p medio: %.1f mV en la salida (~%.0f uV en la entrada, banda %s)\n",
                  qrs, qrs / GANANCIA_MODULO * 1000.0f, banda);
  }
  if (nOk > 0 && e.nLat >= 2) {
    Serial.printf("# FC %.0f lpm (mediana de %d R-R) | CV de R-R %.1f %%", fc, nOk, cv * 100.0f);
    if (nPares >= 3) Serial.printf(" | RMSSD %.0f ms (orientativo, %d pares)", rmssd, nPares);
    Serial.println();
  } else {
    Serial.println("# FC: sin latidos suficientes en esta ventana");
  }
  Serial.printf("# Atrasos de muestreo acumulados: %lu\n", (unsigned long)atrasos);
  Serial.printf("# EDA (A0): %.3f V | lecturas del ADS que fallaron: %lu\n",
                a0Counts * 0.000125f, (unsigned long)erroresADS);

  // --- Diagnostico ---
  // El op-amp de salida llega a ~0,1 V de cada riel (datasheet): 3,228 V
  // medido en banco ya es el riel superior.
  bool enRiel   = (e.crudoMin < 0.12f) || (e.crudoMax > VCC_V - 0.12f);
  bool sinRuido = (rmsFilt < RUIDO_MIN_MV);     // ver RUIDO_MIN_MV
  bool latidosOk = (e.nLat >= 2) && (nOk >= 4) && (cv < 0.15f) &&
                   (fc >= 40.0f) && (fc <= 150.0f) && (qrs >= 5.0f) && !sinRuido;

  if (enRiel && ppCrudo_mV < 100.0f) {
    Serial.printf("# [!] Salida FIJA en el riel %s: electrodo con contacto DC muy alto\n",
                  (media > VCC_V / 2) ? "superior" : "inferior");
    Serial.println("#     (leads-off) o etapa de entrada saturada. Ver la linea de");
    Serial.println("#     leads-off: LO+ en alto = LA sin contacto, LO- = RA.");
  } else if (enRiel) {
    Serial.println("# [!] La salida toca un riel: electrodo sin contacto o saturacion por");
    Serial.println("#     offset del electrodo seco. Esperar 5-10 s quieto; si persiste,");
    Serial.println("#     revisar contacto, presion y RL. Despues mandar 'r'.");
  } else if (sinRuido) {
    Serial.println("# [!] Salida fija y SIN el ruido propio del AD8232: la etapa de entrada");
    Serial.println("#     no esta amplificando (saturada por offset DC de los electrodos o");
    Serial.println("#     RL sin contacto). Control: con RA, LA y RL en corto deberia verse");
    Serial.println("#     ~1,65 V y un RMS filtrado de ~1,5-2,5 mV.");
  } else if (fabsf(media - VCC_V / 2) > 0.5f) {
    Serial.println("# [?] Nivel DC lejos de 1,65 V: offset grande o etapa recuperandose.");
  }
  if (hayPinesLO() && (loMasPct > 0 || loMenosPct > 0)) {
    if (loMasPct > 0 && loMenosPct > 0)
      Serial.println("# [!] Leads-off en LA y RA a la vez: probable RL sin contacto (o ambos");
    else
      Serial.printf ("# [!] Leads-off en %s: ese electrodo no tiene contacto DC (gel, broche,\n",
                     (loMasPct > 0) ? "LA (LO+)" : "RA (LO-)");
    Serial.println(loMasPct > 0 && loMenosPct > 0
                   ? "#     electrodos sueltos). Las muestras con leads-off no son validas."
                   : "#     cable o piel muy seca). Las muestras con leads-off no son validas.");
  }
  if (!sinRuido && fueraBanda > 2.0f * rmsFilt && fueraBanda > 1.0f) {
    Serial.println("# [!] Domina el ruido fuera de banda (tipicamente 50 Hz): notebook con");
    Serial.println("#     cargador, RL con mal contacto o cables RA/LA separados.");
  }
  // Con leads-off mas del 10 % de la ventana, los latidos no se evaluan.
  bool loActivo = hayPinesLO() && (loMasPct > 10.0f || loMenosPct > 10.0f);
  if (!enRiel && !sinRuido && !loActivo) {
    if (latidosOk) {
      Serial.println("# [ok] Latidos regulares detectados. Validar la FC contra el pulso");
      Serial.println("#      tomado a mano durante 30 s (o un reloj/banda de pecho).");
    } else if (e.nLat == 0) {
      Serial.println("# [!] Sin latidos: revisar eje interno-externo, contacto y RL.");
      Serial.println("#     Probar 'w' (banda angosta) y 'r'.");
    } else {
      Serial.println("# [?] Detecciones irregulares: ruido, EMG o latidos perdidos.");
      Serial.println("#     Relajar y apoyar el brazo, mano abierta; probar 'w'.");
    }
  }
}

// Resume en un numero el diagnostico de la ventana (para la app por BLE). Usa
// los mismos criterios que imprimirEstadisticas(), en este orden de prioridad:
//   0 latidos regulares            1 midiendo (todavia sin ventana completa)
//   2 salida fija en un riel       3 la salida toca un riel
//   4 salida sin ruido propio      5 nivel DC lejos de 1,65 V
//   6 domina el ruido fuera de banda (50 Hz)
//   7 leads-off en mas del 10 % de la ventana
//   8 sin latidos                  9 detecciones irregulares
uint8_t codigoDiagnostico(const Estadisticas &e) {
  if (e.n == 0) return 1;
  float media      = (float)(e.crudoSuma / e.n);
  float ppCrudo_mV = (e.crudoMax - e.crudoMin) * 1000.0f;
  float rmsFilt    = sqrtf((float)(e.filtCuad / e.n));
  float rmsHp      = sqrtf((float)(e.hpCuad / e.n));
  float difCuad    = rmsHp * rmsHp - rmsFilt * rmsFilt;
  float fueraBanda = sqrtf(difCuad > 0 ? difCuad : 0);
  float loMasPct   = 100.0f * e.loMasCuenta / e.n;
  float loMenosPct = 100.0f * e.loMenosCuenta / e.n;
  float fc, cv, rmssd; int nPares;
  int nOk = metricasRR(fc, cv, rmssd, nPares);
  float qrs = (e.nLat > 0) ? (float)(e.qrsPPSuma / e.nLat) : 0;

  bool enRiel   = (e.crudoMin < 0.12f) || (e.crudoMax > VCC_V - 0.12f);
  bool sinRuido = (rmsFilt < RUIDO_MIN_MV);
  bool latidosOk = (e.nLat >= 2) && (nOk >= 4) && (cv < 0.15f) &&
                   (fc >= 40.0f) && (fc <= 150.0f) && (qrs >= 5.0f) && !sinRuido;
  bool loActivo = hayPinesLO() && (loMasPct > 10.0f || loMenosPct > 10.0f);

  if (enRiel && ppCrudo_mV < 100.0f) return 2;
  if (enRiel) return 3;
  if (sinRuido) return 4;
  if (loActivo) return 7;
  if (latidosOk) return 0;
  if (fueraBanda > 2.0f * rmsFilt && fueraBanda > 1.0f) return 6;
  if (fabsf(media - VCC_V / 2) > 0.5f) return 5;
  if (e.nLat == 0) return 8;
  return 9;
}

// ============================================================================
//  ESCANEO I2C (comando 's')
//  Verifica el nivel de reposo de SDA y SCL y busca dispositivos en todas las
//  direcciones. Mientras escanea, el muestreo se detiene unos milisegundos.
// ============================================================================
const char *nombreI2C(uint8_t a) {
  switch (a) {
    case 0x48: return "ADS1115 (ADDR a GND)";
    case 0x49: return "ADS1115 (ADDR a VDD)";
    case 0x4A: return "ADS1115 (ADDR a SDA)";
    case 0x4B: return "ADS1115 (ADDR a SCL)";
    case 0x68: return "MPU-6050 (AD0 a GND)";
    case 0x69: return "MPU-6050 (AD0 a VDD)";
    default:   return "desconocido";
  }
}

void escanearI2C() {
  Serial.println("\n# ==== Escaneo I2C ====");
  Serial.printf("# Pines: SDA = GPIO%d, SCL = GPIO%d\n", PIN_SDA, PIN_SCL);

  // 1) Nivel de reposo de las lineas, sin el periferico I2C ni pull-ups internos.
  //    Con los pull-ups externos (4k7 a 3,3 V) las dos deben estar en ALTO.
  Wire.end();
  pinMode(PIN_SDA, INPUT);
  pinMode(PIN_SCL, INPUT);
  delay(2);
  int sda = digitalRead(PIN_SDA);
  int scl = digitalRead(PIN_SCL);
  Serial.printf("# Reposo: SDA %s, SCL %s\n", sda ? "ALTO" : "BAJO", scl ? "ALTO" : "BAJO");
  if (!sda || !scl) {
    Serial.println("# [!] Linea en BAJO en reposo: falta el pull-up externo de esa linea,");
    Serial.println("#     esta en corto a GND, o un dispositivo la mantiene tomada.");
  }

  // 2) Escaneo a 100 kHz (mas tolerante a pull-ups debiles o pistas largas).
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);
  int encontrados = 0, erroresBus = 0;
  bool adsEsperado = false, adsOtraDir = false;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    uint8_t r = Wire.endTransmission();
    if (r == 0) {
      Serial.printf("# Encontrado 0x%02X  %s\n", a, nombreI2C(a));
      encontrados++;
      if (a == ADS_DIRECCION) adsEsperado = true;
      else if (a >= 0x48 && a <= 0x4B) adsOtraDir = true;
    } else if (r == 4 || r == 5) {
      erroresBus++;                  // 4 = error de bus, 5 = timeout
    }
  }
  Wire.setClock(400000);

  Serial.printf("# Dispositivos: %d | errores de bus: %d\n", encontrados, erroresBus);
  if (erroresBus > 0) {
    Serial.println("# [!] Errores de bus: SDA y SCL invertidos, en corto entre si o a GND,");
    Serial.println("#     o sin pull-ups.");
  }
  if (encontrados == 0) {
    Serial.println("# [!] Ningun dispositivo responde. Revisar: alimentacion y GND del ADS1115,");
    Serial.println("#     SDA y SCL en los pines correctos (no invertidos), pull-ups de 4k7,");
    Serial.println("#     soldaduras y orientacion/pinout del integrado o modulo en la PCB.");
  } else if (adsOtraDir && !adsEsperado) {
    Serial.printf("# [!] El ADS1115 responde en otra direccion: el pin ADDR no esta a GND.\n");
    Serial.printf("#     Conectar ADDR a GND o cambiar ADS_DIRECCION (ahora 0x%02X).\n", ADS_DIRECCION);
  } else if (!adsEsperado) {
    Serial.printf("# [!] No hay ADS1115 en 0x%02X ni en 0x49-0x4B. Si ADDR quedo flotando,\n", ADS_DIRECCION);
    Serial.println("#     la direccion es indefinida: conectarlo a GND.");
  }

  // 3) Si el ADS esta en la direccion esperada, reconfigurarlo y seguir midiendo.
  if (adsEsperado) {
    adsOk = iniciarADS();
    Serial.println(adsOk ? "# ADS1115 configurado: el muestreo continua." :
                           "# [!] El ADS1115 responde pero fallo su configuracion.");
  }
  Serial.println("# ==== Fin del escaneo ====");
  proximoUs = micros() + PERIODO_US;   // no contar el escaneo como atraso
}

// ============================================================================
//  MPU6050
// ============================================================================
bool i2cPresente(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

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

// Despierta el MPU6050 (sale de sleep). Rangos por defecto: +-2 g, +-250 gps.
bool iniciarMPU() {
  if (i2cPresente(MPU_ADDR_1)) mpuAddr = MPU_ADDR_1;
  else if (i2cPresente(MPU_ADDR_2)) mpuAddr = MPU_ADDR_2;
  else { mpuAddr = 0; return false; }
  return mpuEscribir(0x6B, 0x00);
}

void leerIMU() {
  imuValido = false;
  if (!mpuAddr) return;
  uint8_t d[14];
  if (!mpuLeer(0x3B, d, 14)) return;
  for (int i = 0; i < 7; i++) imuRaw[i] = (int16_t)((d[2 * i] << 8) | d[2 * i + 1]);
  imuValido = true;
}

// ============================================================================
//  TEST DEL CIRCUITO (arranque y comando 't'). Todas las lineas empiezan con '#'.
// ============================================================================
void resultado(const char *nombre, bool ok) {
  Serial.printf("# [%s] %s\n", ok ? " OK " : "FALLA", nombre);
}

void testMPU() {
  Serial.println("\n# -- MPU6050 --");
  okMPU = false;
  if (!iniciarMPU()) {
    Serial.println("# MPU6050 no responde en 0x68 ni 0x69.");
    resultado("MPU6050", false);
    return;
  }
  Serial.printf("# Direccion: 0x%02X\n", mpuAddr);
  uint8_t who = 0;
  if (!mpuLeer(0x75, &who, 1)) {
    Serial.println("# No se pudo leer WHO_AM_I.");
    resultado("MPU6050", false);
    return;
  }
  Serial.printf("# WHO_AM_I = 0x%02X (esperado 0x68; algunos clones dan 0x70/0x72)\n", who);
  delay(100);
  leerIMU();
  if (!imuValido) {
    Serial.println("# No se pudieron leer los datos.");
    resultado("MPU6050", false);
    return;
  }
  float ax = imuRaw[0], ay = imuRaw[1], az = imuRaw[2];
  float g = sqrtf(ax * ax + ay * ay + az * az) / 16384.0f;   // rango +-2 g
  Serial.printf("# Aceleracion total = %.2f g (quieto deberia dar ~1.00)\n", g);
  Serial.printf("# Temperatura = %.1f C\n", imuRaw[3] / 340.0f + 36.53f);
  okMPU = (g > 0.7f && g < 1.3f);
  if (!okMPU) Serial.println("# Datos fuera de lo esperado (todo en 0 o -1 suele indicar mal contacto).");
  resultado("MPU6050", okMPU);
}

void testADS() {
  Serial.println("\n# -- ADS1115 --");
  okADS = false;
  if (!adsOk) {
    Serial.println("# ADS1115 no responde en 0x48. Revisar pin ADDR (a GND = 0x48).");
    resultado("ADS1115", false);
    return;
  }
  int16_t c1 = 0, c0 = 0;
  bool l1 = adsLeer(ADS_MUX_ECG, c1);
  bool l0 = adsLeer(ADS_MUX_AUX, c0);
  if (!l1 || !l0) Serial.println("# [!] El ADS1115 no termino una conversion en 4 ms.");
  float v1 = ads.computeVolts(c1);
  float v0 = ads.computeVolts(c0);
  Serial.printf("# A0 (EDA)         = %.3f V (esperado 1 a 3 V)\n", v0);
  Serial.printf("# A1 (AD8232)      = %.3f V (esperado ~1.65 V)\n", v1);
  bool a0ok = (v0 > 0.8f && v0 < 3.2f);
  bool a1ok = (v1 > 0.12f && v1 < VCC_V - 0.12f);
  if (!a0ok) Serial.println("# A0 fuera de rango: revisar el operacional y su conexion al ADS.");
  if (!a1ok) Serial.println("# A1 en un riel: revisar alimentacion y OUTPUT del AD8232 (con electrodos");
  if (!a1ok) Serial.println("#    sueltos puede ser normal).");
  if (l0 && l1 && fabsf(v0 - v1) < 0.005f)
    Serial.println("# [?] A0 y A1 dan casi lo mismo: si no deberian coincidir, revisar el cableado de AIN0 y AIN1.");
  okADS = a0ok && a1ok && l0 && l1;
  resultado("ADS1115 A0", a0ok);
  resultado("ADS1115 A1", a1ok);
#if !LEER_EDA
  ads.startADCReading(ADS_MUX_ECG, /*continuous=*/true);   // volver al modo continuo del ECG
  delay(5);
#endif
}

void testSD() {
  Serial.println("\n# -- Tarjeta SD --");
  okSD = false;
  SPI.begin(PIN_SD_CLK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
  if (!SD.begin(PIN_SD_CS, SPI, 4000000)) {
    Serial.println("# No se pudo montar la SD. Revisar: tarjeta insertada, formato FAT32,");
    Serial.println("# conexiones MISO/MOSI/CLK/CS y alimentacion del modulo (muchos piden 5 V).");
    resultado("SD", false);
    return;
  }
  uint8_t tipo = SD.cardType();
  Serial.print("# Tipo: ");
  if (tipo == CARD_MMC) Serial.println("MMC");
  else if (tipo == CARD_SD) Serial.println("SDSC");
  else if (tipo == CARD_SDHC) Serial.println("SDHC");
  else Serial.println("desconocido");
  Serial.printf("# Tamano: %llu MB\n", SD.cardSize() / (1024ULL * 1024ULL));

  File f = SD.open("/test_esp32.txt", FILE_WRITE);
  if (!f) {
    Serial.println("# No se pudo crear el archivo de prueba.");
    resultado("SD", false);
    return;
  }
  f.println("Hola desde el XIAO ESP32-C6");
  f.close();

  f = SD.open("/test_esp32.txt");
  if (!f) {
    Serial.println("# No se pudo reabrir el archivo.");
    resultado("SD", false);
    return;
  }
  String contenido = f.readStringUntil('\n');
  f.close();
  Serial.print("# Leido: ");
  Serial.println(contenido);
  okSD = contenido.startsWith("Hola desde el XIAO");
  SD.remove("/test_esp32.txt");
  resultado("SD (escritura/lectura)", okSD);
}

void testLeadsOff() {
  Serial.println("\n# -- Leads-off del AD8232 --");
  uint8_t m = leerLeadsOff();
  Serial.printf("# LO+ (D1) = %d | LO- (D0) = %d -> %s\n", (m & 1) ? 1 : 0, (m & 2) ? 1 : 0,
                m ? "electrodo DESCONECTADO" : "electrodos conectados");
  Serial.println("# Sin electrodos deberian dar 1; con los electrodos en la piel, 0.");
}

void testCircuito() {
  Serial.println("\n# ===== TEST DEL CIRCUITO =====");
  escanearI2C();
  testMPU();
  testADS();
  testSD();
  testLeadsOff();
  Serial.println("\n# -- RESUMEN --");
  resultado("MPU6050", okMPU);
  resultado("ADS1115", okADS);
  resultado("SD", okSD);
  Serial.println("# ===== FIN DEL TEST =====");
  // El test interrumpio el muestreo: arrancar de nuevo limpio.
  configurarFiltros();
  resetDetector();
  resetEst(est);
  resetAcumuladores();
  hayEst = false;
  proximoUs = micros() + PERIODO_US;
}

void imprimirAyuda() {
  Serial.println("# ==== ECG AD8232 SparkFun + BLE (AIN1) ====");
  Serial.printf ("# fs %d Hz | perfil %d (1=IDE2, 2=IDE1.8, 3=Teleplot)\n", FS_HZ, perfil);
  Serial.println("# f=filtrado  c=crudo  a=ambos  d=diagnostico  i=estadisticas");
  Serial.println("# w=banda angosta/ancha  n=notch  p=polaridad  l=lineas de escala");
  Serial.println("# +/- escala  b=marcadores  r=reiniciar  1/2/3=perfil  s=I2C");
  Serial.println("# t=test del circuito  h=ayuda");
}

void mensaje(const char *texto) {
  if (textoPermitido()) Serial.println(texto);
}

void procesarComandos() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    switch (c) {
      case 'f': modo = MODO_FILTRADO; resetAcumuladores(); break;
      case 'c': modo = MODO_CRUDO;    resetAcumuladores(); break;
      case 'a': modo = MODO_AMBOS;    resetAcumuladores(); break;
      case 'd': modo = MODO_DIAG; mensaje("# Modo diagnostico (texto cada 2 s)"); break;
      case 'i': if (hayEst) imprimirEstadisticas(ultimaEst); break;
      case 'w':
        bandaAngosta = !bandaAngosta;
        configurarFiltros();
        mensaje(bandaAngosta ? "# Banda angosta 7-24 Hz" : "# Banda ancha 0,5-40 Hz");
        break;
      case 'n':
        notchActivo = !notchActivo;
        fNotch.reset();
        filtrosCebados = false;
        mensaje(notchActivo ? "# Notch ON" : "# Notch OFF");
        break;
      case 'p': polaridad = -polaridad; break;
      case 'l': limites = !limites; break;
      case '+': if (escalaMv < 2000.0f) escalaMv *= 2.0f; break;
      case '-': if (escalaMv > 10.0f)   escalaMv *= 0.5f; break;
      case 'b': marcadores = !marcadores; break;
      case 'r': configurarFiltros(); resetDetector(); mensaje("# Reinicio de filtros y detector"); break;
      case '1': perfil = 1; resetAcumuladores(); break;
      case '2': perfil = 2; resetAcumuladores(); break;
      case '3': perfil = 3; resetAcumuladores(); break;
      case 's': escanearI2C(); break;
      case 't': testCircuito(); break;
      case 'h': imprimirAyuda(); break;
      default: break;   // ignora \r, \n y otras teclas
    }
  }
}

// ============================================================================
//  BLE
// ============================================================================
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *s) {
    deviceConnected = true;
  }
  void onDisconnect(BLEServer *s) {
    deviceConnected = false;
  }
};

static inline void putI16(uint8_t *p, int16_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)(((uint16_t)v) >> 8);
}

static inline void putU16(uint8_t *p, uint16_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)(v >> 8);
}

void iniciarBLE() {
  BLEDevice::init(BLE_NOMBRE);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);
  pEcgChar = pService->createCharacteristic(ECG_CHAR_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pAuxChar = pService->createCharacteristic(AUX_CHAR_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pMetChar = pService->createCharacteristic(MET_CHAR_UUID, BLECharacteristic::PROPERTY_NOTIFY);
#if defined(CONFIG_BLUEDROID_ENABLED)
  // Con NimBLE el descriptor 0x2902 se agrega solo; con Bluedroid hay que ponerlo a mano.
  pEcgChar->addDescriptor(new BLE2902());
  pAuxChar->addDescriptor(new BLE2902());
  pMetChar->addDescriptor(new BLE2902());
#endif
  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();
  Serial.printf("# BLE: publicitando como \"%s\". Abrir la app y tocar Conectar.\n", BLE_NOMBRE);
}

// Reconexion sin frenar el muestreo
void gestionarBLE() {
  if (!deviceConnected && oldDeviceConnected) {
    oldDeviceConnected = false;
    reanunciar = true;
    reanunciarMs = millis() + 300;        // darle tiempo al stack antes de publicitar
    mensaje("# BLE: celular desconectado");
  }
  if (reanunciar && (int32_t)(millis() - reanunciarMs) >= 0) {
    reanunciar = false;
    pServer->startAdvertising();
  }
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = true;
    bleN = 0; bleLo = 0; bleBeat = 0;
    mensaje("# BLE: celular conectado");
  }
}

void enviarEcg() {
  uint16_t sq = seqPkt++;
  if (!deviceConnected) return;
  uint8_t pkt[4 + 2 * BLE_MUESTRAS_PKT];
  putU16(pkt, sq);
  pkt[2] = bleLo & 0x03;
  pkt[3] = bleBeat;
  for (int i = 0; i < BLE_MUESTRAS_PKT; i++) putI16(pkt + 4 + 2 * i, bleBuf[i]);
  pEcgChar->setValue(pkt, sizeof(pkt));
  pEcgChar->notify();
}

// Agrega una muestra de ECG filtrado (mV) al paquete en armado. Con marca =
// true (latido confirmado en esta muestra) se informa donde quedo la R.
void bleMuestra(double filt_mV, uint8_t loMask, bool marca) {
  double s = filt_mV * 10.0;                     // decimas de mV
  if (s > 32767.0) s = 32767.0;
  if (s < -32768.0) s = -32768.0;
  bleBuf[bleN] = (int16_t)(s >= 0 ? s + 0.5 : s - 0.5);
  bleLo |= loMask;
  if (marca) { bleBeat = 1; bleBeatN = det.ultimoN; }
  bleN++;
  if (bleN >= BLE_MUESTRAS_PKT) {
    if (bleBeat) {
      // k = 1 -> la R es la ultima muestra de este paquete; k = 2, la anterior...
      uint32_t k = nMuestra - bleBeatN + 1;      // nMuestra = indice de esta muestra
      bleBeat = (uint8_t)(k < 1 ? 1 : (k > 255 ? 255 : k));
    }
    enviarEcg();
    bleN = 0; bleLo = 0; bleBeat = 0;
  }
}

void enviarAux() {
  if (!deviceConnected) return;
  uint8_t pkt[17];
  for (int i = 0; i < 7; i++) putI16(pkt + 2 * i, imuValido ? imuRaw[i] : 0);
  putI16(pkt + 14, a0Counts);
  pkt[16] = (imuValido ? 1 : 0) | (adsOk ? 2 : 0);
  pAuxChar->setValue(pkt, sizeof(pkt));
  pAuxChar->notify();
}

void enviarMetricas() {
  if (!deviceConnected) return;
  float fc, cv, rmssd; int nPares;
  int nOk = metricasRR(fc, cv, rmssd, nPares);

  uint16_t fcV = (nOk >= 2) ? (uint16_t)(fc + 0.5f) : 0;
  float cvPct = cv * 100.0f;
  uint8_t cvV = (nOk >= 2) ? (uint8_t)(cvPct > 255.0f ? 255.0f : cvPct) : 0;
  uint16_t rmssdV = (nPares >= 3) ? (uint16_t)(rmssd > 65535.0f ? 65535.0f : rmssd) : 0;

  float qrsUv = 0, dcMv = 0;
  uint8_t diag = 1;
  if (hayEst) {
    if (ultimaEst.nLat > 0) qrsUv = (float)(ultimaEst.qrsPPSuma / ultimaEst.nLat) / GANANCIA_MODULO * 1000.0f;
    if (ultimaEst.n > 0) dcMv = (float)(ultimaEst.crudoSuma / ultimaEst.n) * 1000.0f;
    diag = codigoDiagnostico(ultimaEst);
  }
  if (qrsUv > 65535.0f) qrsUv = 65535.0f;
  if (dcMv < 0) dcMv = 0;
  if (dcMv > 65535.0f) dcMv = 65535.0f;
  uint32_t atr = atrasos > 65535UL ? 65535UL : atrasos;

  uint8_t pkt[14];
  putU16(pkt + 0, fcV);
  pkt[2] = (uint8_t)(nOk > 255 ? 255 : nOk);
  pkt[3] = cvV;
  putU16(pkt + 4, rmssdV);
  putU16(pkt + 6, (uint16_t)(qrsUv + 0.5f));
  pkt[8] = diag;
  pkt[9] = (bandaAngosta ? 1 : 0) | (notchActivo ? 2 : 0);
  putU16(pkt + 10, (uint16_t)(dcMv + 0.5f));
  putU16(pkt + 12, (uint16_t)atr);
  pMetChar->setValue(pkt, sizeof(pkt));
  pMetChar->notify();
}

// Tareas lentas, repartidas en distintas muestras para no acumular demoras:
//   k = 5  -> MPU6050 + paquete AUX (10 Hz)
//   k = 15 -> lectura de A0 / EDA (10 Hz, ~2 ms)
//   m = 60 -> paquete de metricas (2 Hz)
void tareasPeriodicas() {
  uint32_t k = nMuestra % AUX_PERIODO_N;
  if (k == 5) {
    leerIMU();
    enviarAux();
  } else if (k == 15) {
#if LEER_EDA
    if (adsOk) leerA0Counts();
#endif
  }
  if ((nMuestra % MET_PERIODO_N) == 60) enviarMetricas();
}

// ============================================================================
//  SETUP
// ============================================================================
void setup() {
  // AD8232 encendido lo antes posible (SDN en alto) y para siempre.
  if (PIN_SDN >= 0)      { pinMode(PIN_SDN, OUTPUT); digitalWrite(PIN_SDN, HIGH); }
  Serial.begin(SERIAL_BAUD);
#if ARDUINO_USB_MODE && ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxTimeoutMs(0);   // sin monitor serie abierto, no frenar el muestreo
#endif
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0 < 3000)) { ; }

  if (PIN_LO_MAS >= 0)   pinMode(PIN_LO_MAS, INPUT);
  if (PIN_LO_MENOS >= 0) pinMode(PIN_LO_MENOS, INPUT);

  resetDetector();
  configurarFiltros();
  resetEst(est);
  resetAcumuladores();

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);

  adsOk = iniciarADS();
  iniciarMPU();
  if (textoPermitido()) imprimirAyuda();
  if (!adsOk) escanearI2C();
  // Sin numeros en este mensaje: no crea series falsas en el plotter.
  if (!adsOk) Serial.println("# [ERROR] ADS no responde. Revisar SDA SCL y alimentacion. Reintentando...");

#if TEST_AL_ARRANQUE
  testCircuito();
#endif

  iniciarBLE();
  proximoUs = micros() + PERIODO_US;
}

// ============================================================================
//  LOOP
// ============================================================================
void loop() {
  procesarComandos();
  gestionarBLE();

  if (!adsOk) {
    if (millis() - ultimoReintento > 2000) {
      ultimoReintento = millis();
      adsOk = iniciarADS();
      if (adsOk) {
        mensaje("# ADS OK");
        resetDetector();             // hubo un hueco sin muestras: R-R invalidos
        filtrosCebados = false;
      }
      proximoUs = micros() + PERIODO_US;
    }
    delay(1);
    return;
  }

  // --- Base de tiempo fija ---
  uint32_t ahora = micros();
  int32_t falta = (int32_t)(proximoUs - ahora);
  if (falta > 0) {
    if (falta > 2000) delay(1);     // ceder la CPU al stack BLE si falta mucho
    return;
  }
  // Atraso de hasta ATRASO_MAX_US: se recupera tomando las muestras seguidas.
  // Mayor: se resincroniza, pero el contador de muestras avanza lo que
  // corresponde al tiempo perdido. Antes se descartaba ese tiempo, los R-R
  // medidos en muestras quedaban mas cortos y la FC daba de mas.
  uint32_t atraso = ahora - proximoUs;
  if (atraso > PERIODO_US) {
    if (!enAtraso) { atrasos++; enAtraso = true; }
  } else {
    enAtraso = false;
  }
  if (atraso > ATRASO_MAX_US) {
    nMuestra += atraso / PERIODO_US;
    proximoUs = ahora;
  }
  proximoUs += PERIODO_US;

  // --- Adquisicion (single-shot: la conversion arranca ahora) ---
#if LEER_EDA
  int16_t cuentas = ultimaCuentaECG;
  if (adsLeer(ADS_MUX_ECG, cuentas)) {
    ultimaCuentaECG = cuentas;
    adsFallasSeguidas = 0;
  } else {
    erroresADS++;                   // se repite la muestra anterior
    if (++adsFallasSeguidas >= 50) { adsOk = false; adsFallasSeguidas = 0; }   // se reintenta iniciar
  }
#else
  int16_t cuentas = ads.getLastConversionResults();   // identico a test_ad8232_biceps
#endif
  float v = ads.computeVolts(cuentas);
  double v_mV = (double)v * 1000.0;
  uint8_t loMask = leerLeadsOff();
  bool lo = (loMask != 0);

  // --- Filtrado de visualizacion ---
  if (!filtrosCebados) { cebarFiltros(v_mV); filtrosCebados = true; }
  double hp   = fHP.procesar(v_mV);
  double tmp  = notchActivo ? fNotch.procesar(hp) : hp;
  double filt = fLP.procesar(tmp);

  // --- Detector de latidos ---
  bool marca = procesarDetector(v_mV, (float)filt);

  // --- Estadisticas de ventana ---
  est.n++;
  if (v < est.crudoMin) est.crudoMin = v;
  if (v > est.crudoMax) est.crudoMax = v;
  est.crudoSuma += v;
  if (filt < est.filtMin) est.filtMin = (float)filt;
  if (filt > est.filtMax) est.filtMax = (float)filt;
  est.filtCuad += filt * filt;
  est.hpCuad   += hp * hp;
  if (lo) est.loCuenta++;
  if (loMask & 0x01) est.loMasCuenta++;
  if (loMask & 0x02) est.loMenosCuenta++;

  if (est.n >= N_VENTANA) {
    ultimaEst = est;
    hayEst = true;
    resetEst(est);
    if (modo == MODO_DIAG) imprimirEstadisticas(ultimaEst);
  }

  salida(v, hp, filt, lo, marca);
  bleMuestra(filt, loMask, marca);
  nMuestra++;
  tareasPeriodicas();
}
