/*
 * ============================================================================
 *  test_ad8232_biceps.ino — PRUEBA DEL MODULO SPARKFUN AD8232 EN EL BRAZO
 *  Wearable pre-ictal · XIAO ESP32-C6 + ADS1115 + modulo SparkFun AD8232
 *  Version 2.2 (basada en test_ad8232_ecg.ino)
 *   2.1: el diagnostico ya no informa FC con R-R viejos, ignora "latidos"
 *        por debajo del ruido propio del AD8232 y detecta salida bloqueada.
 *   2.2: el detector exige energia >= 8 veces el ruido medio (con entradas
 *        en corto ya no informa latidos). Limite de deteccion en banco:
 *        QRS de ~30 uV p-p referidos a la entrada.
 * ============================================================================
 *
 *  OBJETIVO
 *   Verificar si el modulo SparkFun, con su topologia original (0,5-41 Hz,
 *   ganancia ~1100), entrega un ECG utilizable con los electrodos en la parte
 *   superior del brazo izquierdo (opcion B). Ademas de graficar, el sketch
 *   detecta latidos y reporta:
 *     - frecuencia cardiaca (FC) y regularidad de los intervalos R-R;
 *     - RMSSD orientativo (resolucion de 4 ms, no es la medicion final);
 *     - amplitud pico a pico del QRS referida a la entrada, para compararla
 *       con la bibliografia (~70 uV p-p en la linea media del brazo,
 *       Escalona et al., Sensors 2022).
 *
 *  COLOCACION DE ELECTRODOS (brazo izquierdo, a la altura del punto medio
 *  entre el acromion y el olecranon)
 *   LA (cable azul,  +IN) -> cara EXTERNA del brazo
 *   RA (cable negro, -IN) -> cara INTERNA del brazo, enfrentado a LA (180°)
 *   RL (cable rojo)       -> cara POSTERIOR (sobre el triceps), a ~90°
 *   Si el QRS aparece invertido, mandar 'p' (o intercambiar LA y RA).
 *   Postura: sentado, brazo relajado y apoyado, codo a ~90°, antebrazo sobre
 *   la mesa, mano abierta. Contraer el biceps agrega EMG (sirve de prueba).
 *
 *  MODIFICACIONES RECOMENDADAS AL MODULO (no obligatorias para esta prueba)
 *   - Cortar el puente SJ4 (desconecta el LED, que carga la salida).
 *   - 100 ohm a 1 kohm en serie entre OUTPUT y el capacitor del ADC; bajar
 *     ese capacitor (C5 de la placa de banco) de 100 nF a 10 nF.
 *
 *  PERFILES DE SALIDA (definen cuantos puntos se mandan al graficador)
 *   1 = Serial Plotter de Arduino IDE 2.x. Muestra SOLO 50 puntos: se mandan
 *       ~21 puntos/s con diezmado que conserva picos -> ~2,4 s en pantalla.
 *       Sirve para ver latidos; la morfologia fina del QRS se pierde.
 *   2 = Serial Plotter de Arduino IDE 1.8.x (500 puntos): 125 puntos/s
 *       -> 4 s en pantalla, con morfologia razonable.
 *   3 = Teleplot (extension de VS Code): 250 muestras/s con marca de tiempo,
 *       ventana configurable. Es la mejor opcion para ver la onda completa.
 *   Se elige con PERFIL_INICIAL o enviando '1', '2' o '3'.
 *
 *  COMANDOS POR SERIE (una letra; se pueden mandar desde la caja de texto
 *  del Serial Plotter de IDE 2, o desde el Serial Monitor)
 *   f  ECG filtrado en mV                                  [por defecto]
 *   c  crudo en V absolutos (nivel DC y saturacion)
 *   a  ambos: "solo pasa-altos" (con 50 Hz) y filtrado, en mV
 *   d  diagnostico en texto cada 2 s (usar el Serial Monitor)
 *   i  imprimir una vez las estadisticas de la ultima ventana
 *   w  banda angosta 7-24 Hz (Fig. 64) <-> banda ancha 0,5-40 Hz
 *   n  notch de 50 Hz on/off
 *   p  invertir polaridad del grafico
 *   l  lineas de escala fija on/off (estabilizan el eje Y)
 *   +  duplicar la escala fija      -  dividir la escala fija por 2
 *   b  marcadores de latido detectado on/off
 *   r  reiniciar filtros y detector (tras recolocar electrodos o saturar)
 *   s  escaneo I2C: nivel de SDA/SCL y dispositivos presentes (Serial Monitor)
 *   h  ayuda
 *
 *  REQUISITOS
 *   - Core arduino-esp32 3.x, placa "XIAO_ESP32C6", USB CDC On Boot: Enabled.
 *   - Libreria Adafruit ADS1X15 (v2.x).
 *   - Conexion sin cambios: OUTPUT del modulo -> AIN1 del ADS1115.
 *
 *  NUMERACION DE PINES: GPIO del ESP32-C6. I2C: SDA = D9 (GPIO20), SCL = D10
 *  (GPIO18). Leads-off: LO+ = D0 (GPIO0), LO- = D1 (GPIO1).
 *  mapadeconexiones.txt usa la numeracion del XIAO ESP32-S3.
 * ============================================================================
 */

#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <math.h>

// ----------------------------- PINES ----------------------------------------
#define PIN_SDA        20   // D9  (nuevo esquema de conexiones)
#define PIN_SCL        18   // D10 (nuevo esquema de conexiones)
// Leads-off del modulo (salidas digitales del AD8232, nivel 3,3 V). En ALTO, ese
// electrodo no tiene contacto DC. LO+ corresponde a LA (+IN), LO- a RA (-IN).
// -1 = sin cablear.
#define PIN_LO_MAS     0    // D0 (GPIO0) -> LO+ del modulo
#define PIN_LO_MENOS   1    // D1 (GPIO1) -> LO- del modulo
// SDN es activo en BAJO (datasheet + esquematico v10, con pull-up de 10 kohm
// en el modulo). Si se cablea a un GPIO, el codigo lo fuerza a ALTO.
#define PIN_SDN        -1

// ----------------------------- ADS1115 --------------------------------------
#define ADS_DIRECCION  0x48
#define ADS_MUX_ECG    ADS1X15_REG_CONFIG_MUX_SINGLE_1   // AIN1 = OUTPUT AD8232
#define VCC_V          3.3f

// ----------------------------- MUESTREO -------------------------------------
#define FS_HZ          250
#define PERIODO_US     (1000000UL / FS_HZ)               // 4000 us
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
// Pan-Tompkins simplificado: pasa-banda 5-20 Hz, cuadrado, integracion de
// 80 ms y umbral adaptativo. Es independiente de la polaridad y del filtro
// de visualizacion elegido.
#define DET_HP_HZ        5.0
#define DET_LP_HZ        20.0
#define DET_N_INT        20          // 80 ms de integracion
#define DET_REFRACT_N    63          // ~250 ms -> FC maxima ~240 lpm
#define DET_PICO_MAX_N   38          // 150 ms maximo para cerrar un pico
#define DET_APREND_N     (2 * FS_HZ) // 2 s de aprendizaje inicial
#define DET_N_QRS        40          // 160 ms hacia atras para medir el QRS
#define RR_MIN_MS        270.0f
#define RR_MAX_MS        2000.0f
#define RR_TOL           0.30f       // +-30 % de la mediana: si no, irregular
#define N_RR             16
#define RR_EDAD_MAX_N    (10 * FS_HZ) // solo R-R de los ultimos 10 s
// Energia minima (mV^2, banda 5-20 Hz integrada) para aceptar un latido. El
// ruido del ADS1115 da ~0,01 y el ruido propio del AD8232 ~1-2; un QRS de
// brazo (~70 mV p-p en la salida) da cientos. Evita "latidos" sobre ruido.
#define DET_Y_MIN        9.0
// Ademas, un latido debe superar DET_K_RUIDO veces el nivel medio de ruido
// (npk). Con entradas en corto, el ruido del AD8232 genera picos de energia
// de hasta ~5 veces su media; un QRS de brazo, decenas a cientos de veces.
#define DET_K_RUIDO      8.0
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
uint32_t atrasos = 0;
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
  Biquad hp, lp;
  double cuad[DET_N_INT];
  int    idx;
  double suma;
  uint32_t n;                 // muestras desde el reinicio
  double maxAprend, sumAprend;
  uint32_t nAprend;
  double spk, npk;            // nivel de senal y de ruido (integrado)
  bool   enPico;
  double picoVal;
  uint32_t picoN;
  bool   hayUltimo;
  uint32_t ultimoN;           // muestra del ultimo latido
  uint32_t nRef;              // ultimo latido o fin del aprendizaje
  float  rrCrudo[N_RR];       // ms
  bool   rrOk[N_RR];
  uint32_t rrN[N_RR];         // muestra en la que termina cada R-R
  int    iRR, nRR;
};
Detector det;
float bufFilt[128];           // ultimas 128 muestras filtradas (para el QRS)
float fcInstantanea = 0;      // para Teleplot
bool  fcNueva = false;

// ============================================================================
//  DISENO DE FILTROS
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
  ads.startADCReading(ADS_MUX_ECG, /*continuous=*/true);
  delay(5);
  return true;
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
  disenarBiquad(det.hp, PASA_ALTOS,    DET_HP_HZ,   Q_BUTTER, FS_HZ);
  disenarBiquad(det.lp, PASA_BAJOS,    DET_LP_HZ,   Q_BUTTER, FS_HZ);
  filtrosCebados = false;   // se ceban con la proxima muestra
}

void cebarFiltros(double x0_mV) {
  double y = fHP.estadoEstable(x0_mV);
  y = fNotch.estadoEstable(y);
  fLP.estadoEstable(y);
  double d = det.hp.estadoEstable(x0_mV);
  det.lp.estadoEstable(d);
}

// ============================================================================
//  DETECTOR DE LATIDOS
// ============================================================================
void resetDetector() {
  for (int i = 0; i < DET_N_INT; i++) det.cuad[i] = 0;
  det.idx = 0;  det.suma = 0;  det.n = 0;
  det.maxAprend = 0;  det.sumAprend = 0;  det.nAprend = 0;
  det.spk = 0;  det.npk = 0;
  det.enPico = false;  det.picoVal = 0;  det.picoN = 0;
  det.hayUltimo = false;  det.ultimoN = 0;  det.nRef = 0;
  det.iRR = 0;  det.nRR = 0;
  for (int i = 0; i < N_RR; i++) { det.rrCrudo[i] = 0; det.rrOk[i] = false; det.rrN[i] = 0; }
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

void registrarLatido() {
  det.spk = 0.125 * det.picoVal + 0.875 * det.spk;

  // Amplitud p-p del QRS en la senal mostrada: ventana de 160 ms que termina
  // en el pico de la senal integrada (que llega ~50-80 ms despues de la R).
  float mx = -1e9f, mn = 1e9f;
  for (int k = 0; k <= DET_N_QRS; k++) {
    float a = bufFilt[(det.picoN - k) & 127];
    if (a > mx) mx = a;
    if (a < mn) mn = a;
  }
  est.nLat++;
  est.qrsPPSuma += (mx - mn);

  if (det.hayUltimo) agregarRR((det.picoN - det.ultimoN) * 1000.0f / FS_HZ, det.picoN);
  det.hayUltimo = true;
  det.ultimoN = det.picoN;
  det.nRef = det.picoN;
}

// Procesa una muestra. Devuelve true en el instante en que empieza un latido
// (para el marcador del grafico).
bool procesarDetector(double x_mV, float filt) {
  bufFilt[nMuestra & 127] = filt;

  double b = det.lp.procesar(det.hp.procesar(x_mV));
  double c = b * b;
  det.suma += c - det.cuad[det.idx];
  det.cuad[det.idx] = c;
  det.idx = (det.idx + 1) % DET_N_INT;
  if (det.suma < 0) det.suma = 0;
  double y = det.suma / DET_N_INT;
  det.n++;

  // Aprendizaje: 2 s para estimar niveles iniciales de senal y ruido
  if (det.n <= DET_APREND_N) {
    if (det.n > DET_N_INT + 25) {
      if (y > det.maxAprend) det.maxAprend = y;
      det.sumAprend += y;
      det.nAprend++;
    }
    if (det.n == DET_APREND_N) {
      det.npk  = det.nAprend ? det.sumAprend / det.nAprend : 0;
      det.spk  = det.maxAprend;
      det.nRef = nMuestra;
    }
    return false;
  }

  bool marca = false;
  double umbral = det.npk + 0.25 * (det.spk - det.npk);

  if (!det.enPico) {
    bool fueraRefractario = !det.hayUltimo || ((nMuestra - det.ultimoN) > DET_REFRACT_N);
    if (y > umbral && y > DET_Y_MIN && y > DET_K_RUIDO * det.npk && fueraRefractario) {
      det.enPico = true;  det.picoVal = y;  det.picoN = nMuestra;
      marca = true;
    } else {
      det.npk += (y - det.npk) / FS_HZ;              // media movil, tau ~1 s
      // Sin latidos durante 2 s: el nivel de senal cae a la mitad por
      // segundo, para recuperar la deteccion si la amplitud bajo.
      if ((nMuestra - det.nRef) > 2 * FS_HZ) det.spk *= 0.99723;
    }
  } else {
    if (y > det.picoVal) { det.picoVal = y; det.picoN = nMuestra; }
    if (y < 0.5 * det.picoVal || (nMuestra - det.picoN) > DET_PICO_MAX_N) {
      det.enPico = false;
      registrarLatido();
    }
  }
  return marca;
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

void imprimirAyuda() {
  Serial.println("# ==== Prueba AD8232 SparkFun en el brazo (AIN1) ====");
  Serial.printf ("# fs %d Hz | perfil %d (1=IDE2, 2=IDE1.8, 3=Teleplot)\n", FS_HZ, perfil);
  Serial.println("# f=filtrado  c=crudo  a=ambos  d=diagnostico  i=estadisticas");
  Serial.println("# w=banda angosta/ancha  n=notch  p=polaridad  l=lineas de escala");
  Serial.println("# +/- escala  b=marcadores  r=reiniciar  1/2/3=perfil  s=I2C  h=ayuda");
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
      case 'h': imprimirAyuda(); break;
      default: break;   // ignora \r, \n y otras teclas
    }
  }
}

// ============================================================================
//  SETUP
// ============================================================================
void setup() {
  Serial.begin(SERIAL_BAUD);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0 < 3000)) { ; }

  if (PIN_SDN >= 0)      { pinMode(PIN_SDN, OUTPUT); digitalWrite(PIN_SDN, HIGH); }
  if (PIN_LO_MAS >= 0)   pinMode(PIN_LO_MAS, INPUT);
  if (PIN_LO_MENOS >= 0) pinMode(PIN_LO_MENOS, INPUT);

  resetDetector();
  configurarFiltros();
  resetEst(est);
  resetAcumuladores();

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);

  adsOk = iniciarADS();
  if (textoPermitido()) imprimirAyuda();
  if (!adsOk) escanearI2C();
  // Sin numeros en este mensaje: no crea series falsas en el plotter.
  if (!adsOk) Serial.println("# [ERROR] ADS no responde. Revisar SDA SCL y alimentacion. Reintentando...");

  proximoUs = micros() + PERIODO_US;
}

// ============================================================================
//  LOOP
// ============================================================================
void loop() {
  procesarComandos();

  if (!adsOk) {
    if (millis() - ultimoReintento > 2000) {
      ultimoReintento = millis();
      adsOk = iniciarADS();
      if (adsOk) mensaje("# ADS OK");
      proximoUs = micros() + PERIODO_US;
    }
    return;
  }

  // --- Base de tiempo fija ---
  uint32_t ahora = micros();
  if ((int32_t)(ahora - proximoUs) < 0) return;
  if ((uint32_t)(ahora - proximoUs) > PERIODO_US) {
    atrasos++;
    proximoUs = ahora;
  }
  proximoUs += PERIODO_US;

  // --- Adquisicion ---
  int16_t cuentas = ads.getLastConversionResults();
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
  nMuestra++;
}
