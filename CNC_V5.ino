/*
 * CNC XYZW V5.1 - Firmware ESP32
 * Sobre la base V5 (25 hallazgos de "Auditoria CNC XYZW V3") se remedia la
 * "Auditoria CNC XYZW V5" (objetivo: scoring > 80/100):
 *  N19 manualStop ya no aborta bloques encolados: solo stopEpoch/jogCancelEpoch
 *      deciden; asi '$H', G-code, $J y move_multi_abs funcionan tras un stop
 *      ('!', 0x18, timeout o E-stop fisico) sin rearmado manual.
 *  N20 polaridad del E-stop corregida: con boton NC a GND e INPUT_PULLUP el
 *      estado activo (pulsado o cable cortado) es HIGH: ISR en RISING y todos
 *      los chequeos == HIGH. Fail-safe real.
 *  N21 heartbeat del cliente en AMBOS modos (ver CNC_V5.py: timer 500 ms).
 *  N22 ok diferido hasta que haya hueco en la cola (control de flujo estilo
 *      GRBL) en G0/G1/G4/G28/$J/move_multi_abs; el cliente reintenta ante FULL.
 *  N12 toda salida de ALARM ($X / enable_actuators) purga cola y solicitudes
 *      pendientes y resincroniza plannedPos; si la purga no se logra, no se
 *      desbloquea (fail-safe).
 *  N13 G10/G92/set_zero_axis fijan el WCO contra plannedPos (final de cola).
 *  N25 arcos con espera de hueco extendida y telemetria durante la espera,
 *      validacion de error de radio (tolerancia GRBL 0.5 mm) y G53 aceptado.
 *  N6  NaN/Inf rechazados en inLim y en el planificador (isfinite).
 *  N14 $J absoluto aplica el WCO (consistente con G1).
 *  N16 la telemetria compacta incluye wco (campo 28); la UI muestra WPos.
 *  N17 reset_step_counter y set_calibration_axis invalidan homed: exigen
 *      re-referenciado del eje.
 *  A5  rampa con velocidades reales de union, pasada hacia adelante + hacia
 *      atras y junction deviation estandar (sin discontinuidad cerca de 5.7°).
 *  A1r 0x85 solo resincroniza plannedPos de los ejes con jog cancelado.
 *  C5  limite de intentos AUTH, cierre de sesion sin autenticar (timeout) y
 *      requerimiento de corte de potencia del E-stop documentado (C2r).
 *  C2r escrituras TCP con contador de reintentos: un cliente colgado no
 *      puede bloquear loop() indefinidamente.
 *  M5  cliente: excepciones registradas (logging) y cierre de camara con
 *      espera real del hilo.
 *  N26 jog GRBL con feed derivado de la velocidad configurada y profundidad
 *      de cola acotada.
 *  M2/M3/M7/M8: sin cambio estructural (piso de paso, pines, lazo abierto);
 *      documentados en la transferencia de ingenieria.
 *  N1  planificacion desde el final de la cola (plannedPos protegido por mutex)
 *  C2  E-stop: pin fisico con ISR que deshabilita drivers + ACK con reintento (cliente)
 *  C3  watchdog funcional (wasConnected), heartbeat con timeout en Run/Jog, server.begin() en GOT_IP
 *  C5  autenticacion por token por sesion (NVS, comando serial SET_AUTH_TOKEN)
 *  C1r soft limits siempre activos; movimiento bloqueado en ejes sin referenciar
 *  A5  aceleracion real (mm/s2) con look-ahead y velocidades de union
 *  A1r seccion critica en cola, reintento de mutex, stop con contador de epoca, sin doble Give
 *  N5  jog GRBL cancelable (0x85 solo cancela jog, no deshabilita drivers)
 *  N6  todos los setters validados y acotados; espera de paso con timeout
 *  N7  WCO separado de coordenadas de maquina; soft limits siempre en MPos
 *  N8  '~' solo reanuda (no re-energiza ni borra ALARM); tiempo real inmediato en TCP y serial
 *  N9  limite de velocidad por eje explicito e informado ([MSG:Feed limitado...])
 *  N10 codigos de error distintos (LIMIT / FULL / NOT_HOMED / REJECTED / error:...)
 *  N11 abortos de homing limpian estado; cola de homing del cliente se detiene en ALARM
 *  A7r distancia de calibracion separada del recorrido maximo
 *  M1r parser G-code: comentarios, minusculas, G4, G28, G2/G3 (G17), $J sin espacios
 *  M2  mitigado: tarea de motores con prioridad mayor que loop(); piso de paso configurable
 *  M3  hardware: ver notas de pines al final del encabezado de pines
 *  M4  JOG_FREE encolado en la tarea de motores con limites (no bloquea loop)
 *  M7  compensacion de backlash por eje; cliente con cargador de archivos G-code
 *  M8  version de firmware (GET_VERSION / $I), parametros de red por NVS
 *
 * NOTA HARDWARE (M3): GPIO34 (limite X) requiere pull-up fisico. GPIO4/GPIO15 son
 * pines de strapping: mantener sin carga durante el arranque o reasignar en AxisHW.
 * E-stop fisico: boton NC entre PIN_ESTOP y GND. El ISR corta ENABLE al instante;
 * para corte de potencia real, cablear el E-stop en serie con la alimentacion de drivers.
 */
#include <WiFi.h>
#include <Preferences.h>
#include <math.h>
#include <ctype.h>
#include <stdlib.h>

#define FW_VERSION "5.1.0"

const uint16_t SERVER_PORT = 5000;
const size_t MAX_LINE_LEN = 256;
const uint8_t PIN_ENABLE_ACTUATORS = 27; // LOW = Habilitado
const uint8_t PIN_ESTOP = 32;            // C2: E-stop fisico, NC a GND (INPUT_PULLUP)
const uint32_t MIN_STEP_US = 80;         // M2/N9: piso de periodo de paso

String wifi_ssid    = "";
String wifi_pass    = "";
String wifi_devname = "CNC-XYZW-NETLOG";

uint8_t ip_oct1 = 192;
uint8_t ip_oct2 = 168;
uint8_t ip_oct3 = 1;
uint8_t ip_oct4 = 167;

IPAddress ESP32_GATEWAY(192, 168, 1, 1);
IPAddress ESP32_SUBNET(255, 255, 255, 0);
IPAddress ESP32_DNS1(8, 8, 8, 8);
IPAddress ESP32_DNS2(1, 1, 1, 1);

WiFiServer server(SERVER_PORT);
WiFiClient cl;
Preferences prefs;
bool clientWasConnected = false;
bool serverStarted = false;            // C3: server.begin() tambien en evento GOT_IP

// C3: heartbeat de comunicacion durante programas (Run/Jog)
unsigned long lastRxMs = 0;
const unsigned long COMM_TIMEOUT_MS = 3000;

// C5: autenticacion por token (vacio = modo abierto, configurar por serial)
String authToken = "";
bool clientAuthenticated = false;
// C5: endurecimiento: limite de intentos y cierre de sesion sin autenticar
uint8_t authFailCount = 0;
const uint8_t AUTH_MAX_ATTEMPTS = 5;
unsigned long authGraceStartMs = 0;
const unsigned long AUTH_GRACE_MS = 10000; // 10 s para autenticarse

// C2r: escrituras TCP acotadas (un cliente colgado no bloquea loop())
const unsigned long TCP_WRITE_TIMEOUT_MS = 3000;

// C2: E-stop fisico
volatile bool estopTriggered = false;

// A1r: contador de epoca de stop (un stop recien llegado nunca se pierde)
volatile uint32_t stopEpoch = 0;
volatile bool queuePurgePending = false;

enum AxisId { AXIS_X=0, AXIS_Y=1, AXIS_Z=2, AXIS_W=3, AXIS_COUNT=4 };
const char* AXIS_NAME[AXIS_COUNT] = {"x", "y", "z", "w"};

struct SCurveProfile {
  float startSpeedMmS;
  float cruiseSpeedMmS;
  float endSpeedMmS;
  float rampRatio;
};

struct AxisHW {
  uint8_t pinStep;
  uint8_t pinDir;
  uint8_t pinLimitHome;
  bool dirPositive;
};

struct AxisState {
  float pos = 0.0f;       // Coordenada maquina absoluta (MPos)
  float wco = 0.0f;       // Offset de trabajo (WPos = pos - wco)
  float targetPos = 0.0f;
  float stepsPerMm = 568.0f;
  float maxTravel = 110.0f;
  
  int64_t stepCount = 0;

  uint32_t softLimitOffsetSteps = 2840;
  uint32_t backoffSteps = 1136;

  bool homed = false;
  bool calibrated = false;
  bool firstRun = true;
  bool useSCurve = true;
  uint8_t dirForwardLevel = HIGH;
  char lastCalibration[32] = "Sin datos";

  bool limitHome = false;
  bool isMoving = false;
  char moveDir[12] = "none";

  volatile bool manualForward = false;
  volatile bool manualBackward = false;
  // N19: manualStop queda solo como residuo del JOG libre por serial (JOG_FREE);
  // ya no participa en la ejecucion de bloques ni del homing: los abortos de
  // movimientos encolados los decide stopEpoch (y jogCancelEpoch para jog).
  volatile bool manualStop = false;

  int homingSeekUs = 1200;
  int homingFeedUs = 2800;
  int homingBackoffUs = 1500;
  int manualUs = 1000;
  int jogUs = 1000;

  volatile bool runAbsMoveRequested = false;
  volatile bool runHomeRequested = false;
  volatile bool isJogMove = false;
  volatile float pendingTargetPos = 0.0f;

  // A5/N9: parametros de movimiento reales
  float accelMmS2 = 20.0f;      // aceleracion real del eje
  float maxSpeedMmS = 22.0f;    // limite de velocidad por eje (568 stp/mm @ 80 us ~= 22 mm/s)
  float backlashMm = 0.0f;      // M7: compensacion de backlash

  SCurveProfile scurve = {1.5f, 15.0f, 1.5f, 0.25f};
  char lastError[32] = "None";
};

// M7: direccion previa para compensacion de backlash
bool lastDirKnown[4] = {false, false, false, false};
bool lastDirPos[4] = {true, true, true, true};

// M4: JOG_FREE encolado en la tarea de motores (pasos restantes y periodo)
volatile int32_t jogFreeSteps[4] = {0, 0, 0, 0};
volatile uint32_t jogFreeUs[4] = {1000, 1000, 1000, 1000};

// N5: cancelacion real de jog GRBL (0x85)
volatile uint32_t jogCancelEpoch = 0;

AxisHW hw[AXIS_COUNT] = {
  {23, 22, 34, LOW},   // X (Requiere pull-up fisico en placa para GPIO34)
  {21, 17, 25, LOW},   // Y
  {16, 4,  14, LOW},   // Z
  {13, 15, 18, LOW}    // W
};

AxisState ax[AXIS_COUNT];
float plannedPos[AXIS_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};

enum MachineState { IDLE, HOMING, CALIBRATING, RUNNING, MANUAL, ALARM };
volatile MachineState machineState = IDLE;
volatile bool actuatorsEnabled = false;
volatile bool forceStatusPush = false;

volatile unsigned long lastManualHeartbeatMs = 0;
const unsigned long MANUAL_DEADMAN_TIMEOUT_MS = 400;

int gcode_active_motion_mode = 1; // 0=G0, 1=G1
bool gcode_is_relative = false;       
bool gcode_is_inches = false;         
float gcode_feed_rate_mmpm = 600.0f;  

// A5: bloque con velocidades reales de union (mm/s) calculadas con look-ahead
struct MotionBlock {
  float target[AXIS_COUNT];
  float unitVec[AXIS_COUNT];   // direccion unitaria del bloque (para velocidad de union)
  float distanceMm;            // distancia vectorial del bloque
  float cruiseSpeedMmS;        // velocidad de crucero vectorial (limitada por eje)
  float entrySpeedMmS;         // velocidad de entrada (look-ahead)
  float exitSpeedMmS;          // velocidad de salida (look-ahead)
  float accelMmS2;             // aceleracion del eje dominante
  float rampRatio;
  uint32_t deltaSteps[AXIS_COUNT];
  bool dirPos[AXIS_COUNT];
  uint32_t maxSteps;
  bool active;
  bool isJog;                  // N5: cancelable con 0x85
  bool isDwell;                // M1r: G4
  uint32_t dwellMs;
};

const int BLOCK_QUEUE_SIZE = 16;
MotionBlock blockQueue[BLOCK_QUEUE_SIZE];
volatile int qHead = 0;
volatile int qTail = 0;

// A5: junction deviation estandar (mm), modelo GRBL: vj = sqrt(accel * jd)
const float JUNCTION_DEVIATION_MM = 0.05f;

// N1: posicion planeada = final de la cola. Solo se modifica bajo stateMutex
// (al encolar, al purgar, al ejecutar jog manual o al abortar).

// Resultados de planificacion con codigos distintos (N10)
enum PlanResult { PLAN_OK, PLAN_FULL, PLAN_SOFT_LIMIT, PLAN_NOT_HOMED, PLAN_INVALID };

unsigned long lastTelemetryMs = 0;
const unsigned long TELEMETRY_INTERVAL_MS = 60; 

TaskHandle_t TaskMotorsHandle = NULL;
SemaphoreHandle_t stateMutex = NULL;
hw_timer_t * stepTimer = NULL;

char tcpBuffer[MAX_LINE_LEN];
uint16_t tcpBufIdx = 0;
char serialBuffer[MAX_LINE_LEN];
uint16_t serialBufIdx = 0;

void IRAM_ATTR onStepTimer() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if (TaskMotorsHandle != NULL) {
    vTaskNotifyGiveFromISR(TaskMotorsHandle, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
  }
}

// N6: espera acotada. Con los setters validados (>= MIN_STEP_US) el timeout nunca
// deberia dispararse, pero evita un bloqueo permanente de la tarea de motores.
inline void waitStepHardwareTimer(uint32_t delayUs) {
  if (delayUs < MIN_STEP_US) delayUs = MIN_STEP_US;
  if (delayUs > 20000) delayUs = 20000;
  timerWrite(stepTimer, 0);
  timerAlarm(stepTimer, (uint64_t)delayUs, false, 0);
  uint32_t timeoutTicks = pdMS_TO_TICKS(delayUs / 500 + 10); // ~2x el periodo esperado
  ulTaskNotifyTake(pdTRUE, timeoutTicks);
}

// C2: ISR del E-stop fisico: corta ENABLE de inmediato y marca el evento.
// N20: con boton NC a GND e INPUT_PULLUP, el estado ACTIVO (pulsado o cable
// cortado) es HIGH -> fail-safe. ISR disparado por flanco RISING.
void IRAM_ATTR onEstopISR() {
  digitalWrite(PIN_ENABLE_ACTUATORS, HIGH); // drivers deshabilitados
  actuatorsEnabled = false;
  estopTriggered = true;
}

// N20: boton NC a GND: el E-stop esta activo cuando la linea queda en HIGH
// (pulsado = abre el circuito; cable cortado = mismo estado, fail-safe)
inline bool estopActive() {
  return digitalRead(PIN_ESTOP) == HIGH;
}

void setActuatorsState(bool enable) {
  // C2/N20: no permitir re-energizar mientras el E-stop fisico siga activo
  if (enable && estopActive()) {
    enable = false;
    estopTriggered = true;
  }
  actuatorsEnabled = enable;
  digitalWrite(PIN_ENABLE_ACTUATORS, enable ? LOW : HIGH);
}

inline void setAxisDirection(AxisId a, bool isForward) {
  uint8_t level = isForward ? ax[a].dirForwardLevel : (ax[a].dirForwardLevel == HIGH ? LOW : HIGH);
  digitalWrite(hw[a].pinDir, level);
}

inline float safeSpm(AxisId a) {
  // N6: stepsPerMm=0 daria division por cero
  return (ax[a].stepsPerMm > 0.1f) ? ax[a].stepsPerMm : 568.0f;
}

inline void pulseAxis(AxisId a, bool isForward) {
  if (!actuatorsEnabled) return;
  digitalWrite(hw[a].pinStep, HIGH);
  delayMicroseconds(4);
  digitalWrite(hw[a].pinStep, LOW);

  if (isForward) ax[a].stepCount++;
  else ax[a].stepCount--;

  ax[a].pos = (float)ax[a].stepCount / safeSpm(a);
}

// M7: pasos extra al invertir el sentido para absorber el backlash mecanico.
// No modifican pos ni stepCount (son desplazamiento muerto).
void compensateBacklash(AxisId a, bool newDirPos, uint32_t stepUs) {
  float bl = ax[a].backlashMm;
  if (bl <= 0.0f) { lastDirKnown[a] = true; lastDirPos[a] = newDirPos; return; }
  if (lastDirKnown[a] && lastDirPos[a] == newDirPos) return;
  lastDirKnown[a] = true;
  lastDirPos[a] = newDirPos;
  uint32_t extra = (uint32_t)lroundf(bl * safeSpm(a));
  if (extra == 0 || extra > 20000) return;
  setAxisDirection(a, newDirPos);
  for (uint32_t i = 0; i < extra; i++) {
    if (!actuatorsEnabled || estopTriggered) return;
    digitalWrite(hw[a].pinStep, HIGH);
    delayMicroseconds(4);
    digitalWrite(hw[a].pinStep, LOW);
    waitStepHardwareTimer(stepUs);
  }
}

// N6: NaN/Inf no pasan: un valor no finito nunca esta dentro de limites
inline bool inLim(AxisId a, float v){
  if (!isfinite(v)) return false;
  return !(v < -0.05f || v > (ax[a].maxTravel + 0.05f));
}

void clearManualFlags(AxisId a){
  ax[a].manualForward = false;
  ax[a].manualBackward = false;
  ax[a].manualStop = false; // N19: residuo de JOG_FREE; no debe sobrevivir a un stop
  ax[a].isMoving = false;
  strncpy(ax[a].moveDir, "none", sizeof(ax[a].moveDir) - 1);
  ax[a].moveDir[sizeof(ax[a].moveDir) - 1] = '\0';
}

// N12: toda salida de ALARM deja la maquina en un estado conocido: cola
// vacia, sin solicitudes pendientes de homing/movimiento y plannedPos
// resincronizado con la posicion real. Devuelve false si no se logro la
// purga bajo mutex (quien llama NO debe desbloquear: fail-safe).
static bool clearAlarmState() {
  stopEpoch++;      // invalida cualquier bloque en vuelo
  jogCancelEpoch++; // y cualquier jog en vuelo
  for (int i = 0; i < AXIS_COUNT; i++) {
    clearManualFlags((AxisId)i);
    ax[i].runAbsMoveRequested = false;
    ax[i].runHomeRequested = false;
    jogFreeSteps[i] = 0;
  }
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    qHead = 0;
    qTail = 0;
    for (int i = 0; i < BLOCK_QUEUE_SIZE; i++) blockQueue[i].active = false;
    for (int i = 0; i < AXIS_COUNT; i++) plannedPos[i] = ax[i].pos;
    queuePurgePending = false;
    xSemaphoreGive(stateMutex);
    return true;
  }
  queuePurgePending = true; // TaskMotors completa la purga con el mutex libre
  return false;
}

// A1r: purga de la cola bajo mutex, con reintento; si no se logra, la tarea
// de motores la purga (queuePurgePending). stopEpoch invalida bloques en vuelo.
void stopAllMotion() {
  stopEpoch++;
  jogCancelEpoch++;
  for(int i=0; i<AXIS_COUNT; i++) {
    clearManualFlags((AxisId)i);
    ax[i].runAbsMoveRequested = false;
    ax[i].runHomeRequested = false;
    jogFreeSteps[i] = 0;
  }
  bool purged = false;
  for (int attempt = 0; attempt < 5 && !purged; attempt++) {
    if (stateMutex && xSemaphoreTake(stateMutex, pdMS_TO_TICKS(25)) == pdTRUE) {
      qHead = 0;
      qTail = 0;
      for(int i=0; i<BLOCK_QUEUE_SIZE; i++) blockQueue[i].active = false;
      for(int i=0; i<AXIS_COUNT; i++) plannedPos[i] = ax[i].pos;
      purged = true;
      xSemaphoreGive(stateMutex);
    }
  }
  if (!purged) queuePurgePending = true; // TaskMotors la purga con el mutex libre
  if (machineState != ALARM) machineState = IDLE;
  forceStatusPush = true;
}

void refreshAllInputs(){
  for(int i=0; i<AXIS_COUNT; i++) ax[i].limitHome = (digitalRead(hw[i].pinLimitHome) == HIGH);
}

AxisId parseAxis(const char* s){
  if(!s) return AXIS_X;
  char c = tolower(s[0]);
  if(c=='y') return AXIS_Y;
  if(c=='z') return AXIS_Z;
  if(c=='w' || c=='a') return AXIS_W;
  return AXIS_X;
}

const char* machineStateStr(){
  switch(machineState){
    case IDLE: return "Idle";
    case HOMING: return "Home";
    case CALIBRATING: return "Hold";
    case RUNNING: return "Run";
    case MANUAL: return "Jog";
    case ALARM: return "Alarm";
    default: return "Idle";
  }
}

inline int calculateSCurveDelay(float progress, int cruiseUs, int startUs, int endUs, float rampRatio) {
  if (rampRatio <= 0.01f) return cruiseUs;
  
  if (progress < rampRatio) {
    float p = progress / rampRatio;
    float sFactor = 0.5f * (1.0f - cosf(p * (float)M_PI));
    return (int)(startUs - (startUs - cruiseUs) * sFactor);
  } else if (progress <= (1.0f - rampRatio)) {
    return cruiseUs;
  } else {
    float p = (progress - (1.0f - rampRatio)) / rampRatio;
    float sFactor = 0.5f * (1.0f - cosf(p * (float)M_PI));
    return (int)(cruiseUs + (endUs - cruiseUs) * sFactor);
  }
}

void loadNVS(){
  if (!prefs.begin("cnc_xyzw", false)) return;
  wifi_ssid    = prefs.getString("w_ssid", "");
  wifi_pass    = prefs.getString("w_pass", "");
  wifi_devname = prefs.getString("w_devname", "CNC-XYZW-NETLOG");
  ip_oct4      = prefs.getUChar("ip_oct4", 167);
  authToken    = prefs.getString("w_token", ""); // C5

  for(int i=0; i<AXIS_COUNT; i++){
    String k = String(AXIS_NAME[i]);
    ax[i].firstRun = prefs.getBool((k + "_fr").c_str(), true);
    ax[i].dirForwardLevel = prefs.getUChar((k + "_dfl").c_str(), HIGH);
    // N6: todos los valores cargados se validan/acotan antes de usarse
    ax[i].stepsPerMm = constrain(prefs.getFloat((k + "_spm").c_str(), 568.0f), 0.1f, 100000.0f);
    ax[i].maxTravel = constrain(prefs.getFloat((k + "_max").c_str(), 110.0f), 1.0f, 5000.0f);
    ax[i].calibrated = prefs.getBool((k + "_cal").c_str(), false);
    ax[i].backoffSteps = constrain(prefs.getUInt((k + "_bo_st").c_str(), 1136), 10u, 200000u);
    ax[i].softLimitOffsetSteps = constrain(prefs.getUInt((k + "_so_st").c_str(), 2840), 10u, 200000u);
    ax[i].useSCurve = prefs.getBool((k + "_sc").c_str(), true);
    ax[i].homingSeekUs = constrain(prefs.getInt((k + "_hseek").c_str(), 1200), (int)MIN_STEP_US, 20000);
    ax[i].homingFeedUs = constrain(prefs.getInt((k + "_hfeed").c_str(), 2800), (int)MIN_STEP_US, 20000);
    ax[i].homingBackoffUs = constrain(prefs.getInt((k + "_hbo").c_str(), 1500), (int)MIN_STEP_US, 20000);
    ax[i].manualUs = constrain(prefs.getInt((k + "_man").c_str(), 1000), (int)MIN_STEP_US, 20000);
    ax[i].jogUs = constrain(prefs.getInt((k + "_jog").c_str(), 1000), (int)MIN_STEP_US, 20000);

    ax[i].scurve.startSpeedMmS = constrain(prefs.getFloat((k + "_sc_s").c_str(), 1.5f), 0.1f, 50.0f);
    ax[i].scurve.cruiseSpeedMmS = constrain(prefs.getFloat((k + "_sc_c").c_str(), 15.0f), 0.5f, 100.0f);
    ax[i].scurve.endSpeedMmS = constrain(prefs.getFloat((k + "_sc_e").c_str(), 1.5f), 0.1f, 50.0f);
    ax[i].scurve.rampRatio = constrain(prefs.getFloat((k + "_sc_r").c_str(), 0.25f), 0.05f, 0.45f);

    ax[i].accelMmS2 = constrain(prefs.getFloat((k + "_acc").c_str(), 20.0f), 1.0f, 500.0f);      // A5
    ax[i].maxSpeedMmS = constrain(prefs.getFloat((k + "_vmax").c_str(), 22.0f), 0.5f, 100.0f);    // N9
    ax[i].backlashMm = constrain(prefs.getFloat((k + "_bl").c_str(), 0.0f), 0.0f, 5.0f);          // M7

    String calStr = prefs.getString((k + "_date").c_str(), "Sin datos");
    strncpy(ax[i].lastCalibration, calStr.c_str(), sizeof(ax[i].lastCalibration) - 1);
    ax[i].lastCalibration[sizeof(ax[i].lastCalibration) - 1] = '\0';
    plannedPos[i] = ax[i].pos;
  }
  prefs.end();
}

void saveAxisNVS(AxisId i){
  prefs.begin("cnc_xyzw", false);
  String k = String(AXIS_NAME[i]);
  prefs.putBool((k + "_fr").c_str(), ax[i].firstRun);
  prefs.putUChar((k + "_dfl").c_str(), ax[i].dirForwardLevel);
  prefs.putFloat((k + "_spm").c_str(), ax[i].stepsPerMm);
  prefs.putFloat((k + "_max").c_str(), ax[i].maxTravel);
  prefs.putBool((k + "_cal").c_str(), ax[i].calibrated);
  prefs.putUInt((k + "_bo_st").c_str(), ax[i].backoffSteps);
  prefs.putUInt((k + "_so_st").c_str(), ax[i].softLimitOffsetSteps);
  prefs.putBool((k + "_sc").c_str(), ax[i].useSCurve);
  prefs.putInt((k + "_hseek").c_str(), ax[i].homingSeekUs);
  prefs.putInt((k + "_hfeed").c_str(), ax[i].homingFeedUs);
  prefs.putInt((k + "_hbo").c_str(), ax[i].homingBackoffUs);
  prefs.putInt((k + "_man").c_str(), ax[i].manualUs);
  prefs.putInt((k + "_jog").c_str(), ax[i].jogUs);
  prefs.putFloat((k + "_sc_s").c_str(), ax[i].scurve.startSpeedMmS);
  prefs.putFloat((k + "_sc_c").c_str(), ax[i].scurve.cruiseSpeedMmS);
  prefs.putFloat((k + "_sc_e").c_str(), ax[i].scurve.endSpeedMmS);
  prefs.putFloat((k + "_sc_r").c_str(), ax[i].scurve.rampRatio);
  prefs.putFloat((k + "_acc").c_str(), ax[i].accelMmS2);
  prefs.putFloat((k + "_vmax").c_str(), ax[i].maxSpeedMmS);
  prefs.putFloat((k + "_bl").c_str(), ax[i].backlashMm);
  prefs.putString((k + "_date").c_str(), String(ax[i].lastCalibration));
  prefs.end();
}

void saveNetworkNVS(){
  prefs.begin("cnc_xyzw", false);
  prefs.putString("w_ssid", wifi_ssid);
  prefs.putString("w_pass", wifi_pass);
  prefs.putString("w_devname", wifi_devname);
  prefs.putUChar("ip_oct4", ip_oct4);
  prefs.putString("w_token", authToken);
  prefs.end();
}

// C3: el servidor TCP arranca ante el evento GOT_IP, no solo si hubo WiFi al boot
void onWiFiEvent(WiFiEvent_t event) {
  if (event == ARDUINO_EVENT_WIFI_STA_GOT_IP) {
    if (!serverStarted) {
      server.begin();
      server.setNoDelay(true);
      serverStarted = true;
    }
    Serial.printf("[WIFI] GOT_IP: %s\n", WiFi.localIP().toString().c_str());
  } else if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
    Serial.println("[WIFI] Desconectado");
  }
}

void connectWiFiBlocking(){
  if (wifi_ssid.length() == 0) return;
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(wifi_devname.c_str());
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);
  WiFi.onEvent(onWiFiEvent);
  IPAddress customIp(ip_oct1, ip_oct2, ip_oct3, ip_oct4);
  WiFi.config(customIp, ESP32_GATEWAY, ESP32_SUBNET, ESP32_DNS1, ESP32_DNS2);
  WiFi.begin(wifi_ssid.c_str(), wifi_pass.c_str());
  unsigned long startAtt = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAtt < 8000) delay(200);
  if (WiFi.status() == WL_CONNECTED && !serverStarted) {
    server.begin();
    server.setNoDelay(true);
    serverStarted = true;
    Serial.printf("[WIFI] Conectado. IP: %s\n", WiFi.localIP().toString().c_str());
  }
}

void sendCompactStatus(WiFiClient& c) {
  if (!c || !c.connected()) return;
  refreshAllInputs();
  // N3: profundidad de cola para que el cliente detecte fin de movimiento real
  int qDepth = 0;
  if (stateMutex && xSemaphoreTake(stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    qDepth = (qHead - qTail + BLOCK_QUEUE_SIZE) % BLOCK_QUEUE_SIZE;
    xSemaphoreGive(stateMutex);
  }
  char outBuf[1100];
  int n = snprintf(outBuf, sizeof(outBuf), "ST|%s|%d|%d", machineStateStr(), actuatorsEnabled ? 1 : 0, qDepth);
  for (int i = 0; i < AXIS_COUNT && n < (int)sizeof(outBuf) - 150; i++) {
    n += snprintf(outBuf + n, sizeof(outBuf) - n,
      "|%.3f,%.3f,%lld,%d,%d,%d,%d,%d,%s,%.2f,%.2f,%lu,%lu,%d,%d,%d,%d,%d,%s,%.2f,%.2f,%.2f,%.2f,%s,%d,%.3f,%.2f,%.1f,%.3f",
      ax[i].pos, ax[i].targetPos, (long long)ax[i].stepCount,
      ax[i].homed ? 1 : 0, ax[i].calibrated ? 1 : 0, ax[i].firstRun ? 1 : 0, ax[i].dirForwardLevel,
      ax[i].isMoving ? 1 : 0, ax[i].moveDir, ax[i].stepsPerMm, ax[i].maxTravel,
      (unsigned long)ax[i].backoffSteps, (unsigned long)ax[i].softLimitOffsetSteps, ax[i].useSCurve ? 1 : 0,
      ax[i].manualUs, ax[i].jogUs, ax[i].homingSeekUs, ax[i].homingBackoffUs, ax[i].lastCalibration,
      ax[i].scurve.startSpeedMmS, ax[i].scurve.cruiseSpeedMmS, ax[i].scurve.endSpeedMmS, ax[i].scurve.rampRatio,
      ax[i].lastError,
      // Campos nuevos V5 (24-27): feed de homing, backlash, vel maxima, aceleracion
      ax[i].homingFeedUs, ax[i].backlashMm, ax[i].maxSpeedMmS, ax[i].accelMmS2,
      // Campo nuevo V5.1 (28): WCO (N16: la UI muestra WPos = pos - wco)
      ax[i].wco
    );
  }
  strncat(outBuf, "\n", sizeof(outBuf) - strlen(outBuf) - 1);
  c.print(outBuf);
}

// N11: todos los abortos limpian isMoving/moveDir y resincronizan plannedPos
static void homingAbort(AxisId a, const char* err) {
  ax[a].isMoving = false;
  strncpy(ax[a].moveDir, "none", sizeof(ax[a].moveDir) - 1);
  ax[a].moveDir[sizeof(ax[a].moveDir) - 1] = '\0';
  ax[a].targetPos = ax[a].pos;
  if (stateMutex && xSemaphoreTake(stateMutex, pdMS_TO_TICKS(25)) == pdTRUE) {
    plannedPos[a] = ax[a].pos;
    xSemaphoreGive(stateMutex);
  }
  machineState = ALARM;
  strncpy(ax[a].lastError, err, sizeof(ax[a].lastError) - 1);
  ax[a].lastError[sizeof(ax[a].lastError) - 1] = '\0';
  forceStatusPush = true;
}

bool doHoming(AxisId a) {
  if (!actuatorsEnabled) {
    homingAbort(a, "Actuadores OFF");
    return false;
  }
  uint32_t epoch = stopEpoch; // A1r: un stop posterior a esta captura aborta el homing
  machineState = HOMING;
  ax[a].homed = false;
  ax[a].isMoving = true;
  strncpy(ax[a].moveDir, "backward", sizeof(ax[a].moveDir) - 1);

  setAxisDirection(a, false);
  uint32_t maxSeekSteps = (uint32_t)(ax[a].maxTravel * safeSpm(a) * 1.5f);
  uint32_t stepCounter = 0;
  bool hitSwitch = false;

  while (stepCounter < maxSeekSteps) {
    if (stopEpoch != epoch || !actuatorsEnabled) break;
    if (digitalRead(hw[a].pinLimitHome) == HIGH) {
      hitSwitch = true;
      break;
    }
    pulseAxis(a, false);
    waitStepHardwareTimer(ax[a].homingSeekUs);
    stepCounter++;
  }

  if (!hitSwitch || stopEpoch != epoch || !actuatorsEnabled) {
    homingAbort(a, "Homing Seek Fail");
    return false;
  }

  // Retroceso
  setAxisDirection(a, true);
  strncpy(ax[a].moveDir, "forward", sizeof(ax[a].moveDir) - 1);
  compensateBacklash(a, true, ax[a].homingBackoffUs);
  for (uint32_t i = 0; i < ax[a].backoffSteps; i++) {
    if (stopEpoch != epoch || !actuatorsEnabled) { homingAbort(a, "Homing Abortado"); return false; }
    pulseAxis(a, true);
    waitStepHardwareTimer(ax[a].homingBackoffUs);
  }

  // Pasada Fina
  setAxisDirection(a, false);
  strncpy(ax[a].moveDir, "backward", sizeof(ax[a].moveDir) - 1);
  compensateBacklash(a, false, ax[a].homingFeedUs);
  hitSwitch = false;
  stepCounter = 0;
  while (stepCounter < ax[a].backoffSteps * 2) {
    if (stopEpoch != epoch || !actuatorsEnabled) { homingAbort(a, "Homing Abortado"); return false; }
    if (digitalRead(hw[a].pinLimitHome) == HIGH) {
      hitSwitch = true;
      break;
    }
    pulseAxis(a, false);
    waitStepHardwareTimer(ax[a].homingFeedUs);
    stepCounter++;
  }

  if (!hitSwitch) {
    homingAbort(a, "Homing Feed Miss");
    return false;
  }

  // Soft Limit Offset
  setAxisDirection(a, true);
  strncpy(ax[a].moveDir, "forward", sizeof(ax[a].moveDir) - 1);
  compensateBacklash(a, true, ax[a].homingBackoffUs);
  for (uint32_t i = 0; i < ax[a].softLimitOffsetSteps; i++) {
    if (stopEpoch != epoch || !actuatorsEnabled) { homingAbort(a, "Homing Abortado"); return false; }
    pulseAxis(a, true);
    waitStepHardwareTimer(ax[a].homingBackoffUs);
  }

  ax[a].stepCount = 0;
  ax[a].pos = 0.0f;
  ax[a].wco = 0.0f;
  ax[a].targetPos = 0.0f;
  ax[a].homed = true;
  ax[a].firstRun = false;
  ax[a].isMoving = false;
  if (stateMutex && xSemaphoreTake(stateMutex, pdMS_TO_TICKS(25)) == pdTRUE) {
    plannedPos[a] = 0.0f;
    xSemaphoreGive(stateMutex);
  }
  strncpy(ax[a].moveDir, "none", sizeof(ax[a].moveDir) - 1);
  strncpy(ax[a].lastError, "None", sizeof(ax[a].lastError) - 1);

  saveAxisNVS(a);
  machineState = IDLE;
  forceStatusPush = true;
  return true;
}

bool moveAbsExecution(AxisId a, float target, bool isJog){
  if(!actuatorsEnabled){ strncpy(ax[a].lastError, "Actuadores OFF", sizeof(ax[a].lastError) - 1); return false; }
  // C1r: los soft limits aplican siempre (no solo con homed)
  if(!inLim(a, target)){
    strncpy(ax[a].lastError, "Soft Limit", sizeof(ax[a].lastError) - 1);
    return false;
  }
  uint32_t epoch = stopEpoch;

  float dz = target - ax[a].pos;
  bool dirPos = dz >= 0.0f;

  uint32_t steps = (uint32_t) llabs(lroundf(dz * safeSpm(a)));
  if(!steps) {
    ax[a].pos = target;
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      plannedPos[a] = target;
      xSemaphoreGive(stateMutex);
    }
    return true;
  }

  machineState = RUNNING;
  ax[a].targetPos = target;
  ax[a].isMoving = true;
  strncpy(ax[a].moveDir, dirPos ? "forward" : "backward", sizeof(ax[a].moveDir) - 1);
  setAxisDirection(a, dirPos);
  compensateBacklash(a, dirPos, isJog ? ax[a].jogUs : ax[a].manualUs);

  // N6: periodos ya validados en los setters; re-acotar por seguridad
  int cruiseUs = constrain(isJog ? ax[a].jogUs : ax[a].manualUs, (int)MIN_STEP_US, 20000);
  // A5: velocidades de entrada/salida desde el perfil S (mm/s reales), no feed*3
  float mmPerStep = 1.0f / safeSpm(a);
  int startUs = (int)constrain(mmPerStep / max(0.2f, ax[a].scurve.startSpeedMmS) * 1e6f, (float)cruiseUs, 20000.0f);
  int endUs   = (int)constrain(mmPerStep / max(0.2f, ax[a].scurve.endSpeedMmS)   * 1e6f, (float)cruiseUs, 20000.0f);
  float rampRatio = ax[a].useSCurve ? ax[a].scurve.rampRatio : 0.0f;

  for(uint32_t i=0; i<steps; i++){
    if(stopEpoch != epoch || !actuatorsEnabled) break;
    if(!dirPos && digitalRead(hw[a].pinLimitHome) == HIGH){
      strncpy(ax[a].lastError, "Limit Hit", sizeof(ax[a].lastError) - 1);
      machineState = ALARM;
      break;
    }

    pulseAxis(a, dirPos);

    float progress = (float)i / (float)steps;
    int currentDelayUs = ax[a].useSCurve ?
                         calculateSCurveDelay(progress, cruiseUs, startUs, endUs, rampRatio) :
                         cruiseUs;

    waitStepHardwareTimer(currentDelayUs);
  }

  // N1: pos = stepCount real (pulseAxis lo mantiene); solo se "cierra" al target
  // si el movimiento completo termino sin stop ni alarma.
  if (stopEpoch == epoch && actuatorsEnabled && machineState != ALARM) {
    ax[a].pos = target;
    ax[a].stepCount = lroundf(target * safeSpm(a));
  }

  ax[a].targetPos = ax[a].pos;
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    plannedPos[a] = ax[a].pos;
    xSemaphoreGive(stateMutex);
  }
  ax[a].isMoving = false;
  strncpy(ax[a].moveDir, "none", sizeof(ax[a].moveDir) - 1);
  if (machineState != ALARM) machineState = IDLE;
  forceStatusPush = true;
  return true;
}

// Convierte velocidad vectorial (mm/s) a periodo del paso dominante (us)
static inline int speedToStepUs(float speedMmS, float mmPerDominantStep) {
  if (speedMmS < 0.2f) speedMmS = 0.2f;
  float us = mmPerDominantStep / speedMmS * 1e6f;
  if (us > 20000.0f) us = 20000.0f;
  if (us < (float)MIN_STEP_US) us = (float)MIN_STEP_US;
  return (int)us;
}

bool executeBlock(const MotionBlock& blk) {
  if (!actuatorsEnabled) return false;
  uint32_t epoch = stopEpoch;      // A1r: un stop posterior aborta, nunca se pierde
  uint32_t jogEpoch = jogCancelEpoch; // N5: 0x85 aborta solo bloques jog

  // M1r: G4 dwell (sin movimiento, abortable)
  if (blk.isDwell) {
    machineState = RUNNING;
    uint32_t t0 = millis();
    while (millis() - t0 < blk.dwellMs) {
      if (!actuatorsEnabled || stopEpoch != epoch) break;
      vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (machineState != ALARM) machineState = IDLE;
    forceStatusPush = true;
    return true;
  }

  if (blk.maxSteps == 0) return true;

  machineState = RUNNING;
  long errAccum[AXIS_COUNT] = {0, 0, 0, 0};

  // N19: el aborto lo decide stopEpoch (y jogCancelEpoch en bloques jog);
  // manualStop ya no se consulta aqui. Los comandos recien aceptados tras un
  // stop se ejecutan con normalidad (nada queda "pegado").
  for (int i = 0; i < AXIS_COUNT; i++) {
    setAxisDirection((AxisId)i, blk.dirPos[i]);
    ax[i].targetPos = blk.target[i];
    ax[i].isMoving = (blk.deltaSteps[i] > 0);
    strncpy(ax[i].moveDir, blk.dirPos[i] ? "forward" : "backward", sizeof(ax[i].moveDir) - 1);
    if (blk.deltaSteps[i] > 0) compensateBacklash((AxisId)i, blk.dirPos[i], MIN_STEP_US * 4);
  }

  // A5: perfil con velocidades de union reales (mm/s) del look-ahead
  float mmPerStep = blk.distanceMm / (float)blk.maxSteps;
  int cruiseUs = speedToStepUs(blk.cruiseSpeedMmS, mmPerStep);
  int startUs  = speedToStepUs(blk.entrySpeedMmS,  mmPerStep);
  int endUs    = speedToStepUs(blk.exitSpeedMmS,   mmPerStep);
  float rampRatio = blk.rampRatio;

  bool aborted = false;

  for (uint32_t step = 0; step < blk.maxSteps; step++) {
    if (!actuatorsEnabled || stopEpoch != epoch) { aborted = true; break; }
    if (blk.isJog && jogCancelEpoch != jogEpoch) { aborted = true; break; }

    for (int i = 0; i < AXIS_COUNT; i++) {
      if (blk.deltaSteps[i] > 0 && !blk.dirPos[i] && digitalRead(hw[i].pinLimitHome) == HIGH) {
        strncpy(ax[i].lastError, "Limit Hit", sizeof(ax[i].lastError) - 1);
        machineState = ALARM;
        aborted = true;
        break;
      }
    }
    if (aborted) break;

    for (int i = 0; i < AXIS_COUNT; i++) {
      if (blk.deltaSteps[i] > 0) {
        errAccum[i] += blk.deltaSteps[i];
        if (errAccum[i] >= (long)blk.maxSteps) {
          errAccum[i] -= blk.maxSteps;
          pulseAxis((AxisId)i, blk.dirPos[i]);
        }
      }
    }

    float progress = (float)step / (float)blk.maxSteps;
    int currentFeedUs = calculateSCurveDelay(progress, cruiseUs, startUs, endUs, rampRatio);
    waitStepHardwareTimer(currentFeedUs);
  }

  // N1/N15: al abortar, pos conserva el valor real de stepCount. Al completar,
  // solo se ajusta si el error acumulado es sub-paso (<0.5 pasos); un error
  // mayor indica perdida de pasos y queda visible en la telemetria.
  bool completed = !aborted && actuatorsEnabled && machineState != ALARM;
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(25)) == pdTRUE) {
    for (int i = 0; i < AXIS_COUNT; i++) {
      if (completed && blk.deltaSteps[i] > 0 && fabsf(ax[i].pos - blk.target[i]) < (0.5f / safeSpm((AxisId)i))) {
        ax[i].pos = blk.target[i];
        ax[i].stepCount = lroundf(blk.target[i] * safeSpm((AxisId)i));
      }
      ax[i].targetPos = ax[i].pos;
      ax[i].isMoving = false;
      strncpy(ax[i].moveDir, "none", sizeof(ax[i].moveDir) - 1);
      plannedPos[i] = ax[i].pos; // resincroniza el planificador con la realidad
    }
    xSemaphoreGive(stateMutex);
  }

  if (machineState != ALARM) machineState = IDLE;
  forceStatusPush = true;
  return true;
}

// A1r: extraccion atomica del siguiente bloque (lectura de cola siempre bajo mutex)
static bool popBlock(MotionBlock& out) {
  bool found = false;
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    if (qHead != qTail) {
      out = blockQueue[qTail];
      blockQueue[qTail].active = false;
      qTail = (qTail + 1) % BLOCK_QUEUE_SIZE;
      found = true;
    }
    xSemaphoreGive(stateMutex);
  }
  // N19: un stop anterior (JOG_FREE/manual) no debe abortar bloques aceptados
  // despues; limpiar el residuo ANTES de ejecutar el bloque.
  if (found) {
    for (int i = 0; i < AXIS_COUNT; i++) ax[i].manualStop = false;
  }
  return found;
}

// A1r: extraccion atomica de un move absoluto pendiente
static bool popAbsMove(AxisId& outAxis, float& outTarget, bool& outJog) {
  bool found = false;
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    for (int i = 0; i < AXIS_COUNT; i++) {
      if (ax[i].runAbsMoveRequested) {
        ax[i].runAbsMoveRequested = false;
        outAxis = (AxisId)i;
        outTarget = ax[i].pendingTargetPos;
        outJog = ax[i].isJogMove;
        found = true;
        break;
      }
    }
    xSemaphoreGive(stateMutex);
  }
  return found;
}

// A1r: extraccion atomica de una solicitud de homing
static bool popHomeRequest(AxisId& outAxis) {
  bool found = false;
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    for (int i = 0; i < AXIS_COUNT; i++) {
      if (ax[i].runHomeRequested) {
        ax[i].runHomeRequested = false;
        outAxis = (AxisId)i;
        found = true;
        break;
      }
    }
    xSemaphoreGive(stateMutex);
  }
  return found;
}

void TaskMotors(void * pvParameters) {
  for(;;) {
    // A1r: purga diferida si stopAllMotion no pudo tomar el mutex
    if (queuePurgePending) {
      if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        qHead = 0; qTail = 0;
        for (int i = 0; i < BLOCK_QUEUE_SIZE; i++) blockQueue[i].active = false;
        for (int i = 0; i < AXIS_COUNT; i++) plannedPos[i] = ax[i].pos;
        queuePurgePending = false;
        xSemaphoreGive(stateMutex);
      }
    }

    if (actuatorsEnabled && machineState != ALARM) {
      if (machineState == MANUAL) {
        if (millis() - lastManualHeartbeatMs > MANUAL_DEADMAN_TIMEOUT_MS) {
          for (int i = 0; i < AXIS_COUNT; i++) clearManualFlags((AxisId)i);
          machineState = IDLE;
          forceStatusPush = true;
        } else {
          for (int i = 0; i < AXIS_COUNT; i++) {
            if (ax[i].manualForward || ax[i].manualBackward) {
              bool isFwd = ax[i].manualForward;

              // C1r: limites siempre activos, con o sin referencia
              if (!isFwd && ax[i].pos <= 0.001f) {
                clearManualFlags((AxisId)i);
                strncpy(ax[i].lastError, "Soft Limit Min", sizeof(ax[i].lastError) - 1);
                machineState = IDLE;
                forceStatusPush = true;
                continue;
              }

              if (isFwd && ax[i].pos >= ax[i].maxTravel) {
                clearManualFlags((AxisId)i);
                strncpy(ax[i].lastError, "Soft Limit Max", sizeof(ax[i].lastError) - 1);
                machineState = IDLE;
                forceStatusPush = true;
                continue;
              }

              setAxisDirection((AxisId)i, isFwd);

              if (!isFwd && digitalRead(hw[i].pinLimitHome) == HIGH) {
                clearManualFlags((AxisId)i);
                strncpy(ax[i].lastError, "Limit Hit", sizeof(ax[i].lastError) - 1);
                machineState = ALARM;
                forceStatusPush = true;
              } else {
                compensateBacklash((AxisId)i, isFwd, ax[i].manualUs);
                pulseAxis((AxisId)i, isFwd);
                if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                  plannedPos[i] = ax[i].pos;
                  xSemaphoreGive(stateMutex);
                }
                waitStepHardwareTimer(ax[i].manualUs);
              }
            }
          }
        }
      }

      // M4: JOG_FREE encolado: pasos con limite, sin bloquear loop()
      for (int i = 0; i < AXIS_COUNT; i++) {
        if (jogFreeSteps[i] != 0 && machineState != ALARM) {
          bool isFwd = jogFreeSteps[i] > 0;
          if (isFwd && ax[i].pos >= ax[i].maxTravel) { jogFreeSteps[i] = 0; continue; }
          if (!isFwd && ax[i].pos <= 0.001f) { jogFreeSteps[i] = 0; continue; }
          if (!isFwd && digitalRead(hw[i].pinLimitHome) == HIGH) {
            jogFreeSteps[i] = 0;
            strncpy(ax[i].lastError, "Limit Hit", sizeof(ax[i].lastError) - 1);
            machineState = ALARM;
            forceStatusPush = true;
            continue;
          }
          setAxisDirection((AxisId)i, isFwd);
          compensateBacklash((AxisId)i, isFwd, jogFreeUs[i]);
          pulseAxis((AxisId)i, isFwd);
          jogFreeSteps[i] += isFwd ? -1 : 1;
          if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            plannedPos[i] = ax[i].pos;
            xSemaphoreGive(stateMutex);
          }
          ax[i].isMoving = (jogFreeSteps[i] != 0);
          strncpy(ax[i].moveDir, isFwd ? "forward" : "backward", sizeof(ax[i].moveDir) - 1);
          if (jogFreeSteps[i] == 0) {
            ax[i].isMoving = false;
            strncpy(ax[i].moveDir, "none", sizeof(ax[i].moveDir) - 1);
          }
          waitStepHardwareTimer(jogFreeUs[i]);
        }
      }

      AxisId hAxis;
      if (popHomeRequest(hAxis)) {
        doHoming(hAxis);
      }

      if (machineState == IDLE) {
        MotionBlock blk;
        if (popBlock(blk)) {
          executeBlock(blk);
        } else {
          AxisId mAxis; float mTgt; bool mJog;
          if (popAbsMove(mAxis, mTgt, mJog)) {
            moveAbsExecution(mAxis, mTgt, mJog);
          }
        }
      }
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

// ============================================================================
// PLANIFICADOR VECTORIAL CON LOOK-AHEAD (A5)
// - N1: planifica desde plannedPos (final de la cola), no desde la pos actual.
// - N9: limita el feed por la velocidad maxima de cada eje e informa el recorte.
// - A5: velocidades de entrada/salida por union con junction deviation estandar
//       (modelo GRBL: vj = sqrt(accel * jd * sin(angulo/2) / (1-sin(angulo/2)))),
//       pasada hacia adelante (limita la entrada segun el bloque previo, incluido
//       el que ya esta en la cola listo para ejecutarse) y pasada hacia atras
//       (garantiza el frenado hasta la entrada del siguiente bloque).
// ============================================================================

// Velocidad maxima de union entre dos bloques segun el angulo entre ellos.
// Modelo junction deviation continuo (GRBL): sin la discontinuidad que el
// modelo ad-hoc anterior presentaba cerca de 5.7 grados.
static float junctionSpeed(const float u1[AXIS_COUNT], const float u2[AXIS_COUNT],
                           float v1, float v2, float accelMmS2) {
  float dot = 0.0f;
  for (int i = 0; i < AXIS_COUNT; i++) dot += u1[i] * u2[i];
  if (dot > 1.0f) dot = 1.0f;
  if (dot < -1.0f) dot = -1.0f;
  if (dot >= 0.9999f) return min(v1, v2);     // colineal: sin reduccion
  if (dot <= 0.0f) return 0.0f;               // >=90 grados: detener en la esquina
  float sinHalf = sqrtf((1.0f - dot) * 0.5f);
  float vj = sqrtf(accelMmS2 * JUNCTION_DEVIATION_MM * sinHalf / (1.0f - sinHalf));
  return min(min(v1, v2), vj);
}

// A5: pasada hacia adelante. La entrada del primer bloque encolado no puede
// superar lo que su distancia permite alcanzar desde la velocidad de salida del
// bloque que se esta ejecutando (o esta a punto de ejecutarse) en la cola.
// Debe llamarse con stateMutex tomado.
static void plannerForwardPass() {
  int n = (qHead - qTail + BLOCK_QUEUE_SIZE) % BLOCK_QUEUE_SIZE;
  if (n == 0) return;
  int prev = qTail;
  for (int k = 1; k < n; k++) {
    int idx = (qTail + k) % BLOCK_QUEUE_SIZE;
    MotionBlock& prevB = blockQueue[prev];
    MotionBlock& b = blockQueue[idx];
    if (!b.active) break;
    if (prevB.active && !prevB.isDwell && !b.isDwell) {
      float maxEntry = sqrtf(prevB.exitSpeedMmS * prevB.exitSpeedMmS +
                             2.0f * b.accelMmS2 * b.distanceMm);
      if (b.entrySpeedMmS > maxEntry) b.entrySpeedMmS = maxEntry;
      if (b.entrySpeedMmS > b.cruiseSpeedMmS) b.entrySpeedMmS = b.cruiseSpeedMmS;
    }
    prev = idx;
  }
}

// Pasada hacia atras sobre la cola: la velocidad de entrada de cada bloque debe
// permitir frenar hasta la velocidad de entrada del siguiente (v^2 = v0^2 + 2ad).
// A1r/A5: la pasada se detiene ANTES de la cabeza (qTail): ese bloque puede
// estar ya en ejecucion (popBlock trabaja sobre una copia) y no debe
// modificarse. La coherencia de la union con la cabeza la garantiza la pasada
// hacia adelante (plannerForwardPass), que si limita la entrada de cada bloque
// nuevo segun la salida del que esta en la cola.
// Debe llamarse con stateMutex tomado.
static void plannerBackwardPass() {
  int idx = (qHead - 1 + BLOCK_QUEUE_SIZE) % BLOCK_QUEUE_SIZE;
  int next = -1;
  while (idx != qTail) {
    MotionBlock& b = blockQueue[idx];
    if (!b.active) break;
    if (next >= 0) {
      MotionBlock& nb = blockQueue[next];
      if (!b.isDwell && !nb.isDwell) {
        float maxEntry = sqrtf(nb.entrySpeedMmS * nb.entrySpeedMmS +
                               2.0f * b.accelMmS2 * b.distanceMm);
        if (b.exitSpeedMmS > nb.entrySpeedMmS) b.exitSpeedMmS = nb.entrySpeedMmS;
        if (b.entrySpeedMmS > maxEntry) b.entrySpeedMmS = maxEntry;
        if (b.entrySpeedMmS > b.cruiseSpeedMmS) b.entrySpeedMmS = b.cruiseSpeedMmS;
      }
    }
    next = idx;
    idx = (idx - 1 + BLOCK_QUEUE_SIZE) % BLOCK_QUEUE_SIZE;
  }
}

// N1/N9/N10: valida limites, calcula geometria desde plannedPos, limita feed por
// eje y encola. Devuelve codigo de resultado distinto por causa.
PlanResult planAndEnqueueBlock(float target[AXIS_COUNT], float feedMmPm,
                               bool isJog, bool isDwell, uint32_t dwellMs,
                               bool* feedLimited) {
  if (feedLimited) *feedLimited = false;

  if (!isDwell) {
    for (int i = 0; i < AXIS_COUNT; i++) {
      // C1r: limites siempre; movimiento bloqueado si el eje no esta referenciado
      if (!inLim((AxisId)i, target[i])) {
        strncpy(ax[i].lastError, "Soft Limit Target", sizeof(ax[i].lastError) - 1);
        return PLAN_SOFT_LIMIT;
      }
      if (!ax[i].homed && fabsf(target[i] - plannedPos[i]) > 0.0005f) {
        strncpy(ax[i].lastError, "Not Homed", sizeof(ax[i].lastError) - 1);
        return PLAN_NOT_HOMED;
      }
    }
  }

  // N15/A1r: los deltas se calculan desde plannedPos (final de la cola) bajo
  // mutex; con la cola vacia plannedPos == pos real. Asi los movimientos
  // encolados nunca parten de una posicion intermedia obsoleta.
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(25)) != pdTRUE) return PLAN_FULL;

  int nextHead = (qHead + 1) % BLOCK_QUEUE_SIZE;
  if (nextHead == qTail) {
    xSemaphoreGive(stateMutex);
    return PLAN_FULL;
  }

  MotionBlock& blk = blockQueue[qHead];
  memset(&blk, 0, sizeof(MotionBlock));
  blk.isJog = isJog;
  blk.isDwell = isDwell;
  blk.dwellMs = dwellMs;
  blk.rampRatio = 0.25f;

  if (isDwell) {
    blk.active = true;
    qHead = nextHead;
    xSemaphoreGive(stateMutex);
    return PLAN_OK;
  }

  if (feedMmPm <= 1.0f) feedMmPm = 100.0f;
  float feedMmPs = feedMmPm / 60.0f;

  // Geometria desde el FINAL DE LA COLA (N1)
  float sumSq = 0.0f;
  blk.maxSteps = 0;
  int dominantAxis = 0;
  float dominantDeltaMm = 0.0f;

  for (int i = 0; i < AXIS_COUNT; i++) {
    float d = target[i] - plannedPos[i];
    blk.dirPos[i] = (d >= 0.0f);
    blk.deltaSteps[i] = (uint32_t)llabs(lroundf(d * safeSpm((AxisId)i)));
    if (blk.deltaSteps[i] > blk.maxSteps) {
      blk.maxSteps = blk.deltaSteps[i];
      blk.rampRatio = ax[i].scurve.rampRatio;
      dominantAxis = i;
      dominantDeltaMm = fabsf(d);
    }
    sumSq += d * d;
    blk.target[i] = target[i];
  }

  blk.distanceMm = sqrtf(sumSq);
  blk.accelMmS2 = ax[dominantAxis].accelMmS2;

  if (blk.distanceMm < 0.001f || blk.maxSteps == 0) {
    // Movimiento nulo: solo actualiza la posicion planeada
    for (int i = 0; i < AXIS_COUNT; i++) plannedPos[i] = target[i];
    xSemaphoreGive(stateMutex);
    return PLAN_OK;
  }

  for (int i = 0; i < AXIS_COUNT; i++) blk.unitVec[i] = (target[i] - plannedPos[i]) / blk.distanceMm;

  // N9: limite de velocidad por eje. La velocidad del eje i es
  // v_vector * |d_i|/dist; debe quedar <= maxSpeedMmS[i] y <= cruise del perfil S.
  float maxVectorSpeed = feedMmPs;
  for (int i = 0; i < AXIS_COUNT; i++) {
    float share = fabsf(target[i] - plannedPos[i]) / blk.distanceMm;
    if (share < 1e-6f) continue;
    float axisCap = min(ax[i].maxSpeedMmS, ax[i].scurve.cruiseSpeedMmS);
    float cap = axisCap / share;
    if (cap < maxVectorSpeed) {
      maxVectorSpeed = cap;
      if (feedLimited) *feedLimited = true;
    }
  }
  blk.cruiseSpeedMmS = maxVectorSpeed;

  // A5: velocidad de entrada segun la union con el bloque anterior de la cola
  int prevIdx = (qHead - 1 + BLOCK_QUEUE_SIZE) % BLOCK_QUEUE_SIZE;
  if (prevIdx != qTail && blockQueue[prevIdx].active && !blockQueue[prevIdx].isDwell) {
    MotionBlock& prev = blockQueue[prevIdx];
    float vj = junctionSpeed(prev.unitVec, blk.unitVec, prev.cruiseSpeedMmS,
                             blk.cruiseSpeedMmS, blk.accelMmS2);
    blk.entrySpeedMmS = min(vj, blk.cruiseSpeedMmS);
    prev.exitSpeedMmS = blk.entrySpeedMmS; // la union manda en ambos extremos
  } else {
    blk.entrySpeedMmS = min(ax[dominantAxis].scurve.startSpeedMmS, blk.cruiseSpeedMmS);
  }
  // Velocidad de salida provisional: la del perfil S (fin de programa)
  blk.exitSpeedMmS = min(ax[dominantAxis].scurve.endSpeedMmS, blk.cruiseSpeedMmS);

  blk.active = true;
  for (int i = 0; i < AXIS_COUNT; i++) plannedPos[i] = target[i];
  qHead = nextHead;

  plannerBackwardPass(); // A5: garantiza el frenado hasta el siguiente bloque
  plannerForwardPass();  // A5: limita la entrada del nuevo bloque tras la pasada atras
  xSemaphoreGive(stateMutex);
  return PLAN_OK;
}

// Enqueue de dwell G4 (M1r)
PlanResult planDwell(uint32_t dwellMs) {
  float dummy[AXIS_COUNT] = {0, 0, 0, 0};
  return planAndEnqueueBlock(dummy, 100.0f, false, true, dwellMs, NULL);
}

// N22/N25: espera de hueco en la cola (control de flujo estilo GRBL: el "ok" se
// difiere hasta que el bloque entra). Mientras espera NO se bloquea el sistema:
// se atienden los comandos de tiempo real y se mantiene la telemetria viva.
static bool waitQueueSpace(WiFiClient& client, uint32_t timeoutMs) {
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    bool space = false;
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      space = ((qHead + 1) % BLOCK_QUEUE_SIZE) != qTail;
      xSemaphoreGive(stateMutex);
    }
    if (space) return true;
    while (client.available() > 0) {
      char ch = (char)client.peek();
      if (ch == '!') { client.read(); stopAllMotion(); setActuatorsState(false); }
      else if (ch == (char)0x18) { client.read(); stopAllMotion(); }
      else if (ch == (char)0x85) { client.read(); jogCancelEpoch++; }
      // N25: '?' responde incluso durante la espera de hueco (arcos)
      else if (ch == '?') {
        client.read();
        if (authToken.length() == 0 || clientAuthenticated) {
          client.printf("<%s|MPos:%.3f,%.3f,%.3f,%.3f|WPos:%.3f,%.3f,%.3f,%.3f|FS:%.0f,0>\r\n",
            machineStateStr(),
            ax[0].pos, ax[1].pos, ax[2].pos, ax[3].pos,
            ax[0].pos - ax[0].wco, ax[1].pos - ax[1].wco, ax[2].pos - ax[2].wco, ax[3].pos - ax[3].wco,
            gcode_feed_rate_mmpm);
        }
      }
      else break;
    }
    if (machineState == ALARM) return false;
    refreshAllInputs();
    if (millis() - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
      lastTelemetryMs = millis();
      sendCompactStatus(client); // N25: la UI sigue viva durante arcos largos
    }
    delay(2);
  }
  return false;
}

// M1r: escaner de palabras letra+numero, sin exigir espacios entre palabras
static bool nextWord(char*& p, char& letter, float& val) {
  while (*p == ' ' || *p == '\t') p++;
  if (!*p || !isalpha((unsigned char)*p)) return false;
  letter = *p++;
  char* endPtr = p;
  val = strtof(p, &endPtr);
  p = endPtr;
  return true;
}

// A1r: copia de plannedPos bajo mutex (posicion planeada = final de la cola)
static void snapshotPlanned(float dst[AXIS_COUNT]) {
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    for (int i = 0; i < AXIS_COUNT; i++) dst[i] = plannedPos[i];
    xSemaphoreGive(stateMutex);
  } else {
    for (int i = 0; i < AXIS_COUNT; i++) dst[i] = ax[i].pos;
  }
}

// N10: respuestas con causa distinta
static void printPlanResult(WiFiClient& client, PlanResult r, bool feedLimited) {
  switch (r) {
    case PLAN_OK:
      if (feedLimited) client.print("[MSG:Feed limitado por velocidad de eje]\r\n");
      client.print("ok\r\n");
      break;
    case PLAN_FULL:       client.print("error:Queue full\r\n");   break;
    case PLAN_SOFT_LIMIT: client.print("error:Soft limit\r\n");   break;
    case PLAN_NOT_HOMED:  client.print("error:Not homed\r\n");    break;
    default:              client.print("error:Invalid command\r\n"); break;
  }
}

// Pre-valida los extremos de un arco (inicio, fin y puntos cardinales) contra
// los soft limits antes de encolar el primer segmento.
static bool arcWithinLimits(float cx, float cy, float r, float a0, float sweep,
                            const float target[AXIS_COUNT]) {
  float pts[6][2];
  pts[0][0] = cx + r * cosf(a0);            pts[0][1] = cy + r * sinf(a0);
  pts[1][0] = cx + r * cosf(a0 + sweep);    pts[1][1] = cy + r * sinf(a0 + sweep);
  int n = 2;
  for (int k = 0; k < 4; k++) {
    float card = (float)k * (float)M_PI * 0.5f;
    float rel = card - a0;
    while (rel < 0) rel += 2.0f * (float)M_PI;
    while (rel >= 2.0f * (float)M_PI) rel -= 2.0f * (float)M_PI;
    bool inSweep = (sweep >= 0) ? (rel <= sweep) : (rel >= 2.0f * (float)M_PI + sweep);
    if (inSweep && n < 6) {
      pts[n][0] = cx + r * cosf(card);
      pts[n][1] = cy + r * sinf(card);
      n++;
    }
  }
  for (int k = 0; k < n; k++) {
    if (!inLim(AXIS_X, pts[k][0]) || !inLim(AXIS_Y, pts[k][1])) return false;
  }
  return inLim(AXIS_Z, target[2]) && inLim(AXIS_W, target[3]);
}

bool processGrblGCode(WiFiClient& client, char* rawLine) {
  // M1r: copia de trabajo sin comentarios (; y (...)) y en mayusculas
  char line[MAX_LINE_LEN];
  {
    size_t o = 0;
    bool inParen = false;
    for (size_t k = 0; rawLine[k] && o < MAX_LINE_LEN - 1; k++) {
      char c = rawLine[k];
      if (inParen) { if (c == ')') inParen = false; continue; }
      if (c == '(') { inParen = true; continue; }
      if (c == ';') break;
      line[o++] = (char)toupper((unsigned char)c);
    }
    while (o > 0 && (line[o - 1] == ' ' || line[o - 1] == '\t')) o--;
    line[o] = '\0';
  }
  if (line[0] == '\0' || line[0] == '%') { client.print("ok\r\n"); return true; }

  bool looksGCode = (line[0] == 'G' || line[0] == 'M' || line[0] == 'T' ||
                     line[0] == 'S' || line[0] == 'F' || line[0] == 'X' ||
                     line[0] == 'Y' || line[0] == 'Z' || line[0] == 'W' ||
                     line[0] == 'A' || line[0] == '$');
  if (!looksGCode) return false;

  // ---------------- Comandos $ ----------------
  if (line[0] == '$') {
    if (line[1] == '$') {
      client.printf("$100=%.3f\r\n$101=%.3f\r\n$102=%.3f\r\n$103=%.3f\r\n", ax[0].stepsPerMm, ax[1].stepsPerMm, ax[2].stepsPerMm, ax[3].stepsPerMm);
      client.printf("$110=%.2f\r\n$111=%.2f\r\n$112=%.2f\r\n$113=%.2f\r\n", ax[0].scurve.cruiseSpeedMmS*60, ax[1].scurve.cruiseSpeedMmS*60, ax[2].scurve.cruiseSpeedMmS*60, ax[3].scurve.cruiseSpeedMmS*60);
      client.printf("$130=%.2f\r\n$131=%.2f\r\n$132=%.2f\r\n$133=%.2f\r\n", ax[0].maxTravel, ax[1].maxTravel, ax[2].maxTravel, ax[3].maxTravel);
      client.printf("$120=%.1f\r\n$121=%.1f\r\n$122=%.1f\r\n$123=%.1f\r\nok\r\n", ax[0].accelMmS2, ax[1].accelMmS2, ax[2].accelMmS2, ax[3].accelMmS2);
      return true;
    }
    if (line[1] == 'I') {
      client.printf("[VER:CNC-XYZW %s]\r\n[OPT:V,4 ejes]\r\nok\r\n", FW_VERSION);
      return true;
    }
    if (line[1] == 'H') {
      if (!actuatorsEnabled) { client.print("error:Actuators off\r\n"); return true; }
      for (int i = 0; i < AXIS_COUNT; i++) ax[i].runHomeRequested = true;
      client.print("ok\r\n");
      return true;
    }
    if (line[1] == 'X') {
      // N8/N20: $X desbloquea ALARM solo si el E-stop fisico esta liberado (NC: HIGH)
      if (estopActive()) {
        client.print("error:E-stop activo\r\n");
        return true;
      }
      // N12: salir de ALARM purga cola y solicitudes pendientes y resincroniza
      // plannedPos; si la purga no se logra, no se desbloquea (fail-safe).
      if (!clearAlarmState()) {
        client.print("error:Unlock failed (busy)\r\n");
        return true;
      }
      setActuatorsState(true);
      for (int i = 0; i < AXIS_COUNT; i++) strncpy(ax[i].lastError, "None", sizeof(ax[i].lastError) - 1);
      if (machineState == ALARM) machineState = IDLE;
      client.print("[MSG:Caution: Unlocked]\r\nok\r\n");
      return true;
    }
    if (strncmp(line, "$J=", 3) == 0) {
      // N5: jog real. Se planifica como bloque cancelable (0x85) y los limites
      // se validan en el planificador (error:Soft limit si se excede).
      // N14: el $J absoluto se referencia a coordenadas de TRABAJO (aplica wco),
      // igual que G1 en modo absoluto.
      float snap[AXIS_COUNT];
      snapshotPlanned(snap);
      float target[AXIS_COUNT] = {snap[0], snap[1], snap[2], snap[3]};
      bool jog_rel = gcode_is_relative;
      bool jog_inches = gcode_is_inches;
      float feed = gcode_feed_rate_mmpm;

      char* p = line + 3;
      char w; float val;
      while (nextWord(p, w, val)) {
        if (w == 'G') {
          int g = (int)val;
          if (g == 90) jog_rel = false;
          else if (g == 91) jog_rel = true;
          else if (g == 20) jog_inches = true;
          else if (g == 21) jog_inches = false;
          else if (g == 53) { /* movimiento en MPos: aceptado (N25) */ }
        } else if (w == 'F') {
          feed = val;
        } else {
          int ai = (w == 'X') ? 0 : (w == 'Y') ? 1 : (w == 'Z') ? 2 : 3;
          float mm = jog_inches ? val * 25.4f : val;
          // N14: absoluto en coordenadas de trabajo (como G1), relativo desde el
          // final de la cola
          target[ai] = jog_rel ? (snap[ai] + mm) : (mm + ax[ai].wco);
        }
      }
      if (!actuatorsEnabled) { client.print("error:Actuators off\r\n"); return true; }
      bool limited = false;
      PlanResult pr = planAndEnqueueBlock(target, feed, true, false, 0, &limited);
      // N22: si la cola esta llena, el ok se difiere hasta que haya hueco
      while (pr == PLAN_FULL) {
        if (!waitQueueSpace(client, 30000)) {
          client.print("error:Queue full\r\n");
          return true;
        }
        pr = planAndEnqueueBlock(target, feed, true, false, 0, &limited);
      }
      printPlanResult(client, pr, limited);
      return true;
    }
    client.print("error:Unsupported command\r\n");
    return true;
  }

  // ---------------- Linea G/M ----------------
  float snap[AXIS_COUNT];
  snapshotPlanned(snap);
  float target[AXIS_COUNT] = {snap[0], snap[1], snap[2], snap[3]};
  int motionMode = -1;          // 0,1,2,3 o -1 (modal)
  bool axisWordSeen = false;
  bool wcoLine = false;         // G10/G92
  bool dwellLine = false;       // G4
  bool g28Line = false;         // G28
  bool g28AxisSeen[AXIS_COUNT] = {false, false, false, false};
  float dwellSec = -1.0f;
  bool hasI = false, hasJ = false;
  float iVal = 0.0f, jVal = 0.0f;
  bool badLine = false;
  bool g53Line = false;         // G53: este movimiento ignora el WCO (MPos)
  bool lWordSeen = false;       // G10/G92 con palabra L
  int lWord = -1;               // valor de L (-1 = ausente)

  char* p = line;
  char w; float val;
  while (nextWord(p, w, val)) {
    switch (w) {
      case 'G': {
        int g = (int)val;
        if (g == 0 || g == 1 || g == 2 || g == 3) { motionMode = g; gcode_active_motion_mode = g; }
        else if (g == 4) dwellLine = true;
        else if (g == 10 || g == 92) wcoLine = true;
        else if (g == 28) g28Line = true;
        else if (g == 90) gcode_is_relative = false;
        else if (g == 91) gcode_is_relative = true;
        else if (g == 20) gcode_is_inches = true;
        else if (g == 21) gcode_is_inches = false;
        else if (g == 17) { /* unico plano soportado */ }
        else if (g == 53) g53Line = true; // M1r/N25: este movimiento en MPos
        else if (g == 18 || g == 19) { client.print("error:Unsupported command\r\n"); return true; }
        break;
      }
      case 'M': {
        int m = (int)val;
        if (m == 2 || m == 30 || m == 3 || m == 5 || m == 0) { /* no-op aceptado */ }
        else { client.print("error:Unsupported command\r\n"); return true; }
        break;
      }
      case 'X': case 'Y': case 'Z': case 'W': case 'A': {
        int ai = (w == 'X') ? 0 : (w == 'Y') ? 1 : (w == 'Z') ? 2 : 3;
        float mm = gcode_is_inches ? val * 25.4f : val;
        if (wcoLine) {
          // N7/N13: WCO separado y fijado contra el FINAL DE LA COLA
          // (plannedPos), no contra la posicion en vivo: con bloques pendientes
          // el cero de trabajo queda donde el programa realmente termina.
          ax[ai].wco = plannedPos[ai] - mm;
        } else if (g28Line) {
          g28AxisSeen[ai] = true; // G28: solo marca el eje (punto intermedio ignorado)
        } else {
          target[ai] = gcode_is_relative ? (snap[ai] + mm)
                       : (g53Line ? mm : (mm + ax[ai].wco)); // G53: MPos directo
          axisWordSeen = true;
        }
        break;
      }
      case 'I': iVal = gcode_is_inches ? val * 25.4f : val; hasI = true; break;
      case 'J': jVal = gcode_is_inches ? val * 25.4f : val; hasJ = true; break;
      case 'P': dwellSec = val; break;
      case 'L': lWordSeen = true; lWord = (int)val; break; // G10/G92 L-word
      case 'F': gcode_feed_rate_mmpm = val; break;
      case 'T': case 'S': break; // herramienta/spindle: aceptados sin efecto
      case 'N': break;           // numero de linea: ignorado
      default: badLine = true; break;
    }
    if (badLine) break;
  }

  if (badLine) { client.print("error:Invalid gcode\r\n"); return true; }

  if (wcoLine) {
    // N6/N13: solo se soporta fijar el cero de trabajo actual:
    //   - G92 (sin L) o G92 L0/L20
    //   - G10 L20 (P se ignora: un solo sistema de trabajo)
    // Cualquier otro L (L1/L2: tool offset / offset de sistema de coordenadas)
    // se RECHAZA en lugar de aplicar una semantica distinta a la pedida.
    if (lWordSeen && lWord != 0 && lWord != 20) {
      client.print("error:Unsupported command\r\n");
      return true;
    }
    forceStatusPush = true;
    client.print("ok\r\n");
    return true;
  }

  if (dwellLine) {
    if (dwellSec < 0.0f || !isfinite(dwellSec)) { client.print("error:Invalid gcode\r\n"); return true; }
    uint32_t ms = (uint32_t)constrain(dwellSec * 1000.0f, 0.0f, 60000.0f);
    PlanResult pr = planDwell(ms);
    // N22: ok diferido hasta que haya hueco en la cola
    while (pr == PLAN_FULL) {
      if (!waitQueueSpace(client, 30000)) { client.print("error:Queue full\r\n"); return true; }
      pr = planDwell(ms);
    }
    printPlanResult(client, pr, false);
    return true;
  }

  if (g28Line) {
    for (int i = 0; i < AXIS_COUNT; i++) {
      if (!ax[i].homed) { client.print("error:Not homed\r\n"); return true; }
      bool any = g28AxisSeen[0] || g28AxisSeen[1] || g28AxisSeen[2] || g28AxisSeen[3];
      if (!any || g28AxisSeen[i]) target[i] = 0.0f; // ir al cero de maquina
    }
    bool limited = false;
    PlanResult pr = planAndEnqueueBlock(target, 100000.0f, false, false, 0, &limited);
    while (pr == PLAN_FULL) { // N22: ok diferido hasta que haya hueco
      if (!waitQueueSpace(client, 30000)) { client.print("error:Queue full\r\n"); return true; }
      pr = planAndEnqueueBlock(target, 100000.0f, false, false, 0, &limited);
    }
    printPlanResult(client, pr, false); // G0/G28: el recorte a velocidad de eje es normal
    return true;
  }

  bool hasMotion = (motionMode >= 0) || axisWordSeen;
  if (!hasMotion) { client.print("ok\r\n"); return true; }

  if (motionMode < 0) motionMode = gcode_active_motion_mode;

  if (motionMode == 2 || motionMode == 3) {
    // M1r: arcos G2/G3 en plano G17 (XY), formato centro I/J, con Z/W helicoidal
    if (!hasI && !hasJ) { client.print("error:Invalid arc\r\n"); return true; }
    float cx = snap[0] + iVal, cy = snap[1] + jVal;
    float r = hypotf(iVal, jVal);
    if (r < 0.01f) { client.print("error:Invalid arc\r\n"); return true; }
    float a0 = atan2f(snap[1] - cy, snap[0] - cx);
    bool full = (fabsf(target[0] - snap[0]) < 0.001f && fabsf(target[1] - snap[1]) < 0.001f);
    float sweep;
    if (full) {
      sweep = (motionMode == 2) ? -2.0f * (float)M_PI : 2.0f * (float)M_PI;
    } else {
      // N25: validacion del error de radio como GRBL (tolerancia 0.5 mm y 0.1%)
      float rEnd = hypotf(target[0] - cx, target[1] - cy);
      if (fabsf(rEnd - r) > 0.5f && fabsf(rEnd - r) > 0.001f * r) {
        client.print("error:Invalid arc\r\n");
        return true;
      }
      float a1 = atan2f(target[1] - cy, target[0] - cx);
      sweep = a1 - a0;
      if (motionMode == 2) { while (sweep >= 0.0f) sweep -= 2.0f * (float)M_PI; }
      else                 { while (sweep <= 0.0f) sweep += 2.0f * (float)M_PI; }
    }
    if (!arcWithinLimits(cx, cy, r, a0, sweep, target)) {
      client.print("error:Soft limit\r\n");
      return true;
    }
    float arcLen = r * fabsf(sweep);
    int nSeg = (int)ceilf(arcLen / 1.0f);
    if (nSeg < 2) nSeg = 2;
    if (nSeg > 400) { client.print("error:Arc too long\r\n"); return true; }

    float feed = gcode_feed_rate_mmpm;
    for (int s = 1; s <= nSeg; s++) {
      float t = (float)s / (float)nSeg;
      float ang = a0 + sweep * t;
      float seg[AXIS_COUNT];
      if (s == nSeg) {
        for (int i = 0; i < AXIS_COUNT; i++) seg[i] = target[i];
      } else {
        seg[0] = cx + r * cosf(ang);
        seg[1] = cy + r * sinf(ang);
        seg[2] = snap[2] + (target[2] - snap[2]) * t;
        seg[3] = snap[3] + (target[3] - snap[3]) * t;
      }
      bool limited = false;
      PlanResult pr = planAndEnqueueBlock(seg, feed, false, false, 0, &limited);
      while (pr == PLAN_FULL) {
        // N22/N25: arco largo: el ok se difiere hasta que haya hueco. Durante la
        // espera se atienden '?'/tiempo real y se mantiene viva la telemetria.
        if (!waitQueueSpace(client, 30000)) {
          stopAllMotion();
          client.print("error:Queue full\r\n");
          return true;
        }
        pr = planAndEnqueueBlock(seg, feed, false, false, 0, &limited);
      }
      if (pr != PLAN_OK) {
        stopAllMotion(); // arco parcial: detener con seguridad
        printPlanResult(client, pr, false);
        return true;
      }
    }
    client.print("ok\r\n");
    return true;
  }

  // G0/G1 (o modal con palabras de eje)
  bool limited = false;
  float feed = (motionMode == 0) ? 100000.0f : gcode_feed_rate_mmpm; // N9: G0 limitado por eje
  PlanResult pr = planAndEnqueueBlock(target, feed, false, false, 0, &limited);
  // N22: control de flujo estilo GRBL: si la cola esta llena, el ok se difiere
  // hasta que el bloque entre (el cliente nunca aborta por Queue full).
  while (pr == PLAN_FULL) {
    if (!waitQueueSpace(client, 30000)) {
      client.print("error:Queue full\r\n");
      return true;
    }
    pr = planAndEnqueueBlock(target, feed, false, false, 0, &limited);
  }
  printPlanResult(client, pr, motionMode != 0 && limited);
  return true;
}

void parseCompactLine(WiFiClient& client, char* line) {
  lastRxMs = millis(); // C3: heartbeat (cualquier linea recibida cuenta)

  // C5: autenticacion por token de sesion con limite de intentos; el token
  // nunca se imprime ni se registra (viaja por TCP plano: usar en red segmentada)
  if (strncmp(line, "AUTH|", 5) == 0) {
    const char* tok = line + 5;
    if (authToken.length() == 0 || authToken.equals(tok)) {
      clientAuthenticated = true;
      authFailCount = 0;
      client.print("AUTH|OK\n");
      sendCompactStatus(client);
    } else {
      clientAuthenticated = false;
      authFailCount++;
      client.print("AUTH|FAIL\n");
      if (authFailCount >= AUTH_MAX_ATTEMPTS) {
        client.print("error:Too many auth attempts\n");
        client.stop(); // C5: liberar el unico cupo de conexion
      }
    }
    return;
  }
  if (authToken.length() > 0 && !clientAuthenticated) {
    client.print("error:Not authenticated\n");
    return;
  }

  // Comandos compactos primero (GET_STATUS empieza con 'G' y no debe ir al parser G-code)
  if (strcmp(line, "GET_STATUS") == 0) { sendCompactStatus(client); return; }
  if (strcmp(line, "GET_VERSION") == 0) { client.printf("VER|%s\n", FW_VERSION); return; }

  if (strncmp(line, "CMD|", 4) != 0) {
    // Protocolo GRBL / G-code
    if (processGrblGCode(client, line)) return;
    client.print("error:Unknown command\n");
    return;
  }

  char* tag = strtok(line, "|"); // "CMD"
  if (!tag) return;
  char* cmdName = strtok(NULL, "|");
  if (!cmdName) return;

  if (strcmp(cmdName, "enable_actuators") == 0) {
    // N8/C2/N20: no re-energizar con el E-stop fisico activo (NC: HIGH)
    if (estopActive()) {
      client.print("ACK|enable_actuators|REJECTED\n");
      return;
    }
    // N12: salir de ALARM purga cola y solicitudes pendientes y resincroniza
    // plannedPos; si la purga no se logra, no se desbloquea (fail-safe).
    if (machineState == ALARM && !clearAlarmState()) {
      client.print("ACK|enable_actuators|FAIL\n");
      return;
    }
    setActuatorsState(true);
    for (int i = 0; i < AXIS_COUNT; i++) strncpy(ax[i].lastError, "None", sizeof(ax[i].lastError) - 1);
    if (machineState == ALARM) machineState = IDLE;
    client.print("ACK|enable_actuators|OK\n");
    sendCompactStatus(client);
    return;
  }
  if (strcmp(cmdName, "emergency_stop") == 0) {
    stopAllMotion();
    setActuatorsState(false);
    client.print("ACK|emergency_stop|OK\n");
    sendCompactStatus(client);
    return;
  }
  if (strcmp(cmdName, "manual_start") == 0) {
    char* axStr = strtok(NULL, "|");
    char* dirStr = strtok(NULL, "|");
    if (axStr && dirStr && actuatorsEnabled && machineState != ALARM) {
      AxisId a = parseAxis(axStr);
      // C1r: jog manual solo en ejes referenciados
      if (!ax[a].homed) {
        client.print("ACK|manual_start|NOT_HOMED\n");
        return;
      }
      machineState = MANUAL;
      ax[a].isMoving = true;
      strncpy(ax[a].moveDir, dirStr, sizeof(ax[a].moveDir) - 1);
      ax[a].manualForward = (strcmp(dirStr, "forward") == 0);
      ax[a].manualBackward = (strcmp(dirStr, "backward") == 0);
      lastManualHeartbeatMs = millis();
      client.print("ACK|manual_start|OK\n");
    } else {
      client.print("ACK|manual_start|REJECTED\n");
    }
    return;
  }
  if (strcmp(cmdName, "manual_stop") == 0) {
    char* axStr = strtok(NULL, "|");
    if (axStr) {
      AxisId a = parseAxis(axStr);
      clearManualFlags(a);
      bool anyMoving = false;
      for (int i = 0; i < AXIS_COUNT; i++) {
        if (ax[i].manualForward || ax[i].manualBackward) anyMoving = true;
      }
      if (!anyMoving && machineState != ALARM) machineState = IDLE;
      client.print("ACK|manual_stop|OK\n");
    }
    return;
  }
  if (strcmp(cmdName, "manual_ping") == 0) {
    lastManualHeartbeatMs = millis();
    return;
  }
  if (strcmp(cmdName, "move_multi_abs") == 0) {
    char* xStr = strtok(NULL, "|"); char* yStr = strtok(NULL, "|");
    char* zStr = strtok(NULL, "|"); char* wStr = strtok(NULL, "|"); char* fStr = strtok(NULL, "|");
    if (xStr && yStr && zStr && wStr) {
      float tgt[AXIS_COUNT] = { (float)atof(xStr), (float)atof(yStr), (float)atof(zStr), (float)atof(wStr) };
      float feedMmpm = fStr ? constrain((float)atof(fStr), 1.0f, 100000.0f) : 600.0f;
      PlanResult pr = planAndEnqueueBlock(tgt, feedMmpm, false, false, 0, NULL);
      // N22: si la cola esta llena, el ACK se difiere hasta que haya hueco
      while (pr == PLAN_FULL) {
        if (!waitQueueSpace(client, 30000)) { client.print("ACK|move_multi_abs|FULL\n"); return; }
        pr = planAndEnqueueBlock(tgt, feedMmpm, false, false, 0, NULL);
      }
      // N10: codigos de error distintos por causa
      if (pr == PLAN_OK) client.print("ACK|move_multi_abs|OK\n");
      else if (pr == PLAN_SOFT_LIMIT) client.print("ACK|move_multi_abs|LIMIT\n");
      else if (pr == PLAN_NOT_HOMED) client.print("ACK|move_multi_abs|NOT_HOMED\n");
      else client.print("ACK|move_multi_abs|FULL\n");
    }
    return;
  }
  if (strcmp(cmdName, "move_axis_abs") == 0) {
    char* axStr = strtok(NULL, "|"); char* valStr = strtok(NULL, "|");
    if (axStr && valStr) {
      AxisId a = parseAxis(axStr);
      float tgt = (float)atof(valStr);
      // N10/C1r: validar antes de aceptar el comando
      if (!ax[a].homed) { client.print("ACK|move_axis_abs|NOT_HOMED\n"); return; }
      if (!inLim(a, tgt)) { strncpy(ax[a].lastError, "Soft Limit", sizeof(ax[a].lastError) - 1); client.print("ACK|move_axis_abs|LIMIT\n"); return; }
      ax[a].manualStop = false; // N19: limpiar residuo de JOG_FREE al aceptar
      ax[a].pendingTargetPos = tgt; ax[a].isJogMove = false; ax[a].runAbsMoveRequested = true;
      client.print("ACK|move_axis_abs|OK\n");
    }
    return;
  }
  if (strcmp(cmdName, "move_axis_rel") == 0) {
    char* axStr = strtok(NULL, "|"); char* valStr = strtok(NULL, "|");
    if (axStr && valStr) {
      AxisId a = parseAxis(axStr);
      float tgt = ax[a].pos + (float)atof(valStr);
      if (!ax[a].homed) { client.print("ACK|move_axis_rel|NOT_HOMED\n"); return; }
      if (!inLim(a, tgt)) { strncpy(ax[a].lastError, "Soft Limit", sizeof(ax[a].lastError) - 1); client.print("ACK|move_axis_rel|LIMIT\n"); return; }
      ax[a].manualStop = false; // N19: limpiar residuo de JOG_FREE al aceptar
      ax[a].pendingTargetPos = tgt; ax[a].isJogMove = false; ax[a].runAbsMoveRequested = true;
      client.print("ACK|move_axis_rel|OK\n");
    }
    return;
  }
  if (strcmp(cmdName, "set_zero_axis") == 0) {
    char* axStr = strtok(NULL, "|");
    if (axStr) {
      AxisId a = parseAxis(axStr);
      // N13: WCO contra el final de la cola (plannedPos), no contra la
      // posicion en vivo: consistente con G10/G92.
      ax[a].wco = plannedPos[a]; // N7: solo WCO, pos de maquina intacta
      client.print("ACK|set_zero_axis|OK\n"); sendCompactStatus(client);
    }
    return;
  }
  if (strcmp(cmdName, "home_axis") == 0) {
    char* axStr = strtok(NULL, "|");
    char* boStr = strtok(NULL, "|");
    char* soStr = strtok(NULL, "|");
    if (axStr) {
      if (!actuatorsEnabled) { client.print("ACK|home_axis|REJECTED\n"); return; }
      AxisId a = parseAxis(axStr);
      // N6: validar rangos
      if (boStr) ax[a].backoffSteps = (uint32_t)constrain(atoi(boStr), 10, 200000);
      if (soStr) ax[a].softLimitOffsetSteps = (uint32_t)constrain(atoi(soStr), 10, 200000);
      ax[a].runHomeRequested = true;
      client.print("ACK|home_axis|OK\n");
    }
    return;
  }
  if (strcmp(cmdName, "invert_axis_dir") == 0) {
    char* axStr = strtok(NULL, "|");
    if (axStr) {
      AxisId a = parseAxis(axStr);
      ax[a].dirForwardLevel = (ax[a].dirForwardLevel == HIGH) ? LOW : HIGH;
      lastDirKnown[a] = false; // M7: direccion invertida, backlash se re-aprende
      saveAxisNVS(a);
      client.print("ACK|invert_axis_dir|OK\n");
    }
    return;
  }
  if (strcmp(cmdName, "set_axis_mode") == 0) {
    char* axStr = strtok(NULL, "|"); char* mStr = strtok(NULL, "|");
    if (axStr && mStr) {
      AxisId a = parseAxis(axStr);
      ax[a].useSCurve = (atoi(mStr) == 1);
      saveAxisNVS(a);
      client.print("ACK|set_axis_mode|OK\n");
    }
    return;
  }
  if (strcmp(cmdName, "reset_step_counter") == 0) {
    char* axStr = strtok(NULL, "|");
    if (axStr) {
      AxisId a = parseAxis(axStr);
      ax[a].stepCount = 0;
      ax[a].pos = 0.0f;
      ax[a].wco = 0.0f;
      // N17: la posicion mecanica queda desconocida: exigir re-referenciado
      ax[a].homed = false;
      if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(25)) == pdTRUE) {
        plannedPos[a] = 0.0f; // A1r/N1: resincronizar bajo mutex
        xSemaphoreGive(stateMutex);
      }
      client.print("ACK|reset_step_counter|OK\n");
      sendCompactStatus(client); // N17: la UI refleja la perdida de referencia
    }
    return;
  }
  if (strcmp(cmdName, "set_calibration_axis") == 0) {
    // A7r: la calibracion guarda steps/mm y fecha; el recorrido maximo es un
    // parametro aparte (set_travel_axis) y no se pisa con la distancia medida.
    // N17: cambiar steps/mm invalida la referencia (pos/steps ya no significan
    // lo mismo): se exige re-homing del eje.
    char* axStr = strtok(NULL, "|");
    char* spmStr = strtok(NULL, "|");
    char* distStr = strtok(NULL, "|"); // distancia medida (informativa, no se aplica)
    char* dateStr = strtok(NULL, "|");
    (void)distStr;
    if (axStr && spmStr) {
      AxisId a = parseAxis(axStr);
      float spmVal = (float)atof(spmStr);
      if (spmVal < 0.1f || spmVal > 100000.0f) { client.print("ACK|set_calibration_axis|INVALID\n"); return; } // N6
      bool spmChanged = fabsf(ax[a].stepsPerMm - spmVal) > 1e-6f;
      ax[a].stepsPerMm = spmVal;
      if (spmChanged) {
        // N17: re-referenciado obligatorio tras cambiar la calibracion
        ax[a].homed = false;
        strncpy(ax[a].lastError, "Re-Home req", sizeof(ax[a].lastError) - 1);
      }
      if (dateStr) {
        strncpy(ax[a].lastCalibration, dateStr, sizeof(ax[a].lastCalibration) - 1);
        ax[a].lastCalibration[sizeof(ax[a].lastCalibration) - 1] = '\0';
      }
      ax[a].calibrated = true;
      saveAxisNVS(a);
      client.print("ACK|set_calibration_axis|OK\n");
      sendCompactStatus(client); // N17: la UI refleja la perdida de referencia
    }
    return;
  }
  if (strcmp(cmdName, "set_travel_axis") == 0) {
    // A7r: recorrido real del eje, separado de la calibracion
    char* axStr = strtok(NULL, "|"); char* mtStr = strtok(NULL, "|");
    if (axStr && mtStr) {
      float mt = (float)atof(mtStr);
      if (mt < 1.0f || mt > 5000.0f) { client.print("ACK|set_travel_axis|INVALID\n"); return; }
      AxisId a = parseAxis(axStr);
      ax[a].maxTravel = mt;
      saveAxisNVS(a);
      client.print("ACK|set_travel_axis|OK\n");
    }
    return;
  }
  if (strcmp(cmdName, "set_accel_axis") == 0) {
    // A5: aceleracion real del eje (mm/s2)
    char* axStr = strtok(NULL, "|"); char* aStr = strtok(NULL, "|");
    if (axStr && aStr) {
      float av = (float)atof(aStr);
      if (av < 1.0f || av > 500.0f) { client.print("ACK|set_accel_axis|INVALID\n"); return; }
      AxisId a = parseAxis(axStr);
      ax[a].accelMmS2 = av;
      saveAxisNVS(a);
      client.print("ACK|set_accel_axis|OK\n");
    }
    return;
  }
  if (strcmp(cmdName, "set_maxspeed_axis") == 0) {
    // N9: limite de velocidad por eje (mm/s)
    char* axStr = strtok(NULL, "|"); char* vStr = strtok(NULL, "|");
    if (axStr && vStr) {
      float vv = (float)atof(vStr);
      if (vv < 0.5f || vv > 100.0f) { client.print("ACK|set_maxspeed_axis|INVALID\n"); return; }
      AxisId a = parseAxis(axStr);
      ax[a].maxSpeedMmS = vv;
      saveAxisNVS(a);
      client.print("ACK|set_maxspeed_axis|OK\n");
    }
    return;
  }
  if (strcmp(cmdName, "set_backlash_axis") == 0) {
    // M7: compensacion de backlash (mm)
    char* axStr = strtok(NULL, "|"); char* bStr = strtok(NULL, "|");
    if (axStr && bStr) {
      float bv = (float)atof(bStr);
      if (bv < 0.0f || bv > 5.0f) { client.print("ACK|set_backlash_axis|INVALID\n"); return; }
      AxisId a = parseAxis(axStr);
      ax[a].backlashMm = bv;
      lastDirKnown[a] = false;
      saveAxisNVS(a);
      client.print("ACK|set_backlash_axis|OK\n");
    }
    return;
  }
  if (strcmp(cmdName, "set_scurve_profile_axis") == 0) {
    char* axStr = strtok(NULL, "|");
    char* sStr = strtok(NULL, "|");
    char* cStr = strtok(NULL, "|");
    char* eStr = strtok(NULL, "|");
    char* rStr = strtok(NULL, "|");
    if (axStr && sStr && cStr && eStr && rStr) {
      AxisId a = parseAxis(axStr);
      ax[a].scurve.startSpeedMmS = constrain((float)atof(sStr), 0.1f, 50.0f);
      ax[a].scurve.cruiseSpeedMmS = constrain((float)atof(cStr), ax[a].scurve.startSpeedMmS, 100.0f);
      ax[a].scurve.endSpeedMmS = constrain((float)atof(eStr), 0.1f, 50.0f);
      ax[a].scurve.rampRatio = constrain((float)atof(rStr), 0.05f, 0.45f); // N6
      saveAxisNVS(a);
      client.print("ACK|set_scurve_profile_axis|OK\n");
    }
    return;
  }
  if (strcmp(cmdName, "set_manual_speed_axis") == 0) {
    char* axStr = strtok(NULL, "|");
    char* manStr = strtok(NULL, "|");
    char* jogStr = strtok(NULL, "|");
    if (axStr && manStr && jogStr) {
      AxisId a = parseAxis(axStr);
      // N6: rango duro [MIN_STEP_US, 20000]; negativos ya no pasan a uint32_t
      ax[a].manualUs = constrain(atoi(manStr), (int)MIN_STEP_US, 20000);
      ax[a].jogUs = constrain(atoi(jogStr), (int)MIN_STEP_US, 20000);
      saveAxisNVS(a);
      client.print("ACK|set_manual_speed_axis|OK\n");
    }
    return;
  }
  if (strcmp(cmdName, "set_homing_speed_axis") == 0) {
    char* axStr = strtok(NULL, "|");
    char* seekStr = strtok(NULL, "|");
    char* feedStr = strtok(NULL, "|");
    char* boStr = strtok(NULL, "|");
    char* boStepsStr = strtok(NULL, "|");
    char* soStepsStr = strtok(NULL, "|");
    if (axStr && seekStr && feedStr && boStr) {
      AxisId a = parseAxis(axStr);
      ax[a].homingSeekUs = constrain(atoi(seekStr), (int)MIN_STEP_US, 20000);
      ax[a].homingFeedUs = constrain(atoi(feedStr), (int)MIN_STEP_US, 20000);
      ax[a].homingBackoffUs = constrain(atoi(boStr), (int)MIN_STEP_US, 20000);
      if (boStepsStr) ax[a].backoffSteps = (uint32_t)constrain(atoi(boStepsStr), 10, 200000);
      if (soStepsStr) ax[a].softLimitOffsetSteps = (uint32_t)constrain(atoi(soStepsStr), 10, 200000);
      saveAxisNVS(a);
      client.print("ACK|set_homing_speed_axis|OK\n");
    }
    return;
  }
  client.print("error:Unknown command\n");
}

void parseSerialCommand(char* line) {
  if (strcmp(line, "GET_VERSION") == 0) {
    Serial.printf("VER|%s\n", FW_VERSION);
    return;
  }
  // C5: configurar token de autenticacion TCP (vacio = modo abierto)
  if (strncmp(line, "SET_AUTH_TOKEN|", 15) == 0) {
    authToken = String(line + 15);
    authToken.trim();
    if (authToken.length() > 32) authToken = authToken.substring(0, 32);
    saveNetworkNVS();
    clientAuthenticated = (authToken.length() == 0);
    Serial.println("ACK|SET_AUTH_TOKEN|OK");
    return;
  }
  if (strcmp(line, "SCAN_WIFI") == 0) {
    int n = WiFi.scanNetworks();
    Serial.print("WIFI_LIST|");
    for (int i = 0; i < n; ++i) { Serial.print(WiFi.SSID(i)); if (i < n - 1) Serial.print(","); }
    Serial.println(); 
    WiFi.scanDelete(); 
    return;
  }
  if (strncmp(line, "SET_WIFI_NVS|", 13) == 0) {
    char* ssid = strtok(line + 13, "|");
    char* pass = strtok(NULL, "|");
    char* devname = strtok(NULL, "|");
    char* ip4 = strtok(NULL, "|");
    if (ssid) {
      wifi_ssid = String(ssid); 
      wifi_pass = pass ? String(pass) : "";
      wifi_devname = (devname && strlen(devname) > 0) ? String(devname) : "CNC-XYZW-NETLOG";
      if (ip4) ip_oct4 = (uint8_t)atoi(ip4);
      saveNetworkNVS(); 
      Serial.println("ACK|SET_WIFI_NVS|OK");
    } else { 
      Serial.println("ACK|SET_WIFI_NVS|FAIL"); 
    }
    return;
  }
  if (strcmp(line, "DUMP_NVS") == 0) {
    Serial.print("NVS_DATA|{");
    String currentIpStr = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : "0.0.0.0";
    Serial.printf("\"current_ip\":\"%s\",", currentIpStr.c_str());
    Serial.printf("\"ssid\":\"%s\",", wifi_ssid.c_str());
    Serial.print("\"pass\":\"******\",");
    Serial.printf("\"devname\":\"%s\",", wifi_devname.c_str());
    Serial.printf("\"ip_oct4\":%d,", (int)ip_oct4);
    Serial.print("\"axes\":{");
    for (int i = 0; i < AXIS_COUNT; i++) {
      Serial.printf("\"%s\":{\"spm\":%.3f,\"max\":%.2f,\"bo\":%lu,\"so\":%lu,\"sc\":%d,\"seek\":%d,\"feed\":%d,\"bo_us\":%d,\"man_us\":%d,\"jog_us\":%d,\"dfl\":%d,\"sc_s\":%.2f,\"sc_c\":%.2f,\"sc_e\":%.2f,\"sc_r\":%.2f}",
        AXIS_NAME[i], ax[i].stepsPerMm, ax[i].maxTravel, (unsigned long)ax[i].backoffSteps,
        (unsigned long)ax[i].softLimitOffsetSteps, ax[i].useSCurve ? 1 : 0, ax[i].homingSeekUs,
        ax[i].homingFeedUs, ax[i].homingBackoffUs, ax[i].manualUs, ax[i].jogUs,
        ax[i].dirForwardLevel, ax[i].scurve.startSpeedMmS, ax[i].scurve.cruiseSpeedMmS,
        ax[i].scurve.endSpeedMmS, ax[i].scurve.rampRatio);
      if (i < AXIS_COUNT - 1) Serial.print(",");
    }
    Serial.println("}}"); 
    return;
  }
  if (strncmp(line, "LOAD_AXIS_PARAM|", 16) == 0) {
    char* axStr = strtok(line + 16, "|");
    if (axStr) {
      AxisId a = parseAxis(axStr);
      // N6: validacion y acotado de todos los parametros (igual que via TCP)
      char* spm  = strtok(NULL, "|"); if (spm && atof(spm) > 0.1f) ax[a].stepsPerMm = constrain((float)atof(spm), 0.1f, 100000.0f);
      char* maxT = strtok(NULL, "|"); if (maxT && atof(maxT) > 0.0f) ax[a].maxTravel = constrain((float)atof(maxT), 1.0f, 5000.0f);
      char* bo   = strtok(NULL, "|"); if (bo) ax[a].backoffSteps = (uint32_t)constrain(atoi(bo), 10, 200000);
      char* so   = strtok(NULL, "|"); if (so) ax[a].softLimitOffsetSteps = (uint32_t)constrain(atoi(so), 10, 200000);
      char* sc   = strtok(NULL, "|"); if (sc) ax[a].useSCurve = (atoi(sc) == 1);
      char* seek = strtok(NULL, "|"); if (seek) ax[a].homingSeekUs = constrain(atoi(seek), (int)MIN_STEP_US, 20000);
      char* feed = strtok(NULL, "|"); if (feed) ax[a].homingFeedUs = constrain(atoi(feed), (int)MIN_STEP_US, 20000);
      char* bous = strtok(NULL, "|"); if (bous) ax[a].homingBackoffUs = constrain(atoi(bous), (int)MIN_STEP_US, 20000);
      char* man  = strtok(NULL, "|"); if (man) ax[a].manualUs = constrain(atoi(man), (int)MIN_STEP_US, 20000);
      char* jog  = strtok(NULL, "|"); if (jog) ax[a].jogUs = constrain(atoi(jog), (int)MIN_STEP_US, 20000);
      char* dfl  = strtok(NULL, "|"); if (dfl) { ax[a].dirForwardLevel = atoi(dfl) ? HIGH : LOW; lastDirKnown[a] = false; }
      saveAxisNVS(a);
      Serial.println("ACK|LOAD_AXIS_PARAM|OK");
    } else {
      Serial.println("ACK|LOAD_AXIS_PARAM|FAIL");
    }
    return;
  }
  if (strncmp(line, "JOG_FREE|", 9) == 0) {
    // M4: se encola en la tarea de motores (con limites); ya no bloquea loop()
    char* axStr = strtok(line + 9, "|");
    char* dirStr = strtok(NULL, "|");
    char* usStr = strtok(NULL, "|");
    if (axStr && dirStr && actuatorsEnabled && machineState != ALARM) {
      AxisId a = parseAxis(axStr);
      bool isFwd = (strcmp(dirStr, "forward") == 0);
      uint32_t stepUs = (uint32_t)constrain(usStr ? atoi(usStr) : 1000, (int)MIN_STEP_US, 20000);
      jogFreeUs[a] = stepUs;
      jogFreeSteps[a] = isFwd ? 25 : -25;
      Serial.println("ACK|JOG_FREE|OK");
    } else {
      Serial.println("ACK|JOG_FREE|REJECTED");
    }
    return;
  }
  if (strcmp(line, "JOG_STOP") == 0) {
    for (int i = 0; i < AXIS_COUNT; i++) clearManualFlags((AxisId)i);
    return;
  }
  if (strncmp(line, "CMD|enable_actuators", 20) == 0) {
    if (estopActive()) {
      Serial.println("ACK|enable_actuators|REJECTED");
      return;
    }
    // N12: misma politica que por TCP: purga total al salir de ALARM
    if (machineState == ALARM && !clearAlarmState()) {
      Serial.println("ACK|enable_actuators|FAIL");
      return;
    }
    setActuatorsState(true);
    for (int i = 0; i < AXIS_COUNT; i++) strncpy(ax[i].lastError, "None", sizeof(ax[i].lastError) - 1);
    if (machineState == ALARM) machineState = IDLE;
    Serial.println("ACK|enable_actuators|OK");
    return;
  }
  if (strncmp(line, "CMD|emergency_stop", 18) == 0) {
    stopAllMotion();
    setActuatorsState(false);
    Serial.println("ACK|emergency_stop|OK");
    return;
  }
  if (strcmp(line, "RESTART_ESP") == 0) { 
    Serial.println("ACK|RESTART_ESP|OK"); 
    delay(200); 
    ESP.restart(); 
    return; 
  }
}

void setup(){
  Serial.begin(115200);
  delay(150);
  Serial.printf("[BOOT] CNC XYZW FW %s\n", FW_VERSION);

  stateMutex = xSemaphoreCreateMutex();
  stepTimer = timerBegin(1000000);
  timerAttachInterrupt(stepTimer, &onStepTimer);

  loadNVS();
  clientAuthenticated = (authToken.length() == 0);
  if (authToken.length() > 0) Serial.println("[SEC] Autenticacion TCP habilitada");

  pinMode(PIN_ENABLE_ACTUATORS, OUTPUT);
  setActuatorsState(false);

  // C2/N20: E-stop fisico con ISR. Boton NC a GND con INPUT_PULLUP: el estado
  // ACTIVO (pulsado o cable cortado) es HIGH -> flanco RISING. Fail-safe real.
  pinMode(PIN_ESTOP, INPUT_PULLUP);
  attachInterrupt(PIN_ESTOP, onEstopISR, RISING);
  if (estopActive()) {
    estopTriggered = true;
    Serial.println("[E-STOP] Pulsado al arrancar: actuadores bloqueados");
  }

  for (int i = 0; i < AXIS_COUNT; i++){
    pinMode(hw[i].pinStep, OUTPUT);
    pinMode(hw[i].pinDir, OUTPUT);
    pinMode(hw[i].pinLimitHome, INPUT_PULLUP);
    digitalWrite(hw[i].pinStep, LOW);
  }

  connectWiFiBlocking();
  // M2: prioridad mayor que loop() (prioridad 1) para reducir jitter de pasos
  xTaskCreatePinnedToCore(TaskMotors, "TaskMotors", 8192, NULL, 5, &TaskMotorsHandle, 1);
}

void loop(){
  // C2: E-stop fisico (marcado por el ISR)
  if (estopTriggered) {
    estopTriggered = false;
    stopAllMotion();
    setActuatorsState(false);
    machineState = ALARM;
    for (int i = 0; i < AXIS_COUNT; i++) strncpy(ax[i].lastError, "E-Stop Fisico", sizeof(ax[i].lastError) - 1);
    forceStatusPush = true;
  }

  while (Serial.available() > 0) {
    char ch = (char)Serial.read();
    // N8: tiempo real inmediato tambien por serial
    if (ch == '!') { stopAllMotion(); setActuatorsState(false); Serial.println("ok"); continue; }
    if (ch == (char)0x18) { stopAllMotion(); Serial.printf("\r\nCNC-XYZW %s ['$' para ayuda]\r\n", FW_VERSION); continue; }
    if (ch == (char)0x85) { jogCancelEpoch++; continue; } // N5: solo cancela jog
    if (ch == '\n' || ch == '\r') {
      if (serialBufIdx > 0) {
        serialBuffer[serialBufIdx] = '\0';
        parseSerialCommand(serialBuffer);
        serialBufIdx = 0;
      }
    } else {
      if (serialBufIdx < (MAX_LINE_LEN - 1)) serialBuffer[serialBufIdx++] = ch;
    }
  }

  // C3: watchdog de conexion TCP con bandera wasConnected (alcance corregido)
  if (!cl || !cl.connected()) {
    if (clientWasConnected) {
      clientWasConnected = false;
      clientAuthenticated = (authToken.length() == 0);
      if (machineState == RUNNING || machineState == MANUAL) {
        stopAllMotion();
      }
      cl.stop();
    }
    if (serverStarted) {
      cl = server.accept();
      if (cl) {
        tcpBufIdx = 0;
        clientWasConnected = true;
        clientAuthenticated = (authToken.length() == 0);
        authFailCount = 0;                    // C5: limite de intentos por sesion
        authGraceStartMs = millis();          // C5: plazo para autenticarse
        lastRxMs = millis();
        cl.setNoDelay(true);
        // C2r: escrituras TCP acotadas: un cliente colgado no bloquea loop()
        cl.setTimeout(TCP_WRITE_TIMEOUT_MS);
      } else {
        delay(2);
        return;
      }
    } else {
      delay(2);
      return;
    }
  }

  // C5: una sesion sin autenticar no puede ocupar el unico cupo para siempre
  if (cl && cl.connected() && authToken.length() > 0 && !clientAuthenticated &&
      (millis() - authGraceStartMs > AUTH_GRACE_MS)) {
    cl.print("error:Auth timeout\n");
    cl.stop();
    return;
  }

  if (cl && cl.connected()) {
    while (cl.available() > 0) {
      char ch = (char)cl.read();
      lastRxMs = millis(); // C3: cualquier byte recibido es heartbeat

      // Intercepcion inmediata de comandos de tiempo real (con o sin '\n')
      if (ch == '?') {
        if (authToken.length() == 0 || clientAuthenticated) {
          cl.printf("<%s|MPos:%.3f,%.3f,%.3f,%.3f|WPos:%.3f,%.3f,%.3f,%.3f|FS:%.0f,0>\r\n",
            machineStateStr(),
            ax[0].pos, ax[1].pos, ax[2].pos, ax[3].pos,
            ax[0].pos - ax[0].wco, ax[1].pos - ax[1].wco, ax[2].pos - ax[2].wco, ax[3].pos - ax[3].wco,
            gcode_feed_rate_mmpm);
        }
        continue;
      }
      if (ch == '!') {
        stopAllMotion();
        setActuatorsState(false);
        cl.print("ok\r\n");
        continue;
      }
      if (ch == (char)0x85) { // N5/A1r: cancela solo bloques jog, sin tocar el
        // resto del programa; plannedPos se resincroniza solo en los ejes cuyo
        // jog fue cancelado (los demas bloques mantienen su planificacion).
        jogCancelEpoch++;
        for (int i = 0; i < AXIS_COUNT; i++) clearManualFlags((AxisId)i);
        if (machineState == MANUAL) machineState = IDLE;
        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          // A1r: recalcular donde termina realmente la cola tras la
          // cancelacion: replanificar las posiciones desde la posicion actual
          // sumando solo los deltas de los bloques NO jog que quedan activos.
          // plannedPos pasa a ser el objetivo del ultimo bloque no jog vivo
          // (o la posicion actual si no queda ninguno). No se reescribe a
          // ciegas con ax[].pos si quedaban movimientos de programa.
          float replanned[AXIS_COUNT] = {ax[0].pos, ax[1].pos, ax[2].pos, ax[3].pos};
          int n = (qHead - qTail + BLOCK_QUEUE_SIZE) % BLOCK_QUEUE_SIZE;
          for (int k = 0; k < n; k++) {
            int idx = (qTail + k) % BLOCK_QUEUE_SIZE;
            if (!blockQueue[idx].active) continue;
            if (blockQueue[idx].isJog) {
              blockQueue[idx].active = false;
              blockQueue[idx].maxSteps = 0;
              blockQueue[idx].isDwell = true; // se ejecuta como dwell nulo
              blockQueue[idx].dwellMs = 0;
              for (int a = 0; a < AXIS_COUNT; a++) blockQueue[idx].deltaSteps[a] = 0;
              continue; // no aporta desplazamiento
            }
            // bloque no jog: acumula el desplazamiento eje a eje
            for (int a = 0; a < AXIS_COUNT; a++) {
              float dmm = (float)blockQueue[idx].deltaSteps[a] / safeSpm((AxisId)a);
              replanned[a] += blockQueue[idx].dirPos[a] ? dmm : -dmm;
            }
          }
          for (int a = 0; a < AXIS_COUNT; a++) plannedPos[a] = replanned[a];
          xSemaphoreGive(stateMutex);
        }
        continue;
      }
      if (ch == '~') {
        // N8: solo reanuda; NO re-energiza drivers ni borra ALARM
        if (machineState == ALARM) cl.print("error:Alarm\r\n");
        else cl.print("ok\r\n");
        continue;
      }
      if (ch == (char)0x18) {
        stopAllMotion();
        cl.printf("\r\nCNC-XYZW %s ['$' for help]\r\n", FW_VERSION);
        continue;
      }

      if (ch == '\n' || ch == '\r') {
        if (tcpBufIdx > 0) {
          tcpBuffer[tcpBufIdx] = '\0';
          parseCompactLine(cl, tcpBuffer);
          tcpBufIdx = 0;
        }
      } else {
        if (tcpBufIdx < (MAX_LINE_LEN - 1)) tcpBuffer[tcpBufIdx++] = ch;
      }
    }

    // C3: heartbeat con timeout durante programas (Run/Jog).
    // Si el cliente deja de hablar en medio de un movimiento, se detiene todo.
    if ((machineState == RUNNING || machineState == MANUAL) &&
        (millis() - lastRxMs > COMM_TIMEOUT_MS)) {
      stopAllMotion();
      for (int i = 0; i < AXIS_COUNT; i++) strncpy(ax[i].lastError, "Comm Timeout", sizeof(ax[i].lastError) - 1);
      forceStatusPush = true;
    }

    refreshAllInputs();
    if (forceStatusPush || (millis() - lastTelemetryMs >= TELEMETRY_INTERVAL_MS)) {
      lastTelemetryMs = millis();
      forceStatusPush = false;
      sendCompactStatus(cl);
    }
    delay(1);
  }
}