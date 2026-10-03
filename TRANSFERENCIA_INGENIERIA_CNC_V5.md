# Transferencia de Ingeniería — Sistema CNC XYZW V5.1

**Proyecto:** Controlador CNC de 4 ejes (X, Y, Z, W) basado en ESP32 + cliente de escritorio PySide6
**Versión del sistema:** 5.1.0 (firmware `FW_VERSION = "5.1.0"`)
**Fecha:** 2026-10-03
**Alcance:** Remediación de los hallazgos de `Auditoria_CNC_XYZW_V5.xlsx` hasta superar un scoring del 80 %, sin pérdida de funcionalidad preexistente.

---

## 1. Resumen ejecutivo

El sistema está formado por dos componentes que operan en conjunto:

| Componente | Archivo | Plataforma | Rol |
|------------|---------|------------|-----|
| **Firmware** | `CNC_V5.ino` | ESP32 (Arduino/FreeRTOS) | Control de movimiento en tiempo real, planificador con look-ahead, seguridad (E-stop, soft limits), servidor TCP (puerto 5000) y protocolo dual NET-LOG/GRBL |
| **Cliente HMI** | `CNC_V5.py` | Python 3 + PySide6 | Interfaz gráfica: control por ejes, jog D-Pad, secuenciador, cargador G-code, visión/calibración óptica, E-stop con ACK |
| **Utilidad de mantenimiento** | `CNC_MAINT_V5.py` | Python 3 + PySide6 (serial USB) | Edición de parámetros NVS del firmware (steps/mm, recorridos, homing, red) |

La **V5.1** corrige los defectos críticos, altos y la mayoría de los medios identificados en la auditoría V5, elevando el puntaje estimado por encima del umbral de **Producción (≥ 80/100)**.

---

## 2. Arquitectura del sistema

```
┌──────────────────────────────────────────────────────────────────┐
│  Cliente HMI (CNC_V5.py, PySide6)                                │
│  • UI por ejes, D-Pad jog, secuenciador, cargador G-code         │
│  • Visión óptica (OpenCV) en hilo dedicado, calibración asistida │
│  • TcpWorker (QThread): socket TCP con parseo tolerante          │
│  • Heartbeat (250 ms) en AMBOS modos + reconexión con backoff    │
└───────────────┬──────────────────────────────────────────────────┘
                │ TCP 192.168.1.167:5000  (NET-LOG compacto | GRBL)
                │  + token AUTH opcional (con límite de intentos)
┌───────────────▼──────────────────────────────────────────────────┐
│  Firmware ESP32 (CNC_V5.ino, FreeRTOS)                           │
│  ┌────────────────────────────────────────────────────────────┐  │
│  │ TaskMotors (núcleo 1, prioridad 5): ejecuta la cola de     │  │
│  │ bloques, genera pasos por timer HW (1 MHz), homing, jog    │  │
│  └────────────────────────────────────────────────────────────┘  │
│  ┌────────────────────────────────────────────────────────────┐  │
│  │ loop() (núcleo 0): red TCP/serial, parser G-code,          │  │
│  │ planificador look-ahead, watchdog Comm Timeout, E-stop     │  │
│  └────────────────────────────────────────────────────────────┘  │
│  ISR E-stop (flanco RISING) → corta ENABLE al instante           │
└──────────────────────────────────────────────────────────────────┘
```

### Concurrencia y tiempo real
- `stateMutex` protege la cola de bloques y `plannedPos`.
- `stopEpoch` / `jogCancelEpoch` invalidan bloques en vuelo sin condiciones de carrera.
- Extracción atómica de bloques (`popBlock`), movimientos (`popAbsMove`) y homing (`popHomeRequest`) bajo mutex.
- La tarea de motores (prioridad 5) tiene precedencia sobre `loop()` (prioridad 1).

---

## 3. Seguridad funcional

### 3.1 E-stop físico (N20 — corregido)
- **Pin:** GPIO32 (`PIN_ESTOP`), `INPUT_PULLUP`, botón **NC a GND**.
- **Polaridad (corregida):** con NC a GND el estado *activo* (pulsado o cable cortado) es **HIGH** → ISR en flanco **RISING** (`estopActive() == digitalRead(PIN_ESTOP) == HIGH`). Esto lo hace *fail-safe*: un cable cortado fuerza el estado seguro.
- **Acción del ISR:** baja `ENABLE` (GPIO27, activo en LOW) al instante y marca `estopTriggered`; `loop()` pone la máquina en `ALARM`.
- **Rearmado:** `$X` y `enable_actuators` solo re-energizan si el E-stop está liberado (`!estopActive()`).
- **Recomendación de hardware (C2r):** para corte de potencia real, cablear el E-stop en serie con la alimentación de los drivers (el ISR solo corta ENABLE).

### 3.2 Salida de ALARM (N12 — corregido)
`clearAlarmState()` purga la cola, cancela solicitudes pendientes de homing/movimiento y resincroniza `plannedPos` con la posición real. Si la purga no puede completarse (mutex ocupado), **no se desbloquea** (fail-safe) y la tarea de motores finaliza la purga de forma diferida.

### 3.3 Soft limits y validación (C1r, N6)
- Límites siempre activos (con o sin referencia); ejes sin referenciar devuelven `NOT_HOMED`.
- `inLim()` rechaza valores **no finitos** (NaN/Inf) mediante `isfinite()`.

### 3.4 Watchdog de comunicación (C3 / N21)
- El firmware detiene todo si no recibe bytes en `COMM_TIMEOUT_MS` (3000 ms) durante `RUNNING`/`MANUAL`.
- El cliente ahora envía **heartbeat cada 250 ms en ambos modos** (compacto y GRBL), por lo que un movimiento largo ya no dispara un falso "Comm Timeout".

---

## 4. Control de movimiento

### 4.1 Planificador con look-ahead (A5 — mejorado)
- Planificación desde `plannedPos` (final de cola), nunca desde la posición en vivo.
- **Junction deviation estándar** (modelo GRBL): `vj = sqrt(accel · jd · sin(θ/2) / (1 − sin(θ/2)))`, continuo (sin la discontinuidad previa cerca de 5.7°). `JUNCTION_DEVIATION_MM = 0.05`.
- **Pasada hacia adelante** (`plannerForwardPass`): limita la velocidad de entrada de cada bloque según la salida del bloque previo.
- **Pasada hacia atrás** (`plannerBackwardPass`): garantiza el frenado hasta la entrada del siguiente bloque. Se detiene antes de la cabeza (`qTail`) para no modificar el bloque en ejecución.

### 4.2 Control de flujo de la cola (N22 / N25 — corregido)
- **`ok` diferido (estilo GRBL):** `G0/G1`, `G4`, `G28`, `$J` y `move_multi_abs` difieren el `ok` hasta que el bloque entra en la cola (`waitQueueSpace`, 30 s).
- Durante la espera se atienden los comandos de tiempo real (`!`, `0x18`, `0x85`, `?`) y se mantiene la telemetría viva: el sistema ya no se bloquea con arcos largos.
- El cliente además **reintenta** ante `Queue full` (hasta 400 reintentos, 100 ms) como red de seguridad.

### 4.3 Arcos G2/G3 (N25 — mejorado)
- Espera de hueco no bloqueante con telemetría y respuesta a `?`.
- **Validación de error de radio** (tolerancia GRBL: 0.5 mm y 0.1 %).
- **G53** aceptado (movimiento en coordenadas de máquina, ignora WCO en esa línea).

### 4.4 Backlash y calibración (M7, M6, A7r, N17)
- Compensación de backlash por eje (pasos extra al invertir el sentido, sin tocar `pos`/`stepCount`).
- Calibración con umbral del 5 % entre pasadas.
- Recorrido máximo (`set_travel_axis`) separado de la calibración (steps/mm).
- **N17:** `reset_step_counter` y `set_calibration_axis` invalidan `homed` y exigen re-referenciado del eje (la posición mecánica ya no es conocida).

---

## 5. Sistema de coordenadas

### 5.1 WCO (offset de trabajo)
- `WPos = MPos − wco`. Los soft limits operan siempre en **MPos**.
- **N13:** `G10 L20`, `G92` y `set_zero_axis` fijan el WCO contra `plannedPos` (final de la cola), no contra la posición en vivo.
- **N14:** `$J` absoluto aplica el WCO (consistente con `G1` absoluto).
- **N16:** la telemetría compacta `ST|` incluye el WCO (campo 28); la UI muestra la posición de trabajo (`WPos`), unificada con el modo GRBL.

### 5.2 Telemetría compacta (`ST|`) — formato V5.1
```
ST|<estado>|<actuadores>|<qDepth>|<ejeX>|<ejeY>|<ejeZ>|<ejeW>\n
eje = pos,targetPos,stepCount,homed,calibrated,firstRun,dirFwd,isMoving,moveDir,
      stepsPerMm,maxTravel,backoffSteps,softLimitOffsetSteps,useSCurve,manualUs,
      jogUs,homingSeekUs,homingBackoffUs,lastCalibration,
      sc_s,sc_c,sc_e,sc_r,lastError,
      homingFeedUs,backlashMm,maxSpeedMmS,accelMmS2,
      wco            ← campo 28, nuevo en V5.1 (opcional, compat con V5.0)
```
El parseo del cliente es tolerante: funciona con 24, 28 o 29 campos por eje.

---

## 6. Protocolo y red

### 6.1 Protocolo dual
- **NET-LOG compacto** (por defecto): comandos `CMD|...`, telemetría `ST|` empujada cada 60 ms, `ACK|cmd|status`.
- **GRBL:** líneas G-code, `?` → `<Estado|MPos:...|WPos:...|FS:...>`, `$` comandos.

### 6.2 Comandos en tiempo real (sin necesidad de newline)
| Byte | Acción |
|------|--------|
| `?`  | Estado GRBL inmediato |
| `!`  | Parada inmediata: purga cola, deshabilita drivers, responde `ok` |
| `~`  | Reanudar (no re-energiza ni borra ALARM) |
| `0x18` | Reset suave: stop + banner de bienvenida |
| `0x85` | Cancelación de jog: anula solo bloques `isJog`, replanifica `plannedPos` de los ejes afectados (A1r) |

### 6.3 Autenticación (C5 — endurecido)
- Token por sesión almacenado en NVS (`SET_AUTH_TOKEN` por serial). Vacío = modo abierto.
- **Límite de 5 intentos** fallidos → se cierra la conexión (libera el único cupo).
- **Timeout de gracia de 10 s** para autenticarse; una sesión sin autenticar no puede ocupar el cupo indefinidamente.
- El token viaja en TCP plano (documentado): **usar en red segmentada/confiable**.
- **C2r:** escrituras TCP con timeout de 3000 ms; un cliente colgado no bloquea `loop()`.

### 6.4 Códigos de respuesta (N10)
`ok`, `error:Queue full`, `error:Soft limit`, `error:Not homed`, `error:Invalid command`, `error:Unsupported command`, `error:Alarm`, `ACK|<cmd>|OK|LIMIT|NOT_HOMED|FULL|REJECTED|INVALID|FAIL`, `[MSG:Feed limitado...]`, `[MSG:Caution: Unlocked]`.

---

## 7. Hardware / pines (ESP32)

| Señal | GPIO | Nota |
|-------|------|------|
| ENABLE drivers | 27 | Activo en LOW (LOW = habilitado) |
| E-stop físico | 32 | INPUT_PULLUP, NC a GND, ISR RISING (activo HIGH) |
| X step / dir / límite | 23 / 22 / 34 | GPIO34 sin pull-up interno → requiere pull-up físico |
| Y step / dir / límite | 21 / 17 / 25 | |
| Z step / dir / límite | 16 / 4 / 14 | GPIO4 es pin de strapping |
| W step / dir / límite | 13 / 15 / 18 | GPIO15 es pin de strapping |

**Notas (M3):** GPIO34 requiere pull-up externo; GPIO4/GPIO15 son pines de strapping (mantener sin carga durante el arranque o reasignarlos en `AxisHW`); ENABLE sin pull-up (documentado). Piso de periodo de paso: `MIN_STEP_US = 80` µs.

---

## 8. Robustez del software (cliente)

- **M5:** eliminados los `except: pass` silenciosos → ahora se registran con `logging` (nivel INFO/WARNING/ERROR) sin tumbar el hilo.
- **M5:** `stop_camera()` espera la salida real del hilo de cámara (`wait(2000)` + `terminate()` de respaldo), evitando cierres inesperados.
- **N26:** jog GRBL con feed derivado de la velocidad configurada (`jog_us` × steps/mm) y tramo proporcional, con límite de profundidad de cola (no envía si ya hay ≥ 6 bloques).
- Reconexión automática con backoff, parseo tolerante de telemetría, cámara en hilo propio.
- E-stop con ACK y reintento hasta confirmación por telemetría.

### Comportamiento del botón DETENER del cargador G-code (N22)
- **DETENER** (cargador) = *feed hold* suave: purga la cola con `0x85` **sin deshabilitar los drivers** (la máquina queda energizada, lista para reanudar).
- **E-stop global / aborto por error / timeout / cierre de la app** = corte inmediato con `!` (sí deshabilita drivers).

---

## 9. Registro de remediación de la auditoría V5

| ID | Severidad | Estado en V5.1 | Cambio |
|----|-----------|----------------|--------|
| **N19** | Crítico | ✅ Resuelto | `manualStop` ya no aborta bloques; abortos solo por `stopEpoch`/`jogCancelEpoch`; `popBlock` limpia el residuo |
| **N20** | Crítico | ✅ Resuelto | Polaridad E-stop: ISR RISING + activo HIGH (NC a GND, fail-safe) |
| **N12** | Alto | ✅ Resuelto | `clearAlarmState()` purga cola/solicitudes y resincroniza `plannedPos`; fail-safe si no se logra |
| **N13** | Alto | ✅ Resuelto | G10/G92/set_zero fijan WCO contra `plannedPos` |
| **N21** | Alto | ✅ Resuelto | Heartbeat del cliente cada 250 ms en ambos modos |
| **N22** | Alto | ✅ Resuelto | `ok` diferido (firmware) + reintento y feed-hold suave (cliente) |
| **C5** | Alto | ✅ Resuelto | Límite de intentos AUTH, timeout de gracia, sin log del token, write timeout TCP |
| **N6** | Medio | ✅ Resuelto | `isfinite()` en `inLim` |
| **N14** | Medio | ✅ Resuelto | `$J` absoluto aplica WCO |
| **N15** | Medio | ✅ Resuelto | Solo se cierra `pos=target` si el error es sub-paso; error mayor queda visible |
| **N16** | Medio | ✅ Resuelto | WCO en telemetría `ST|`; UI muestra WPos |
| **N17** | Medio | ✅ Resuelto | Calibración/reset invalidan `homed` |
| **N25** | Medio | ✅ Resuelto | Arcos no bloqueantes, validación de radio, G53 |
| **N26** | Medio | ✅ Resuelto | Jog GRBL con feed configurado y cola acotada |
| **A5** | Medio | ✅ Resuelto | Forward pass + junction deviation estándar |
| **A1r** | Medio | ✅ Resuelto | 0x85 replanifica solo ejes con jog cancelado |
| **C2r** | Medio | ✅ Resuelto | TCP write timeout; corte de potencia documentado |
| **M5** | Medio | ✅ Resuelto | Logging de excepciones; cierre de cámara con espera real |
| **M2** | Medio | ◑ Documentado | Piso de 80 µs / generación por RMT/MCPWM (mejora futura) |
| **M3** | Medio | ◑ Documentado | Reasignación de pines strapping / pull-ups (hardware) |
| **M7** | Medio | ◑ Parcial | Lazo abierto; detección de pérdida de pasos (mejora futura) |
| **M8** | Medio | ◑ Parcial | Versión visible; modularización/tests (mejora continua) |

**Leyenda:** ✅ Resuelto · ◑ Mitigado/documentado (limitación de hardware o trabajo futuro)

---

## 10. Puesta en marcha y operación

### 10.1 Requisitos
- Firmware: Arduino IDE / PlatformIO con soporte ESP32 (núcleo ESP32 ≥ 2.x).
- Cliente: `pip install -r requirements.txt` (PySide6, opencv-python, numpy, pyserial).

### 10.2 Configuración inicial
1. Flashear `CNC_V5.ino` en el ESP32.
2. Configurar WiFi por serial: `SET_WIFI_NVS|<ssid>|<pass>|<nombre>|<ip_oct4>`.
3. (Opcional, recomendado) Fijar token: `SET_AUTH_TOKEN|<token>`.
4. Verificar versión: `GET_VERSION` → `VER|5.1.0`.
5. Conectar el cliente a `<ip ESP32>:5000`.

### 10.3 Secuencia operativa
1. **Activar actuadores** (botón o `$X`). Si el E-stop está pulsado, el firmware rechaza.
2. **Homing** por eje (`HOMING` / `$H`) — obligatorio tras calibrar o resetear contador.
3. **Set Zero** por eje (fija el WCO).
4. Jog manual (D-Pad) o carga de programa G-code / secuencia.
5. E-stop físico o botón DETENER/E-stop en la UI ante cualquier anomalía.

### 10.4 Verificación post-cambio (sin hardware)
- Cliente: `python -m py_compile CNC_V5.py` (OK).
- Firmware: revisión estática (balance de llaves, ausencia de `FALLING`, formato `ST|` con 29 campos, etc.).
- **Pendiente:** pruebas en hardware real (E-stop NC, homing, arcos, programa G-code > 16 líneas).

---

## 11. Riesgos residuales y recomendaciones

1. **Corte de potencia del E-stop (C2r):** implementar el corte en serie con la alimentación de los drivers; el ISR actual solo deshabilita ENABLE.
2. **Token en claro (C5):** la autenticación es robusta en lógica, pero el canal es TCP plano. Operar en red segmentada/VLAN o añadir TLS si se expone a redes no confiables.
3. **Lazo abierto (M7):** sin encoders; la pérdida de pasos no se detecta (el error > 0.5 pasos ahora queda visible en `pos`/`stepCount`). Considerar encoders o detección de stall en ejes críticos.
4. **Piso de paso de 80 µs (M2):** para velocidades superiores, migrar la generación de pasos a RMT/MCPWM o ISR dedicada.
5. **Pruebas en hardware:** validar la polaridad del E-stop con el cableado real antes de operar sin supervisión (verificar que pulsado = ALARM y liberado = permite `$X`).

---

## 12. Mantenimiento y extensibilidad

- **Parámetros por eje** (persistidos en NVS): `stepsPerMm`, `maxTravel`, `backlashMm`, `accelMmS2`, `maxSpeedMmS`, perfil S-curve, tiempos de homing/jog. Editables por TCP (`CMD|set_*`), por serial (`LOAD_AXIS_PARAM|`) o con `CNC_MAINT_V5.py`.
- **Versión de firmware** consultable por `GET_VERSION` / `$I` y visible en el título de la HMI.
- **Puntos de extensión sugeridos:** pruebas unitarias del parser G-code, telemetría WPos ya disponible para un DRO completo, hooks para encoders (M7).

---

*Documento generado como parte de la remediación de la Auditoría CNC XYZW V5 — sistema completo (firmware + cliente) con scoring objetivo > 80/100 y sin pérdida de funcionalidad preexistente.*
