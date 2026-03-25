# RISE Motion – Entwicklungsdokumentation

## Inhaltsverzeichnis
1. [Build & Ausführen](#1-build--ausführen)
2. [Projektstruktur](#2-projektstruktur)
3. [Architekturübersicht](#3-architekturübersicht)
4. [Komponenten](#4-komponenten)
5. [ROS2 Interface](#5-ros2-interface)
6. [Admittanzregelung](#6-admittanzregelung)
7. [Entwicklungsplan](#7-entwicklungsplan)

---

## 1. Build & Ausführen

### Voraussetzungen
- ROS2 Jazzy
- SOEM Submodul initialisiert:
  ```bash
  git submodule init
  git submodule update
  ```

### Bauen
```bash
cd rise_motion_dev_ws
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/setup.bash
```

Das Skript `rise_motion_dev_ws/build.sh` führt alle drei Schritte aus.

```bash
cd ~/rise-motion/rise_motion_dev_ws
./build.sh
```

### Netzwerkrechte setzen (einmalig nach Build)
EtherCAT braucht Raw-Socket-Zugriff:
```bash
sudo setcap cap_net_admin,cap_net_raw+eip build/rise_motion/rise_motion_main
```

### Ausführen (mit Hardware)
Setzt eine EtherCAT-Verbindung auf Interface `enp1s0` voraus (Linux-PC, **nicht WSL2**):
```bash
# Terminal 1 – EtherCAT Node starten (benötigt Raw-Socket-Rechte)
sudo -E ros2 run rise_motion rise_motion_main

# Terminal 2 – Test-Node starten
# Ruft /enable_ethercat automatisch auf, bevor Commands gesendet werden.
# Optionales Argument: Inkrement pro Zyklus (default: 10, besser: 1)
ros2 run rise_motion testing_node 1
```

Der `testing_node` ruft `/enable_ethercat` selbst auf – kein manueller Service-Call nötig. Der Node wartet blockierend bis der Service verfügbar ist (`waiting for service to appear...`).

Manueller Service-Call (z.B. zur Diagnose ohne testing_node):
```bash
ros2 service call /enable_ethercat rise_motion_messages/srv/EnableEthercatSrv "{enable: true}"
```

Der `/enable_ethercat`-Call löst intern automatisch aus:
1. EtherCAT-Zustände `INIT → PRE_OP → SAFE_OP → OPERATIONAL`
2. Mode of Operation → `CyclicSyncPositionMode`
3. CiA402-State-Machine → `OPERATION_ENABLED`

SDO-Calls (`/sdo_read`, `/sdo_write`) sind optional und dienen nur zur Diagnose einzelner Drive-Register.

### Ausführen (ohne Hardware, WSL2)
```bash
# Immer:
source install/setup.bash

# Terminal 1
ros2 run rise_motion rise_motion_mock

# (Terminal 2) Not needed!
ros2 service call /enable_ethercat rise_motion_messages/srv/EnableEthercatSrv "{enable: true}"

# Terminal 3
ros2 run rise_motion testing_node

# Terminal 4: Motor position validieren
ros2 topic echo /motor_feedback
```

Der Mock simuliert einen Motor im RAM: Kommandos werden sofort als Feedback zurückgegeben. Kein EtherCAT, kein SOEM, keine Netzwerkrechte erforderlich.

---

## 2. State Machines

Beim Start müssen zwei voneinander unabhängige State Machines hochgefahren werden.

### EtherCAT Bus States (SOEM)

Zustand des **Kommunikationsbusses** – unabhängig von den Motoren.

```
INIT → PRE_OP → SAFE_OP → OPERATIONAL
```

| State | Was passiert |
|---|---|
| `INIT` | Bus initialisiert, keine Kommunikation |
| `PRE_OP` | SDO-Kommunikation möglich (Konfiguration lesen/schreiben) |
| `SAFE_OP` | PDOs werden empfangen, aber noch nicht gesendet – Motoren kriegen noch keine Commands |
| `OPERATIONAL` | PDOs bidirektional aktiv – Motoren können gesteuert werden |

`init_ec()` bringt den Bus bis `SAFE_OP`. `cyclic_loop()` bringt ihn auf `OPERATIONAL`.

### CiA402 Drive States (pro Motor)

Zustand **jedes einzelnen Servo-Drives** (Synapticon Circulo 9). Wird über das `Controlword`-PDO gesteuert, aktueller State steht im `Statusword`.

```
NOT_READY_TO_SWITCH_ON
        ↓
SWITCH_ON_DISABLED
        ↓
READY_TO_SWITCH_ON
        ↓
SWITCHED_ON
        ↓
OPERATION_ENABLED  ← hier laufen die Motoren
```

Fehler-States:
- `QUICK_STOP_ACTIVE` – Notbremse aktiv
- `FAULT_REACTION_ACTIVE` → `FAULT` – Drive hat einen Fehler, muss manuell zurückgesetzt werden

### Startreihenfolge

```
enable_ethercat Service →
  init_ec():
    Bus: INIT → PRE_OP → SAFE_OP

  cyclic_loop():
    Bus: SAFE_OP → OPERATIONAL
    Motoren: Mode = CyclicSyncPositionMode
    Motoren: → OPERATION_ENABLED
    → 1kHz Loop startet
```

EtherCAT muss `OPERATIONAL` sein, bevor die CiA402-State-Machine auf `OPERATION_ENABLED` geht.

**Fehlerverhalten:**
- Motor im `FAULT`-State → `transition_motors_to()` bricht ab, Node fährt herunter (`Motor X in fault` im Log)
- Im cyclic loop wird **jeden Zyklus** geprüft ob alle Motoren `OPERATION_ENABLED` sind – wenn nicht, sofortiger Shutdown

**FAULT zurücksetzen:**

Tritt `Motor X in fault` auf, muss der FAULT-State zurückgesetzt werden bevor der Drive wieder in `OPERATION_ENABLED` gebracht werden kann.

Einfachster Weg: **Power-Cycle des Synapticon Circulo** (Strom aus/ein). Faults werden bei Stromverlust zurückgesetzt.

Alternativ per Controlword (Bit 7 = Fault Reset, CiA402 Index `0x6040`). Funktioniert nur wenn der Bus bereits `OPERATIONAL` ist:
```bash
ros2 service call /sdo_write rise_motion_messages/srv/SDOWriteSrv \
  "{device_id: 1, index: 0x6040, subindex: 0x00, value: [0x80]}"
```

Mögliche Ursachen für FAULT: unsauberer Shutdown beim letzten Lauf, Überstrom/Überspannung, oder Drive war beim Start nicht in Ruheposition.

**Für die Admittanzregelung:** Wechsel von `CyclicSyncPositionMode` auf `CyclicSyncVelocityMode` (Mode 9) – an der Stelle `m.set_mode_of_operation(...)` in `cyclic_loop()`.

---

## 3. Architekturübersicht

```
┌─────────────────────────────────────────────────────────────┐
│                        ROS2 Thread                          │
│                                                             │
│   [AdmittanzNode]  ──cmd──►  [EthercatNode]                │
│        ▲                          │                         │
│        │ feedback                 │ cmd                     │
│        └──────────────────────────┘                         │
└──────────────────────────┬──────────────────────────────────┘
                           │  APSA (lock-free)
┌──────────────────────────▼──────────────────────────────────┐
│                   EtherCAT Thread (1 kHz)                   │
│                                                             │
│              [ECManager::cyclic_loop()]                     │
│                      │                                      │
│        ┌─────────────▼─────────────┐                       │
│        │       SOEM / IgH           │                       │
│        └─────────────┬─────────────┘                       │
└──────────────────────┼──────────────────────────────────────┘
                       │ EtherCAT Bus
          ┌────────────┼────────────┐
     [Motor 1]    [Motor 2]   ... [Motor 6]
   Synapticon Circulo 9 (CiA402)
```

**Threads:**
- **ROS2-Thread:** `EthercatNode` empfängt Kommandos, veröffentlicht Feedback
- **EtherCAT-Thread:** `ECManager::cyclic_loop()` läuft mit 1 kHz, kommuniziert direkt mit den Drives

**Datenaustausch:** APSA (Atomic Pointer Swap Algorithm) – Triple-Buffer, komplett lock-free, kein Mutex zwischen den Threads.

---

## 4. Komponenten

### ECManager
Verwaltet den EtherCAT-Bus und den Cyclic Loop.

| Methode | Beschreibung |
|---|---|
| `init_ec()` | Bus initialisieren, Drives in OP-State bringen |
| `cyclic_loop()` | 1-kHz-Loop: PDO senden/empfangen, State-Machine |
| `is_running()` | Gibt zurück ob der Loop aktiv ist |
| `stop()` | Loop sauber beenden |
| `get_motor_values_apsa()` | Feedback aus EtherCAT-Thread lesen (ROS-Seite) |
| `set_motor_values_apsa()` | Kommando in EtherCAT-Thread schreiben (ROS-Seite) |
| `sdo_read/write()` | SDO-Zugriff für Drive-Konfiguration |

### EthercatNode
ROS2-Node als Brücke zwischen ROS-Welt und ECManager.

| Interface | Typ | Beschreibung |
|---|---|---|
| Sub: `motor_commands` | `MotorPositions` | Positionsbefehle empfangen |
| Pub: `motor_feedback` | `MotorPositions` | Motorpositionen publizieren |
| Srv: `enable_ethercat` | `EnableEthercatSrv` | EtherCAT starten/stoppen |
| Srv: `sdo_read` | `SDOReadSrv` | SDO-Register lesen |
| Srv: `sdo_write` | `SDOWriteSrv` | SDO-Register schreiben |

### MockECManager
Implementiert `IECManager` ohne SOEM-Abhängigkeit. Läuft in WSL2 ohne Hardware.

| Parameter | Default | Beschreibung |
|---|---|---|
| `cycle_period_ms` | 1 | Zykluszeit in ms |
| `num_motors` | 1 | Anzahl simulierter Motoren |
| `alpha` | 1.0 | Tracking-Faktor (1.0 = sofortiges Echo, <1.0 = Lag) |

`init_ec()` gibt sofort `EXIT_SUCCESS` zurück. `cyclic_loop()` liest Kommandos aus `cmd_apsa`, berechnet `pos += alpha * (cmd - pos)` und schreibt das Ergebnis in `feedback_apsa`.

### CiA402Motor
Implementiert das CiA402-Antriebsprofil (IEC 61800-7-201) für Synapticon Circulo 9.

**State Machine:** `NOT_READY_TO_SWITCH_ON` → `SWITCH_ON_DISABLED` → `READY_TO_SWITCH_ON` → `SWITCHED_ON` → `OPERATION_ENABLED`

**Betriebsmodi (ModeOfOperation):**
- `CyclicSyncPositionMode` (8) – aktuell verwendet
- `CyclicSyncVelocityMode` (9) – für Admittanzregelung geplant
- `CyclicSyncTorqueMode` (10)
- `ImpedanceMode` (-6), `JointTorqueMode` (-5) – Synapticon-spezifisch

**PDO-Daten (Inputs von Drive):** Statusword, Position, Velocity, Torque, AnalogInput1–4, Timestamp
**PDO-Daten (Outputs an Drive):** Controlword, OpMode, TargetPosition, TargetVelocity, TargetTorque

### APSA (Lock-free Buffer)
Triple-Buffer für sicheren, blockierungsfreien Datenaustausch zwischen ROS- und EtherCAT-Thread.

| Methode | Seite | Beschreibung |
|---|---|---|
| `comm_write()` | ROS-Thread | Kommando schreiben |
| `comm_read()` | ROS-Thread | Feedback lesen |
| `perf_write()` | EtherCAT-Thread | Feedback schreiben |
| `perf_read()` | EtherCAT-Thread | Kommando lesen |

**Verwendung im Projekt:**
- `cmd_apsa`: ROS → EtherCAT (Motorkommandos)
- `feedback_apsa`: EtherCAT → ROS (Motorpositionen)

---

## 5. ROS2 Interface

### Messages

**`MotorPositions.msg`**
```
int32[] positions   # Motorpositionen (encoder-Inkremente), 6 Achsen
int8 target
```

**Achsen-Mapping** (Index 0–5):
| Index | Achse |
|---|---|
| 0 | Hüfte Abduktion/Adduktion Links |
| 1 | Hüfte Abduktion/Adduktion Rechts |
| 2 | Hüfte Flexion/Extension Links |
| 3 | Hüfte Flexion/Extension Rechts |
| 4 | Knie Flexion/Extension Links |
| 5 | Knie Flexion/Extension Rechts |

### Services

**`EnableEthercatSrv.srv`**
```
bool enable
---
int8 status_enable
```

**`SDOReadSrv.srv`**
```
uint16 device_id
uint16 index
uint8 subindex
uint8 value_type
---
int8 status_code
uint16 device_id
uint16 index
uint8 subindex
uint8 value_type
uint8[] value
```

---

## 6. Admittanzregelung

### Regelungsgesetz
```
M_v * v̇ + D_v * v + K_v * q = τ_int

Euler-Integration (dt = 1 ms):
  v̇  = (τ_int - D_v * v - K_v * q) / M_v
  v  += v̇ * dt
  q  += v  * dt

pos_target = pos_current + q
```

**Parameter:**
| Symbol | Bedeutung | Transparenter Modus |
|---|---|---|
| M_v | Virtuelle Masse | > 0 |
| D_v | Virtuelle Dämpfung | > 0 |
| K_v | Virtuelle Steifigkeit | **0** (transparent) |

### Kraftsensor
- Wägezelle zwischen Manschette und Exoskelett
- Moment: `τ = F * l` (konstanter Hebelarm `l`)
- ADC-Wert in `AnalogInput1` (und ggf. `AnalogInput2`) der CiA402-PDOs
- Kalibrierung: ADC-Ticks → Newton → Nm (Woche 3)

### Geplante ROS2-Nodes

| Node | Datei | Status |
|---|---|---|
| `EthercatNode` | `ethercat_node.cpp` | vorhanden |
| `AdmittanzNode` | *geplant* | Woche 1 |
| `TestNode` | `testing_node.cpp` | vorhanden (Positions-Inkrement) |

### Offene Punkte
- Velocity-Commands an Servo: restlicher RISE-OS Stack arbeitet mit Positionscommands → Integration noch offen
- Geplantes Topic `motor_feedback_full` mit `WrenchMsg` (Woche 2)

### Safety (minimal)
- `v_max`: maximale Gelenkgeschwindigkeit
- `dv_max`: maximale Beschleunigung
- Watchdog: bei ausbleibendem Kraftsensor-Signal stoppen

---

## 7. Entwicklungsplan

### Woche 1 – Mock & Grundstruktur
- [x] Mock `ECManager` (lokale Tests ohne Hardware)
- [ ] `AdmittanzNode` mit Fake-Kraft-Input (`F = 10 * sin(2π * 0.5 * t)`)
- [ ] Mathematik validieren (Plots)

### Woche 2 – Kraftsensor-Integration (Software)
- [ ] `FeedbackData` Struct erweitern (AnalogInput1/2 aus CiA402 PDO)
- [ ] ECManager: Analog Inputs in `feedback_apsa` durchleiten
- [ ] Neue Message `WrenchMsg.msg` definieren

### Woche 3 – Hardware-Tests
- [ ] Echten Kraftsensor testen (Linux-PC, Interface `enp1s0`)
- [ ] Kalibrierung (ADC-Ticks → Newton → Nm)
- [ ] Erste Admittanz-Tests am Exoskelett

### Woche 4+ – Integration & Tuning
- [ ] Parameter M, D, K über ROS2 Parameter Server konfigurierbar
- [ ] Integration in RISE-OS Stack (`motor_command_safe`)
- [ ] Safety-Logik: v_max, pos_limits, Watchdog, Notaus
