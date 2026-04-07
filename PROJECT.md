# Projektdefinition: Autonom GPS-styrd Robotgräsklippare

## Sammanfattning

Projektet syftar till att bygga en autonom robotgräsklippare baserad på ett **Robomow RL2000**-chassi, styrd med centimeternoggrann **RTK-GPS**. Systemet består av två enheter — en stationär **basstation** och den mobila **rovern** — som båda kör samma ESP32-S3-firmware med rollbaserad konfiguration. Basstationen streamar RTCM-korrigeringsdata trådlöst via **ESP-NOW** till rovern, som matar in korrektionerna i sin GNSS-mottagare för RTK-fix. Roverns ESP32 kommunicerar över USB med en **Raspberry Pi** som kör **ROS 2 Jazzy** i Docker-containrar, där navigering, kinematik och styrlogik hanteras. Roboten har **differentialdrift** (två oberoende drivhjul + ett passivt stödhjul).

## Arkitektur

```
┌─────────────────────┐     ESP-NOW (wireless)     ┌─────────────────────┐
│   BASSTATION        │◄──────────────────────────►│   ROVER (ESP32-S3)  │
│   (ESP32-S3)        │  RTCM corrections + NMEA   │                     │
│   + GNSS-mottagare  │                            │  + GNSS-mottagare   │
└─────────────────────┘                            │  + Motorstyring     │
                                                   │  + LEDs             │
                                                   └────────┬────────────┘
                                                            │ USB Serial
                                                            │ (COBS + HDLC)
                                                   ┌────────▼────────────┐
                                                   │  microros_proxy.py  │
                                                   │  (Python, Docker)   │
                                                   └────────┬────────────┘
                                                            │ TCP :8888
                                                   ┌────────▼────────────┐
                                                   │ micro-ROS Agent     │
                                                   │ (Docker, Jazzy)     │
                                                   └────────┬────────────┘
                                                            │ DDS/ROS 2
                                                   ┌────────▼────────────┐
                                                   │  ROS 2 Core Stack   │
                                                   │  - robot_state_pub  │
                                                   │  - ros2_control     │
                                                   │  - diff_drive_ctrl  │
                                                   │  - nmea_navsat_drv  │
                                                   └─────────────────────┘
```

### Komponenter

| Komponent | Hårdvara | Uppgift |
|---|---|---|
| **Basstation** | ESP32-S3 + GNSS-mottagare | Läser GNSS, genererar RTCM-korrektioner, sänder via ESP-NOW |
| **Rover MCU** | ESP32-S3 ("LynxAdapter v1.0", custom PCB) | Realtidsreglering: motorstyring, GNSS-läsning, ESP-NOW-mottagare, micro-ROS-nod |
| **Rover SBC** | Raspberry Pi + Docker | ROS 2-stack: diff_drive_controller, navigering, positionering, planering |

### Kommunikation

| Länk | Protokoll | Beskrivning |
|---|---|---|
| Bas ↔ Rover | ESP-NOW | Trådlös direktlänk mellan ESP32-enheterna, längre räckvidd än WiFi |
| ESP32 ↔ Raspberry Pi | USB-serial med COBS+HDLC-framing | micro-ROS-transport med CRC-16, demuxad av Python-proxy |
| Proxy ↔ micro-ROS Agent | TCP :8888 | Standard micro-ROS agent-anslutning |
| ROS 2-noder | DDS | Standard ROS 2 middleware |

### Drivlina

- 2× drivhjul (höger/vänster) — oberoende styrda för differentialdrift
- 1× passivt stödhjul framtill (caster, liknande kontorsstol)
- Hjulradie: 0.127 m, hjulavstånd: 0.66 m
- Chassivikt: 23.7 kg, dimensioner: 0.9 × 0.66 × 0.315 m

## ESP32 Firmware (rover/baseboard/)

### Rollsystem

Samma firmware-image körs på **båda** ESP32-korten. Aktiv roll sparas i NVS (Preferences) och väljs vid boot:

| Roll | Klass | Beskrivning |
|---|---|---|
| **Unknown / CLI** | `CLI` | Seriell CLI för att välja roll (`b` = bas, `r` = rover), startar om efteråt |
| **Rover** | `Rover` | Huvudenheten — kör micro-ROS, motorer, GPS, ESP-NOW-mottagare |
| **Basstation** | `Basestation` | RTK-bas — läser GNSS, vidarebefordrar RTCM/NMEA via ESP-NOW |

### FreeRTOS Tasks

| Task | Roll | Syfte |
|---|---|---|
| `ledTask` | Alla | Boot-knapp-polling, LED-statushantering |
| `motorTask` | Rover | Motorstyrning (placeholder — tom loop) |
| `urosTask` | Rover | micro-ROS state machine (ping → connect → spin → reconnect) |
| `gnssReceiveTask` | Båda | UART-läsare för GNSS-modul vid 460800 baud; parsar RTCM och NMEA |
| `pairTask` | Båda | ESP-NOW-parningshandskaning med 60s timeout |

### Hårdvara (LynxAdapter v1.0)

| Pinnar | Funktion | Status |
|---|---|---|
| GPIO_0 | Boot-knapp | Implementerad |
| GPIO_1-2 | TWAI (CAN) TX/RX | Definierad, ej använd |
| GPIO_3-4 | UART RX/TX → GNSS | Implementerad |
| GPIO_5 | UART fault pin | Definierad, ej använd |
| GPIO_6-7 | I2C SDA/SCL | Definierad, ej använd |
| GPIO_40-42 | Status/Error/TWAI LED | Implementerad |
| GPIO_45-48 | LYNX A-D LED (anslutning & parning) | Implementerad |

### ROS 2-entiteter (micro-ROS)

| Entitet | Typ | Topic | Status |
|---|---|---|---|
| Publisher | `std_msgs/Int32` | `baseboard` | Aktiv — heartbeat (1 Hz) |
| Publisher | `nmea_msgs/Sentence` | `nmea_sentence` | Aktiv — råa NMEA-strängar från GNSS |
| Subscription | `geometry_msgs/Twist` | `/cmd_vel` | Skapad men bortkommenterad i executor |

### ESP-NOW-meddelanden

| Typ | Kod | Riktning | Syfte |
|---|---|---|---|
| `MSG_TYPE_PAIR_REQ` | 0xAA | Bas → Rover (broadcast) | Parningsförfrågan |
| `MSG_TYPE_PAIR_ACK` | 0xAB | Rover → Bas | Parningsbekräftelse |
| `MSG_TYPE_RTCM` | 0x03 | Bas → Rover | RTK-korrigeringsdata (chunked, max 250 byte/paket) |
| `MSG_TYPE_NMEA` | 0x04 | Bas → Rover | NMEA-strängar från bas-GNSS |
| `MSG_TYPE_FIRMWARE_REQ/RES` | 0x01/0x02 | — | Definierad, ej implementerad |

## ROS 2 Stack (rover/rover_description/)

### URDF (robomow_rl2000.xacro)

Modellerar en Robomow RL2000 med:
- Kropp, drivhjul (continuous joints), stödhjul (fixed joint)
- `ros2_control`-hårdvara: `mock_components/GenericSystem` (enbart simulering)
- `SimpleTransmission` för båda hjulen (1:1 utväxling)

### Launch (robot_control.launch.py)

Startar: `robot_state_publisher`, `ros2_control_node`, `joint_state_broadcaster`, `diff_drive_controller`, `nmea_navsat_driver`

### Controller-konfiguration

- **DiffDriveController**: wheel_separation=0.66m, wheel_radius=0.127m, 50 Hz, open_loop=true
- **JointStateBroadcaster**: Standard joint state-publicering

### Simuleringsscript

- `simple_diff_drive_sim.py` — fullständig mjukvarusimulering med kinematik
- `simple_wheel_cmd_simulator.py` — hardware-in-the-loop-simulering

## Docker-stack (docker-compose.yml)

| Tjänst | Image | Syfte |
|---|---|---|
| **core** | Custom (ros:jazzy-ros-base) | Fullständig ROS 2-stack |
| **microros_proxy** | Custom (python:3.13-slim) | USB-serial ↔ TCP-brygga |
| **micro_ros_agent** | microros/micro-ros-agent:jazzy | Officiell micro-ROS agent, TCP :8888 |

## Implementationsstatus

### Implementerat och fungerande

- Rollsystem med NVS-persistens och CLI-val
- ESP-NOW-parning med kanalscanning och MAC-persistens
- RTCM-vidarebefordran: bas-GNSS → ESP-NOW → rover-GNSS (RTK-korrektioner)
- GNSS-läsning på båda enheter (460 800 baud UART, RTCM + NMEA-parsing)
- NMEA-publicering till ROS 2 via micro-ROS (nmea_sentence-topic)
- micro-ROS-klient med komplett livscykel och COBS+HDLC-transport
- Python USB-proxy med protokoll-demux, CRC-validering, reconnect-logik
- URDF-modell av Robomow RL2000 med korrekta dimensioner
- ros2_control med mock-hårdvara + diff_drive_controller
- nmea_navsat_driver-integration (NMEA → NavSatFix)
- Docker multi-container-stack
- 7 status-LEDs
- 10 NMEA-meddelandetyper definierade som custom ROS 2-meddelanden

### Återstår att implementera

| Område | Status | Beskrivning |
|---|---|---|
| **Motorstyring** | Stub | motorTask är en tom loop; setVelocities() beräknar men aktuerar inget; /cmd_vel-subscription bortkommenterad |
| **CAN-buss (TWAI)** | Ej påbörjad | Pinnar definierade men ingen CAN-kod — troligen tänkt för motorstyrning |
| **Encoder-feedback** | Ej påbörjad | Encoder-länkar i URDF men ingen encoder-läsning; diff_drive_controller kör open_loop=true |
| **Riktig ros2_control HW-interface** | Ej påbörjad | Behöver custom HardwareInterface-plugin som bryggar till ESP32 |
| **NMEA-parsning** | Delvis | 9 strukturerade meddelandetyper definierade men bara Sentence publiceras |
| **Navigation/path planning** | Ej påbörjad | Ingen nav2-integration, inga gränsdefinitioner, ingen waypoint-planering |
| **Klippmotor** | Ej påbörjad | Ingen styrlogik för klippaggregatet |
| **Säkerhetssystem** | Ej påbörjad | Inga bumpers, tilt-sensorer, nödstopp, eller geofencVing |
| **OTA-uppdatering** | Definierad | ESP-NOW-meddelandetyper finns men ingen implementation |

## Fas-plan

1. **Fas 1 — Grundläggande körning** *(pågår)*: ESP32 tar emot `/cmd_vel` och styr drivhjulen, encoder-feedback ger odometri tillbaka till ROS 2
2. **Fas 2 — Positionering**: RTK-GPS → NavSatFix → fusionerad lokalisering (GPS + odometri), kartjustering
3. **Fas 3 — Autonom navigering**: Gränsdefinition (geofence), banplanering (coverage path planning), hinderundvikande
4. **Fas 4 — Komplett gräsklippare**: Klippmotorstyrning, säkerhetssystem (bumpers, tilt, nödstopp), laddstationsnavigering
5. **Fas 5 — Drift**: OTA-uppdateringar, fjärrövervakning, schemaläggning

## Teknisk stack

- **MCU**: ESP32-S3 (Arduino + PlatformIO, espressif32 6.11.0)
- **SBC**: Raspberry Pi
- **ROS 2**: Jazzy (micro-ROS + full desktop)
- **Containerisering**: Docker Compose (3 tjänster)
- **Trådlös länk**: ESP-NOW
- **GNSS**: Airoha/MediaTek-baserad mottagare, 460 800 baud, RTCM3 + NMEA
- **Chassi**: Robomow RL2000, differentialdrift

## Kodkvalitetsregler

Dessa regler MÅSTE följas vid all kodändring i projektet.

### Kompilator-flaggor (platformio.ini)

Följande varningsflaggor är aktiverade och ska inte stängas av:
- `-Wall -Wextra` — alla standardvarningar
- `-Werror=return-type` — **FEL** vid saknad return (kompilerar inte)
- `-Wshadow` — lokal variabel skuggar annan
- `-Wswitch-enum` — switch på enum missar case
- `-Wimplicit-fallthrough` — switch case fallthrough utan `[[fallthrough]]`
- `-Wuninitialized` — ej initierade variabler
- `-Wnull-dereference` — null-pekar-dereference
- `-Wformat=2` — strikt printf/snprintf-formatcheck
- `-Wdouble-promotion` — implicit float→double (prestandaproblem på ESP32)
- `-Wvla` — förbjud variable-length arrays (stack overflow-risk)

### FreeRTOS trådsäkerhet

- Varje delad variabel som nås från flera FreeRTOS-tasks **MÅSTE** skyddas med `std::atomic<>` (flaggor/enums) eller `SemaphoreHandle_t`/mutex (komplexa strukturer)
- Anrop `vTaskDelete(NULL)` — gör ALDRIG `vTaskDelete(ownHandle)`. Städa upp **FÖRE** self-delete, eller signalera tasken att avsluta via en flagga
- Alla `xTaskCreate`-anrop **MÅSTE** kontrollera returvärdet (`pdPASS`). Vid misslyckande: felindikering + reboot
- Använd `pdMS_TO_TICKS()` istället för manuell `/ portTICK_RATE_MS` (undviker heltalsdivision till 0)

### Minneshantering

- Varje `malloc`/`new` **MÅSTE** null-checkas
- Undvik Arduino `String`-klassen — använd fasta `char[]`-buffertar med `snprintf`. Definiera projektomfattande bufferstorlekskonstanter
- Undvik heap-allokering i tight loops och callbacks. Föredra pre-allokerade buffertar
- Alla buffertstorlekar, stackstorlekar och timeouts ska vara namngivna `constexpr`-konstanter, inte magiska siffror

### Felhantering

- Alla `esp_now_send`-anrop **MÅSTE** kontrollera returvärdet
- `RCSOFTCHECK`-makrot **MÅSTE** logga vid fel (aldrig tom felkropp)
- `RCCHECK`-makrot fungerar inte i lambdor/void-funktioner — returnera explicit och kontrollera
- Vid delvis misslyckad initiering, säkerställ att `destroy_entities()` bara finaliserar det som faktiskt skapades

### Kodstil

- Använd `static_cast<>` / `reinterpret_cast<>` — aldrig C-style casts `(Type*)x`
- Markera `static void task(void*)` som `private` — de är implementationsdetaljer
- En funktion per uppgift — undvik långrandiga funktioner som gör flera saker
- Kommentera alla `[[fallthrough]]` i switch-satser explicit
