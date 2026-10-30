# CitySync RT+ — Updated Data Acquisition Flow

The current program has five sensor threads. Fire, Water, and Current worker threads also control local LED outputs; the LEDs do not add separate processes or worker threads.

```mermaid
flowchart TD
  START["Start DataAcquisition"] --> INIT["Initialize GPIO, I2C, ADC and mutexes"]
  INIT --> PINS["Configure sensor inputs and LED outputs"]
  PINS --> OFF["Set all three LEDs OFF"]
  OFF --> THREADS["Create five QNX sensor threads"]

  THREADS --> FIRE["Fire · GPIO27"]
  THREADS --> WATER["Water · HC-SR04 GPIO23/24"]
  THREADS --> CURRENT["Current · PCF8591 AIN1"]
  THREADS --> VEHICLE["Vehicle · IR GPIO17"]
  THREADS --> TEMP["Temperature · PCF8591 AIN0"]

  FIRE --> FDET["Read active-low Flame input"]
  FDET --> L1["LED1 GPIO5 ON when Flame detected"]
  L1 --> FM["Send CITYSYNC_MSG_UPDATE_FLAME"]

  WATER --> DIST["Measure distance in cm"]
  DIST --> RANGE{"Valid 2–400 cm?"}
  RANGE -->|No / no echo| L2OFF["LED2 OFF; send NO_ECHO"]
  RANGE -->|Yes| GT30{"Distance > 30 cm?"}
  GT30 -->|Yes| L2ON["LED2 GPIO6 ON"]
  GT30 -->|No| L2CLEAR["LED2 GPIO6 OFF"]
  L2ON --> WM["Send Water distance update"]
  L2CLEAR --> WM

  CURRENT --> ADC["Read AIN1 under ADC mutex"]
  ADC --> SCALE["Scale ADC input to simulated mA"]
  SCALE --> GT1500{"Simulated current > 1500 mA?"}
  GT1500 -->|Yes| L3ON["LED3 GPIO13 ON"]
  GT1500 -->|No| L3OFF["LED3 GPIO13 OFF"]
  L3ON --> CM["Send Current update"]
  L3OFF --> CM

  VEHICLE --> COUNT["Count detection edges"]
  COUNT --> TM["Send Traffic update"]
  TEMP --> AIN0["Read AIN0 and convert LM35 value"]
  AIN0 --> EM["Send Temperature update"]

  FM --> IPC["QNX MsgSend"]
  WM --> IPC
  CM --> IPC
  TM --> IPC
  EM --> IPC
  IPC --> RX["TwinSync MsgReceive"]
  RX --> VALID["Validate messages and status"]
  VALID --> STATE["Update subsystem and Fire states"]
  STATE --> FRESH["ACTIVE / STALE / UNAVAILABLE"]
  STATE --> PERF["Latency, jitter, gaps, deadlines, rejected"]
  FRESH --> DASH["Monitoring dashboard"]
  PERF --> SUMMARY["Performance summary"]
```

## Sampling intervals [Cfg]

| Thread | Interval |
|---|---:|
| Fire | 100 ms |
| Current | 200 ms |
| Water | 300 ms |
| Vehicle | 500 ms |
| Temperature | 1000 ms |

## LED control summary

- Flame detection (active-low input) turns LED1 GPIO5 ON.
- Valid ultrasonic distance greater than 30 cm turns LED2 GPIO6 ON; invalid/no echo clears it.
- Simulated current greater than 1500 mA turns LED3 GPIO13 ON; ADC failure clears it.
- The states are protected by a QNX mutex, so multiple LEDs can stay ON independently.
- All LEDs initialize OFF and turn OFF on normal shutdown.

The ultrasonic threshold is a distance comparison, not a water-height measurement. The current threshold is based on a potentiometer-derived simulated value.

## TwinSync receive path

TwinSync handles CITYSYNC_MSG_UPDATE, CITYSYNC_MSG_UPDATE_FLAME, CITYSYNC_MSG_SENSOR_STATUS, and CITYSYNC_MSG_GET_STATE. It maintains Fire state separately and includes Fire in the dashboard and performance summary.
