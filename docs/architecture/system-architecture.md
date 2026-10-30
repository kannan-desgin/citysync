# CitySync RT+ — System Architecture

## 1. Project overview

CitySync RT+ is a QNX SDP 8.0 real-time monitoring and synchronization application for Raspberry Pi 4. It collects sensor readings from five domains, transfers updates through QNX native message passing, maintains latest state in TwinSync, monitors freshness/performance, and controls three independent LED alerts.

## 2. Application processes

| Process | Responsibility |
|---|---|
| DataAcquisition | GPIO/I2C setup, five sensor threads, LED outputs, timestamps, message transmission |
| TwinSync | Message reception/validation, subsystem and Fire state, freshness checks, performance reports |

DataAcquisition has its main thread plus five sensor worker threads: Fire, Water, Current, Vehicle, Temperature. TwinSync processes messages and timer pulses in its main receive loop. The LEDs are controlled by existing acquisition threads; they are not extra processes or threads.

## 3. Hardware layer

| Component | Interface | Purpose |
|---|---|---|
| Flame sensor | GPIO27 | Fire detection, active-low |
| IR sensor | GPIO17 | Vehicle detection, active-low |
| HC-SR04 | TRIG GPIO23 / ECHO GPIO24 | Distance measurement |
| PCF8591 ADC | I2C /dev/i2c1, address 0x48 | Analog conversion |
| LM35 | PCF8591 AIN0 | Temperature input |
| Potentiometer | PCF8591 AIN1 | Simulated current input |
| LED1 | GPIO5 / physical pin 29 | Flame alert |
| LED2 | GPIO6 / physical pin 31 | Distance threshold alert |
| LED3 | GPIO13 / physical pin 33 | Simulated-current alert |

The potentiometer is scaled to simulated current; it is not a calibrated current measurement. HC-SR04 measures distance, not tank water height.

## 4. Data acquisition layer

Source: src/DataAcquisition.c

Responsibilities:
1. Install SIGINT/SIGTERM handling using SignalAction().
2. Initialize GPIO and configure sensors and LED output directions.
3. Initialize PCF8591 on /dev/i2c1.
4. Connect to TwinSync using name_open().
5. Create ADC and LED-state QNX mutexes.
6. Create five sensor worker threads using ThreadCreate().
7. Read sensors at configured sampling intervals.
8. Update local LEDs from Fire, Water, and Current thread logic.
9. Timestamp and send messages using MsgSend().
10. Join workers and turn LEDs OFF on normal shutdown.

| Thread | Input / calculation | Message | Local LED |
|---|---|---|---|
| Fire | Active-low Flame on GPIO27 | Dedicated Flame update | LED1 GPIO5 ON on detection |
| Water | HC-SR04 distance | Standard Water update | LED2 GPIO6 ON if valid distance > 30 cm |
| Current | PCF8591 AIN1 scaled to simulated mA | Standard Energy update | LED3 GPIO13 ON if simulated current > 1500 mA |
| Vehicle | IR GPIO17 edge count | Standard Traffic update | — |
| Temperature | PCF8591 AIN0 / LM35 conversion | Standard Environment update | — |

A dedicated mutex protects independent LED state variables and GPIO writes. This allows multiple LEDs to remain ON simultaneously. Outputs initialize OFF and are turned OFF on normal shutdown.

## 5. Sampling and scheduling

| Worker | Sampling interval [Cfg] | Requested SCHED_FIFO priority [Cfg] |
|---|---:|---:|
| Fire | 100 ms | 60 |
| Water | 300 ms | 55 |
| Current | 200 ms | 50 |
| Vehicle | 500 ms | 45 |
| Temperature | 1000 ms | 40 |
| TwinSync main receive loop | — | 63 |

Acquisition timing uses ClockTime(CLOCK_MONOTONIC) and absolute TimerTimeout() waits. TwinSync timer pulse delivery priority is 10; that pulse is not a separate application thread. Requested priority values must be confirmed on target at runtime.

## 6. QNX native message passing

DataAcquisition opens the CitySyncTwin named channel with name_open(). TwinSync registers the channel with name_attach().

| API | Responsibility |
|---|---|
| MsgSend() | Sends a message and waits for a reply |
| MsgReceive() | Receives a client message or timer pulse |
| MsgReply() | Replies to sender |
| MsgError() | Returns an error reply |

Message types:
- CITYSYNC_MSG_UPDATE — Traffic, Current, Water-distance, Temperature.
- CITYSYNC_MSG_UPDATE_FLAME — dedicated Flame detected/clear.
- CITYSYNC_MSG_SENSOR_STATUS — OK, FAILED, Water NO_ECHO.
- CITYSYNC_MSG_GET_STATE — state snapshot request.

TwinSync has a dedicated receive handler for the Flame message and maintains Fire state separately from the standard numeric subsystem state.

## 7. TwinSync processing

Source: src/TwinSync.c

TwinSync registers the channel, creates its periodic QNX timer, receives messages and pulses, validates type/status/value/timestamps/IDs/sequences, rejects invalid or out-of-order messages, maintains latest subsystem and Fire states, refreshes freshness, and updates the dashboard/performance summary.

Water NO_ECHO retains the last valid distance and marks it STALE; if no valid distance has ever existed, the state remains UNAVAILABLE. NO_ECHO is not proof of physical disconnection.

## 8. LED threshold logic

| Indicator | BCM GPIO | ON condition |
|---|---:|---|
| LED1 — Fire | GPIO5 | Flame input indicates detection |
| LED2 — Water / distance | GPIO6 | Valid measured distance > 30 cm |
| LED3 — Current | GPIO13 | Simulated current > 1500 mA |

LED2 compares sensor-to-surface distance, not calculated water height. LED3 uses a potentiometer-derived simulated current value, not a calibrated current-sensor value. Use one 2.2 kΩ series resistor per LED.

## 9. Performance summary

TwinSync reports Updates, MinLat, AvgLat, MaxLat, MaxJit, GapMiss, LatMiss, and Rejected. The configured 20 ms latency threshold is a monitoring target; actual target performance must be measured.

## 10. Conclusion

CitySync RT+ combines multi-rate sensor acquisition, QNX message-based synchronization, freshness/performance monitoring, and three independently controlled GPIO alert LEDs. The two application processes remain DataAcquisition and TwinSync.
