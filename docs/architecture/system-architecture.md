# CitySync RT+ – System Architecture

## 1. Project Overview

CitySync RT+ is a QNX SDP 8.0-based real-time sensor monitoring system implemented on Raspberry Pi 4.

The system acquires data from multiple sensors through the Data Acquisition program and transfers the acquired information to TwinSync using QNX Native Message Passing. TwinSync receives and validates sensor messages, updates subsystem state, monitors freshness and performance, and displays results through a Command Line Interface (CLI).

## 2. System Architecture

The system consists of the physical sensor layer, Data Acquisition layer, QNX Native Message Passing communication layer, TwinSync processing layer, and CLI monitoring layer.

See the [hardware connections](../hardware-connections.md) and [data acquisition flowchart](../data-acquisition-flowchart.md).

## 3. Hardware Layer

| Component | Function |
|---|---|
| Raspberry Pi 4 | Main processing platform |
| IR Sensor | Vehicle detection |
| Flame Sensor | Fire detection |
| HC-SR04 Ultrasonic | Distance measurement |
| PCF8591 ADC | Analog-to-digital conversion |
| LM35 | Temperature measurement |
| Potentiometer | Simulated current input |

The Raspberry Pi 4 runs QNX SDP 8.0 and provides GPIO and I2C interfaces for sensor acquisition.

## 4. Data Acquisition Layer

**Source file: `src/DataAcquisition.c`**

Responsibilities:
1. Initialize GPIO and PCF8591 ADC.
2. Configure sensor input and output pins.
3. Establish a connection to TwinSync using `name_open()`.
4. Initialize the ADC mutex to protect shared ADC access.
5. Create five sensor acquisition threads.
6. Sample sensors according to configured intervals.
7. Prepare messages with subsystem IDs, sequence numbers, timestamps, values, and validity/status fields.
8. Transmit messages to TwinSync through `MsgSend()`.

### Sensor Threads

| Thread | Acquisition operation |
|---|---|
| Fire Thread | Reads flame sensor |
| Water Thread | Measures ultrasonic distance |
| Current Thread | Reads PCF8591 ADC channel 1 |
| Vehicle Thread | Reads IR sensor and counts detections |
| Temperature Thread | Reads PCF8591 ADC channel 0 |

## 5. Communication Layer – QNX Native Message Passing

| API | Responsibility |
|---|---|
| `name_open()` | Connects Data Acquisition to TwinSync |
| `MsgSend()` | Sends sensor messages |
| `MsgReceive()` | Receives messages in TwinSync |
| `MsgReply()` | Replies to the sending process |
| `MsgError()` | Rejects a received message with an error |

Communication flow:

```text
DataAcquisition.c
       |
       v
   name_open()
       |
       v
    MsgSend()
       |
       v
    MsgReceive()
       |
       v
    TwinSync.c
```

## 6. TwinSync Processing Layer

**Source file: `src/TwinSync.c`**

TwinSync is the central message-reception, synchronization, and monitoring component.

Its responsibilities include:
1. Register the `CitySyncTwin` channel using `name_attach()`.
2. Receive sensor messages using `MsgReceive()`.
3. Identify and validate message types and sizes.
4. Validate subsystem IDs, values, validity, and sequence ordering.
5. Reject malformed, invalid, duplicate, or out-of-order updates.
6. Maintain the latest accepted subsystem state.
7. Refresh data freshness and report sensor health.
8. Track latency, jitter, update gaps, deadline misses, and rejected messages.
9. Answer `CITYSYNC_MSG_GET_STATE` requests with a state response.

## 7. Monitoring and CLI Layer

### A. Sensor Monitoring
- Fire status
- Water distance
- Current input
- Vehicle count
- Temperature

### B. Fault Monitoring
- Sensor availability
- Data freshness
- Invalid or failed sensor readings
- Configured fault conditions

### C. Analytics Summary
- Accepted updates
- Rejected messages
- Minimum latency
- Average latency
- Maximum latency
- Maximum jitter
- Update gap misses
- Deadline misses

## 8. Software Architecture Summary

| Layer | Main component | Responsibility |
|---|---|---|
| Hardware | Raspberry Pi 4 and sensors | Physical data acquisition |
| Data Acquisition | `DataAcquisition.c` | Read and prepare sensor data |
| Communication | QNX Native Message Passing | Transfer messages between programs |
| Synchronization and Monitoring | `TwinSync.c` | Receive, validate, and update sensor information |
| Monitoring Output | CLI | Display sensor and performance results |

## 9. Technologies Used

- Operating System: QNX SDP 8.0
- Hardware Platform: Raspberry Pi 4
- Programming Language: C
- Communication: QNX Native Message Passing
- Sensor Interfaces: GPIO and I2C
- Analog-to-Digital Conversion: PCF8591
- Thread Management: QNX Native Thread APIs
- Synchronization: QNX Native Mutex APIs
- Timing: POSIX Clock APIs
- Scheduling: POSIX Scheduling APIs

## 10. Conclusion

CitySync RT+ follows a two-program architecture. DataAcquisition handles hardware initialization, multithreaded sensor acquisition, and message transmission. TwinSync receives and validates sensor messages, maintains subsystem state, monitors freshness and performance, and provides CLI output. QNX Native Message Passing connects the two programs.
