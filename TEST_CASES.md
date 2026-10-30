# CitySync RT+ – System Test Cases

## 1. Overview

This document describes the functional and timing test cases designed to verify the CitySync RT+ Digital Twin system running on QNX SDP 8.0.

Testing covers sensor acquisition, QNX message passing, synchronization, fault handling, ADC mutual exclusion and real-time performance monitoring.

## 2. Test Cases

| Test Scenario | Expected Result |
|---|---|
| Valid sensor update | TwinSync accepts the update and updates the sensor value and sequence number. |
| Sensor failure | Sensor status changes to UNAVAILABLE. |
| No update received within configured timeout | Sensor freshness changes to STALE. |
| Duplicate or old sequence number | Update is rejected and Rejected count increases. |
| Invalid measurement or timestamp | Invalid update is rejected. |
| Current and temperature threads access PCF8591 simultaneously | ADC mutex ensures mutually exclusive access. |
| Update latency exceeds 20 ms | Latency deadline miss (LatMiss) increases. |
| Inter-arrival interval exceeds expected period plus tolerance | GapMiss increases. |

## 3. Verification Parameters

The following parameters are monitored during testing:

- Sensor value and sequence number
- Sensor freshness: ACTIVE, STALE and UNAVAILABLE
- Minimum, average and maximum latency
- Maximum jitter
- Inter-arrival deadline misses (GapMiss)
- Latency deadline misses (LatMiss)
- Rejected messages

**Configured latency deadline:** 20 ms.

## 4. Testing Environment

- Operating System: QNX SDP 8.0
- Hardware: Raspberry Pi 4
- Communication: QNX Native Message Passing
- Synchronization: QNX Mutex
- ADC: PCF8591
- Monitoring: QNX Momentics System Profiler

## 5. Conclusion

These test cases provide a verification framework for evaluating sensor acquisition, inter-process communication, fault detection, synchronization and real-time performance in the CitySync RT+ system.
