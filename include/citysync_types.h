#ifndef CITYSYNC_TYPES_H
#define CITYSYNC_TYPES_H
#include <stdint.h>

typedef enum {
    SUBSYSTEM_TRAFFIC=1, SUBSYSTEM_ENERGY=2, SUBSYSTEM_WATER=3,
    SUBSYSTEM_ENVIRONMENT=4, SUBSYSTEM_FIRE=5
} SubsystemID;

#define CITYSYNC_SUBSYSTEM_COUNT 5
#define CITYSYNC_SUBSYSTEM_ARRAY_SIZE 6

#define CITYSYNC_MSG_UPDATE 1
#define CITYSYNC_MSG_GET_STATE 2
#define CITYSYNC_MSG_UPDATE_FLAME 3
#define CITYSYNC_MSG_SENSOR_STATUS 4

#define CITYSYNC_DATA_UNAVAILABLE 0
#define CITYSYNC_DATA_ACTIVE 1
#define CITYSYNC_DATA_STALE 2

#define CITYSYNC_SENSOR_STATUS_FAILED 0
#define CITYSYNC_SENSOR_STATUS_OK 1
#define CITYSYNC_SENSOR_STATUS_NO_ECHO 2

/* Sampling intervals */
#define TRAFFIC_INTERVAL_MS 500U
#define FIRE_INTERVAL_MS 100U
#define ENERGY_INTERVAL_MS 200U
#define WATER_INTERVAL_MS 300U
#define ENVIRONMENT_INTERVAL_MS 1000U

/* Requested QNX SCHED_FIFO priorities; confirm success at runtime. */
#define CITYSYNC_PRIORITY_FIRE 60
#define CITYSYNC_PRIORITY_WATER 55
#define CITYSYNC_PRIORITY_ENERGY 50
#define CITYSYNC_PRIORITY_TRAFFIC 45
#define CITYSYNC_PRIORITY_ENVIRONMENT 40
#define CITYSYNC_PRIORITY_TWINSYNC 63
#define CITYSYNC_PRIORITY_MONITORING 30
#define CITYSYNC_PRIORITY_DIAGNOSTICS 10

/* Timer pulse delivery priority, not an application thread priority. */
#define CITYSYNC_TIMER_PULSE_PRIORITY 10

#define TRAFFIC_DEADLINE_TOLERANCE_MS 50U
#define FIRE_DEADLINE_TOLERANCE_MS 50U
#define ENERGY_DEADLINE_TOLERANCE_MS 50U
#define WATER_DEADLINE_TOLERANCE_MS 50U
#define ENVIRONMENT_DEADLINE_TOLERANCE_MS 100U

#define TRAFFIC_TIMEOUT_MS 1500U
#define FIRE_TIMEOUT_MS 500U
#define ENERGY_TIMEOUT_MS 600U
#define WATER_TIMEOUT_MS 900U
#define ENVIRONMENT_TIMEOUT_MS 3000U

#define CITYSYNC_TARGET_LATENCY_MS 20U
#define CITYSYNC_REPORT_INTERVAL_MS 1000U
#define CITYSYNC_MONITOR_INTERVAL_MS 10U

typedef struct {
    int subsystem_id;
    uint32_t sequence_number;
    uint64_t timestamp_ms;
    uint64_t timestamp_us;
    float value;
    int valid;
    int freshness;
} SubsystemData;

typedef struct {
    int type;
    SubsystemData data;
} CitySyncMessage;

typedef struct {
    int type;
    uint32_t sequence_number;
    uint64_t timestamp_ms;
    uint64_t timestamp_us;
    int detected;
    int valid;
    int freshness;
} CitySyncFlameMessage;

typedef struct {
    int type;
    int subsystem_id;
    uint32_t sequence_number;
    uint64_t timestamp_ms;
    int status;
    int error_code;
} CitySyncSensorStatusMessage;

typedef struct { int type; } CitySyncRequest;

typedef struct {
    int type;
    SubsystemData data[CITYSYNC_SUBSYSTEM_ARRAY_SIZE];
    CitySyncFlameMessage flame;
} CitySyncStateResponse;

#endif
