#ifndef CITYSYNC_TYPES_H
#define CITYSYNC_TYPES_H

#include <stdint.h>

/* =========================================================
 * CitySync RT+ — Common Communication Protocol
 * Platform: QNX SDP 8.0 / Raspberry Pi 4
 * ========================================================= */

/* ---------------- Subsystem IDs ---------------- */

typedef enum {
    SUBSYSTEM_TRAFFIC     = 1,
    SUBSYSTEM_ENERGY      = 2,
    SUBSYSTEM_WATER       = 3,
    SUBSYSTEM_ENVIRONMENT = 4,
    SUBSYSTEM_FIRE        = 5
} SubsystemID;

#define CITYSYNC_SUBSYSTEM_COUNT       5
#define CITYSYNC_SUBSYSTEM_ARRAY_SIZE  6

/* ---------------- Message Types ---------------- */

#define CITYSYNC_MSG_UPDATE         1
#define CITYSYNC_MSG_GET_STATE      2
#define CITYSYNC_MSG_UPDATE_FLAME   3
#define CITYSYNC_MSG_SENSOR_STATUS  4

/* ---------------- Data Availability ---------------- */

#define CITYSYNC_DATA_UNAVAILABLE  0
#define CITYSYNC_DATA_ACTIVE       1
#define CITYSYNC_DATA_STALE        2

/* ---------------- Sensor Status ---------------- */

#define CITYSYNC_SENSOR_STATUS_FAILED   0
#define CITYSYNC_SENSOR_STATUS_OK       1
#define CITYSYNC_SENSOR_STATUS_NO_ECHO  2

/* =========================================================
 * Sensor Sampling Intervals
 *
 * Traffic     = 500 ms
 * Fire        = 100 ms
 * Current     = 200 ms
 * Water       = 300 ms
 * Temperature = 1000 ms
 * ========================================================= */

#define TRAFFIC_INTERVAL_MS      500U
#define FIRE_INTERVAL_MS         100U
#define ENERGY_INTERVAL_MS       200U
#define WATER_INTERVAL_MS        300U
#define ENVIRONMENT_INTERVAL_MS  1000U

/* ---------------- QNX Thread Priorities ----------------
 * Proposed priorities; verify successful runtime scheduling.
 * Higher numeric priority generally means higher priority
 * for QNX FIFO scheduling.
 */

#define CITYSYNC_PRIORITY_FIRE          60
#define CITYSYNC_PRIORITY_WATER         55
#define CITYSYNC_PRIORITY_ENERGY        50
#define CITYSYNC_PRIORITY_TRAFFIC       45
#define CITYSYNC_PRIORITY_ENVIRONMENT  40
#define CITYSYNC_PRIORITY_TWINSYNC      35
#define CITYSYNC_PRIORITY_MONITORING    30
#define CITYSYNC_PRIORITY_DIAGNOSTICS   10

/* =========================================================
 * Inter-arrival Deadline Tolerances
 *
 * TwinSync can use these to identify delayed updates.
 * They are tolerances, not execution-time guarantees.
 * ========================================================= */

#define TRAFFIC_DEADLINE_TOLERANCE_MS       50U
#define FIRE_DEADLINE_TOLERANCE_MS          50U
#define ENERGY_DEADLINE_TOLERANCE_MS        50U
#define WATER_DEADLINE_TOLERANCE_MS         50U
#define ENVIRONMENT_DEADLINE_TOLERANCE_MS  100U

/* =========================================================
 * Stale Data Timeouts
 *
 * Data becomes stale when its age exceeds its timeout.
 * ========================================================= */

#define TRAFFIC_TIMEOUT_MS       1500U
#define FIRE_TIMEOUT_MS           500U
#define ENERGY_TIMEOUT_MS         600U
#define WATER_TIMEOUT_MS           900U
#define ENVIRONMENT_TIMEOUT_MS   3000U

/* ---------------- Latency and Monitoring ---------------- */

/* Target message transport/processing latency */
#define CITYSYNC_TARGET_LATENCY_MS     20U

/* Dashboard/performance report interval */
#define CITYSYNC_REPORT_INTERVAL_MS    1000U

/* Monitoring timer interval */
#define CITYSYNC_MONITOR_INTERVAL_MS   10U

/* =========================================================
 * Standard Subsystem Data
 * ========================================================= */

typedef struct {
    int      subsystem_id;
    uint32_t sequence_number;
    /* Sensor acquisition timestamp */
    uint64_t timestamp_ms;
    uint64_t timestamp_us;
    float    value;
    /* Validity and freshness state */
    int      valid;
    int      freshness;
} SubsystemData;

/* ---------------- Standard Sensor Update ---------------- */

typedef struct {
    int type;
    SubsystemData data;
} CitySyncMessage;

/* =========================================================
 * Dedicated Fire Sensor Message
 * ========================================================= */

typedef struct {
    int      type;
    uint32_t sequence_number;
    uint64_t timestamp_ms;
    uint64_t timestamp_us;
    int      detected;
    int      valid;
    int      freshness;
} CitySyncFlameMessage;

/* =========================================================
 * Sensor Health / Error Status Message
 * ========================================================= */

typedef struct {
    int      type;
    int      subsystem_id;
    uint32_t sequence_number;
    uint64_t timestamp_ms;
    int      status;
    int      error_code;
} CitySyncSensorStatusMessage;

/* ---------------- State Request ---------------- */

typedef struct {
    int type;
} CitySyncRequest;

/* =========================================================
 * State Response
 *
 * Includes subsystem snapshots and dedicated fire state.
 * Array index 0 is reserved; subsystem IDs are 1–5.
 * ========================================================= */

typedef struct {
    int type;
    SubsystemData data[CITYSYNC_SUBSYSTEM_ARRAY_SIZE];
    CitySyncFlameMessage flame;
} CitySyncStateResponse;

#endif /* CITYSYNC_TYPES_H */
