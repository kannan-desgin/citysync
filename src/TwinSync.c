/*
 * TwinSync.c
 * CitySync RT+ - Real-Time State Synchronizer
 * Platform: QNX SDP 8.0 / Raspberry Pi 4
 *
 * Receives sensor messages from DataAcquisition.c through QNX native
 * message passing, validates them, maintains the latest subsystem state,
 * detects stale data, and reports synchronization performance.
 *
 * Protocol definitions are shared through ../include/citysync_types.h.
 */

#define _QNX_SOURCE
#define _POSIX_C_SOURCE 200809L

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <time.h>
#include <math.h>
#include <sys/neutrino.h>
#include <sys/dispatch.h>
#include <sys/netmgr.h>
#include <sys/siginfo.h>

#include "../include/citysync_types.h"

#define TWINSYNC_CHANNEL_NAME "CitySyncTwin"
#define TIMER_PULSE_CODE      (_PULSE_CODE_MINAVAIL + 1)
#define RECEIVE_BUFFER_SIZE   ((sizeof(CitySyncStateResponse) > sizeof(CitySyncSensorStatusMessage)) ? \
                               sizeof(CitySyncStateResponse) : sizeof(CitySyncSensorStatusMessage))

typedef struct {
    float value;
    uint32_t sequence_number;
    uint64_t source_timestamp_ms;
    uint64_t source_timestamp_us;
    uint64_t last_receive_ms;
    int valid;
    int freshness;
    int last_error;
} SubsystemState;

typedef struct {
    uint64_t accepted;
    uint64_t rejected;
    uint64_t latency_samples;
    uint64_t latency_sum_us;
    uint64_t min_latency_us;
    uint64_t max_latency_us;
    uint64_t max_jitter_us;
    uint64_t last_receive_us;
    uint64_t last_gap_us;
    uint64_t gap_misses;
    uint64_t latency_misses;
    uint64_t invalid_timestamps;
} PerformanceStats;

static SubsystemState state[CITYSYNC_SUBSYSTEM_ARRAY_SIZE];
static PerformanceStats performance[CITYSYNC_SUBSYSTEM_ARRAY_SIZE];
static volatile sig_atomic_t keep_running = 1;
static name_attach_t *name_attachment = NULL;
static int timer_channel = -1;
static int timer_connection = -1;
static timer_t monitor_timer;
static int timer_created = 0;

static void stop_handler(int signo)
{
    (void)signo;
    keep_running = 0;
}

static uint64_t monotonic_us(void)
{
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) == -1)
        return 0;
    return (uint64_t)ts.tv_sec * 1000000ULL +
           (uint64_t)ts.tv_nsec / 1000ULL;
}

static uint64_t monotonic_ms(void)
{
    return monotonic_us() / 1000ULL;
}

static const char *subsystem_name(int id)
{
    switch (id) {
        case SUBSYSTEM_TRAFFIC: return "Vehicle Count";
        case SUBSYSTEM_ENERGY: return "Current";
        case SUBSYSTEM_WATER: return "Water Distance";
        case SUBSYSTEM_ENVIRONMENT: return "Temperature";
        case SUBSYSTEM_FIRE: return "Fire";
        default: return "Unknown";
    }
}

static const char *freshness_name(int freshness)
{
    switch (freshness) {
        case CITYSYNC_DATA_UNAVAILABLE: return "UNAVAILABLE";
        case CITYSYNC_DATA_ACTIVE: return "ACTIVE";
        case CITYSYNC_DATA_STALE: return "STALE";
        default: return "UNKNOWN";
    }
}

static uint64_t expected_interval_ms(int id)
{
    switch (id) {
        case SUBSYSTEM_TRAFFIC: return TRAFFIC_INTERVAL_MS;
        case SUBSYSTEM_ENERGY: return ENERGY_INTERVAL_MS;
        case SUBSYSTEM_WATER: return WATER_INTERVAL_MS;
        case SUBSYSTEM_ENVIRONMENT: return ENVIRONMENT_INTERVAL_MS;
        case SUBSYSTEM_FIRE: return FIRE_INTERVAL_MS;
        default: return 0;
    }
}

static uint64_t stale_timeout_ms(int id)
{
    switch (id) {
        case SUBSYSTEM_TRAFFIC: return TRAFFIC_TIMEOUT_MS;
        case SUBSYSTEM_ENERGY: return ENERGY_TIMEOUT_MS;
        case SUBSYSTEM_WATER: return WATER_TIMEOUT_MS;
        case SUBSYSTEM_ENVIRONMENT: return ENVIRONMENT_TIMEOUT_MS;
        case SUBSYSTEM_FIRE: return FIRE_TIMEOUT_MS;
        default: return 0;
    }
}

static uint64_t deadline_tolerance_ms(int id)
{
    switch (id) {
        case SUBSYSTEM_TRAFFIC: return TRAFFIC_DEADLINE_TOLERANCE_MS;
        case SUBSYSTEM_ENERGY: return ENERGY_DEADLINE_TOLERANCE_MS;
        case SUBSYSTEM_WATER: return WATER_DEADLINE_TOLERANCE_MS;
        case SUBSYSTEM_ENVIRONMENT: return ENVIRONMENT_DEADLINE_TOLERANCE_MS;
        case SUBSYSTEM_FIRE: return FIRE_DEADLINE_TOLERANCE_MS;
        default: return 0;
    }
}

static int valid_subsystem_id(int id)
{
    return id >= SUBSYSTEM_TRAFFIC && id <= SUBSYSTEM_FIRE;
}

static void refresh_freshness(void)
{
    int id;
    uint64_t now = monotonic_ms();

    for (id = 1; id <= CITYSYNC_SUBSYSTEM_COUNT; ++id) {
        if (!state[id].valid) {
            state[id].freshness = CITYSYNC_DATA_UNAVAILABLE;
        } else if (now < state[id].last_receive_ms ||
                   now - state[id].last_receive_ms >= stale_timeout_ms(id)) {
            state[id].freshness = CITYSYNC_DATA_STALE;
        } else {
            state[id].freshness = CITYSYNC_DATA_ACTIVE;
        }
    }
}

static void update_performance(int id, const SubsystemData *data,
                               uint64_t receive_us)
{
    PerformanceStats *p = &performance[id];
    uint64_t expected_us = expected_interval_ms(id) * 1000ULL;

    p->accepted++;

    /* Latency is valid only when both timestamps use the same CLOCK_MONOTONIC
       clock domain. */
    if (data->timestamp_us == 0 || receive_us < data->timestamp_us) {
        p->invalid_timestamps++;
    } else {
        uint64_t latency = receive_us - data->timestamp_us;
        if (p->latency_samples == 0 || latency < p->min_latency_us)
            p->min_latency_us = latency;
        if (p->latency_samples == 0 || latency > p->max_latency_us)
            p->max_latency_us = latency;
        p->latency_sum_us += latency;
        p->latency_samples++;
        if (latency > (uint64_t)CITYSYNC_TARGET_LATENCY_MS * 1000ULL)
            p->latency_misses++;
    }

    if (p->last_receive_us != 0 && receive_us >= p->last_receive_us) {
        uint64_t gap = receive_us - p->last_receive_us;
        uint64_t jitter = gap >= expected_us ? gap - expected_us : expected_us - gap;
        p->last_gap_us = gap;
        if (jitter > p->max_jitter_us)
            p->max_jitter_us = jitter;
        if (expected_us != 0 &&
            gap > expected_us + deadline_tolerance_ms(id) * 1000ULL)
            p->gap_misses++;
    }
    p->last_receive_us = receive_us;
}

static void accept_update(const CitySyncMessage *msg)
{
    int id = msg->data.subsystem_id;
    uint64_t received_ms = monotonic_ms();
    uint64_t received_us = monotonic_us();

    if (!valid_subsystem_id(id)) {
        fprintf(stderr, "[REJECTED] Invalid subsystem ID: %d\n", id);
        return;
    }
    if (msg->type != CITYSYNC_MSG_UPDATE ||
        msg->data.valid != 1 ||
        !isfinite((double)msg->data.value) ||
        msg->data.timestamp_ms == 0) {
        performance[id].rejected++;
        fprintf(stderr, "[REJECTED] Invalid measurement from %s\n", subsystem_name(id));
        return;
    }
    if (state[id].valid &&
        msg->data.sequence_number <= state[id].sequence_number) {
        performance[id].rejected++;
        fprintf(stderr, "[REJECTED] Duplicate/out-of-order %s sequence=%u (latest=%u)\n",
                subsystem_name(id), msg->data.sequence_number,
                state[id].sequence_number);
        return;
    }

    update_performance(id, &msg->data, received_us);
    state[id].value = msg->data.value;
    state[id].sequence_number = msg->data.sequence_number;
    state[id].source_timestamp_ms = msg->data.timestamp_ms;
    state[id].source_timestamp_us = msg->data.timestamp_us;
    state[id].last_receive_ms = received_ms;
    state[id].valid = 1;
    state[id].freshness = CITYSYNC_DATA_ACTIVE;
    state[id].last_error = 0;
}

static void accept_sensor_status(const CitySyncSensorStatusMessage *msg)
{
    int id = msg->subsystem_id;

    if (!valid_subsystem_id(id)) {
        fprintf(stderr, "[REJECTED] Invalid sensor-status subsystem ID: %d\n", id);
        return;
    }

    state[id].last_error = msg->error_code;

    if (msg->status == CITYSYNC_SENSOR_STATUS_FAILED) {
        /* Preserve the last numeric value for diagnostics, but do not treat
           it as valid for safety decisions. */
        state[id].valid = 0;
        state[id].freshness = CITYSYNC_DATA_UNAVAILABLE;
        fprintf(stderr, "[FAULT] %s sensor failed (error=%d)\n",
                subsystem_name(id), msg->error_code);
    } else if (msg->status == CITYSYNC_SENSOR_STATUS_NO_ECHO &&
               id == SUBSYSTEM_WATER) {
        /* No ultrasonic echo is not proof of disconnection. Keep the last
           value, but mark it stale so consumers know it is not current. */
        state[id].freshness = state[id].valid
            ? CITYSYNC_DATA_STALE : CITYSYNC_DATA_UNAVAILABLE;
        fprintf(stderr, "[FAULT] Water sensor returned NO_ECHO\n");
    } else if (msg->status == CITYSYNC_SENSOR_STATUS_OK) {
        fprintf(stdout, "[HEALTH] %s sensor reports OK\n", subsystem_name(id));
    } else {
        fprintf(stderr, "[REJECTED] Unknown sensor status=%d for %s\n",
                msg->status, subsystem_name(id));
    }
}

static void display_sensor_monitoring(void)
{
    int id;
    refresh_freshness();

    printf("\n");
    printf("==========================================================================\n");
    printf("                       CitySync RT+ SENSOR MONITORING\n");
    printf("==========================================================================\n");
    printf("%-17s %-14s %-12s %-12s %-14s\n",
           "Subsystem", "Reading", "Sequence", "Freshness", "Last Error");
    printf("--------------------------------------------------------------------------\n");

    for (id = 1; id <= CITYSYNC_SUBSYSTEM_COUNT; ++id) {
        if (state[id].valid) {
            printf("%-17s %-14.2f %-12u %-12s %-14d\n",
                   subsystem_name(id), state[id].value,
                   state[id].sequence_number,
                   freshness_name(state[id].freshness), state[id].last_error);
        } else {
            printf("%-17s %-14s %-12s %-12s %-14d\n",
                   subsystem_name(id), "--", "--",
                   freshness_name(state[id].freshness), state[id].last_error);
        }
    }
    printf("==========================================================================\n");
    fflush(stdout);
}

static void display_fault_monitoring(void)
{
    int id;
    refresh_freshness();

    printf("\n--------------------------- FAULT MONITORING -----------------------------\n");
    printf("%-17s %-15s %-12s\n", "Subsystem", "Condition", "Error Code");
    for (id = 1; id <= CITYSYNC_SUBSYSTEM_COUNT; ++id) {
        const char *condition = "NORMAL";
        if (!state[id].valid)
            condition = "UNAVAILABLE";
        else if (state[id].freshness == CITYSYNC_DATA_STALE)
            condition = "STALE";
        if (state[id].last_error != 0 || strcmp(condition, "NORMAL") != 0)
            printf("%-17s %-15s %-12d\n",
                   subsystem_name(id), condition, state[id].last_error);
    }
    printf("-------------------------------------------------------------------------\n");
}

static void display_performance_summary(void)
{
    int id;
    printf("\n");
    printf("============================================================================================\n");
    printf("                         CitySync RT+ ANALYTICS / PERFORMANCE SUMMARY\n");
    printf("============================================================================================\n");
    printf("%-16s %8s %9s %9s %9s %9s %9s %9s %9s\n",
           "Subsystem", "Updates", "Rejected", "MinLat", "AvgLat",
           "MaxLat", "MaxJit", "GapMiss", "LatMiss");
    printf("%-16s %8s %9s %9s %9s %9s %9s %9s %9s\n",
           "", "(count)", "(count)", "(ms)", "(ms)", "(ms)",
           "(ms)", "(count)", "(count)");
    printf("--------------------------------------------------------------------------------------------\n");

    for (id = 1; id <= CITYSYNC_SUBSYSTEM_COUNT; ++id) {
        PerformanceStats *p = &performance[id];
        double avg = p->latency_samples
            ? (double)p->latency_sum_us / (double)p->latency_samples / 1000.0
            : 0.0;
        printf("%-16s %8llu %9llu %9.3f %9.3f %9.3f %9.3f %9llu %9llu\n",
               subsystem_name(id),
               (unsigned long long)p->accepted,
               (unsigned long long)p->rejected,
               p->latency_samples ? (double)p->min_latency_us / 1000.0 : 0.0,
               avg,
               (double)p->max_latency_us / 1000.0,
               (double)p->max_jitter_us / 1000.0,
               (unsigned long long)p->gap_misses,
               (unsigned long long)p->latency_misses);
    }
    printf("--------------------------------------------------------------------------------------------\n");
    printf("Latency = TwinSync receive time - acquisition timestamp (same monotonic clock required).\n");
    printf("Jitter = absolute deviation of arrival gap from configured sampling interval.\n");
    printf("GapMiss = arrival gap exceeds interval + subsystem tolerance.\n");
    printf("LatMiss = measured latency exceeds CITYSYNC_TARGET_LATENCY_MS.\n");
    printf("============================================================================================\n");
    fflush(stdout);
}

static void copy_latest_state(CitySyncStateResponse *response)
{
    int id;
    refresh_freshness();
    memset(response, 0, sizeof(*response));
    response->type = CITYSYNC_MSG_GET_STATE;

    /* The protocol reserves array index zero; subsystem IDs are 1..5. */
    for (id = 1; id <= CITYSYNC_SUBSYSTEM_COUNT; ++id) {
        response->data[id].subsystem_id = id;
        response->data[id].value = state[id].value;
        response->data[id].sequence_number = state[id].sequence_number;
        response->data[id].timestamp_ms = state[id].source_timestamp_ms;
        response->data[id].valid = state[id].valid;
        response->data[id].freshness = state[id].freshness;
    }

    response->flame.type = CITYSYNC_MSG_UPDATE_FLAME;
    response->flame.sequence_number = state[SUBSYSTEM_FIRE].sequence_number;
    response->flame.timestamp_ms = state[SUBSYSTEM_FIRE].source_timestamp_ms;
    response->flame.detected = state[SUBSYSTEM_FIRE].valid &&
                               state[SUBSYSTEM_FIRE].value > 0.5f;
    response->flame.valid = state[SUBSYSTEM_FIRE].valid;
    response->flame.freshness = state[SUBSYSTEM_FIRE].freshness;
}

static int start_monitor_timer(int chid, int *connection, timer_t *timer_id)
{
    struct sigevent event;
    struct itimerspec timer_spec;

    *connection = ConnectAttach(ND_LOCAL_NODE, 0, chid, _NTO_SIDE_CHANNEL, 0);
    if (*connection == -1) {
        perror("ConnectAttach for monitor timer");
        return -1;
    }

    SIGEV_PULSE_INIT(&event, *connection, CITYSYNC_PRIORITY_TWINSYNC,
                     TIMER_PULSE_CODE, 0);

    if (timer_create(CLOCK_MONOTONIC, &event, timer_id) == -1) {
        perror("timer_create");
        ConnectDetach(*connection);
        *connection = -1;
        return -1;
    }

    memset(&timer_spec, 0, sizeof(timer_spec));
    timer_spec.it_value.tv_sec = CITYSYNC_MONITOR_INTERVAL_MS / 1000U;
    timer_spec.it_value.tv_nsec =
        (long)(CITYSYNC_MONITOR_INTERVAL_MS % 1000U) * 1000000L;
    timer_spec.it_interval = timer_spec.it_value;

    if (timer_settime(*timer_id, 0, &timer_spec, NULL) == -1) {
        perror("timer_settime");
        timer_delete(*timer_id);
        ConnectDetach(*connection);
        *connection = -1;
        return -1;
    }
    return 0;
}

static void cleanup(void)
{
    if (timer_created)
        timer_delete(monitor_timer);
    if (timer_connection != -1)
        ConnectDetach(timer_connection);
    if (name_attachment != NULL)
        name_detach(name_attachment, 0);
}

int main(void)
{
    char buffer[RECEIVE_BUFFER_SIZE];
    struct _msg_info info;
    int receive_id;
    uint64_t last_report_ms;
    uint64_t last_dashboard_ms = 0;

    signal(SIGINT, stop_handler);
    signal(SIGTERM, stop_handler);
    memset(state, 0, sizeof(state));
    memset(performance, 0, sizeof(performance));

    name_attachment = name_attach(NULL, TWINSYNC_CHANNEL_NAME, 0);
    if (name_attachment == NULL) {
        perror("name_attach CitySyncTwin");
        return EXIT_FAILURE;
    }

    printf("\n==========================================================================\n");
    printf("                       CitySync RT+ TwinSync\n");
    printf("==========================================================================\n");
    printf("Channel: %s | QNX Native Message Passing\n", TWINSYNC_CHANNEL_NAME);
    printf("Subsystems: Vehicle, Current, Water Distance, Temperature, Fire\n");
    printf("Latency target: %u ms | Monitor period: %u ms\n",
           CITYSYNC_TARGET_LATENCY_MS, CITYSYNC_MONITOR_INTERVAL_MS);
    printf("TwinSync priority configured as a project target: %d\n",
           CITYSYNC_PRIORITY_TWINSYNC);

    timer_channel = name_attachment->chid;
    if (start_monitor_timer(timer_channel, &timer_connection, &monitor_timer) == 0) {
        timer_created = 1;
        printf("Periodic freshness timer started.\n");
    } else {
        fprintf(stderr, "WARNING: timer unavailable; freshness is refreshed on message activity.\n");
    }

    last_report_ms = monotonic_ms();
    display_sensor_monitoring();

    while (keep_running) {
        memset(buffer, 0, sizeof(buffer));
        memset(&info, 0, sizeof(info));

        receive_id = MsgReceive(name_attachment->chid, buffer, sizeof(buffer), &info);
        if (receive_id == -1) {
            if (errno == EINTR)
                continue;
            perror("MsgReceive");
            break;
        }

        /* Timer pulses are delivered with rcvid == 0. */
        if (receive_id == 0) {
            struct _pulse *pulse = (struct _pulse *)buffer;
            if (pulse->code == TIMER_PULSE_CODE) {
                uint64_t now = monotonic_ms();
                refresh_freshness();
                if (now - last_dashboard_ms >= CITYSYNC_REPORT_INTERVAL_MS) {
                    display_sensor_monitoring();
                    display_fault_monitoring();
                    last_dashboard_ms = now;
                }
            }
            continue;
        }

        if (info.srcmsglen < (int)sizeof(int)) {
            performance[0].rejected++;
            MsgError(receive_id, EINVAL);
            continue;
        }

        {
            int type;
            memcpy(&type, buffer, sizeof(type));

            if (type == CITYSYNC_MSG_UPDATE) {
                CitySyncMessage msg;
                if (info.srcmsglen != (int)sizeof(msg)) {
                    MsgError(receive_id, EINVAL);
                    continue;
                }
                memcpy(&msg, buffer, sizeof(msg));
                if (!valid_subsystem_id(msg.data.subsystem_id)) {
                    MsgError(receive_id, EINVAL);
                    continue;
                }
                {
                    uint64_t before = performance[msg.data.subsystem_id].accepted;
                    accept_update(&msg);
                    if (performance[msg.data.subsystem_id].accepted == before) {
                        MsgError(receive_id, EINVAL);
                        continue;
                    }
                }
                if (MsgReply(receive_id, EOK, NULL, 0) == -1)
                    perror("MsgReply update");
            } else if (type == CITYSYNC_MSG_SENSOR_STATUS) {
                CitySyncSensorStatusMessage msg;
                if (info.srcmsglen != (int)sizeof(msg)) {
                    MsgError(receive_id, EINVAL);
                    continue;
                }
                memcpy(&msg, buffer, sizeof(msg));
                if (!valid_subsystem_id(msg.subsystem_id)) {
                    MsgError(receive_id, EINVAL);
                    continue;
                }
                accept_sensor_status(&msg);
                if (MsgReply(receive_id, EOK, NULL, 0) == -1)
                    perror("MsgReply sensor status");
            } else if (type == CITYSYNC_MSG_GET_STATE) {
                CitySyncRequest request;
                CitySyncStateResponse response;
                if (info.srcmsglen != (int)sizeof(request)) {
                    MsgError(receive_id, EINVAL);
                    continue;
                }
                memcpy(&request, buffer, sizeof(request));
                copy_latest_state(&response);
                if (MsgReply(receive_id, EOK, &response, sizeof(response)) == -1)
                    perror("MsgReply state");
                continue;
            } else {
                MsgError(receive_id, EINVAL);
                continue;
            }
        }

        {
            uint64_t now = monotonic_ms();
            if (now >= last_report_ms &&
                now - last_report_ms >= CITYSYNC_REPORT_INTERVAL_MS) {
                display_sensor_monitoring();
                display_fault_monitoring();
                display_performance_summary();
                last_report_ms = now;
            }
        }
    }

    cleanup();
    printf("TwinSync stopped.\n");
    return EXIT_SUCCESS;
}
