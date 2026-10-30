#define _QNX_SOURCE 1

#include <stdio.h>  //printf();fprintf();perror();

#include <stdlib.h>

#include <stdint.h>

#include <string.h>

#include <errno.h>

#include <time.h>

#include <signal.h>

#include <sched.h>

#include <process.h>

#include <sys/neutrino.h> //QNX Neutrino kernel functionality.

#include <sys/dispatch.h> //name_attach() name_detach() name_attach_t

#include "../include/citysync_types.h"

#define TWINSYNC_CHANNEL_NAME "CitySyncTwin"

#define PERFORMANCE_REPORT_INTERVAL_MS CITYSYNC_REPORT_INTERVAL_MS

#define SYNC_TIMER_PERIOD_MS CITYSYNC_MONITOR_INTERVAL_MS

#define SYNC_MAX_LATENCY_MS CITYSYNC_TARGET_LATENCY_MS

#define FLAME_TIMEOUT_MS 500ULL

#define FLAME_EXPECTED_PERIOD_MS FIRE_INTERVAL_MS

#define FLAME_DEADLINE_TOLERANCE_MS 50ULL

#define SYNC_TIMER_PULSE_CODE _PULSE_CODE_MINAVAIL

typedef struct {

       float value;

       uint32_t sequence_number;

       uint64_t source_timestamp_ms;

       uint64_t source_timestamp_us;

       uint64_t last_receive_ms;

       int valid;

       int freshness;

} SubsystemState;

typedef struct {

       int detected;

       uint32_t sequence_number;

       uint64_t source_timestamp_ms;

       uint64_t source_timestamp_us;

       uint64_t last_receive_ms;

       int valid;

       int freshness;

       uint64_t rejected_updates;

} FlameState;

typedef struct {

       uint64_t accepted_updates;

       uint64_t latency_samples;

       uint64_t latency_sum_us;

       uint64_t min_latency_us;

       uint64_t max_latency_us;

       uint64_t latency_deadline_misses;

       uint64_t interval_samples;

       uint64_t interval_sum_us;

       uint64_t last_interval_us;

       uint64_t jitter_samples;

       uint64_t jitter_sum_us;

       uint64_t max_jitter_us;

       uint64_t interarrival_deadline_misses;

       uint64_t invalid_timestamps;

       uint64_t rejected_updates;

       uint64_t last_receive_us;

       int has_previous_receive;

} PerformanceStats;

//Group A – Update counting

//- accepted_updates: Number of accepted messages.

//- rejected_updates: Number of rejected messages.

//Group B – Latency measurement

//- latency_samples: Number of valid latency measurements.

//- latency_sum_us: Sum of all measured latencies.

//- min_latency_us: Lowest latency.

//- max_latency_us: Highest latency.

//- latency_deadline_misses: Number of latency deadline violations.

//Group C – Interarrival time and jitter

//- interval_samples: Number of arrival intervals measured.

//- interval_sum_us: Sum of arrival intervals.

//- last_interval_us: Latest arrival interval.

//- jitter_samples: Number of jitter measurements.

//- jitter_sum_us: Sum of jitter values.

//- max_jitter_us: Maximum observed jitter.

//- interarrival_deadline_misses: Number of late arrivals.

//Group D – Validation

//- invalid_timestamps: Number of invalid timestamp cases.

//- last_receive_us: Previous/latest reception time.

//- has_previous_receive: Indicates whether an earlier reception exists.

/* Public subsystem IDs 1..4 are preserved. Slot zero is unused. */

static SubsystemState city_state[CITYSYNC_SUBSYSTEM_ARRAY_SIZE];

static PerformanceStats performance[CITYSYNC_SUBSYSTEM_ARRAY_SIZE];

static FlameState flame_state;

static PerformanceStats flame_performance;

static int water_no_echo = 0;

static uint64_t monotonic_ns(void)

{

       uint64_t now_ns = 0;

       if (ClockTime(CLOCK_MONOTONIC, NULL, &now_ns) == -1) {//Reads or sets the time associated with a QNX system clock.

           perror("ClockTime");

           return 0;

       }

       //CLOCK_MONOTONIC-t is a clock that measures elapsed time and does not jump backwards or forwards

       return now_ns;

}

static uint64_t monotonic_ms(void)

{

       return monotonic_ns() / 1000000ULL;

}//1  millisecond=1,000,000 nano second

static uint64_t monotonic_us(void)

{

       return monotonic_ns() / 1000ULL;

}// 1 microsecond =1,000 nanoseconds

//Nanoseconds	Obtaining clock readings

//Microseconds	Latency and jitter calculations

//Milliseconds	Freshness and timeout monitoring

static const char *subsystem_name(SubsystemID id)

{

       switch (id) {

           case SUBSYSTEM_TRAFFIC:     return "Vehicle Count";

           case SUBSYSTEM_ENERGY:      return "Current";

           case SUBSYSTEM_WATER:       return "Water Level";

           case SUBSYSTEM_ENVIRONMENT: return "Temperature";

           default:                    return "Unknown";

       }

}

static const char *subsystem_unit(SubsystemID id)

{

       switch (id) {

           case SUBSYSTEM_TRAFFIC:     return "count";

           case SUBSYSTEM_ENERGY:      return "mA";

           case SUBSYSTEM_WATER:       return "cm";

           case SUBSYSTEM_ENVIRONMENT: return "C";

           default:                    return "";

       }

}

static const char *freshness_name(int freshness)

{

       switch (freshness) {

           case CITYSYNC_DATA_UNAVAILABLE: return "UNAVAILABLE";

           case CITYSYNC_DATA_ACTIVE:      return "ACTIVE";

           case CITYSYNC_DATA_STALE:       return "STALE";

           default:                        return "UNKNOWN";

       }

}

static uint64_t expected_period_ms(SubsystemID id)

{

       switch (id) {

           case SUBSYSTEM_TRAFFIC:     return TRAFFIC_INTERVAL_MS;

           case SUBSYSTEM_ENERGY:      return ENERGY_INTERVAL_MS;

           case SUBSYSTEM_WATER:       return WATER_INTERVAL_MS;

           case SUBSYSTEM_ENVIRONMENT: return ENVIRONMENT_INTERVAL_MS;

           default:                    return 0;

       }

}

static uint64_t deadline_tolerance_ms(SubsystemID id)

{

       switch (id) {

           case SUBSYSTEM_TRAFFIC:     return TRAFFIC_DEADLINE_TOLERANCE_MS;

           case SUBSYSTEM_ENERGY:      return ENERGY_DEADLINE_TOLERANCE_MS;

           case SUBSYSTEM_WATER:       return WATER_DEADLINE_TOLERANCE_MS;

           case SUBSYSTEM_ENVIRONMENT: return ENVIRONMENT_DEADLINE_TOLERANCE_MS;

           default:                    return 0;

       }

}

static uint64_t timeout_ms(SubsystemID id)

{

       switch (id) {

           case SUBSYSTEM_TRAFFIC:     return TRAFFIC_TIMEOUT_MS;

           case SUBSYSTEM_ENERGY:      return ENERGY_TIMEOUT_MS;

           case SUBSYSTEM_WATER:       return WATER_TIMEOUT_MS;

           case SUBSYSTEM_ENVIRONMENT: return ENVIRONMENT_TIMEOUT_MS;

           default:                    return 0;

       }

}

static void refresh_freshness(uint64_t now_ms)

{

       int id;

       for (id = SUBSYSTEM_TRAFFIC; id <= SUBSYSTEM_ENVIRONMENT; ++id) {

           uint64_t timeout = timeout_ms((SubsystemID)id);

           if (!city_state[id].valid)

               city_state[id].freshness = CITYSYNC_DATA_UNAVAILABLE;

           else if ((id == SUBSYSTEM_WATER && water_no_echo) ||

                    now_ms < city_state[id].last_receive_ms ||

                    now_ms - city_state[id].last_receive_ms >= timeout)

               city_state[id].freshness = CITYSYNC_DATA_STALE;

           else

               city_state[id].freshness = CITYSYNC_DATA_ACTIVE;

       }

       if (!flame_state.valid)

           flame_state.freshness = CITYSYNC_DATA_UNAVAILABLE;

       else if (now_ms < flame_state.last_receive_ms ||

                now_ms - flame_state.last_receive_ms >= FLAME_TIMEOUT_MS)

           flame_state.freshness = CITYSYNC_DATA_STALE;

       else

           flame_state.freshness = CITYSYNC_DATA_ACTIVE;

}

/* Latency and jitter are measured in microseconds internally. */

static void update_performance(PerformanceStats *s,

                                  uint64_t receive_us,

                                  uint64_t source_timestamp_us,

                                  uint64_t expected_period_us,

                                  uint64_t tolerance_us)

{

       uint64_t latency_us;

       ++s->accepted_updates;

       if (source_timestamp_us == 0 || receive_us < source_timestamp_us) {

           ++s->invalid_timestamps;

       } else {

           latency_us = receive_us - source_timestamp_us;

           if (s->latency_samples == 0 || latency_us < s->min_latency_us)

               s->min_latency_us = latency_us;

           if (s->latency_samples == 0 || latency_us > s->max_latency_us)

               s->max_latency_us = latency_us;

           s->latency_sum_us += latency_us;

           ++s->latency_samples;

           if (latency_us > SYNC_MAX_LATENCY_MS * 1000ULL)

               ++s->latency_deadline_misses;

       }

       if (s->has_previous_receive && receive_us >= s->last_receive_us) {

           uint64_t interval_us = receive_us - s->last_receive_us;

           uint64_t jitter_us = interval_us >= expected_period_us

                              ? interval_us - expected_period_us

                              : expected_period_us - interval_us;

           s->last_interval_us = interval_us;

           s->interval_sum_us += interval_us; /* interval is stored in microseconds */

           ++s->interval_samples;

           s->jitter_sum_us += jitter_us;

           ++s->jitter_samples;

           if (jitter_us > s->max_jitter_us)

               s->max_jitter_us = jitter_us;

           if (expected_period_us > 0 && interval_us > expected_period_us + tolerance_us)

               ++s->interarrival_deadline_misses;

       }

       s->last_receive_us = receive_us;

       s->has_previous_receive = 1;

}

static int valid_subsystem_id(int id)

{

       return id >= SUBSYSTEM_TRAFFIC && id <= SUBSYSTEM_ENVIRONMENT;

}

static int process_update(const CitySyncMessage *message, uint64_t receive_ms, uint64_t receive_us)

{

       SubsystemID id = message->data.subsystem_id;

       SubsystemState *state;

       if (!valid_subsystem_id((int)id)) {

           return -1;

       }

       state = &city_state[id];

       if (message->type != CITYSYNC_MSG_UPDATE ||

           message->data.valid != 1 ||

           message->data.value != message->data.value ||

           message->data.value > 3.4028234e+38F ||

           message->data.value < -3.4028234e+38F) {

           ++performance[id].rejected_updates;

           return -1;

       }

       if (message->data.timestamp_ms == 0 ||

           message->data.timestamp_ms > receive_ms ||

           message->data.timestamp_us == 0 ||

           message->data.timestamp_us > receive_us) {

           ++performance[id].invalid_timestamps;

           ++performance[id].rejected_updates;

           return -1;

       }

       if (state->valid &&

           message->data.sequence_number <= state->sequence_number) {

           ++performance[id].rejected_updates;

           return -1;

       }

       update_performance(&performance[id], receive_us, message->data.timestamp_us,

                          expected_period_ms(id) * 1000ULL,

                          deadline_tolerance_ms(id) * 1000ULL);

       state->value = message->data.value;

       if (id == SUBSYSTEM_WATER)

           water_no_echo = 0;

       state->sequence_number = message->data.sequence_number;

       state->source_timestamp_ms = message->data.timestamp_ms;

       state->source_timestamp_us = message->data.timestamp_us;

       state->last_receive_ms = receive_ms;

       state->valid = 1;

       state->freshness = CITYSYNC_DATA_ACTIVE;

       return 0;

}

static int process_sensor_status(const CitySyncSensorStatusMessage *message,

                                    uint64_t receive_ms)

{

       int id = message->subsystem_id;

       if (message->type != CITYSYNC_MSG_SENSOR_STATUS ||

           (!valid_subsystem_id(id) && id != SUBSYSTEM_FIRE) ||

           (message->status != CITYSYNC_SENSOR_STATUS_OK &&

            message->status != CITYSYNC_SENSOR_STATUS_FAILED &&

            !(id == SUBSYSTEM_WATER && message->status == CITYSYNC_SENSOR_STATUS_NO_ECHO)) ||

           message->timestamp_ms == 0 || message->timestamp_ms > receive_ms) {

           /* Rejected status messages are counted below; keep the console quiet. */

           if (valid_subsystem_id(id)) ++performance[id].rejected_updates;

           else if (id == SUBSYSTEM_FIRE) {

               ++flame_performance.rejected_updates;

               ++flame_state.rejected_updates;

           }

           return -1;

       }

       if (id == SUBSYSTEM_FIRE) {

           if (message->status == CITYSYNC_SENSOR_STATUS_FAILED) {

               flame_state.valid = 0;

               flame_state.freshness = CITYSYNC_DATA_UNAVAILABLE;

           } else {

               if (flame_state.valid) flame_state.freshness = CITYSYNC_DATA_ACTIVE;

           }

       } else {

           SubsystemState *state = &city_state[id];

           if (id == SUBSYSTEM_WATER &&

               message->status == CITYSYNC_SENSOR_STATUS_NO_ECHO) {

               /* Keep the last valid distance; do not pretend the timeout is a

                * confirmed unplug. If there has never been a valid reading, it

                * remains UNAVAILABLE. Otherwise the display becomes STALE. */

               water_no_echo = 1;

               if (state->valid)

                   state->freshness = CITYSYNC_DATA_STALE;

               else

                   state->freshness = CITYSYNC_DATA_UNAVAILABLE;

           } else if (message->status == CITYSYNC_SENSOR_STATUS_FAILED) {

               if (id == SUBSYSTEM_WATER)

                   water_no_echo = 0;

               state->valid = 0;

               state->freshness = CITYSYNC_DATA_UNAVAILABLE;

           } else {

               if (state->valid) state->freshness = CITYSYNC_DATA_ACTIVE;

           }

       }

       fflush(stdout);

       return 0;

}

static int process_flame_update(const CitySyncFlameMessage *message,

                                   uint64_t receive_ms, uint64_t receive_us)

{

       if (message->type != CITYSYNC_MSG_UPDATE_FLAME ||

           message->valid != 1 ||

           (message->detected != 0 && message->detected != 1) ||

           message->timestamp_ms == 0 ||

           message->timestamp_ms > receive_ms ||

           message->timestamp_us == 0 ||

           message->timestamp_us > receive_us) {

           ++flame_state.rejected_updates;

           ++flame_performance.rejected_updates;

           return -1;

       }

       if (flame_state.valid &&

           message->sequence_number <= flame_state.sequence_number) {

           ++flame_state.rejected_updates;

           ++flame_performance.rejected_updates;

           return -1;

       }

       update_performance(&flame_performance, receive_us, message->timestamp_us,

                          FLAME_EXPECTED_PERIOD_MS * 1000ULL,

                          FLAME_DEADLINE_TOLERANCE_MS * 1000ULL);

       flame_state.detected = message->detected;

       flame_state.sequence_number = message->sequence_number;

       flame_state.source_timestamp_ms = message->timestamp_ms;

       flame_state.source_timestamp_us = message->timestamp_us;

       flame_state.last_receive_ms = receive_ms;

       flame_state.valid = 1;

       flame_state.freshness = CITYSYNC_DATA_ACTIVE;

       return 0;

}

static void copy_latest_state(CitySyncStateResponse *response, uint64_t now_ms)

{

       int id;

       refresh_freshness(now_ms);

       memset(response, 0, sizeof(*response));

       response->type = CITYSYNC_MSG_GET_STATE;

       for (id = SUBSYSTEM_TRAFFIC; id <= SUBSYSTEM_ENVIRONMENT; ++id) {

           int index = id;

           response->data[index].subsystem_id = (SubsystemID)id;

           response->data[index].value = city_state[id].value;

           response->data[index].sequence_number = city_state[id].sequence_number;

           response->data[index].timestamp_ms = city_state[id].source_timestamp_ms;

           response->data[index].timestamp_us = city_state[id].source_timestamp_us;

           response->data[index].valid = city_state[id].valid;

           response->data[index].freshness = city_state[id].freshness;

       }

       response->flame.type = CITYSYNC_MSG_UPDATE_FLAME;

       response->flame.sequence_number = flame_state.sequence_number;

       response->flame.timestamp_ms = flame_state.source_timestamp_ms;

       response->flame.timestamp_us = flame_state.source_timestamp_us;

       response->flame.detected = flame_state.detected;

       response->flame.valid = flame_state.valid;

       response->flame.freshness = flame_state.freshness;

}

static void print_performance_row(const char *name, const PerformanceStats *s)

{

       double min_latency_ms = s->latency_samples ? (double)s->min_latency_us / 1000.0 : 0.0;

       double avg_latency_ms = s->latency_samples

           ? ((double)s->latency_sum_us / (double)s->latency_samples) / 1000.0 : 0.0;

       double max_latency_ms = (double)s->max_latency_us / 1000.0;

       double max_jitter_ms = (double)s->max_jitter_us / 1000.0;

       printf("%-15s %8llu %8.3f %9.3f %8.3f %8.3f %9llu %9llu %9llu\n",

              name,

              (unsigned long long)s->accepted_updates,

              min_latency_ms,

              avg_latency_ms,

              max_latency_ms,

              max_jitter_ms,

              (unsigned long long)s->interarrival_deadline_misses,

              (unsigned long long)s->latency_deadline_misses,

              (unsigned long long)s->rejected_updates);

}

static void display_performance_summary(void)

{

       int id;

       printf("\n=========================================================================" "\n");

       printf("             TwinSync Real-Time Performance Summary\n");

       printf("=========================================================================" "\n");

       printf("%-15s %8s %8s %9s %8s %8s %9s %9s %9s\n",

              "Subsystem", "Updates", "MinLat", "AvgLat", "MaxLat",

              "MaxJit", "GapMiss", "LatMiss", "Rejected");

       printf("%-15s %8s %8s %9s %8s %8s %9s %9s %9s\n",

              "", "(count)", "(ms)", "(ms)", "(ms)", "(ms)",

              "(count)", "(count)", "(count)");

       printf("-------------------------------------------------------------------------\n");

       for (id = SUBSYSTEM_TRAFFIC; id <= SUBSYSTEM_ENVIRONMENT; ++id)

           print_performance_row(subsystem_name((SubsystemID)id), &performance[id]);

       print_performance_row("Fire System", &flame_performance);

       printf("-------------------------------------------------------------------------\n");

       printf("Latency deadline threshold: %llu ms\n",

              (unsigned long long)SYNC_MAX_LATENCY_MS);

       printf("=========================================================================" "\n");

       fflush(stdout);

}

static void display_city_state(uint64_t now_ms)

{

       int id;

       refresh_freshness(now_ms);

       printf("\n============================================================\n");

       printf("                 CitySync RT+ Digital Twin\n");

       printf("============================================================\n");

       printf("%-15s %-12s %-8s %-10s %-12s %-10s\n",

              "Subsystem", "Value", "Unit", "Sequence", "Freshness", "Age(ms)");

       printf("------------------------------------------------------------\n");

       for (id = SUBSYSTEM_TRAFFIC; id <= SUBSYSTEM_ENVIRONMENT; ++id) {

           SubsystemState *s = &city_state[id];

           uint64_t age = s->valid && now_ms >= s->last_receive_ms

                            ? now_ms - s->last_receive_ms : 0;

           if (s->valid) {

               printf("%-15s %-12.2f %-8s %-10u %-12s %-10llu\n",

                      subsystem_name((SubsystemID)id), s->value,

                      subsystem_unit((SubsystemID)id), s->sequence_number,

                      freshness_name(s->freshness), (unsigned long long)age);

           } else {

               printf("%-15s %-12s %-8s %-10s %-12s %-10s\n",

                      subsystem_name((SubsystemID)id), "--",

                      subsystem_unit((SubsystemID)id), "--",

                      freshness_name(s->freshness), "--");

           }

       }

       {

           uint64_t age = flame_state.valid && now_ms >= flame_state.last_receive_ms

                            ? now_ms - flame_state.last_receive_ms : 0;

           if (flame_state.valid) {

               printf("%-15s %-12s %-8s %-10u %-12s %-10llu\n",

                      "Fire System", flame_state.detected ? "FIRE" : "CLEAR",

                      "bool", flame_state.sequence_number,

                      freshness_name(flame_state.freshness),

                      (unsigned long long)age);

           } else {

               printf("%-15s %-12s %-8s %-10s %-12s %-10s\n",

                      "Fire System", "--", "bool", "--",

                      freshness_name(flame_state.freshness), "--");

           }

       }

       printf("============================================================\n");

       fflush(stdout);

}

static int create_monitoring_timer(int channel_id, int *timer_id, int *timer_coid)

{

       struct sigevent event;

       struct _itimer timer_spec;

       int coid;

       coid = ConnectAttach(0, 0, channel_id, _NTO_SIDE_CHANNEL, 0);

       if (coid == -1) {

           perror("ConnectAttach");

           return -1;

       }

       SIGEV_PULSE_INIT(&event, coid, CITYSYNC_TIMER_PULSE_PRIORITY, SYNC_TIMER_PULSE_CODE, 0);

       /*

        * QNX-native timer API:

        * TimerCreate() returns the timer ID directly.

        */

       *timer_id = TimerCreate(CLOCK_MONOTONIC, &event);

       if (*timer_id == -1) {

           perror("TimerCreate");

           ConnectDetach(coid);

           return -1;

       }

       memset(&timer_spec, 0, sizeof(timer_spec));

       timer_spec.nsec = (uint64_t)SYNC_TIMER_PERIOD_MS * 1000000ULL;

       timer_spec.interval_nsec = (uint64_t)SYNC_TIMER_PERIOD_MS * 1000000ULL;

       if (TimerSettime(*timer_id, 0, &timer_spec, NULL) == -1) {

           perror("TimerSettime");

           TimerDestroy(*timer_id);

           ConnectDetach(coid);

           return -1;

       }

       *timer_coid = coid;

       return 0;

}

/*

    * QNX SCHED_FIFO priorities range up to 255 on the system.

    * The default unprivileged range is 1..63. Use 63 here so the

    * synchronizer can request the highest default unprivileged priority.

    * To request a higher priority, configure the required QNX process

    * ability and change this project setting deliberately.

    */

#define TWINSYNC_QNX_FIFO_PRIORITY CITYSYNC_PRIORITY_TWINSYNC

static int configure_synchronizer_priority(void)

{

       struct sched_param param;

       int priority = TWINSYNC_QNX_FIFO_PRIORITY;

       int tid = gettid();

       memset(&param, 0, sizeof(param));

       param.sched_priority = priority;

       if (SchedSet(0, tid, SCHED_FIFO, &param) == -1) {

           perror("SchedSet");

           fprintf(stderr,

                   "TwinSync continues with its existing scheduling policy.\n");

           return -1;

       }

       printf("TwinSync scheduling policy: SCHED_FIFO, priority=%d\n", priority);

       return 0;

}

int main(void)

{

       name_attach_t *attach;

       CitySyncStateResponse response;

       struct _msg_info info;

       union {

           int type;

           CitySyncMessage subsystem;

           CitySyncFlameMessage flame;

           CitySyncSensorStatusMessage sensor_status;

           CitySyncRequest request;

           CitySyncStateResponse state_response;

           struct _pulse pulse;

       } receive_buffer;

       int rcvid;

       int timer_coid = -1;

       int priority_status;

       int timer_id = -1;

       uint64_t last_report_ms;

       int timer_created = 0;

       memset(city_state, 0, sizeof(city_state));

       memset(performance, 0, sizeof(performance));

       memset(&flame_state, 0, sizeof(flame_state));

       memset(&flame_performance, 0, sizeof(flame_performance));

       water_no_echo = 0;

       priority_status = configure_synchronizer_priority();

       printf("\n============================================================\n");

       printf("                 CitySync RT+ Digital Twin\n");

       printf("============================================================\n");

       printf("Highest SCHED_FIFO priority: %s\n",

              priority_status == 0 ? "SET" : "NOT CONFIRMED");

       printf("Timer period: %llu ms | Latency deadline: %llu ms\n",

              (unsigned long long)SYNC_TIMER_PERIOD_MS,

              (unsigned long long)SYNC_MAX_LATENCY_MS);

       printf("Sampling periods: Fire=%u ms, Water=%u ms, Current=%u ms, Vehicle=%u ms, Temperature=%u ms\n",

              FIRE_INTERVAL_MS, WATER_INTERVAL_MS, ENERGY_INTERVAL_MS,

              TRAFFIC_INTERVAL_MS, ENVIRONMENT_INTERVAL_MS);

       fflush(stdout);

       attach = name_attach(NULL, TWINSYNC_CHANNEL_NAME, 0);

       if (attach == NULL) {

           perror("name_attach");

           return EXIT_FAILURE;

       }

       printf("Registered QNX channel: %s (chid=%d)\n",

              TWINSYNC_CHANNEL_NAME, attach->chid);

       if (create_monitoring_timer(attach->chid, &timer_id, &timer_coid) == -1) {

           fprintf(stderr, "ERROR: cannot continue without monitoring timer.\n");

           name_detach(attach, 0);

           return EXIT_FAILURE;

       }

       timer_created = 1;

       last_report_ms = monotonic_ms();

       display_city_state(last_report_ms);

       for (;;) {

           uint64_t receive_ms;

           uint64_t receive_us;

           memset(&receive_buffer, 0, sizeof(receive_buffer));

           memset(&info, 0, sizeof(info));

           rcvid = MsgReceive(attach->chid, &receive_buffer,

                              sizeof(receive_buffer), &info);

           if (rcvid == -1) {

               if (errno == EINTR)

                   continue;

               perror("MsgReceive");

               break;

           }

           receive_ms = monotonic_ms();

           receive_us = monotonic_us();

           if (receive_ms == 0 || receive_us == 0) {

               fprintf(stderr, "ERROR: CLOCK_MONOTONIC read failed.\n");

               if (rcvid > 0)

                   MsgError(rcvid, EIO);

               continue;

           }

           if (rcvid == 0) {

               struct _pulse *pulse = &receive_buffer.pulse;

               if (pulse->code == SYNC_TIMER_PULSE_CODE) {

                   refresh_freshness(receive_ms);

                   if (receive_ms >= last_report_ms &&

                       receive_ms - last_report_ms >= PERFORMANCE_REPORT_INTERVAL_MS) {

                       /* Keep the existing dashboard and summary refresh behavior. */

                       display_city_state(receive_ms);

                       display_performance_summary();

                       last_report_ms = receive_ms;

                   }

               }

               continue;

           }

           /*

            * The first integer in each incoming message is its type.

            * These types come from the user's shared citysync_types.h.

            */

           if (receive_buffer.type == CITYSYNC_MSG_UPDATE) {

               int result;

               result = process_update(&receive_buffer.subsystem, receive_ms, receive_us);

               if (MsgReply(rcvid, result == 0 ? EOK : EINVAL, NULL, 0) == -1 &&

                   errno != ESRCH)

                   perror("MsgReply subsystem");

           } else if (receive_buffer.type == CITYSYNC_MSG_UPDATE_FLAME) {

               int result;

               result = process_flame_update(&receive_buffer.flame, receive_ms, receive_us);

               if (MsgReply(rcvid, result == 0 ? EOK : EINVAL, NULL, 0) == -1 &&

                   errno != ESRCH)

                   perror("MsgReply Fire System");

           } else if (receive_buffer.type == CITYSYNC_MSG_SENSOR_STATUS) {

               int result;

               result = process_sensor_status(&receive_buffer.sensor_status, receive_ms);

               if (MsgReply(rcvid, result == 0 ? EOK : EINVAL, NULL, 0) == -1 &&

                   errno != ESRCH)

                   perror("MsgReply sensor status");

           } else if (receive_buffer.type == CITYSYNC_MSG_GET_STATE) {

               copy_latest_state(&response, receive_ms);

               if (MsgReply(rcvid, EOK, &response, sizeof(response)) == -1 &&

                   errno != ESRCH)

                   perror("MsgReply state");

           } else {

               fprintf(stderr, "TwinSync: unknown message type %d\n",

                       receive_buffer.type);

               MsgError(rcvid, EINVAL);

           }

       }

       if (timer_created) {

           TimerDestroy(timer_id);

           if (timer_coid != -1)

               ConnectDetach(timer_coid);

       }

       name_detach(attach, 0);

       return EXIT_FAILURE;

}
