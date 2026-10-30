#define _QNX_SOURCE 1

#include <stdio.h>

#include <stdlib.h>

#include <stdint.h>

#include <stdbool.h>

#include <unistd.h>

#include <signal.h>

#include <time.h>

#include <errno.h>

#include <string.h>

#include <sched.h>

#include <process.h>

#include <sys/neutrino.h>

#include <sys/dispatch.h>

#include "pcf8591_adc.h"

#include "rpi_gpio_qnx.h"

#include "../include/citysync_types.h"

/* Required by QNX SignalAction(). */

extern void __signalstub(void);

/* Existing sensor pinout. */

#define IR_PIN                  17

#define FLAME_PIN               27

#define ULTRASONIC_TRIG         23

#define ULTRASONIC_ECHO         24

/* Three independent LED outputs. BCM GPIO numbering. */

#define LED1_PIN                 5

#define LED2_PIN                 6

#define LED3_PIN                13

#define ULTRASONIC_THRESHOLD_CM 30.0f

#define CURRENT_THRESHOLD_MA  1500.0f

#define ADC_VREF                 3.3f

#define ADC_MAX                255.0f

#define CURRENT_MAX_MA        5000.0f

#define FLAME_ACTIVE_LOW          1

#define IR_ACTIVE_LOW             1

#define TWINSYNC_CHANNEL "CitySyncTwin"

#define ULTRASONIC_MIN_CM        2.0f

#define ULTRASONIC_MAX_CM      400.0f

#define ERR_SENSOR_READ           1

#define ERR_SENSOR_RANGE          2

#define ERR_SENSOR_INIT           3

/* Older project headers may omit this status code; TwinSync must interpret 2 as NO_ECHO. */

#ifndef CITYSYNC_SENSOR_STATUS_NO_ECHO

#define CITYSYNC_SENSOR_STATUS_NO_ECHO 2

#endif

/*

    * The supplied citysync_types.h defines the message IDs and ordinary

    * subsystem structures, but the version supplied with this project does

    * not declare CitySyncFlameMessage. This local structure preserves the

    * legacy flame-message wire layout used by the existing sender/receiver.

    *

    * TwinSync must use the same field order and types when receiving message 3.

    */

typedef struct {

       int type;

       uint32_t sequence_number;

       uint64_t timestamp_ms;

       uint64_t timestamp_us;

       int detected;

       int valid;

       int freshness;

} CitySyncFlameMessage;

/*

    * The current citysync_types.h has the older sampling periods. These

    * local periods implement the latest schedule requested for this program.

    */

#define TRAFFIC_INTERVAL_MS       500U

#define FIRE_INTERVAL_MS          100U

#define ENERGY_INTERVAL_MS       200U

#define WATER_INTERVAL_MS         300U

#define ENVIRONMENT_INTERVAL_MS  1000U

typedef struct {

       int coid;

       sync_t *adc_mutex;

       int adc_fd;

} SensorContext;

static volatile sig_atomic_t running = 1;

static int gpio_initialized = 0;

static int adc_fd = -1;

static int twin_coid = -1;

static sync_t adc_mutex;

static int adc_mutex_initialized = 0;

static sync_t priority_mutex;

static int priority_mutex_initialized = 0;

/* Latest known state of each threshold condition. */

static int flame_active = 0;

static int ultrasonic_active = 0;

static int current_active = 0;

#define SENSOR_FLAME       1

#define SENSOR_ULTRASONIC  2

#define SENSOR_CURRENT     3

static uint64_t monotonic_ns(void)

{

       uint64_t now_ns = 0;

       if (ClockTime(CLOCK_MONOTONIC, NULL, &now_ns) == -1)

           return 0;

       return now_ns;

}

static uint64_t monotonic_ms(void)

{

       return monotonic_ns() / 1000000ULL;

}

static uint64_t monotonic_us(void)

{

       return monotonic_ns() / 1000ULL;

}

static void stop_program(int sig)

{

       (void)sig;

       running = 0;

}

/* ------------------------------------------------------------------ */

/* QNX native thread creation and joining                              */

/* ------------------------------------------------------------------ */

#define SENSOR_THREAD_STACK_SIZE (256U * 1024U)

typedef struct {

       int tid;

       void *stack;

       int created;

} QnxThread;

static int create_qnx_thread(QnxThread *thread,

                                void *(*entry)(void *),

                                void *arg,

                                const char *name)

{

       struct _thread_attr attr;

       memset(thread, 0, sizeof(*thread));

       thread->stack = malloc(SENSOR_THREAD_STACK_SIZE);

       if (thread->stack == NULL) {

           fprintf(stderr, "Cannot allocate stack for %s thread\n", name);

           return -1;

       }

       memset(&attr, 0, sizeof(attr));

       attr.__flags = 0;

       attr.__stackaddr = thread->stack;

       attr.__stacksize = SENSOR_THREAD_STACK_SIZE;

       thread->tid = ThreadCreate(0, entry, arg, &attr);

       if (thread->tid == -1) {

           int saved_errno = errno;

           fprintf(stderr, "ThreadCreate failed for %s: %s\n",

                   name, strerror(saved_errno));

           free(thread->stack);

           thread->stack = NULL;

           return -1;

       }

       thread->created = 1;

       return 0;

}

static int join_qnx_thread(QnxThread *thread, const char *name)

{

       void *thread_status = NULL;

       if (!thread->created)

           return 0;

       if (ThreadJoin(thread->tid, &thread_status) == -1) {

           fprintf(stderr, "ThreadJoin failed for %s (tid=%d): %s\n",

                   name, thread->tid, strerror(errno));

           /* Keep stack allocated unless termination is confirmed. */

           return -1;

       }

       free(thread->stack);

       thread->stack = NULL;

       thread->created = 0;

       return 0;

}

/* ------------------------------------------------------------------ */

/* ADC mutex                                                           */

/* ------------------------------------------------------------------ */

static int read_adc_channel_locked(SensorContext *ctx,

                                      int channel,

                                      uint8_t *raw)

{

       int rc;

       if (SyncMutexLock(ctx->adc_mutex) == -1) {

           fprintf(stderr, "ADC SyncMutexLock failed: %s\n", strerror(errno));

           return -1;

       }

       rc = pcf8591_read_channel(ctx->adc_fd, channel, raw);

       if (SyncMutexUnlock(ctx->adc_mutex) == -1) {

           fprintf(stderr, "ADC SyncMutexUnlock failed: %s\n", strerror(errno));

           return -1;

       }

       return rc;

}

/* ------------------------------------------------------------------ */

/* Independent LED state management                                    */

/* ------------------------------------------------------------------ */

static void update_priority_outputs(int sensor, int active)

{

       if (!priority_mutex_initialized)

           return;

       if (SyncMutexLock(&priority_mutex) == -1) {

           fprintf(stderr, "Priority mutex lock failed: %s\n", strerror(errno));

           return;

       }

       /*

        * Update only the state belonging to this sensor. Keep the other two

        * sensor states unchanged so all qualifying LEDs can remain ON together.

        */

       switch (sensor) {

           case SENSOR_FLAME:

               if (flame_active != (active ? 1 : 0)) {

                   flame_active = active ? 1 : 0;

                   gpio_write(LED1_PIN, flame_active ? true : false);

               }

               break;

           case SENSOR_ULTRASONIC:

               if (ultrasonic_active != (active ? 1 : 0)) {

                   ultrasonic_active = active ? 1 : 0;

                   gpio_write(LED2_PIN, ultrasonic_active ? true : false);

               }

               break;

           case SENSOR_CURRENT:

               if (current_active != (active ? 1 : 0)) {

                   current_active = active ? 1 : 0;

                   gpio_write(LED3_PIN, current_active ? true : false);

               }

               break;

           default:

               (void)SyncMutexUnlock(&priority_mutex);

               return;

       }

       if (SyncMutexUnlock(&priority_mutex) == -1)

           fprintf(stderr, "Priority mutex unlock failed: %s\n", strerror(errno));

}

/* ------------------------------------------------------------------ */

/* Absolute periodic timing using QNX ClockTime / TimerTimeout         */

/* ------------------------------------------------------------------ */

static void sleep_until_next(uint64_t *next_deadline_ns,

                                unsigned int period_ms)

{

       if (*next_deadline_ns == 0) {

           if (ClockTime(CLOCK_MONOTONIC, NULL, next_deadline_ns) == -1) {

               fprintf(stderr,

                       "ClockTime failed while initializing deadline: %s\n",

                       strerror(errno));

               return;

           }

       }

       *next_deadline_ns += (uint64_t)period_ms * 1000000ULL;

       while (running) {

           int rc = TimerTimeout(CLOCK_MONOTONIC,

                                 _NTO_TIMEOUT_NANOSLEEP | TIMER_ABSTIME,

                                 NULL, next_deadline_ns, NULL);

           if (rc == 0)

               break;

           if (errno == EINTR)

               continue;

           fprintf(stderr, "TimerTimeout failed: %s\n", strerror(errno));

           break;

       }

}

/* ------------------------------------------------------------------ */

/* QNX SCHED_FIFO priority                                              */

/* ------------------------------------------------------------------ */

static int set_thread_priority(int priority, const char *name)

{

       struct sched_param sp;

       int policy = SCHED_FIFO;

       int tid = gettid();

       /* The requested project priorities are within QNX's default 1..63 range. */

       if (priority < 1 || priority > 63) {

           fprintf(stderr, "%s: priority %d is outside 1..63\n",

                   name, priority);

           return -1;

       }

       memset(&sp, 0, sizeof(sp));

       sp.sched_priority = priority;

       if (SchedSet(0, tid, policy, &sp) == -1) {

           fprintf(stderr, "%s: SchedSet priority %d failed: %s\n",

                   name, priority, strerror(errno));

           return -1;

       }

       return 0;

}

/* ------------------------------------------------------------------ */

/* TwinSync IPC message send functions                                  */

/* ------------------------------------------------------------------ */

static int send_subsystem_update(int coid,

                                    SubsystemID id,

                                    uint32_t *sequence,

                                    float value)

{

       CitySyncMessage msg;

       memset(&msg, 0, sizeof(msg));

       msg.type = CITYSYNC_MSG_UPDATE;

       msg.data.subsystem_id = (int)id;

       msg.data.sequence_number = ++(*sequence);

       msg.data.timestamp_ms = monotonic_ms();

       /*

        * Keep the microsecond timestamp used by the original project protocol.

        * TwinSync may validate this field before accepting an update.

        */

       msg.data.timestamp_us = monotonic_us();

       msg.data.value = value;

       msg.data.valid = 1;

       msg.data.freshness = CITYSYNC_DATA_ACTIVE;

       if (msg.data.timestamp_ms == 0 || msg.data.timestamp_us == 0 ||

           value != value ||

           value > 3.4028234e+38F || value < -3.4028234e+38F)

           return -1;

       if (MsgSend(coid, &msg, sizeof(msg), NULL, 0) == -1) {

           fprintf(stderr, "MsgSend subsystem %d failed: %s\n",

                   (int)id, strerror(errno));

           return -1;

       }

       return 0;

}

static int send_flame_update(int coid,

                                uint32_t *sequence,

                                int detected)

{

       CitySyncFlameMessage msg;

       memset(&msg, 0, sizeof(msg));

       msg.type = CITYSYNC_MSG_UPDATE_FLAME;

       msg.sequence_number = ++(*sequence);

       msg.timestamp_ms = monotonic_ms();

       msg.timestamp_us = monotonic_us();

       msg.detected = detected ? 1 : 0;

       msg.valid = 1;

       msg.freshness = CITYSYNC_DATA_ACTIVE;

       if (msg.timestamp_ms == 0 || msg.timestamp_us == 0)

           return -1;

       if (MsgSend(coid, &msg, sizeof(msg), NULL, 0) == -1) {

           fprintf(stderr, "MsgSend Fire System failed: %s\n", strerror(errno));

           return -1;

       }

       return 0;

}

static int send_sensor_status(int coid,

                                 SubsystemID id,

                                 uint32_t *sequence,

                                 int status,

                                 int error_code)

{

       CitySyncSensorStatusMessage msg;

       memset(&msg, 0, sizeof(msg));

       msg.type = CITYSYNC_MSG_SENSOR_STATUS;

       msg.subsystem_id = (int)id;

       msg.sequence_number = ++(*sequence);

       msg.timestamp_ms = monotonic_ms();

       msg.status = status;

       msg.error_code = error_code;

       if (msg.timestamp_ms == 0)

           return -1;

       if (MsgSend(coid, &msg, sizeof(msg), NULL, 0) == -1) {

           fprintf(stderr, "MsgSend sensor-status ID %d failed: %s\n",

                   (int)id, strerror(errno));

           return -1;

       }

       return 0;

}

/* ------------------------------------------------------------------ */

/* Sensor thread 1: Flame / Fire                                        */

/* ------------------------------------------------------------------ */

static void *fire_thread(void *arg)

{

       SensorContext *ctx = (SensorContext *)arg;

       uint64_t next_deadline_ns = 0;

       uint32_t seq = 0;

       int priority_rc = set_thread_priority(CITYSYNC_PRIORITY_FIRE, "Fire");

       fprintf(stdout, "Fire thread started (priority=%d, set=%s)\n",

               CITYSYNC_PRIORITY_FIRE, priority_rc == 0 ? "yes" : "no");

       while (running) {

           bool raw = gpio_read(FLAME_PIN);

           bool detected;

#if FLAME_ACTIVE_LOW

           detected = (raw == false);

#else

           detected = (raw == true);

#endif

           update_priority_outputs(SENSOR_FLAME, detected ? 1 : 0);

           if (send_flame_update(ctx->coid, &seq, detected ? 1 : 0) == -1) {

               fprintf(stderr, "Fire update send failed\n");

               (void)send_sensor_status(ctx->coid, SUBSYSTEM_FIRE, &seq,

                                        CITYSYNC_SENSOR_STATUS_FAILED,

                                        ERR_SENSOR_READ);

           }

           sleep_until_next(&next_deadline_ns, FIRE_INTERVAL_MS);

       }

       return NULL;

}

/* ------------------------------------------------------------------ */

/* Sensor thread 2: Vehicle count / IR                                  */

/* ------------------------------------------------------------------ */

static void *vehicle_thread(void *arg)

{

       SensorContext *ctx = (SensorContext *)arg;

       uint64_t next_deadline_ns = 0;

       uint32_t seq = 0;

       uint32_t vehicle_count = 0;

       bool previous_detected = false;

       int priority_rc = set_thread_priority(CITYSYNC_PRIORITY_TRAFFIC,

                                             "Vehicle");

       fprintf(stdout, "Vehicle thread started (priority=%d, set=%s)\n",

               CITYSYNC_PRIORITY_TRAFFIC, priority_rc == 0 ? "yes" : "no");

       while (running) {

           bool raw = gpio_read(IR_PIN);

           bool detected;

#if IR_ACTIVE_LOW

           detected = (raw == false);

#else

           detected = (raw == true);

#endif

           /* Count one vehicle on each inactive-to-active detection edge. */

           if (detected && !previous_detected)

               vehicle_count++;

           previous_detected = detected;

           if (send_subsystem_update(ctx->coid, SUBSYSTEM_TRAFFIC, &seq,

                                     (float)vehicle_count) == -1)

               fprintf(stderr, "Vehicle count update failed\n");

           sleep_until_next(&next_deadline_ns, TRAFFIC_INTERVAL_MS);

       }

       return NULL;

}

/* ------------------------------------------------------------------ */

/* Sensor thread 3: Current (PCF8591 AIN1)                              */

/* ------------------------------------------------------------------ */

static void *current_thread(void *arg)

{

       SensorContext *ctx = (SensorContext *)arg;

       uint64_t next_deadline_ns = 0;

       uint32_t seq = 0;

       int status_was_failed = 0;

       int priority_rc = set_thread_priority(CITYSYNC_PRIORITY_ENERGY,

                                             "Current");

       fprintf(stdout, "Current thread started (priority=%d, set=%s)\n",

               CITYSYNC_PRIORITY_ENERGY, priority_rc == 0 ? "yes" : "no");

       while (running) {

           uint8_t raw = 0;

           int rc = read_adc_channel_locked(ctx, 1, &raw);

           if (rc == 0) {

               float voltage = ((float)raw / ADC_MAX) * ADC_VREF;

               float simulated_current_ma =

                   ((float)raw / ADC_MAX) * CURRENT_MAX_MA;

               (void)voltage;

               update_priority_outputs(

                   SENSOR_CURRENT,

                   simulated_current_ma > CURRENT_THRESHOLD_MA ? 1 : 0);

               if (send_subsystem_update(ctx->coid, SUBSYSTEM_ENERGY, &seq,

                                         simulated_current_ma) == 0) {

                   if (status_was_failed) {

                       (void)send_sensor_status(ctx->coid, SUBSYSTEM_ENERGY,

                                                &seq, CITYSYNC_SENSOR_STATUS_OK, 0);

                       status_was_failed = 0;

                   }

               }

           } else {

               fprintf(stderr, "PCF8591 AIN1 read failed\n");

               update_priority_outputs(SENSOR_CURRENT, 0);

               (void)send_sensor_status(ctx->coid, SUBSYSTEM_ENERGY, &seq,

                                        CITYSYNC_SENSOR_STATUS_FAILED,

                                        ERR_SENSOR_READ);

               status_was_failed = 1;

           }

           sleep_until_next(&next_deadline_ns, ENERGY_INTERVAL_MS);

       }

       return NULL;

}

/* ------------------------------------------------------------------ */

/* Sensor thread 4: Temperature (PCF8591 AIN0 / LM35)                   */

/* ------------------------------------------------------------------ */

static void *temperature_thread(void *arg)

{

       SensorContext *ctx = (SensorContext *)arg;

       uint64_t next_deadline_ns = 0;

       uint32_t seq = 0;

       int status_was_failed = 0;

       int priority_rc = set_thread_priority(CITYSYNC_PRIORITY_ENVIRONMENT,

                                             "Temperature");

       fprintf(stdout, "Temperature thread started (priority=%d, set=%s)\n",

               CITYSYNC_PRIORITY_ENVIRONMENT,

               priority_rc == 0 ? "yes" : "no");

       while (running) {

           uint8_t raw = 0;

           int rc = read_adc_channel_locked(ctx, 0, &raw);

           if (rc == 0) {

               float voltage = ((float)raw / ADC_MAX) * ADC_VREF;

               float temperature_c = voltage * 100.0f;

               /* Project's configured plausible LM35 range. */

               if (temperature_c == temperature_c &&

                   temperature_c >= -40.0f && temperature_c <= 150.0f) {

                   if (send_subsystem_update(ctx->coid,

                                             SUBSYSTEM_ENVIRONMENT,

                                             &seq, temperature_c) == 0) {

                       if (status_was_failed) {

                           (void)send_sensor_status(

                               ctx->coid, SUBSYSTEM_ENVIRONMENT, &seq,

                               CITYSYNC_SENSOR_STATUS_OK, 0);

                           status_was_failed = 0;

                       }

                   }

               } else {

                   fprintf(stderr,

                           "LM35 value out of plausible range: %.2f C\n",

                           temperature_c);

                   (void)send_sensor_status(

                       ctx->coid, SUBSYSTEM_ENVIRONMENT, &seq,

                       CITYSYNC_SENSOR_STATUS_FAILED, ERR_SENSOR_RANGE);

                   status_was_failed = 1;

               }

           } else {

               fprintf(stderr, "PCF8591 AIN0 read failed\n");

               (void)send_sensor_status(

                   ctx->coid, SUBSYSTEM_ENVIRONMENT, &seq,

                   CITYSYNC_SENSOR_STATUS_FAILED, ERR_SENSOR_READ);

               status_was_failed = 1;

           }

           sleep_until_next(&next_deadline_ns, ENVIRONMENT_INTERVAL_MS);

       }

       return NULL;

}

/* ------------------------------------------------------------------ */

/* Sensor thread 5: Ultrasonic distance / water subsystem               */

/* ------------------------------------------------------------------ */

static void *water_thread(void *arg)

{

       SensorContext *ctx = (SensorContext *)arg;

       uint64_t next_deadline_ns = 0;

       uint32_t seq = 0;

       int status_was_failed = 0;

       int priority_rc = set_thread_priority(CITYSYNC_PRIORITY_WATER, "Water");

       fprintf(stdout, "Water thread started (priority=%d, set=%s)\n",

               CITYSYNC_PRIORITY_WATER, priority_rc == 0 ? "yes" : "no");

       while (running) {

           float distance_cm = measure_ultrasonic_cm();

           if (distance_cm == distance_cm &&

               distance_cm >= ULTRASONIC_MIN_CM &&

               distance_cm <= ULTRASONIC_MAX_CM) {

               update_priority_outputs(

                   SENSOR_ULTRASONIC,

                   distance_cm > ULTRASONIC_THRESHOLD_CM ? 1 : 0);

               if (send_subsystem_update(ctx->coid, SUBSYSTEM_WATER, &seq,

                                         distance_cm) == 0) {

                   if (status_was_failed) {

                       (void)send_sensor_status(

                           ctx->coid, SUBSYSTEM_WATER, &seq,

                           CITYSYNC_SENSOR_STATUS_OK, 0);

                       status_was_failed = 0;

                   }

               }

           } else {

               /*

                * No/invalid echo is not proof of disconnection. TwinSync should

                * retain the last valid value and mark it STALE/NO_ECHO.

                */

               update_priority_outputs(SENSOR_ULTRASONIC, 0);

               (void)send_sensor_status(

                   ctx->coid, SUBSYSTEM_WATER, &seq,

                   CITYSYNC_SENSOR_STATUS_NO_ECHO,

                   distance_cm < 0.0f ? ERR_SENSOR_READ : ERR_SENSOR_RANGE);

               status_was_failed = 0;

           }

           sleep_until_next(&next_deadline_ns, WATER_INTERVAL_MS);

       }

       return NULL;

}

/* ------------------------------------------------------------------ */

/* Main                                                                 */

/* ------------------------------------------------------------------ */

int main(void)

{

       QnxThread threads[5] = {{0}};

       const char *thread_names[5] = {

           "Fire", "Water", "Current", "Vehicle", "Temperature"

       };

       void *(*thread_entries[5])(void *) = {

           fire_thread,

           water_thread,

           current_thread,

           vehicle_thread,

           temperature_thread

       };

       SensorContext ctx;

       struct sigaction sa;

       int rc;

       int all_threads_created = 1;

       int join_failed = 0;

       memset(&sa, 0, sizeof(sa));

       sa.sa_handler = stop_program;

       memset(&sa.sa_mask, 0, sizeof(sa.sa_mask));

       sa.sa_flags = 0;

       if (SignalAction(0, &__signalstub, SIGINT, &sa, NULL) == -1) {

           fprintf(stderr, "SignalAction SIGINT failed: %s\n", strerror(errno));

           return EXIT_FAILURE;

       }

       if (SignalAction(0, &__signalstub, SIGTERM, &sa, NULL) == -1) {

           fprintf(stderr, "SignalAction SIGTERM failed: %s\n", strerror(errno));

           return EXIT_FAILURE;

       }

       printf("\n============================================\n");

       printf(" CitySync RT+ - Multirate Sensor Acquisition\n");

       printf(" Fire | Water | Current | Vehicle | Temperature\n");

       printf("Sampling periods: Fire=%u ms, Water=%u ms, Current=%u ms, Vehicle=%u ms, Temperature=%u ms\n",
              FIRE_INTERVAL_MS, WATER_INTERVAL_MS, ENERGY_INTERVAL_MS,
              TRAFFIC_INTERVAL_MS, ENVIRONMENT_INTERVAL_MS);

       printf("LED outputs: Flame=GPIO5, Distance>%.0f cm=GPIO6, Simulated current>%.0f mA=GPIO13\n",
              ULTRASONIC_THRESHOLD_CM, CURRENT_THRESHOLD_MA);

       printf("============================================\n");

       if (gpio_sys_init() != 0) {

           fprintf(stderr, "ERROR: GPIO initialization failed\n");

           return EXIT_FAILURE;

       }

       gpio_initialized = 1;

       /* Configure sensor GPIO directions. */

       gpio_set_mode_input(IR_PIN);

       gpio_set_mode_input(FLAME_PIN);

       gpio_set_mode_output(ULTRASONIC_TRIG);

       gpio_set_mode_input(ULTRASONIC_ECHO);

       /* Configure LED GPIO directions and start with all LEDs OFF. */

       gpio_set_mode_output(LED1_PIN);

       gpio_set_mode_output(LED2_PIN);

       gpio_set_mode_output(LED3_PIN);

       gpio_write(ULTRASONIC_TRIG, false);

       gpio_write(LED1_PIN, false);

       gpio_write(LED2_PIN, false);

       gpio_write(LED3_PIN, false);

       /* Initialize the PCF8591 ADC. */

       adc_fd = pcf8591_init("/dev/i2c1");

       if (adc_fd < 0) {

           fprintf(stderr, "ERROR: Cannot initialize PCF8591 on /dev/i2c1\n");

           gpio_sys_cleanup();

           gpio_initialized = 0;

           return EXIT_FAILURE;

       }

       /* Open the existing TwinSync named channel. */

       twin_coid = name_open(TWINSYNC_CHANNEL, 0);

       if (twin_coid == -1) {

           fprintf(stderr, "ERROR: Cannot open %s: %s\n",

                   TWINSYNC_CHANNEL, strerror(errno));

           pcf8591_close(adc_fd);

           adc_fd = -1;

           gpio_sys_cleanup();

           gpio_initialized = 0;

           return EXIT_FAILURE;

       }

       /* Initialize ADC mutex. */

       if (SyncTypeCreate(_NTO_SYNC_MUTEX_FREE, &adc_mutex, NULL) == -1) {

           fprintf(stderr, "ERROR: ADC SyncTypeCreate failed: %s\n",

                   strerror(errno));

           name_close(twin_coid);

           twin_coid = -1;

           pcf8591_close(adc_fd);

           adc_fd = -1;

           gpio_sys_cleanup();

           gpio_initialized = 0;

           return EXIT_FAILURE;

       }

       adc_mutex_initialized = 1;

       /* Initialize LED-state mutex. */

       if (SyncTypeCreate(_NTO_SYNC_MUTEX_FREE, &priority_mutex, NULL) == -1) {

           fprintf(stderr, "ERROR: Priority SyncTypeCreate failed: %s\n",

                   strerror(errno));

           (void)SyncTypeDestroy(_NTO_SYNC_MUTEX_FREE, &adc_mutex);

           adc_mutex_initialized = 0;

           name_close(twin_coid);

           twin_coid = -1;

           pcf8591_close(adc_fd);

           adc_fd = -1;

           gpio_sys_cleanup();

           gpio_initialized = 0;

           return EXIT_FAILURE;

       }

       priority_mutex_initialized = 1;

       printf("\nConfigured sensor periods:\n");

       printf("  Traffic     : %u ms\\n", TRAFFIC_INTERVAL_MS);

       printf("  Fire        : %u ms\\n", FIRE_INTERVAL_MS);

       printf("  Current     : %u ms\\n", ENERGY_INTERVAL_MS);

       printf("  Water       : %u ms\\n", WATER_INTERVAL_MS);

       printf("  Environment : %u ms\\n", ENVIRONMENT_INTERVAL_MS);

       printf("\nRequested SCHED_FIFO priorities:\n");

       printf("  Fire        : %d\n", CITYSYNC_PRIORITY_FIRE);

       printf("  Water       : %d\n", CITYSYNC_PRIORITY_WATER);

       printf("  Current     : %d\n", CITYSYNC_PRIORITY_ENERGY);

       printf("  Vehicle     : %d\n", CITYSYNC_PRIORITY_TRAFFIC);

       printf("  Temperature : %d\n", CITYSYNC_PRIORITY_ENVIRONMENT);

       ctx.coid = twin_coid;

       ctx.adc_mutex = &adc_mutex;

       ctx.adc_fd = adc_fd;

       /* Create all five sensor threads. */

       for (int i = 0; i < 5; ++i) {

           rc = create_qnx_thread(&threads[i],

                                  thread_entries[i],

                                  &ctx,

                                  thread_names[i]);

           if (rc != 0) {

               all_threads_created = 0;

               break;

           }

           printf("%s thread created (QNX tid=%d)\n",

                  thread_names[i], threads[i].tid);

       }

       if (!all_threads_created) {

           fprintf(stderr, "ERROR: One or more sensor threads did not start.\n");

           running = 0;

       }

       /* Main thread waits until SIGINT/SIGTERM or a startup failure. */

       while (running) {

           uint64_t pause_ns = 100000000ULL;

           if (TimerTimeout(CLOCK_MONOTONIC, _NTO_TIMEOUT_NANOSLEEP,

                            NULL, &pause_ns, NULL) == -1 && errno != EINTR) {

               fprintf(stderr, "Main-loop TimerTimeout failed: %s\n",

                       strerror(errno));

               break;

           }

       }

       /* Join workers before closing shared device/channel handles. */

       for (int i = 0; i < 5; ++i) {

           if (join_qnx_thread(&threads[i], thread_names[i]) != 0)

               join_failed = 1;

       }

       if (join_failed) {

           fprintf(stderr,

                   "ERROR: Could not confirm termination of every worker thread.\n");

           /* Do not close resources that a possibly-running thread may use. */

           return EXIT_FAILURE;

       }

       /* Turn off LEDs before releasing GPIO resources. */

       if (gpio_initialized) {

           gpio_write(LED1_PIN, false);

           gpio_write(LED2_PIN, false);

           gpio_write(LED3_PIN, false);

           gpio_write(ULTRASONIC_TRIG, false);

       }

       if (priority_mutex_initialized) {

           if (SyncTypeDestroy(_NTO_SYNC_MUTEX_FREE, &priority_mutex) == -1) {

               fprintf(stderr, "SyncTypeDestroy priority mutex failed: %s\n",

                       strerror(errno));

           } else {

               priority_mutex_initialized = 0;

           }

       }

       if (adc_mutex_initialized) {

           if (SyncTypeDestroy(_NTO_SYNC_MUTEX_FREE, &adc_mutex) == -1) {

               fprintf(stderr, "SyncTypeDestroy ADC mutex failed: %s\n",

                       strerror(errno));

           } else {

               adc_mutex_initialized = 0;

           }

       }

       if (twin_coid != -1) {

           name_close(twin_coid);

           twin_coid = -1;

       }

       if (adc_fd >= 0) {

           pcf8591_close(adc_fd);

           adc_fd = -1;

       }

       if (gpio_initialized) {

           gpio_sys_cleanup();

           gpio_initialized = 0;

       }

       printf("Sensor acquisition stopped.\n");

       return all_threads_created ? EXIT_SUCCESS : EXIT_FAILURE;

}
