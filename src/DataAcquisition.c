#define _QNX_SOURCE 1
#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <errno.h>
#include <string.h>
#include <pthread.h>
#include <sched.h>
#include <sys/neutrino.h>
#include <sys/dispatch.h>
#include "pcf8591_adc.h"
#include "rpi_gpio_qnx.h"
#include "../include/citysync_types.h"

/*
 * API selection for this native-API experiment:
 *   Thread management : QNX ThreadCreate() / ThreadJoin()
 *   Synchronization   : QNX SyncTypeCreate(), SyncMutexLock(),
 *                       SyncMutexUnlock(), SyncTypeDestroy()
 *   Time management   : POSIX clock_gettime(), clock_nanosleep(), nanosleep()
 *   Scheduling        : POSIX sched_*() / pthread_setschedparam()
 *   IPC               : QNX name_open(), MsgSend(), name_close()
 *   Signals           : POSIX sigaction()
 *   GPIO / ADC / C    : Existing project interfaces and C library
 *
 * QNX SDP 8.0 cautions against direct use of ThreadCreate() and Sync*
 * kernel calls in production. This implementation uses them because the
 * project requested a native-API learning version. Validate on the target.
 *
 * CitySync RT+ sensor acquisition for QNX SDP 8.0.
 *
 * Keeps the existing pinout. Four analog/distance/count subsystems use CITYSYNC_MSG_UPDATE; Fire uses CITYSYNC_MSG_UPDATE_FLAME.
 * Important limitations:
 * - gpio_read() returns only a Boolean and cannot report GPIO I/O errors.
 * - PCF8591 AIN1 potentiometer value is a simulated current, not measured
 *   current. A real current sensor and calibration are required for that.
 * - Ultrasonic output is distance from the sensor, not tank water level.
 */

#define IR_PIN                  17
#define FLAME_PIN               27
#define ULTRASONIC_TRIG         23
#define ULTRASONIC_ECHO         24
#define ADC_VREF                3.3f
#define ADC_MAX                 255.0f
#define CURRENT_MAX_MA          5000.0f
#define FLAME_ACTIVE_LOW        1
#define IR_ACTIVE_LOW           1
#define TWINSYNC_CHANNEL        "CitySyncTwin"

/* Set these only after confirming your tank geometry and sensor range. */
#define ULTRASONIC_MIN_CM       2.0f
#define ULTRASONIC_MAX_CM       400.0f

#define ERR_SENSOR_READ         1
#define ERR_SENSOR_RANGE        2
#define ERR_SENSOR_INIT         3

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

static uint64_t monotonic_ms(void)
{
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) == -1)
        return 0;
    return (uint64_t)ts.tv_sec * 1000ULL +
           (uint64_t)ts.tv_nsec / 1000000ULL;
}

static uint64_t monotonic_us(void)
{
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) == -1)
        return 0;
    return (uint64_t)ts.tv_sec * 1000000ULL +
           (uint64_t)ts.tv_nsec / 1000ULL;
}

static void stop_program(int sig)
{
    (void)sig;
    running = 0;
}

/*
 * Native QNX thread support.
 * ThreadCreate() requires caller-provided stack memory. Keep each stack
 * allocated until its thread has been successfully joined.
 */

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
    attr.__flags = PTHREAD_CREATE_JOINABLE;
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
        /* Keep the stack allocated if thread termination wasn't confirmed. */
        return -1;
    }
    free(thread->stack);
    thread->stack = NULL;
    thread->created = 0;
    return 0;
}

/* Protect the shared PCF8591 device with the QNX native mutex calls. */
static int read_adc_channel_locked(SensorContext *ctx,
                                   int channel,
                                   uint8_t *raw)
{
    int rc;
    if (SyncMutexLock(ctx->adc_mutex) == -1) {
        fprintf(stderr, "SyncMutexLock failed: %s\n", strerror(errno));
        return -1;
    }
    rc = pcf8591_read_channel(ctx->adc_fd, channel, raw);
    if (SyncMutexUnlock(ctx->adc_mutex) == -1) {
        fprintf(stderr, "SyncMutexUnlock failed: %s\n", strerror(errno));
        return -1;
    }
    return rc;
}

static void sleep_until_next(struct timespec *next, unsigned int period_ms)
{
    next->tv_nsec += (long)(period_ms % 1000U) * 1000000L;
    next->tv_sec += (time_t)(period_ms / 1000U);
    if (next->tv_nsec >= 1000000000L) {
        next->tv_nsec -= 1000000000L;
        next->tv_sec++;
    }
    while (running) {
        int rc = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, next, NULL);
        if (rc == 0)
            break;
        if (rc != EINTR) {
            struct timespec rel = {0, 1000000L};
            nanosleep(&rel, NULL);
            break;
        }
    }
}

static int set_thread_priority(int priority, const char *name)
{
    struct sched_param sp;
    int policy = SCHED_FIFO;
    int minp = sched_get_priority_min(policy);
    int maxp = sched_get_priority_max(policy);
    if (minp == -1 || maxp == -1) {
        fprintf(stderr, "%s: cannot query SCHED_FIFO range: %s\n",
                name, strerror(errno));
        return -1;
    }
    if (priority < minp || priority > maxp) {
        fprintf(stderr, "%s: requested priority %d outside SCHED_FIFO range %d..%d\n",
                name, priority, minp, maxp);
        return -1;
    }
    memset(&sp, 0, sizeof(sp));
    sp.sched_priority = priority;
    {
        int rc = pthread_setschedparam(pthread_self(), policy, &sp);
        if (rc != 0) {
            fprintf(stderr, "%s: cannot set SCHED_FIFO priority %d: %s\n",
                    name, priority, strerror(rc));
            return -1;
        }
    }
    return 0;
}

static int send_subsystem_update(int coid, SubsystemID id,
                                 uint32_t *sequence, float value)
{
    CitySyncMessage msg;
    memset(&msg, 0, sizeof(msg));
    msg.type = CITYSYNC_MSG_UPDATE;
    msg.data.subsystem_id = (int)id;
    msg.data.sequence_number = ++(*sequence);
    msg.data.timestamp_ms = monotonic_ms();
    msg.data.timestamp_us = monotonic_us();
    msg.data.value = value;
    msg.data.valid = 1;
    msg.data.freshness = CITYSYNC_DATA_ACTIVE;
    if (msg.data.timestamp_ms == 0 || msg.data.timestamp_us == 0 || value != value ||
        value > 3.4028234e+38F || value < -3.4028234e+38F)
        return -1;
    if (MsgSend(coid, &msg, sizeof(msg), NULL, 0) == -1) {
        fprintf(stderr, "MsgSend subsystem %d failed: %s\n",
                (int)id, strerror(errno));
        return -1;
    }
    return 0;
}

static int send_flame_update(int coid, uint32_t *sequence, int detected)
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

static int send_sensor_status(int coid, SubsystemID id,
                              uint32_t *sequence, int status, int error_code)
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

static int set_sensor_thread_priority(int priority, const char *name)
{
    /* Failure is reported but does not silently terminate acquisition. */
    return set_thread_priority(priority, name);
}

static void *fire_thread(void *arg)
{
    SensorContext *ctx = (SensorContext *)arg;
    struct timespec next;
    uint32_t seq = 0;
    int priority_rc = set_sensor_thread_priority(CITYSYNC_PRIORITY_FIRE, "Fire");
    clock_gettime(CLOCK_MONOTONIC, &next);
    fprintf(stdout, "Fire thread started (priority request=%d, set=%s)\n",
            CITYSYNC_PRIORITY_FIRE, priority_rc == 0 ? "yes" : "no");
    while (running) {
        bool raw, detected;
        raw = gpio_read(FLAME_PIN);
#if FLAME_ACTIVE_LOW
        detected = (raw == false);
#else
        detected = (raw == true);
#endif
        if (send_flame_update(ctx->coid, &seq, detected ? 1 : 0) == -1) {
            fprintf(stderr, "Fire update send failed\n");
            (void)send_sensor_status(ctx->coid, SUBSYSTEM_FIRE, &seq,
                                     CITYSYNC_SENSOR_STATUS_FAILED, ERR_SENSOR_READ);
        }
        sleep_until_next(&next, FIRE_INTERVAL_MS);
    }
    return NULL;
}

static void *vehicle_thread(void *arg)
{
    SensorContext *ctx = (SensorContext *)arg;
    struct timespec next;
    uint32_t seq = 0;
    uint32_t vehicle_count = 0;
    bool previous_detected = false;
    int priority_rc = set_sensor_thread_priority(CITYSYNC_PRIORITY_TRAFFIC, "Vehicle");
    clock_gettime(CLOCK_MONOTONIC, &next);
    fprintf(stdout, "Vehicle thread started (priority request=%d, set=%s)\n",
            CITYSYNC_PRIORITY_TRAFFIC, priority_rc == 0 ? "yes" : "no");
    while (running) {
        bool raw, detected;
        raw = gpio_read(IR_PIN);
#if IR_ACTIVE_LOW
        detected = (raw == false);
#else
        detected = (raw == true);
#endif
        if (detected && !previous_detected)
            vehicle_count++;
        previous_detected = detected;
        if (send_subsystem_update(ctx->coid, SUBSYSTEM_TRAFFIC, &seq,
                                  (float)vehicle_count) == -1)
            fprintf(stderr, "Vehicle count update failed\n");
        sleep_until_next(&next, TRAFFIC_INTERVAL_MS);
    }
    return NULL;
}

static void *current_thread(void *arg)
{
    SensorContext *ctx = (SensorContext *)arg;
    struct timespec next;
    uint32_t seq = 0;
    int status_was_failed = 0;
    int priority_rc = set_sensor_thread_priority(CITYSYNC_PRIORITY_ENERGY, "Current");
    clock_gettime(CLOCK_MONOTONIC, &next);
    fprintf(stdout, "Current thread started (priority request=%d, set=%s)\n",
            CITYSYNC_PRIORITY_ENERGY, priority_rc == 0 ? "yes" : "no");
    while (running) {
        uint8_t raw = 0;
        int rc;
        rc = read_adc_channel_locked(ctx, 1, &raw);
        if (rc == 0) {
            float voltage = ((float)raw / ADC_MAX) * ADC_VREF;
            float simulated_current_ma = ((float)raw / ADC_MAX) * CURRENT_MAX_MA;
            (void)voltage;
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
            (void)send_sensor_status(ctx->coid, SUBSYSTEM_ENERGY, &seq,
                                     CITYSYNC_SENSOR_STATUS_FAILED, ERR_SENSOR_READ);
            status_was_failed = 1;
        }
        sleep_until_next(&next, ENERGY_INTERVAL_MS);
    }
    return NULL;
}

static void *temperature_thread(void *arg)
{
    SensorContext *ctx = (SensorContext *)arg;
    struct timespec next;
    uint32_t seq = 0;
    int status_was_failed = 0;
    int priority_rc = set_sensor_thread_priority(CITYSYNC_PRIORITY_ENVIRONMENT, "Temperature");
    clock_gettime(CLOCK_MONOTONIC, &next);
    fprintf(stdout, "Temperature thread started (priority request=%d, set=%s)\n",
            CITYSYNC_PRIORITY_ENVIRONMENT, priority_rc == 0 ? "yes" : "no");
    while (running) {
        uint8_t raw = 0;
        int rc;
        rc = read_adc_channel_locked(ctx, 0, &raw);
        if (rc == 0) {
            float voltage = ((float)raw / ADC_MAX) * ADC_VREF;
            float temperature_c = voltage * 100.0f;
            /* A 0..3.3 V ADC input corresponds to 0..330 C by this formula.
               Apply a project-specific plausible range if required. */
            if (temperature_c == temperature_c && temperature_c >= -40.0f &&
                temperature_c <= 150.0f) {
                if (send_subsystem_update(ctx->coid, SUBSYSTEM_ENVIRONMENT,
                                          &seq, temperature_c) == 0) {
                    if (status_was_failed) {
                        (void)send_sensor_status(ctx->coid, SUBSYSTEM_ENVIRONMENT,
                                                 &seq, CITYSYNC_SENSOR_STATUS_OK, 0);
                        status_was_failed = 0;
                    }
                }
            } else {
                fprintf(stderr, "LM35 value out of configured plausible range: %.2f C\n",
                        temperature_c);
                (void)send_sensor_status(ctx->coid, SUBSYSTEM_ENVIRONMENT, &seq,
                                         CITYSYNC_SENSOR_STATUS_FAILED, ERR_SENSOR_RANGE);
                status_was_failed = 1;
            }
        } else {
            fprintf(stderr, "PCF8591 AIN0 read failed\n");
            (void)send_sensor_status(ctx->coid, SUBSYSTEM_ENVIRONMENT, &seq,
                                     CITYSYNC_SENSOR_STATUS_FAILED, ERR_SENSOR_READ);
            status_was_failed = 1;
        }
        sleep_until_next(&next, ENVIRONMENT_INTERVAL_MS);
    }
    return NULL;
}

static void *water_thread(void *arg)
{
    SensorContext *ctx = (SensorContext *)arg;
    struct timespec next;
    uint32_t seq = 0;
    int status_was_failed = 0;
    int priority_rc = set_sensor_thread_priority(CITYSYNC_PRIORITY_WATER, "Water");
    clock_gettime(CLOCK_MONOTONIC, &next);
    fprintf(stdout, "Water thread started (priority request=%d, set=%s)\n",
            CITYSYNC_PRIORITY_WATER, priority_rc == 0 ? "yes" : "no");
    while (running) {
        float distance_cm;
        distance_cm = measure_ultrasonic_cm();
        if (distance_cm == distance_cm && distance_cm >= ULTRASONIC_MIN_CM &&
            distance_cm <= ULTRASONIC_MAX_CM) {
            if (send_subsystem_update(ctx->coid, SUBSYSTEM_WATER, &seq,
                                      distance_cm) == 0) {
                if (status_was_failed) {
                    (void)send_sensor_status(ctx->coid, SUBSYSTEM_WATER,
                                             &seq, CITYSYNC_SENSOR_STATUS_OK, 0);
                    status_was_failed = 0;
                }
            }
        } else {
            /* A missing/invalid echo is not, by itself, proof of disconnection.
             * Tell TwinSync to retain the last good distance and mark it STALE.
             * A dedicated hardware diagnostic is required to distinguish an
             * unplugged HC-SR04 from a target outside its range. */
            (void)send_sensor_status(ctx->coid, SUBSYSTEM_WATER, &seq,
                                     CITYSYNC_SENSOR_STATUS_NO_ECHO,
                                     distance_cm < 0.0f ? ERR_SENSOR_READ : ERR_SENSOR_RANGE);
            status_was_failed = 0;
        }
        sleep_until_next(&next, WATER_INTERVAL_MS);
    }
    return NULL;
}

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
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    if (sigaction(SIGINT, &sa, NULL) == -1) {
        fprintf(stderr, "Cannot install SIGINT handler: %s\n", strerror(errno));
        return EXIT_FAILURE;
    }
    if (sigaction(SIGTERM, &sa, NULL) == -1) {
        fprintf(stderr, "Cannot install SIGTERM handler: %s\n", strerror(errno));
        return EXIT_FAILURE;
    }
    printf("\n============================================\n");
    printf(" CitySync RT+ - Multirate Sensor Acquisition\n");
    printf(" Fire | Water | Current | Vehicle | Temperature\n");
    printf("Periods: Fire 100 ms, Vehicle 100 ms, Current 500 ms, Water 1000 ms, Temperature 2000 ms\n");
    printf("============================================\n");

    if (gpio_sys_init() != 0) {
        fprintf(stderr, "ERROR: GPIO initialization failed\n");
        return EXIT_FAILURE;
    }
    gpio_initialized = 1;
    adc_fd = pcf8591_init("/dev/i2c1");
    if (adc_fd < 0) {
        fprintf(stderr, "ERROR: Cannot initialize PCF8591 on /dev/i2c1\n");
        gpio_sys_cleanup();
        return EXIT_FAILURE;
    }
    gpio_set_mode_input(IR_PIN);
    gpio_set_mode_input(FLAME_PIN);
    gpio_set_mode_output(ULTRASONIC_TRIG);
    gpio_set_mode_input(ULTRASONIC_ECHO);
    gpio_write(ULTRASONIC_TRIG, false);

    twin_coid = name_open(TWINSYNC_CHANNEL, 0);
    if (twin_coid == -1) {
        fprintf(stderr, "ERROR: Cannot open %s: %s\n",
                TWINSYNC_CHANNEL, strerror(errno));
        pcf8591_close(adc_fd);
        gpio_sys_cleanup();
        return EXIT_FAILURE;
    }
    if (SyncTypeCreate(_NTO_SYNC_MUTEX_FREE, &adc_mutex, NULL) == -1) {
        fprintf(stderr, "ERROR: SyncTypeCreate for ADC mutex failed: %s\n",
                strerror(errno));
        name_close(twin_coid);
        pcf8591_close(adc_fd);
        gpio_sys_cleanup();
        return EXIT_FAILURE;
    }
    adc_mutex_initialized = 1;

    printf("Configured periods: Fire=%u ms, Water=%u ms, Current=%u ms, Vehicle=%u ms, Temperature=%u ms\n",
           FIRE_INTERVAL_MS, WATER_INTERVAL_MS, ENERGY_INTERVAL_MS,
           TRAFFIC_INTERVAL_MS, ENVIRONMENT_INTERVAL_MS);
    printf("Requested SCHED_FIFO priorities: Fire=%d, Water=%d, Current=%d, Vehicle=%d, Temperature=%d\n",
           CITYSYNC_PRIORITY_FIRE, CITYSYNC_PRIORITY_WATER,
           CITYSYNC_PRIORITY_ENERGY, CITYSYNC_PRIORITY_TRAFFIC,
           CITYSYNC_PRIORITY_ENVIRONMENT);

    ctx.coid = twin_coid;
    ctx.adc_mutex = &adc_mutex;
    ctx.adc_fd = adc_fd;

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

    while (running) {
        struct timespec pause = {0, 100000000L};
        while (nanosleep(&pause, &pause) == -1 && errno == EINTR) {
            if (!running)
                break;
        }
    }

    /* Join workers before closing shared device/channel handles. */
    for (int i = 0; i < 5; ++i) {
        if (join_qnx_thread(&threads[i], thread_names[i]) != 0)
            join_failed = 1;
    }

    /*
     * If a join fails, do not close resources that a possibly-running
     * worker may still use. Returning from main terminates the process,
     * after which QNX reclaims its process resources.
     */
    if (join_failed) {
        fprintf(stderr, "ERROR: Could not confirm termination of every worker thread.\n");
        return EXIT_FAILURE;
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
        gpio_write(ULTRASONIC_TRIG, false);
        gpio_sys_cleanup();
        gpio_initialized = 0;
    }
    if (adc_mutex_initialized) {
        if (SyncTypeDestroy(_NTO_SYNC_MUTEX_FREE, &adc_mutex) == -1) {
            fprintf(stderr, "SyncTypeDestroy ADC mutex failed: %s\n",
                    strerror(errno));
        } else {
            adc_mutex_initialized = 0;
        }
    }

    printf("Sensor acquisition stopped.\n");
    return all_threads_created ? EXIT_SUCCESS : EXIT_FAILURE;
}
