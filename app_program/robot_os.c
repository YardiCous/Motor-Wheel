/** @file robot_os.c
 * @brief All application tasks and the existing network owner use core 0.
 * Packed shared updates are protected by a saved local interrupt mask.
 * See docs/design_review.md for ownership and the complete data flow.
 */
#include "robot_os.h"

#include "board_io.h"
#include "data_formats.h"
#include "robot_config.h"
#if APP_TEST
#include "selftest.h"
#endif /* APP_TEST */
#include "subsystem2/motion.h"
#if MOTOR_TEST
#include "subsystem2/physical_test.h"
#endif /* MOTOR_TEST */
#if ENCODER_TEST
#include "subsystem2/encoder_test.h"
#endif /* ENCODER_TEST */
#include "subsystem3/line.h"
#include "subsystem4/safety.h"
#include "subsystem4/terrain.h"
#include "subsystem5/navigation.h"
#include "subsystem5/obstacle.h"

#include <tk/tkernel.h>
#if (OBSTACLE_TEST || LINE_TEST)
#include <tm/tmonitor.h>
#endif /* OBSTACLE_TEST || LINE_TEST */

#define ROBOT_TASK_COUNT 7u
#define ROBOT_MOTION_DEPTH 4u
#define ROBOT_RANGE_SAMPLES 3u
#define ROBOT_HEARTBEATS 5u
#define ROBOT_SOURCE_COUNT 6u
#define ROBOT_COMMAND_MESSAGE_BYTES 5u
#define ROBOT_COMMAND_DEPTH 8u
#define ROBOT_TASK_STACK 2048
#define ROBOT_SOURCE_MOTION 0u
#define ROBOT_SOURCE_IMU 1u
#define ROBOT_SOURCE_SCAN 2u
#define ROBOT_SOURCE_NAV 3u
#define ROBOT_SOURCE_TIMING 4u
#define ROBOT_SOURCE_NETWORK 5u

/** @brief Every member is a shared control flag or bounded timestamp. */
typedef struct
{
    bool     b_latched : 1;
    bool     b_stop : 1;
    bool     b_reset : 1;
    bool     b_reset_motion : 1;
    bool     b_reset_imu : 1;
    bool     b_reset_scan : 1;
    bool     b_scan_request : 1;
    bool     b_motion_done : 1;
    bool     b_imu_ready : 1;
    bool     b_line_ready : 1;
    uint32_t sequence : 4;
    bool     b_sequence_valid : 1;
    bool     b_queue_fault : 1;
    bool     b_range_fault : 1;
    bool     b_remote_armed : 1;
    bool     b_network_online : 1;
    bool     b_line_visible : 1;
    bool     b_line_lost : 1;
    uint32_t reset_at_ms;
} shared_t;

/** @brief ISR phase is bounded to one hundred 5-ms releases. */
typedef struct
{
    uint8_t phase : 7;
} cyclic_t;

/** @brief The local-switch filter uses exactly eight information bits. */
typedef struct
{
    uint8_t stable_count : 2;
    uint8_t candidate : 3;
    uint8_t previous : 3;
} buttons_t;

#if OBSTACLE_TEST
/** @brief Console progress tracking for the bench obstacle test. */
typedef struct
{
    uint8_t  state;
    uint8_t  stage;
    uint8_t  look;
    uint16_t faults;
    bool     b_avoided;
} bench_t;
#endif /* OBSTACLE_TEST */

static volatile cyclic_t g_clock = {0};
#if ((!MOTOR_TEST) && (!ENCODER_TEST))
static buttons_t g_buttons = {0};
#endif /* MOTOR_TEST && ENCODER_TEST */
static volatile shared_t g_shared                               = {0};
static volatile uint8_t  g_packet[DATA_FORMATS_TELEMETRY_BYTES] = {0u};
static volatile uint8_t  g_sources[9]                           = {0u};
static volatile uint32_t g_heartbeat[ROBOT_HEARTBEATS]          = {0u};
static ID                gh_tasks[ROBOT_TASK_COUNT]             = {0};
static ID                gh_motion_queue                        = 0;
static ID                gh_barcode_queue                       = 0;
static ID                gh_imu_queue                           = 0;
static ID                gh_command_queue                       = 0;
static ID                gh_telemetry_queue                     = 0;
static ID                gh_echo_flag                           = 0;
static ID                gh_ack_queue                           = 0;
static ID                gh_cyclic                              = 0;

static uint32_t field_read(uint16_t offset, uint8_t width);
static void     field_write(uint16_t offset, uint8_t width, uint32_t value);
static void     fault_source(uint8_t source, uint16_t faults);
static void     heartbeat(uint8_t source);
static void     wait_release(void);
static void     cyclic_isr(void * p_argument);
static ID       create_queue(int32_t capacity, int32_t message_size);
static bool     create_objects(void);
static bool     create_tasks(void);
static void     safety_task(INT start, void * p_argument);
static void     motion_task(INT start, void * p_argument);
static void     line_task(INT start, void * p_argument);
static void     terrain_task(INT start, void * p_argument);
static void     navigation_task(INT start, void * p_argument);
static void     obstacle_task(INT start, void * p_argument);
static void     telemetry_task(INT start, void * p_argument);
#if ((!MOTOR_TEST) && (!ENCODER_TEST))
static void motion_cycle(motion_t * p_motion);
#endif /* MOTOR_TEST && ENCODER_TEST */
#if !OBSTACLE_TEST
static void line_cycle(line_t * p_line);
#else
static void line_stub(void);
static void bench_init(bench_t * p_bench);
static void bench_step(navigation_t * p_navigation, bench_t * p_bench);
#endif /* OBSTACLE_TEST */
#if ((!OBSTACLE_TEST) && (!LINE_TEST))
static void terrain_cycle(terrain_t * p_terrain, bool * pb_configured);
#else
static void terrain_stub(void);
#endif /* OBSTACLE_TEST || LINE_TEST */
#if LINE_TEST
static void line_report_init(void);
static void line_report(void);
#endif /* LINE_TEST */
#if ((!MOTOR_TEST) && (!ENCODER_TEST))
static void navigation_cycle(navigation_t * p_navigation);
static void local_commands(navigation_t * p_navigation);
static void remote_commands(navigation_t * p_navigation);
static void dispatch_navigation(navigation_t * p_navigation);
#endif /* MOTOR_TEST && ENCODER_TEST */
#if !LINE_TEST
static void obstacle_cycle(obstacle_t * p_obstacle, uint8_t * p_angle);
static bool range_sample(uint16_t * p_range);
#endif /* LINE_TEST */
static uint16_t current_faults(void);

/** @brief Errors leave PWM off and the initial task asleep. */
void robot_os_run(void)
{
    g_shared.b_latched   = true;
    g_shared.reset_at_ms = board_io_time_ms();
    bool b_ready         = create_objects();
#if APP_TEST
    b_ready = b_ready && (selftest_run());
#endif /* APP_TEST */
    b_ready = b_ready && (board_io_init(gh_echo_flag));
    b_ready = b_ready && (create_tasks());
    fault_source(ROBOT_SOURCE_NETWORK,
                 DATA_FORMATS_FAULT_WIFI | DATA_FORMATS_FAULT_MQTT);
    field_write(0u, 2u, DATA_FORMATS_VERSION);
    field_write(2u, 3u, DATA_FORMATS_IDLE);

    if (b_ready)
    {
        T_CCYC cyclic = {0};
        cyclic.cycatr = TA_HLNG;
        cyclic.cychdr = cyclic_isr;
        cyclic.cyctim = ROBOT_LINE_PERIOD_MS;
        cyclic.cycphs = ROBOT_LINE_PERIOD_MS;
        gh_cyclic     = tk_cre_cyc(&cyclic);
        b_ready       = (gh_cyclic > 0);
    }

    if (b_ready)
    {
        for (uint8_t task = 0u; task < ROBOT_TASK_COUNT; ++task)
        {
            b_ready = b_ready && (E_OK == tk_sta_tsk(gh_tasks[task], 0));
        }

        b_ready = b_ready && (E_OK == tk_sta_cyc(gh_cyclic));
    }

    if (!b_ready)
    {
        g_shared.b_latched = true;
        g_shared.b_stop    = true;
    }

    for (;;)
    {
        (void)tk_slp_tsk(TMO_FEVR);
    }
}

/** @brief Arrival timestamps allow the receiver to reject stale queue entries.
 */
bool robot_os_submit(uint8_t payload)
{
    uint8_t command  = 0u;
    uint8_t sequence = 0u;
    bool    b_valid  = data_formats_command_read(payload, &command, &sequence);
    uint8_t message[ROBOT_COMMAND_MESSAGE_BYTES] = {0u};
    message[0]                                   = payload;
    (void)data_formats_write(message, 8u, 32u, board_io_time_ms());

    if (b_valid && (gh_command_queue > 0))
    {
        b_valid = (E_OK == tk_snd_mbf(gh_command_queue, message,
                                      ROBOT_COMMAND_MESSAGE_BYTES, TMO_POL));
    }
    else
    {
        b_valid = false;
    }

    if ((!b_valid) && (0u == (payload & 0xc0u)))
    {
        uint32_t mask          = board_io_lock();
        g_shared.b_queue_fault = true;
        board_io_unlock(mask);
        (void)tk_wup_tsk(gh_tasks[0]);
    }

    return b_valid;
}

/** @brief One queue slot replaces stale telemetry during a long outage. */
bool robot_os_telemetry(uint8_t * p_packet)
{
    uint8_t message[DATA_FORMATS_TELEMETRY_BYTES + 4u] = {0u};
    bool    b_fresh = (gh_telemetry_queue > 0) &&
                      (32 == tk_rcv_mbf(gh_telemetry_queue, message, TMO_POL));
    uint32_t arrival = data_formats_read(message, 224u, 32u);
    b_fresh          = b_fresh && ((board_io_time_ms() - arrival) <= 500u);

    if (b_fresh)
    {
        for (uint8_t index = 0u; index < DATA_FORMATS_TELEMETRY_BYTES; ++index)
        {
            p_packet[index] = message[index];
        }
    }

    return b_fresh;
}

/** @brief Link faults have individual bits and remain advisory. */
void robot_os_network(bool b_wifi, bool b_mqtt)
{
    uint16_t faults = b_wifi ? 0u : DATA_FORMATS_FAULT_WIFI;
    faults |= b_mqtt ? 0u : DATA_FORMATS_FAULT_MQTT;
    fault_source(ROBOT_SOURCE_NETWORK, faults);
    field_write(135u, 2u, (b_wifi ? 1u : 0u) | (b_mqtt ? 2u : 0u));
    uint32_t mask             = board_io_lock();
    bool     b_changed        = (b_mqtt != g_shared.b_network_online);
    g_shared.b_network_online = b_mqtt;

    if (b_changed)
    {
        g_shared.b_remote_armed = false;
    }

    board_io_unlock(mask);
}

/** @brief ACK packs command2, sequence4 and acceptance1. */
bool robot_os_ack(uint8_t * p_ack)
{
    return (gh_ack_queue > 0) &&
           (1 == tk_rcv_mbf(gh_ack_queue, p_ack, TMO_POL));
}

/** @brief Bit writes to the shared packet always occur under the same lock. */
static uint32_t field_read(uint16_t offset, uint8_t width)
{
    uint32_t mask  = board_io_lock();
    uint32_t value = data_formats_read(g_packet, offset, width);
    board_io_unlock(mask);
    return value;
}

/** @brief No task can overwrite another task's fields in a packed byte. */
static void field_write(uint16_t offset, uint8_t width, uint32_t value)
{
    uint32_t mask    = board_io_lock();
    bool     b_valid = data_formats_write(g_packet, offset, width, value);
    g_shared.b_range_fault = (g_shared.b_range_fault) || (!b_valid);
    board_io_unlock(mask);
}

/** @brief Each fault producer owns its twelve bits within this packed array.
 */
static void fault_source(uint8_t source, uint16_t faults)
{
    uint32_t mask = board_io_lock();
    (void)data_formats_write(g_sources, source * 12u, 12u, faults);
    board_io_unlock(mask);
}

/** @brief The five watchdog timestamps use full modulo-2^32 milliseconds. */
static void heartbeat(uint8_t source)
{
    uint32_t now        = board_io_time_ms();
    uint32_t mask       = board_io_lock();
    g_heartbeat[source] = now;
    board_io_unlock(mask);
}

/** @brief Collapse excess releases instead of running catch-up bursts. */
static void wait_release(void)
{
    (void)tk_slp_tsk(TMO_FEVR);
    (void)tk_can_wup(TSK_SELF);
}

/** @brief A 5-ms cyclic release preserves periods without accumulated drift.
 */
static void cyclic_isr(void * p_argument)
{
    (void)p_argument;
    uint32_t phase = (g_clock.phase + 1u) % 100u;
    g_clock.phase  = phase;
    (void)tk_wup_tsk(gh_tasks[2]);

    if (0u == (phase % 2u))
    {
        (void)tk_wup_tsk(gh_tasks[1]);
        (void)tk_wup_tsk(gh_tasks[3]);
        (void)tk_wup_tsk(gh_tasks[4]);
    }

    if (0u == (phase % 4u))
    {
        (void)tk_wup_tsk(gh_tasks[0]);
    }

    if (0u == (phase % 50u))
    {
        (void)tk_wup_tsk(gh_tasks[6]);
    }
}

/** @brief Capacity includes the kernel's four-byte per-message overhead. */
static ID create_queue(int32_t capacity, int32_t message_size)
{
    T_CMBF queue = {0};
    queue.bufsz  = capacity;
    queue.maxmsz = message_size;
    queue.mbfatr = TA_TFIFO;
    return tk_cre_mbf(&queue);
}

/** @brief All queues exist before a single producer can run on either core. */
static bool create_objects(void)
{
    T_CFLG flag        = {0};
    flag.flgatr        = TA_WMUL;
    gh_echo_flag       = tk_cre_flg(&flag);
    gh_motion_queue    = create_queue(32, 3);
    gh_barcode_queue   = create_queue(32, 1);
    gh_imu_queue       = create_queue(32, 2);
    gh_command_queue   = create_queue(96, ROBOT_COMMAND_MESSAGE_BYTES);
    gh_telemetry_queue = create_queue(36, 32);
    gh_ack_queue       = create_queue(64, 1);
    return (gh_echo_flag > 0) && (gh_motion_queue > 0) &&
           (gh_barcode_queue > 0) && (gh_imu_queue > 0) &&
           (gh_command_queue > 0) && (gh_telemetry_queue > 0) &&
           (gh_ack_queue > 0);
}

/** @brief Safety outranks control; control outranks kernel USB/WiFi work. */
static bool create_tasks(void)
{
    static const FP g_entries[ROBOT_TASK_COUNT] = {
        safety_task,     motion_task,   line_task,     terrain_task,
        navigation_task, obstacle_task, telemetry_task};
    static const PRI g_priorities[ROBOT_TASK_COUNT] = {1, 2, 2, 3, 5, 6, 8};
    bool             b_valid                        = true;

    for (uint8_t task = 0u; task < ROBOT_TASK_COUNT; ++task)
    {
        T_CTSK configuration  = {0};
        configuration.task    = g_entries[task];
        configuration.itskpri = g_priorities[task];
        configuration.stksz   = ROBOT_TASK_STACK;
        configuration.tskatr  = TA_HLNG | TA_RNG3;
#if TK_SUPPORT_SMP
        configuration.tskatr |= TA_ASSPRC;
        configuration.assprc = TP_PRC1;
#endif
        gh_tasks[task] = tk_cre_tsk(&configuration);
        b_valid        = b_valid && (gh_tasks[task] > 0);
    }

    return b_valid;
}

/** @brief Immediate fault wake-ups supplement the 20-ms watchdog period. */
static void safety_task(INT start, void * p_argument)
{
    (void)start;
    (void)p_argument;
    safety_t monitor = {0};
    safety_init(&monitor);
    bool b_boot_complete = false;

    for (;;)
    {
        wait_release();
        uint32_t now     = board_io_time_ms();
        uint32_t mask    = board_io_lock();
        bool     b_ready = (g_shared.b_imu_ready) && (g_shared.b_line_ready);
        bool     b_reset = g_shared.b_reset;
        uint32_t grace   = now - g_shared.reset_at_ms;
        bool     b_stale =
            ((now - g_heartbeat[0]) > 20u) || ((now - g_heartbeat[1]) > 20u) ||
            ((now - g_heartbeat[2]) > 20u) || ((now - g_heartbeat[3]) > 40u) ||
            ((now - g_heartbeat[4]) > 180u);
        board_io_unlock(mask);
        bool b_watchdog = (b_boot_complete && (!b_reset) && b_stale) ||
                          ((grace > 2000u) && (b_stale || (!b_ready)));
        fault_source(ROBOT_SOURCE_TIMING,
                     b_watchdog ? DATA_FORMATS_FAULT_WATCHDOG : 0u);
        uint16_t active = current_faults();
        safety_update(&monitor, active);

        if (b_ready && ((!b_boot_complete) || b_reset))
        {
            bool b_clear     = safety_reset(&monitor);
            b_boot_complete  = b_boot_complete || b_clear;
            mask             = board_io_lock();
            g_shared.b_reset = !b_clear;
            board_io_unlock(mask);
        }

        mask               = board_io_lock();
        g_shared.b_latched = monitor.b_latched;
        board_io_unlock(mask);
        field_write(37u, 12u, monitor.faults);
    }
}

/** @brief Only this task calls the motor-output function after startup. */
static void motion_task(INT start, void * p_argument)
{
    (void)start;
    (void)p_argument;
#if !ENCODER_TEST
    motion_t controller = {0};
    uint32_t left       = 0u;
    uint32_t right      = 0u;
    (void)board_io_encoders(&left, &right);
    motion_init(&controller, left, right);
#endif /* ENCODER_TEST */
#if MOTOR_TEST
    physical_test_t physical_test = {0};
    physical_test_init(&physical_test);

    for (;;)
    {
        wait_release();
        board_io_trace(true);
        int16_t left_pwm  = 0;
        int16_t right_pwm = 0;
        physical_test_step(&physical_test, &controller, &left_pwm, &right_pwm);
        board_io_motor(left_pwm, right_pwm);
        board_io_trace(false);
    }
#elif ENCODER_TEST
    encoder_test_t encoder_test = {0};
    encoder_test_init(&encoder_test);

    for (;;)
    {
        wait_release();
        encoder_test_step(&encoder_test);
    }
#else

    for (;;)
    {
        wait_release();
        board_io_trace(true);
        motion_cycle(&controller);
        board_io_trace(false);
    }
#endif /* MOTOR_TEST */
}

/** @brief The single ADC owner handles tracking and barcode perception. */
static void line_task(INT start, void * p_argument)
{
    (void)start;
    (void)p_argument;
#if OBSTACLE_TEST
    for (;;)
    {
        wait_release();
        line_stub();
    }
#else
    line_t detector = {0};
    line_init(&detector);

    for (;;)
    {
        wait_release();
        line_cycle(&detector);
    }
#endif /* OBSTACLE_TEST */
}

/** @brief I2C failure recovery never blocks startup on WiFi. */
static void terrain_task(INT start, void * p_argument)
{
    (void)start;
    (void)p_argument;
#if (OBSTACLE_TEST || LINE_TEST)
    for (;;)
    {
        wait_release();
        terrain_stub();
    }
#else
    terrain_t detector = {0};
    terrain_init(&detector);
    bool b_configured = false;

    for (;;)
    {
        wait_release();
        terrain_cycle(&detector, &b_configured);
    }
#endif /* OBSTACLE_TEST || LINE_TEST */
}

/** @brief Navigation owns supervisory commands and motion arbitration. */
static void navigation_task(INT start, void * p_argument)
{
    (void)start;
    (void)p_argument;
#if MOTOR_TEST
    for (;;)
    {
        wait_release();
        heartbeat(3u);
    }
#elif ENCODER_TEST
    for (;;)
    {
        wait_release();
        heartbeat(3u);
    }
#else
    navigation_t manager = {0};
    navigation_init(&manager);
#if OBSTACLE_TEST
    bench_t bench = {0};
    bench_init(&bench);
#endif /* OBSTACLE_TEST */
#if LINE_TEST
    line_report_init();
#endif /* LINE_TEST */

    for (;;)
    {
        wait_release();
        local_commands(&manager);
        remote_commands(&manager);
        navigation_cycle(&manager);
#if OBSTACLE_TEST
        bench_step(&manager, &bench);
#endif /* OBSTACLE_TEST */
#if LINE_TEST
        line_report();
#endif /* LINE_TEST */
        dispatch_navigation(&manager);
        field_write(2u, 3u, manager.state);
        heartbeat(3u);
    }
#endif /* MOTOR_TEST */
}

/** @brief Ranging waits on the echo flag, freeing the CPU between edges. */
static void obstacle_task(INT start, void * p_argument)
{
    (void)start;
    (void)p_argument;
#if LINE_TEST
    /* No ultrasonic is needed here, so it can neither raise a fault nor
     * report a range. */
    for (;;)
    {
        fault_source(ROBOT_SOURCE_SCAN, 0u);
        heartbeat(4u);
        (void)tk_dly_tsk(ROBOT_RANGE_PERIOD_MS);
    }
#else
    obstacle_t scanner = {0};
    obstacle_init(&scanner);
    uint8_t last_angle = 30u;

    for (;;)
    {
        uint32_t start_ms = board_io_time_ms();
        obstacle_cycle(&scanner, &last_angle);
        uint32_t elapsed = board_io_time_ms() - start_ms;

        if (elapsed < ROBOT_RANGE_PERIOD_MS)
        {
            (void)tk_dly_tsk(ROBOT_RANGE_PERIOD_MS - elapsed);
        }

        heartbeat(4u);
    }
#endif /* LINE_TEST */
}

/** @brief Latest-value delivery bounds RAM during arbitrarily long outages. */
static void telemetry_task(INT start, void * p_argument)
{
    (void)start;
    (void)p_argument;
    uint8_t packet[DATA_FORMATS_TELEMETRY_BYTES + 4u]    = {0u};
    uint8_t discarded[DATA_FORMATS_TELEMETRY_BYTES + 4u] = {0u};

    for (;;)
    {
        wait_release();
        uint32_t mask = board_io_lock();

        for (uint8_t index = 0u; index < DATA_FORMATS_TELEMETRY_BYTES; ++index)
        {
            packet[index] = g_packet[index];
        }

        board_io_unlock(mask);
        (void)data_formats_write(packet, 224u, 32u, board_io_time_ms());
        ER result = tk_snd_mbf(gh_telemetry_queue, packet, 32, TMO_POL);

        if (E_TMOUT == result)
        {
            (void)tk_rcv_mbf(gh_telemetry_queue, discarded, TMO_POL);
            (void)tk_snd_mbf(gh_telemetry_queue, packet, 32, TMO_POL);
        }
    }
}

#if ((!MOTOR_TEST) && (!ENCODER_TEST))
/** @brief Stop/reset is processed before accepting another movement. */
static void motion_cycle(motion_t * p_motion)
{
    uint32_t left            = 0u;
    uint32_t right           = 0u;
    bool     b_encoder_valid = board_io_encoders(&left, &right);
    uint32_t mask            = board_io_lock();
    bool     b_stop          = g_shared.b_stop;
    bool     b_reset         = g_shared.b_reset_motion;
    bool     b_latched       = g_shared.b_latched;
    g_shared.b_stop          = false;
    g_shared.b_reset_motion  = false;
    board_io_unlock(mask);
    if ((!b_latched) && (MOTION_ESTOP == p_motion->mode))
    {
        motion_init(p_motion, left, right);
    }

    uint8_t  message[3] = {0u};
    uint32_t command    = 0u;
    INT      received   = 0;

    if (b_stop || b_reset || b_latched)
    {
        for (uint8_t slot = 0u; slot < ROBOT_MOTION_DEPTH; ++slot)
        {
            (void)tk_rcv_mbf(gh_motion_queue, message, TMO_POL);
        }

        (void)motion_accept(p_motion, 0u);
    }
    else
    {
        received = tk_rcv_mbf(gh_motion_queue, message, TMO_POL);

        if (3 == received)
        {
            command = data_formats_read(message, 0u, 24u);
            (void)motion_accept(p_motion, command);
        }
    }

    if (b_reset)
    {
        board_io_clear_fault();
        motion_init(p_motion, left, right);
    }

    bool    b_valid_delta = false;
    int32_t edges_left =
        motion_edge_delta(left, p_motion->last_left, &b_valid_delta);
    int32_t edges_right =
        motion_edge_delta(right, p_motion->last_right, &b_valid_delta);
    int32_t signed_speed = ((edges_left + edges_right) * 10210) / 960;
    signed_speed         = (signed_speed < -511) ? -511 : signed_speed;
    signed_speed         = (signed_speed > 511) ? 511 : signed_speed;
    int16_t left_pwm     = 0;
    int16_t right_pwm    = 0;
    /* Steering is offset binary in eight bits, with bias 127. */
    int16_t steering = (int16_t)((int32_t)field_read(121u, 8u) - 127);
    motion_step(p_motion, left, right, steering,
                b_latched || (!b_encoder_valid), &left_pwm, &right_pwm);
    board_io_motor(left_pwm, right_pwm);
    uint16_t faults = p_motion->b_stall ? DATA_FORMATS_FAULT_STALL : 0u;
    faults |= ((!b_encoder_valid) || (p_motion->b_encoder_fault))
                  ? DATA_FORMATS_FAULT_ENCODER
                  : 0u;
    faults |= p_motion->b_timeout ? DATA_FORMATS_FAULT_TIMEOUT : 0u;
    p_motion->b_timeout = false;
    /* Magnitude is clamped to 511; tick is explicitly modulo 128. */
    uint16_t speed =
        (uint16_t)((signed_speed < 0) ? -signed_speed : signed_speed);
    uint8_t tick = (uint8_t)((board_io_time_ms() / 5u) & 127u);
    field_write(5u, 32u,
                motion_snapshot(p_motion, tick, speed, signed_speed < 0));
    field_write(139u, 32u, left);
    field_write(171u, 32u, right);
    int32_t speed_left  = (edges_left * 20420) / 960;
    int32_t speed_right = (edges_right * 20420) / 960;
    bool    b_range     = (speed_left < -512) || (speed_left > 511) ||
                          (speed_right < -512) || (speed_right > 511);
    faults |= b_range ? DATA_FORMATS_FAULT_RANGE : 0u;
    fault_source(ROBOT_SOURCE_MOTION, faults);
    speed_left  = (speed_left < -512) ? -512 : speed_left;
    speed_left  = (speed_left > 511) ? 511 : speed_left;
    speed_right = (speed_right < -512) ? -512 : speed_right;
    speed_right = (speed_right > 511) ? 511 : speed_right;
    field_write(203u, 10u, data_formats_signed(speed_left, 10u));
    field_write(213u, 10u, data_formats_signed(speed_right, 10u));
    mask                   = board_io_lock();
    g_shared.b_motion_done = (g_shared.b_motion_done) || (p_motion->b_done);
    board_io_unlock(mask);
    p_motion->b_done = false;
    heartbeat(0u);

    if (0u != faults)
    {
        (void)tk_wup_tsk(gh_tasks[0]);
    }
}

#endif /* MOTOR_TEST && ENCODER_TEST */

#if !OBSTACLE_TEST
/** @brief Line errors and full-resolution sensor samples have one writer. */
static void line_cycle(line_t * p_line)
{
    uint16_t samples[ROBOT_IR_COUNT] = {0u};
    bool     b_valid                 = board_io_line(samples);
    uint32_t vehicle                 = field_read(2u, 3u);
    bool     b_departure             = (DATA_FORMATS_AVOID == vehicle) ||
                                       (DATA_FORMATS_SEARCH == vehicle) ||
                                       (DATA_FORMATS_BARCODE == vehicle);
    uint32_t snapshot                = field_read(5u, 32u);
    /* Distance occupies the snapshot's low twelve bits. */
    line_step(p_line, samples, (uint16_t)(snapshot & 4095u), b_departure);
    field_write(121u, 8u, p_line->error);
    field_write(129u, 3u, p_line->state);
    if (p_line->b_barcode_valid)
    {
        field_write(118u, 3u,
                    p_line->barcode | (p_line->b_barcode_valid ? 4u : 0u));
    }

    for (uint8_t sensor = 0u; sensor < ROBOT_IR_COUNT; ++sensor)
    {
        field_write(49u + (sensor * 12u), 12u, samples[sensor]);
    }

    uint32_t mask         = board_io_lock();
    g_shared.b_line_ready = b_valid && (LINE_CALIBRATING != p_line->state);
    board_io_unlock(mask);

    if (p_line->b_barcode_valid)
    {
        uint8_t event  = p_line->barcode;
        ER      result = tk_snd_mbf(gh_barcode_queue, &event, 1, TMO_POL);
        mask           = board_io_lock();
        g_shared.b_queue_fault =
            (g_shared.b_queue_fault) || ((E_OK != result));
        board_io_unlock(mask);
    }

    mask                    = board_io_lock();
    g_shared.b_line_visible = b_valid && (p_line->b_line_valid);
    g_shared.b_line_lost    = p_line->b_lost;
    board_io_unlock(mask);
    heartbeat(1u);
}

#endif /* OBSTACLE_TEST */

#if ((!OBSTACLE_TEST) && (!LINE_TEST))
/** @brief Failed or aged data invalidates height instead of inventing a
 * sample.
 */
static void terrain_cycle(terrain_t * p_terrain, bool * pb_configured)
{
    uint32_t mask        = board_io_lock();
    bool     b_reset     = g_shared.b_reset_imu;
    g_shared.b_reset_imu = false;
    uint32_t motion_time = g_heartbeat[0];
    board_io_unlock(mask);

    if (b_reset)
    {
        terrain_init(p_terrain);
        *pb_configured = false;
    }

    if (!*pb_configured)
    {
        *pb_configured = board_io_imu_init();
    }

    int16_t accel_x = 0;
    int16_t accel_y = 0;
    int16_t accel_z = 0;
    bool    b_read =
        (*pb_configured) && (board_io_imu(&accel_x, &accel_y, &accel_z));
    bool     b_fresh  = b_read && ((board_io_time_ms() - motion_time) <= 20u);
    uint32_t snapshot = field_read(5u, 32u);
    /* The nine-bit speed has maximum magnitude 511. */
    int16_t speed = (int16_t)((snapshot >> 12u) & 511u);
    speed         = (0u != (snapshot & (1u << 21u))) ? -speed : speed;
    terrain_step(p_terrain, accel_x, accel_y, accel_z, speed, snapshot & 4095u,
                 b_fresh);
    uint16_t faults =
        (p_terrain->failures >= 3u) ? DATA_FORMATS_FAULT_IMU : 0u;
    faults |= p_terrain->b_impact ? DATA_FORMATS_FAULT_IMPACT : 0u;
    faults |= p_terrain->b_overflow ? DATA_FORMATS_FAULT_RANGE : 0u;
    fault_source(ROBOT_SOURCE_IMU, faults);
    field_write(85u, 3u, p_terrain->state);
    field_write(88u, 7u, p_terrain->pitch);
    field_write(95u, 8u, p_terrain->peak_mm);
    field_write(137u, 1u, p_terrain->b_on_hump);
    field_write(138u, 1u, p_terrain->b_valid);
    mask                 = board_io_lock();
    g_shared.b_imu_ready = (p_terrain->samples >= ROBOT_IMU_CAL_SAMPLES) &&
                           (p_terrain->failures < 3u);
    board_io_unlock(mask);

    if ((p_terrain->b_started) || (p_terrain->b_finished))
    {
        uint8_t event[2] = {0u};
        (void)data_formats_write(event, 0u, 1u, p_terrain->b_finished);
        (void)data_formats_write(event, 1u, 8u, p_terrain->peak_mm);
        (void)data_formats_write(event, 9u, 1u, p_terrain->b_valid);
        ER result = tk_snd_mbf(gh_imu_queue, event, 2, TMO_POL);
        mask      = board_io_lock();
        g_shared.b_queue_fault =
            (g_shared.b_queue_fault) || ((E_OK != result));
        board_io_unlock(mask);
    }

    *pb_configured = b_read;
    heartbeat(2u);

    if (0u != faults)
    {
        (void)tk_wup_tsk(gh_tasks[0]);
    }
}
#endif /* OBSTACLE_TEST || LINE_TEST */

#if ((!MOTOR_TEST) && (!ENCODER_TEST))
/** @brief Latest line data is combined with consumptive motion events. */
static void navigation_cycle(navigation_t * p_navigation)
{
    uint8_t barcode  = 0u;
    INT     received = tk_rcv_mbf(gh_barcode_queue, &barcode, TMO_POL);

    if (1 == received)
    {
        navigation_barcode(p_navigation, barcode);
    }

    uint8_t imu_event[2] = {0u};
    (void)tk_rcv_mbf(gh_imu_queue, imu_event, TMO_POL);
    uint32_t mask          = board_io_lock();
    bool     b_done        = g_shared.b_motion_done;
    g_shared.b_motion_done = false;
    bool b_latched         = g_shared.b_latched;
    board_io_unlock(mask);
    uint32_t motion_word = field_read(5u, 32u);
    mask                 = board_io_lock();
    bool b_line_valid    = g_shared.b_line_visible;
    bool b_line_lost     = g_shared.b_line_lost;
    board_io_unlock(mask);
    uint32_t obstacle_word  = field_read(103u, 9u);
    uint32_t obstacle_state = field_read(132u, 3u);
    bool     b_scan_ready   = (OBSTACLE_PLAN == obstacle_state);
    bool     b_scan_left    = (field_read(112u, 6u) < 30u);
    /* Packet fields are explicitly bounded to their destination widths. */
    navigation_step(p_navigation, (motion_word >> 22u) & 7u, b_done,
                    b_line_valid, b_line_lost, 0u != field_read(137u, 1u),
                    obstacle_word, obstacle_word >= 2u, b_scan_ready,
                    b_scan_left,
                    (LINE_SCAN == field_read(129u, 3u)) ||
                        (LINE_JUNCTION == field_read(129u, 3u)),
                    b_latched);

    if ((!b_latched) && (DATA_FORMATS_FAULT == p_navigation->state) &&
        (!p_navigation->b_fault))
    {
        navigation_init(p_navigation);
    }

    uint16_t faults = p_navigation->b_fault ? DATA_FORMATS_FAULT_TIMEOUT : 0u;
    faults |= p_navigation->b_estop ? DATA_FORMATS_FAULT_ESTOP : 0u;
    fault_source(ROBOT_SOURCE_NAV, faults);

    if (0u != faults)
    {
        (void)tk_wup_tsk(gh_tasks[0]);
    }
}

/** @brief Local switches must remain stable for three 10-ms samples. */
static void local_commands(navigation_t * p_navigation)
{
    uint8_t buttons = board_io_buttons();

    if (0u != (buttons & 2u))
    {
        (void)navigation_supervise(p_navigation, DATA_FORMATS_ESTOP, true);
    }
    uint32_t stable_count =
        (buttons == g_buttons.candidate) ? (g_buttons.stable_count + 1u) : 0u;
    stable_count           = (stable_count > 3u) ? 3u : stable_count;
    g_buttons.candidate    = buttons;
    g_buttons.stable_count = stable_count;

    if (stable_count >= 3u)
    {
        uint32_t pressed   = (uint32_t)buttons & ~(uint32_t)g_buttons.previous;
        uint32_t mask      = board_io_lock();
        bool     b_latched = g_shared.b_latched;
        board_io_unlock(mask);

        if (0u != (pressed & 2u))
        {
            (void)navigation_supervise(p_navigation, DATA_FORMATS_ESTOP,
                                       b_latched);
        }
        else if (0u != (pressed & 4u))
        {
            (void)navigation_supervise(p_navigation, DATA_FORMATS_RESET,
                                       b_latched);
        }
        else if (0u != (pressed & 1u))
        {
            (void)navigation_supervise(p_navigation, DATA_FORMATS_START,
                                       b_latched);
        }
        else
        {
            /* Held switches do not repeat commands. */
        }

        g_buttons.previous = buttons;
    }
}

/** @brief Network sequence and age validation happen before mission mutation.
 */
static void remote_commands(navigation_t * p_navigation)
{
    uint8_t message[ROBOT_COMMAND_MESSAGE_BYTES] = {0u};
    INT     received = tk_rcv_mbf(gh_command_queue, message, TMO_POL);

    if (ROBOT_COMMAND_MESSAGE_BYTES == received)
    {
        uint8_t command  = 0u;
        uint8_t sequence = 0u;
        bool    b_valid =
            data_formats_command_read(message[0], &command, &sequence);
        uint32_t arrival = data_formats_read(message, 8u, 32u);
        uint32_t mask    = board_io_lock();
        bool b_new = (!g_shared.b_sequence_valid) ||
                     (data_formats_sequence_new(sequence, g_shared.sequence));
        bool b_latched = g_shared.b_latched;
        bool b_armed   = g_shared.b_remote_armed;
        board_io_unlock(mask);
        b_valid = b_valid && ((DATA_FORMATS_START != command) || b_armed);
        b_valid = b_valid && b_new && ((board_io_time_ms() - arrival) <= 500u);
        bool b_accepted =
            b_valid &&
            (navigation_supervise(p_navigation, command, b_latched));
        mask = board_io_lock();

        if (b_valid)
        {
            g_shared.sequence         = sequence;
            g_shared.b_sequence_valid = true;
        }

        bool b_arm_command =
            (DATA_FORMATS_STOP == command) || (DATA_FORMATS_RESET == command);
        g_shared.b_remote_armed =
            (g_shared.b_remote_armed) || (b_accepted && b_arm_command);
        board_io_unlock(mask);
        /* ACK occupies the command's six bits and one acceptance bit. */
        uint8_t ack = (message[0] & 63u) | (b_accepted ? 64u : 0u);
        (void)tk_snd_mbf(gh_ack_queue, &ack, 1, TMO_POL);
    }
}

/** @brief Stop is acknowledged before a deferred replacement command is sent.
 */
static void dispatch_navigation(navigation_t * p_navigation)
{
    uint32_t mask           = board_io_lock();
    bool     b_pending_stop = g_shared.b_stop;
    board_io_unlock(mask);

    if (p_navigation->b_stop_requested)
    {
        mask            = board_io_lock();
        g_shared.b_stop = true;
        board_io_unlock(mask);
        p_navigation->b_stop_requested = false;
    }
    else if ((p_navigation->b_command_ready) && (!b_pending_stop))
    {
        uint8_t message[3] = {0u};
        (void)data_formats_write(message, 0u, 24u, p_navigation->command);
        ER result = tk_snd_mbf(gh_motion_queue, message, 3, TMO_POL);
        mask      = board_io_lock();
        g_shared.b_queue_fault =
            (g_shared.b_queue_fault) || ((E_OK != result));
        board_io_unlock(mask);
        p_navigation->b_command_ready = false;
    }
    else
    {
        /* Wait one cycle for Motion to acknowledge the stop. */
    }

    mask = board_io_lock();
    g_shared.b_scan_request =
        (g_shared.b_scan_request) || (p_navigation->b_scan_requested);
    p_navigation->b_scan_requested = false;

    if (p_navigation->b_reset_requested)
    {
        g_shared.b_reset                = true;
        g_shared.b_reset_motion         = true;
        g_shared.b_reset_imu            = true;
        g_shared.b_reset_scan           = true;
        g_shared.b_queue_fault          = false;
        g_shared.b_range_fault          = false;
        g_shared.b_scan_request         = false;
        g_shared.b_scan_request         = false;
        g_shared.reset_at_ms            = g_heartbeat[3];
        p_navigation->b_reset_requested = false;
        p_navigation->b_fault           = false;
        p_navigation->b_estop           = false;
        p_navigation->b_command_ready   = false;
        p_navigation->elapsed_ms        = 0u;
    }

    board_io_unlock(mask);
}
#endif /* MOTOR_TEST && ENCODER_TEST */

#if (OBSTACLE_TEST || LINE_TEST)
/** @brief Bench stand-in: a healthy IMU that never reports a hump. */
static void terrain_stub(void)
{
    fault_source(ROBOT_SOURCE_IMU, 0u);
    uint32_t mask        = board_io_lock();
    g_shared.b_imu_ready = true;
    board_io_unlock(mask);
    heartbeat(2u);
}
#endif /* OBSTACLE_TEST || LINE_TEST */

#if LINE_TEST
/** @brief Banner for the line bench test; readings work before START. */
static void line_report_init(void)
{
    tm_printf((UB *)"LINE TEST: motors stay off until START (GP20)\n");
    tm_printf((UB *)"LINE TEST: ir = left GP26, centre GP27, right GP28\n");
    tm_printf((UB *)"LINE TEST: black reads 2048 or more, white lower\n");
    tm_printf((UB *)"LINE TEST: state 0 cal 1 track 2 search 3 junction\n");
    tm_printf((UB *)"LINE TEST: state 4 scan 5 decode\n");
    tm_printf((UB *)"LINE TEST: nav 1 idle 2 follow 3 barcode 5 avoid\n");
    tm_printf((UB *)"LINE TEST: nav 6 search 7 fault; E-stop GP21\n");
}

/** @brief One line every 200 ms, or every 50 ms while the wheels move. */
static void line_report(void)
{
    static uint32_t g_last_ms = 0u;
    uint32_t        now       = board_io_time_ms();
    uint32_t        mode      = (field_read(5u, 32u) >> 22u) & 7u;
    uint32_t        period    = (MOTION_IDLE == mode) ? 200u : 50u;

    if ((now - g_last_ms) >= period)
    {
        g_last_ms          = now;
        uint32_t mask      = board_io_lock();
        bool     b_visible = g_shared.b_line_visible;
        bool     b_lost    = g_shared.b_line_lost;
        board_io_unlock(mask);
        uint32_t barcode = field_read(118u, 3u);
        int      code    = (0u != (barcode & 4u)) ? (int)(barcode & 3u) : -1;
        int speed_l = (int)data_formats_unsign(field_read(203u, 10u), 10u);
        int speed_r = (int)data_formats_unsign(field_read(213u, 10u), 10u);
        tm_printf(
            (UB *)"LINE: ir=%u,%u,%u err=%d state=%u nav=%u "
                  "see=%d lost=%d bc=%d f=0x%03x\n",
            (unsigned)field_read(49u, 12u), (unsigned)field_read(61u, 12u),
            (unsigned)field_read(73u, 12u), (int)field_read(121u, 8u) - 127,
            (unsigned)field_read(129u, 3u), (unsigned)field_read(2u, 3u),
            b_visible ? 1 : 0, b_lost ? 1 : 0, code,
            (unsigned)field_read(37u, 12u));

        if (MOTION_IDLE != mode)
        {
            tm_printf((UB *)"MOVE: mode=%u speed mm/s L=%d R=%d "
                            "enc L=%u R=%u\n",
                      (unsigned)mode, speed_l, speed_r,
                      (unsigned)field_read(139u, 32u),
                      (unsigned)field_read(171u, 32u));
        }
    }
}
#endif /* LINE_TEST */

#if OBSTACLE_TEST
/** @brief Bench stand-in: a perfectly centred line that is always visible.
 * Line error is offset binary, so 127 means zero steering.
 */
static void line_stub(void)
{
    field_write(121u, 8u, 127u);
    uint32_t mask           = board_io_lock();
    g_shared.b_line_ready   = true;
    g_shared.b_line_visible = true;
    g_shared.b_line_lost    = false;
    board_io_unlock(mask);
    heartbeat(1u);
}

/** @brief Console text is only printed when something changes. */
static void bench_init(bench_t * p_bench)
{
    p_bench->state     = 0u;
    p_bench->stage     = 0u;
    p_bench->look      = 0u;
    p_bench->faults    = 0u;
    p_bench->b_avoided = false;
    tm_printf((UB *)"OBSTACLE TEST: box ahead, press START (GP20)\n");
    tm_printf((UB *)"OBSTACLE TEST: E-stop is GP21, RESET is GP22\n");
    tm_printf((UB *)"OBSTACLE TEST: scan 0 idle 1 coarse 2 fine 3 plan\n");
    tm_printf((UB *)"OBSTACLE TEST: scan 4 bypass 5 recover 6 abort\n");
    tm_printf((UB *)"FAULT BITS: 001 stall 002 encoder 004 imu 008 noecho\n");
    tm_printf((UB *)"FAULT BITS: 020 impact 040 estop/close 080 watchdog\n");
    tm_printf((UB *)"FAULT BITS: 100 range 200 timeout 400 queue\n");
}

/** @brief Report progress and stop once the bypass has returned to FOLLOW. */
static void bench_step(navigation_t * p_navigation, bench_t * p_bench)
{
    static const char * const g_names[8] = {
        "?", "IDLE", "FOLLOW", "BARCODE", "HUMP", "AVOID", "SEARCH", "FAULT"};
    uint8_t state = (uint8_t)p_navigation->state;
    uint8_t stage = (uint8_t)p_navigation->stage;
    bool    b_changed =
        (state != p_bench->state) ||
        ((DATA_FORMATS_AVOID == state) && (stage != p_bench->stage));

    if (b_changed)
    {
        tm_printf((UB *)"OBSTACLE TEST: %s stage=%u range=%u cm scan=%u\n",
                  (UB *)g_names[state & 7u], (unsigned)stage,
                  (unsigned)field_read(103u, 9u),
                  (unsigned)field_read(132u, 3u));
    }

    if ((uint8_t)p_navigation->look != p_bench->look)
    {
        tm_printf((UB *)"OBSTACLE TEST: look=%u left=%u cm right=%u cm\n",
                  (unsigned)p_navigation->look,
                  (unsigned)p_navigation->left_cm,
                  (unsigned)p_navigation->right_cm);
        p_bench->look = (uint8_t)p_navigation->look;
    }

    uint16_t faults = (uint16_t)field_read(37u, 12u);

    if (faults != p_bench->faults)
    {
        tm_printf((UB *)"OBSTACLE TEST: faults=0x%03x range=%u cm\n",
                  (unsigned)faults, (unsigned)field_read(103u, 9u));
        p_bench->faults = faults;
    }

    p_bench->b_avoided = (p_bench->b_avoided) || (DATA_FORMATS_AVOID == state);

    if ((p_bench->b_avoided) && (DATA_FORMATS_FOLLOW == state))
    {
        (void)navigation_supervise(p_navigation, DATA_FORMATS_STOP, false);
        uint32_t mask = board_io_lock();
        /* Let the scanner rearm, so START can run the whole test again. */
        g_shared.b_reset_scan = true;
        board_io_unlock(mask);
        p_bench->b_avoided = false;
        tm_printf((UB *)"OBSTACLE TEST: bypass finished, stopped\n");
    }

    p_bench->state = state;
    p_bench->stage = stage;
}
#endif /* OBSTACLE_TEST */

#if !LINE_TEST
/** @brief The scan task is the only consumer of the echo flag. */
static void obstacle_cycle(obstacle_t * p_obstacle, uint8_t * p_angle)
{
    uint32_t mask         = board_io_lock();
    bool     b_request    = g_shared.b_scan_request;
    bool     b_reset      = g_shared.b_reset_scan;
    g_shared.b_reset_scan = false;
    board_io_unlock(mask);
    uint32_t motion_mode = (field_read(5u, 32u) >> 22u) & 7u;

    if (b_reset)
    {
        obstacle_init(p_obstacle);
    }

    if (b_request && (MOTION_IDLE == motion_mode))
    {
        (void)obstacle_scan(p_obstacle);
        mask                    = board_io_lock();
        g_shared.b_scan_request = false;
        board_io_unlock(mask);
    }

    uint32_t vehicle = field_read(2u, 3u);

    if ((DATA_FORMATS_AVOID == vehicle) && (MOTION_IDLE != motion_mode) &&
        (OBSTACLE_PLAN == p_obstacle->state))
    {
        p_obstacle->state = OBSTACLE_BYPASS;
    }

    if ((DATA_FORMATS_SEARCH == vehicle) &&
        (OBSTACLE_BYPASS == p_obstacle->state))
    {
        p_obstacle->state = OBSTACLE_RECOVER;
    }

    bool b_recovered = (OBSTACLE_BYPASS == p_obstacle->state) ||
                       (OBSTACLE_RECOVER == p_obstacle->state);

    if (b_recovered && (DATA_FORMATS_FOLLOW == vehicle))
    {
        obstacle_init(p_obstacle);
    }

    if (*p_angle != p_obstacle->angle)
    {
        /* Angle is always 0..60, so it fits its byte-sized driver argument. */
        *p_angle = (uint8_t)p_obstacle->angle;
        board_io_servo(*p_angle);
        (void)tk_dly_tsk(100u);
    }

    uint32_t measured_angle = p_obstacle->angle;
    uint16_t ranges[3]      = {0u};
    bool     b_valid        = true;
    uint32_t start          = board_io_time_ms();

    for (uint8_t sample = 0u; sample < ROBOT_RANGE_SAMPLES; ++sample)
    {
        b_valid = (range_sample(&ranges[sample])) && b_valid;

        if (sample < (ROBOT_RANGE_SAMPLES - 1u))
        {
            heartbeat(4u);
        }
    }

    uint16_t range = obstacle_median(ranges[0], ranges[1], ranges[2]);
    obstacle_sample(p_obstacle, range, b_valid);
#if OBSTACLE_TEST
    tm_printf((UB *)"RANGE: %u cm valid=%d angle=%u raw=%u,%u,%u\n",
              (unsigned)range, b_valid ? 1 : 0, (unsigned)measured_angle,
              (unsigned)ranges[0], (unsigned)ranges[1], (unsigned)ranges[2]);
#endif /* OBSTACLE_TEST */
    /* One scan cycle is bounded below 500 ms and fits uint16_t. */
    obstacle_tick(p_obstacle, (uint16_t)(board_io_time_ms() - start + 100u));
    uint32_t state     = p_obstacle->state;
    bool     b_forward = (30u == measured_angle);
    field_write(103u, 9u, (b_valid && b_forward) ? range : 0u);
    field_write(112u, 6u,
                (OBSTACLE_PLAN == state) ? p_obstacle->best_angle
                                         : p_obstacle->angle);
    field_write(132u, 3u, state);
    bool b_close =
        b_forward && b_valid && ((range * 10u) < ROBOT_CLEARANCE_MM);
    bool     b_abort = (OBSTACLE_ABORT == state);
    uint16_t faults  = b_close ? DATA_FORMATS_FAULT_ESTOP : 0u;
    faults |= b_abort ? DATA_FORMATS_FAULT_TIMEOUT : 0u;
    faults |= !b_valid ? DATA_FORMATS_FAULT_NO_ECHO : 0u;
    fault_source(ROBOT_SOURCE_SCAN, faults);
    heartbeat(4u);
}

/** @brief Edge timestamps are captured by the ISR; this task only waits. */
static bool range_sample(uint16_t * p_range)
{
    (void)tk_clr_flg(gh_echo_flag, 0u);
    uint32_t ping_at = board_io_time_ms();
    board_io_ping();
    UINT pattern = 0u;
    ER   result =
        tk_wai_flg(gh_echo_flag, 1u, TWF_ORW | TWF_BITCLR, &pattern, 25);
    bool     b_valid = board_io_range(p_range);
    uint32_t elapsed = board_io_time_ms() - ping_at;

    if (elapsed < ROBOT_RANGE_PERIOD_MS)
    {
        (void)tk_dly_tsk(ROBOT_RANGE_PERIOD_MS - elapsed);
    }

    return (E_OK == result) && b_valid;
}
#endif /* LINE_TEST */

/** @brief All producers' masks are merged under one interrupt exclusion. */
static uint16_t current_faults(void)
{
    uint32_t mask   = board_io_lock();
    uint32_t faults = 0u;

    for (uint8_t source = 0u; source < ROBOT_SOURCE_COUNT; ++source)
    {
        faults |= data_formats_read(g_sources, source * 12u, 12u);
    }

    faults |= g_shared.b_queue_fault ? DATA_FORMATS_FAULT_QUEUE : 0u;
    faults |= g_shared.b_range_fault ? DATA_FORMATS_FAULT_RANGE : 0u;
    board_io_unlock(mask);
    /* Six twelve-bit sources cannot set a bit above bit eleven. */
    return (uint16_t)faults;
}

/*** End of file. ***/

