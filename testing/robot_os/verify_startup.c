/* Exercise the real OS event gating and saved diagnostics with RTOS mocks. */
#include "../../app_program/robot_os.c"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint32_t g_now;
static uint16_t g_range_cm;
static unsigned g_queued;
static unsigned g_watchdog_lines;
static unsigned g_queue_lines;
static const uint8_t * gp_events;
static size_t g_events_left;
static uint8_t g_telemetry_message[ROBOT_TELEMETRY_MESSAGE_BYTES];
static bool g_telemetry_pending;

uint32_t board_io_lock(void)
{
    return 0u;
}

void board_io_unlock(uint32_t mask)
{
    (void)mask;
}

uint32_t board_io_time_ms(void)
{
    return g_now;
}

bool board_io_encoders(uint32_t *left, uint32_t *right)
{
    *left = 0u;
    *right = 0u;
    return true;
}

ER tk_wup_tsk(ID task)
{
    (void)task;
    return E_OK;
}

void board_io_servo(uint8_t angle)
{
    assert(angle <= 60u);
}

void board_io_range_reset(void)
{
}

void board_io_ping(void)
{
}

bool board_io_range(uint16_t *range)
{
    *range = g_range_cm;
    return true;
}

ER tk_dly_tsk(UINT delay)
{
    g_now += delay;
    return E_OK;
}

ER tk_clr_flg(ID flag, UINT pattern)
{
    assert(flag == gh_echo_flag && pattern == 0u);
    return E_OK;
}

ER tk_wai_flg(ID flag, UINT pattern, UINT mode, UINT *result, INT timeout)
{
    assert(flag == gh_echo_flag && pattern == 1u);
    assert(mode == (TWF_ORW | TWF_BITCLR) && timeout > 0);
    *result = pattern;
    return E_OK;
}

ER tk_snd_mbf(ID queue, const void *message, INT length, INT timeout)
{
    assert(timeout == TMO_POL);
    if (queue == gh_telemetry_queue)
    {
        assert(length == (INT)sizeof(g_telemetry_message));
        if (g_telemetry_pending)
        {
            return E_TMOUT;
        }
        memcpy(g_telemetry_message, message, sizeof(g_telemetry_message));
        g_telemetry_pending = true;
        return E_OK;
    }
    assert(queue == gh_barcode_queue);
    assert(length == 1);
    assert(*(const uint8_t *)message == (uint8_t)JUNCTION_EVENT);
    ++g_queued;
    return g_queued <= 4u ? E_OK : E_TMOUT;
}

INT tk_rcv_mbf(ID queue, void *message, INT timeout)
{
    assert(timeout == TMO_POL);
    if (queue == gh_telemetry_queue && g_telemetry_pending)
    {
        memcpy(message, g_telemetry_message, sizeof(g_telemetry_message));
        g_telemetry_pending = false;
        return (INT)sizeof(g_telemetry_message);
    }
    if (queue == gh_barcode_queue && g_events_left > 0u)
    {
        *(uint8_t *)message = *gp_events++;
        --g_events_left;
        return 1;
    }
    return E_TMOUT;
}

INT tm_printf(const UB *format, ...)
{
    if (strncmp((const char *)format, "WATCHDOG:", 9u) == 0)
    {
        ++g_watchdog_lines;
    }
    if (strncmp((const char *)format, "QUEUE:", 6u) == 0)
    {
        ++g_queue_lines;
    }
    return 0;
}

static void verify_barcode_telemetry(void)
{
    gh_telemetry_queue = 2;
    uint8_t packet[DATA_FORMATS_TELEMETRY_BYTES] = {0u};
    navigation_t manager = {0};
    navigation_init(&manager);
    assert(navigation_supervise(&manager, DATA_FORMATS_START, false));
    telemetry_enqueue();
    assert(robot_os_telemetry(packet));
    assert(data_formats_read(packet, 116u, 3u) == 0u);
    assert(data_formats_read(packet, 224u, 5u) == 0u);

    const uint8_t events[] = {'Z', JUNCTION_EVENT, '1'};
    gp_events = events;
    g_events_left = 1u;
    navigation_events(&manager);
    assert(manager.junction.pending == 'Z');
    telemetry_enqueue();

    g_events_left = sizeof(events) - 1u;
    navigation_events(&manager);
    assert(manager.junction.pending == JUNCTION_NONE);
    assert(manager.state == DATA_FORMATS_BARCODE && !manager.b_left);
    g_now += 501u;
    telemetry_enqueue();
    assert(robot_os_telemetry(packet));
    assert(data_formats_read(packet, 116u, 3u) == (4u | DATA_FORMATS_RIGHT));
    assert(data_formats_read(packet, 224u, 5u) == 26u);
    (void)printf("BARCODE_VECTOR ");
    for (size_t index = 0u; index < sizeof(packet); ++index)
    {
        (void)printf("%02x", packet[index]);
    }
    (void)puts("");

    telemetry_enqueue();
    g_now += 501u;
    assert(!robot_os_telemetry(packet));

    const uint8_t next[] = {'F'};
    gp_events = next;
    g_events_left = sizeof(next);
    navigation_events(&manager);
    telemetry_enqueue();
    assert(robot_os_telemetry(packet));
    assert(data_formats_read(packet, 116u, 3u) == (4u | DATA_FORMATS_UTURN));
    assert(data_formats_read(packet, 224u, 5u) == 6u);
    puts("PASS: telemetry retains the scanned letter and action after a junction");
}

static void verify_obstacle_width_telemetry(void)
{
    navigation_t manager = {0};
    obstacle_t scanner = {0};
    uint8_t angle = 30u;
    uint8_t packet[DATA_FORMATS_TELEMETRY_BYTES] = {0u};
    navigation_init(&manager);
    assert(navigation_supervise(&manager, DATA_FORMATS_START, false));
    obstacle_init(&scanner);
    g_range_cm = 25u;
    obstacle_cycle(&scanner, &angle);
    navigation_cycle(&manager);
    assert(manager.state == DATA_FORMATS_AVOID);
    telemetry_enqueue();
    assert(robot_os_telemetry(packet));
    assert(data_formats_read(packet, 101u, 9u) == 25u);
    assert(data_formats_read(packet, 232u, 12u) == 0u);

    assert(obstacle_scan(&scanner));
    for (uint8_t index = 0u; index < OBSTACLE_SCAN_POINTS; ++index)
    {
        g_range_cm = ((index >= 6u) && (index <= 14u)) ? 25u : 400u;
        obstacle_cycle(&scanner, &angle);
    }
    navigation_cycle(&manager);
    telemetry_enqueue();
    assert(robot_os_telemetry(packet));
    assert(data_formats_read(packet, 130u, 3u) == OBSTACLE_PLAN);
    assert(data_formats_read(packet, 110u, 6u) == scanner.best_angle);
    assert(data_formats_read(packet, 232u, 12u) == 134u);
    (void)printf("WIDTH_VECTOR ");
    for (size_t index = 0u; index < sizeof(packet); ++index)
    {
        (void)printf("%02x", packet[index]);
    }
    (void)puts("");

    manager.state = DATA_FORMATS_FOLLOW;
    obstacle_init(&scanner);
    g_range_cm = 400u;
    obstacle_cycle(&scanner, &angle);
    navigation_cycle(&manager);
    assert(field_read(232u, 12u) == 134u);

    g_range_cm = 25u;
    obstacle_cycle(&scanner, &angle);
    navigation_cycle(&manager);
    assert(field_read(232u, 12u) == 0u);
    assert(obstacle_scan(&scanner));
    for (uint8_t index = 0u; index < OBSTACLE_SCAN_POINTS; ++index)
    {
        g_range_cm = index < 6u ? 400u : 25u;
        obstacle_cycle(&scanner, &angle);
    }
    navigation_cycle(&manager);
    assert(scanner.width_mm == 0u);
    assert(manager.stage == 1u);
    assert(!manager.b_fault);
    assert(field_read(130u, 3u) == OBSTACLE_PLAN);

    field_write(0u, 3u, DATA_FORMATS_AVOID);
    field_write(25u, 3u, MOTION_TURN);
    g_range_cm = 400u;
    obstacle_cycle(&scanner, &angle);
    telemetry_enqueue();
    assert(robot_os_telemetry(packet));
    assert(data_formats_read(packet, 130u, 3u) == OBSTACLE_BYPASS);
    assert(data_formats_read(packet, 232u, 12u) == 0u);
    puts("PASS: coarse scan publishes complete widths and plans both cases");
}

int main(void)
{
    line_t detector = {0};
    scanner_t reader = {0};
    bool junction = false;
    detector.state = LINE_JUNCTION;
    scanner_init(&reader);
    gh_barcode_queue = 1;
    const uint8_t inactive[] = {DATA_FORMATS_BOOT, DATA_FORMATS_IDLE,
                               DATA_FORMATS_FAULT};
    for (unsigned state = 0u; state < sizeof(inactive); ++state)
    {
        field_write(0u, 3u, inactive[state]);
        for (unsigned sample = 0u; sample < 10000u; ++sample)
        {
            detector.state = (sample & 1u) ? LINE_JUNCTION : LINE_TRACKING;
            line_events(&detector, &reader, 4095u, false, &junction);
        }
    }
    assert(g_queued == 0u);
    assert(g_shared.queue_faults == 0u);

    field_write(0u, 3u, DATA_FORMATS_FOLLOW);
    detector.state = LINE_JUNCTION;
    line_events(&detector, &reader, 4095u, false, &junction);
    line_events(&detector, &reader, 4095u, false, &junction);
    assert(g_queued == 1u);
    detector.state = LINE_TRACKING;
    line_events(&detector, &reader, 0u, false, &junction);
    detector.state = LINE_JUNCTION;
    line_events(&detector, &reader, 4095u, false, &junction);
    assert(g_queued == 2u);
    for (unsigned sample = 0u; sample < 3u; ++sample)
    {
        line_event((uint8_t)JUNCTION_EVENT);
    }
    assert(g_shared.queue_faults == 2u);
    assert((current_faults() & DATA_FORMATS_FAULT_QUEUE) != 0u);

    uint32_t ages[ROBOT_HEARTBEATS] = {5u, 5u, 5u, 45u, 60u};
    g_shared.b_line_ready = true;
    g_shared.b_imu_ready = true;
    g_now = 2500u;
    watchdog_capture(g_now, ages);
    ages[3] = 0u;
    watchdog_capture(g_now + 1u, ages);
    assert(g_watchdog.ages[3] == 45u);
    assert(g_watchdog.at_ms == 2500u);
    assert(g_watchdog.ready == 3u);
    watchdog_report();
    assert(g_watchdog_lines == 1u && g_queue_lines == 1u);
    g_now += 250u;
    watchdog_report();
    assert(g_watchdog_lines == 1u && g_queue_lines == 1u);
    g_now += 750u;
    watchdog_report();
    assert(g_watchdog_lines == 2u && g_queue_lines == 2u);
    g_watchdog.b_captured = false;
    g_shared.queue_faults = 0u;
    g_now += 1000u;
    watchdog_report();
    assert(g_watchdog_lines == 2u && g_queue_lines == 2u);
    puts("PASS: real OS idle-event gating, queue source and saved diagnostics");
    verify_barcode_telemetry();
    verify_obstacle_width_telemetry();
    return 0;
}
