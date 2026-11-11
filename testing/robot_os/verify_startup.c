/* Exercise the real OS event gating and saved diagnostics with RTOS mocks. */
#include "../../app_program/robot_os.c"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint32_t g_now;
static unsigned g_queued;
static unsigned g_watchdog_lines;
static unsigned g_queue_lines;

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

ER tk_snd_mbf(ID queue, const void *message, INT length, INT timeout)
{
    (void)queue;
    (void)timeout;
    assert(length == 1);
    assert(*(const uint8_t *)message == (uint8_t)JUNCTION_EVENT);
    ++g_queued;
    return g_queued <= 4u ? E_OK : E_TMOUT;
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
    return 0;
}
