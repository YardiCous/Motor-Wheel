/* Exercise the production GPIO driver with a simulated timer and echo pin. */
#include "board_io_mock.h"
#include "tk/tkernel.h"
#include "../../app_program/board_io.h"
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define SIO 0xd0000000u
#define IO 0x40014000u
#define TIMER 0x40054000u
static uint32_t now_us;
static uint32_t gpio_inputs;
static uint32_t pending_edges;
static uint32_t enabled_edges;
static uint32_t trigger_at;
static uint32_t trigger_width;
static uint32_t pulses;
static uint32_t signals;
static uint32_t locked;
static bool trigger_high;
static void (*gpio_handler)(uint32_t);

uint32_t board_io_mock_lock(void)
{
    uint32_t previous = locked;
    locked = 1u;
    return previous;
}

void board_io_mock_unlock(uint32_t mask)
{
    locked = mask;
}

uint32_t board_io_mock_read(uint32_t address)
{
    if (address == TIMER + 0x28u) return now_us++;
    if (address == SIO + 4u) return gpio_inputs;
    if (address == IO + 0x120u) return pending_edges & enabled_edges;
    if (address == IO + 0x100u) return enabled_edges;
    if (address == IO + 0xf0u) return pending_edges;
    if (address == 0x4000c008u) return 0x4011u;
    return 0u;
}

void board_io_mock_write(uint32_t address, uint32_t value)
{
    if (address == IO + 0x100u) enabled_edges = value;
    if (address == IO + 0x2100u) enabled_edges |= value;
    if (address == IO + 0xf0u) pending_edges &= ~value;
    if ((address == SIO + 0x14u) && ((value & 1u) != 0u))
    {
        assert(locked == 1u);
        trigger_high = true;
        trigger_at = now_us;
    }
    if ((address == SIO + 0x18u) && ((value & 1u) != 0u) && trigger_high)
    {
        trigger_high = false;
        trigger_width = now_us - trigger_at;
        ++pulses;
    }
}

int tk_def_int(uint32_t number, const T_DINT *config)
{
    /* The driver must not need a timer ISR to complete the trigger. */
    assert(number == 13u);
    gpio_handler = config->inthdr;
    return E_OK;
}

void EnableInt(uint32_t number, int level)
{
    assert(number == 13u);
    (void)level;
}

int tk_get_tim(SYSTIM *time)
{
    time->lo = now_us / 1000u;
    return E_OK;
}

int tk_set_flg(int32_t flag, uint32_t pattern)
{
    assert(flag == 7);
    assert(pattern == 1u);
    ++signals;
    return E_OK;
}

static void echo_edge(bool high)
{
    assert(locked == 0u);
    gpio_inputs = (gpio_inputs & ~2u) | (high ? 2u : 0u);
    pending_edges |= high ? 0x80u : 0x40u;
    gpio_handler(13u);
}

static void measure(uint32_t width_us, uint16_t expected_cm)
{
    uint16_t cm = 0u;
    uint32_t before = signals;
    board_io_ping();
    assert(!trigger_high);
    assert(trigger_width >= 12u && trigger_width <= 14u);
    now_us += 100u;
    echo_edge(true);
    now_us += width_us - 1u;
    echo_edge(false);
    assert(signals == before + 1u);
    board_io_range_diag_t diagnostic = {0};
    board_io_range_diagnostics(&diagnostic);
    assert(diagnostic.state == 3u);
    assert(diagnostic.width_us == width_us);
    assert(diagnostic.rises > 0u && diagnostic.falls > 0u);
    assert(diagnostic.irq_enabled == 0xc0u);
    assert(board_io_range(&cm));
    assert(cm == expected_cm);
    assert(!board_io_range(&cm)); /* Completed pulses are consumed once. */
}

int main(void)
{
    assert(board_io_init(7));
    measure(1160u, 20u);
    measure(1740u, 30u);
    /* The reported 74-us pulse must never become an empty/clear batch. */
    measure(74u, 1u);
    measure(1u, 1u);
    measure(87u, 1u);
    measure(116u, 2u);
    /* A missing echo cannot replay the previous measurement. */
    board_io_ping();
    uint16_t cm = 99u;
    board_io_range_diag_t diagnostic = {0};
    board_io_range_diagnostics(&diagnostic);
    assert(diagnostic.state == 1u);
    assert(diagnostic.width_us == 0u);
    assert(!board_io_range(&cm));
    assert(cm == 0u);
    /* Reset during an incomplete echo must allow the next ping to work. */
    board_io_ping();
    echo_edge(true);
    board_io_range_diagnostics(&diagnostic);
    assert(diagnostic.state == 2u);
    assert((diagnostic.pins & 2u) != 0u);
    pending_edges = 0xc0u;
    board_io_range_reset();
    assert(pending_edges == 0u);
    assert(!board_io_range(&cm));
    measure(1450u, 25u);
    /* Reset must not remove encoder interrupts or change encoder counts. */
    uint32_t left = 0u;
    uint32_t right = 0u;
    gpio_inputs = 1u << 4u;
    pending_edges = 0x00080000u;
    gpio_handler(13u);
    assert(board_io_encoders(&left, &right));
    uint32_t saved_left = left;
    uint32_t saved_right = right;
    assert(saved_left != 0u);
    uint32_t saved_edges = enabled_edges;
    board_io_range_reset();
    assert(board_io_encoders(&left, &right));
    assert(left == saved_left && right == saved_right);
    assert(enabled_edges == saved_edges);
    /* Timer wrap cannot leave the trigger high or corrupt echo widths. */
    now_us = UINT32_MAX - 5u;
    measure(1160u, 20u);
    now_us = UINT32_MAX - 300u;
    measure(1160u, 20u);
    assert(pulses == 11u);
    puts("PASS: real ranging driver trigger, echoes, timeout, reset, timer wrap");
    return 0;
}
