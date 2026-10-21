#include "controller.h"
#include "encoder.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static void ready(controller_t *c, uint16_t t) {
    controller_init(c);
    controller_self_test(c, true);
    controller_line(c, LINE_TRACKING, true, t);
    controller_range(c, 1000, true, t);
    assert(controller_command(c, CMD_START));
}
static void packing(void) {
    command_msg_t cmd;
    command_t c;
    uint8_t s;
    for (unsigned op = 0; op < 4; ++op)
        for (unsigned seq = 0; seq < 64; ++seq) {
            assert(command_encode(&cmd, (command_t)op, seq));
            assert(command_decode(cmd.bytes, 1, &c, &s));
            assert((unsigned)c == op && s == seq);
        }
    assert(!command_encode(&cmd, CMD_START, 64));
    assert(!command_decode(cmd.bytes, 0, &c, &s));
    motion_msg_t m;
    move_t k;
    int a;
    unsigned speed;
    for (int v = -8192; v <= 8191; ++v) {
        assert(motion_encode(&m, MOVE_DISTANCE, v, 125));
        assert(motion_decode(&m, &k, &a, &speed));
        assert(k == MOVE_DISTANCE && a == v && speed == 125);
    }
    assert(!motion_encode(&m, MOVE_TURN, 8192, 0));
    assert(!motion_encode(&m, MOVE_TURN, 0, 126));
    telemetry_msg_t t = {{0}};
    int32_t value;
    assert(telemetry_set(&t, T_LEFT_SPEED, -250));
    assert(telemetry_set(&t, T_RIGHT_SPEED, 250));
    assert(telemetry_set(&t, T_DISTANCE, 16777215));
    assert(telemetry_set(&t, T_SEQUENCE, 63));
    assert(telemetry_set(&t, T_VEHICLE, 8));
    assert(!telemetry_set(&t, T_VEHICLE, 9));
    assert(telemetry_get(&t, T_LEFT_SPEED, &value) && value == -250);
    assert(telemetry_get(&t, T_RIGHT_SPEED, &value) && value == 250);
    assert(telemetry_get(&t, T_DISTANCE, &value) && value == 16777215);
    assert((t.bytes[19] & 0xc0u) == 0);
    uint8_t b[5] = {0};
    uint32_t raw;
    assert(bits_put(b, 5, 3, 32, UINT32_MAX));
    assert(bits_get(b, 5, 3, 32, &raw) && raw == UINT32_MAX);
    assert(!bits_put(b, 5, 39, 2, 0));
    assert(!bits_put(b, 5, 0, 3, 8));
}
static void safety(void) {
    controller_t c;
    ready(&c, 65530);
    controller_tick(&c, 4);
    assert(controller_may_drive(&c));
    controller_fault(&c, F_WIFI);
    assert(controller_may_drive(&c));
    controller_fault(&c, F_IMPACT);
    assert(!controller_may_drive(&c));
    assert(!controller_command(&c, CMD_RESET));
    controller_clear_cause(&c, F_IMPACT);
    assert(!controller_may_drive(&c));
    assert(controller_command(&c, CMD_RESET));
    assert(c.vehicle == VEH_IDLE);
    assert(controller_command(&c, CMD_START));
    assert(controller_command(&c, CMD_STOP));
    assert(!controller_may_drive(&c));
    ready(&c, 0);
    controller_tick(&c, 21);
    assert(c.latched && (c.faults & F_WATCHDOG));
    ready(&c, 0);
    controller_range(&c, 100, false, 0);
    assert(c.latched);
    ready(&c, 0);
    controller_range(&c, 250, true, 0);
    controller_tick(&c, 0);
    assert(c.vehicle == VEH_OBSTACLE);
    /* A stopped servo scan can exempt only ranging, never the line watchdog. */
    ready(&c, 0);
    c.vehicle = VEH_OBSTACLE;
    c.range_exempt = 1;
    controller_line(&c, LINE_TRACKING, true, 200);
    controller_tick(&c, 200);
    assert(!c.latched);
    c.range_exempt = 0;
    controller_tick(&c, 200);
    assert(c.latched && (c.faults & F_NO_ECHO));
}
static void line_loss(void) {
    controller_t c;
    ready(&c, 0);
    for (unsigned t = 0; t <= 160; t += 5) {
        controller_line(&c, LINE_TRACKING, false, (uint16_t)t);
        controller_range(&c, 1000, true, (uint16_t)t);
        controller_tick(&c, (uint16_t)t);
    }
    assert(c.vehicle == VEH_SEARCH);
    controller_line(&c, LINE_TRACKING, true, 165);
    controller_tick(&c, 165);
    assert(c.vehicle == VEH_FOLLOW);
    ready(&c, 0);
    for (unsigned t = 0; t <= 200; t += 5) {
        controller_line(&c, LINE_BARCODE_SCAN, false, (uint16_t)t);
        controller_range(&c, 1000, true, (uint16_t)t);
        controller_tick(&c, (uint16_t)t);
    }
    assert(c.vehicle == VEH_FOLLOW);
    controller_line(&c, LINE_BARCODE_SCAN, false, 5005);
    controller_range(&c, 1000, true, 5005);
    controller_tick(&c, 5005);
    assert(c.latched);
}
static void quadrature(void) {
    encoder_t e;
    encoder_init(&e, 0);
    unsigned f[] = {1, 3, 2, 0}, r[] = {2, 3, 1, 0};
    for (unsigned i = 0; i < 4; ++i)
        encoder_edge(&e, f[i]);
    assert(e.count == 4 && !e.invalid);
    for (unsigned i = 0; i < 4; ++i)
        encoder_edge(&e, r[i]);
    assert(e.count == 0);
    encoder_edge(&e, 3);
    assert(e.invalid);
    assert(encoder_delta(2, 65534) == 4);
    assert(encoder_delta(65534, 2) == -4);
}
void robot_core_tests(void) {
    packing();
    safety();
    line_loss();
    quadrature();
    puts("All robot core tests passed.");
}
