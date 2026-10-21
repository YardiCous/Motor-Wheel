#include "hardware/sync.h"
#include "robot.h"
#include <string.h>

static robot_state_t shared;
static struct {
    unsigned reset_generation : 8;
} lifecycle;
uint16_t robot_now(void) {
    return (uint16_t)kernel_milliseconds();
}
void robot_state_init(void) {
    memset(&shared, 0, sizeof(shared));
    controller_init(&shared.control);
}
void robot_snapshot(robot_state_t *out) {
    uint32_t irq = save_and_disable_interrupts();
    *out = shared;
    restore_interrupts(irq);
}
void robot_raise_fault(unsigned mask) {
    uint32_t irq = save_and_disable_interrupts();
    bool changed = (mask & ~shared.control.faults) != 0;
    controller_fault(&shared.control, mask);
    restore_interrupts(irq);
    if (changed && evg_robot > 0)
        (void)tk_set_flg(evg_robot, FLG_SAFETY);
}
void robot_clear_cause(unsigned mask) {
    uint32_t irq = save_and_disable_interrupts();
    controller_clear_cause(&shared.control, mask);
    restore_interrupts(irq);
}
void robot_heartbeat(unsigned task) {
    if (task >= 6)
        return;
    uint16_t now = robot_now();
    uint32_t irq = save_and_disable_interrupts();
    shared.heartbeat[task] = now;
    restore_interrupts(irq);
}
void robot_line_update(const line_msg_t *line, unsigned l, unsigned c, unsigned r) {
    uint32_t mode, seen, at;
    bits_get(line->bytes, sizeof(line->bytes), 11, 3, &mode);
    bits_get(line->bytes, sizeof(line->bytes), 14, 1, &seen);
    bits_get(line->bytes, sizeof(line->bytes), 15, 16, &at);
    uint32_t irq = save_and_disable_interrupts();
    shared.line = *line;
    shared.line_valid = 1;
    shared.line_ready = mode != LINE_CALIBRATING;
    shared.adc_left = l;
    shared.adc_center = c;
    shared.adc_right = r;
    controller_line(&shared.control, (line_mode_t)mode, seen != 0, (uint16_t)at);
    restore_interrupts(irq);
}
void robot_motion_update(int left, int right, uint32_t distance, move_t mode) {
    motion_state_msg_t msg = {{0}};
    if (left < -250 || left > 250 || right < -250 || right > 250) {
        robot_raise_fault(F_ENCODER);
        return;
    }
    bits_put(msg.bytes, 8, 0, 9, (uint32_t)(left + 250));
    bits_put(msg.bytes, 8, 9, 9, (uint32_t)(right + 250));
    bits_put(msg.bytes, 8, 18, 24, distance & 0xffffffu);
    bits_put(msg.bytes, 8, 42, 3, mode);
    bits_put(msg.bytes, 8, 45, 16, robot_now());
    uint32_t irq = save_and_disable_interrupts();
    shared.motion = msg;
    shared.motion_valid = 1;
    restore_interrupts(irq);
}
void robot_encoder_update(uint16_t l, uint16_t r) {
    uint32_t irq = save_and_disable_interrupts();
    shared.encoder_left = l;
    shared.encoder_right = r;
    restore_interrupts(irq);
}
void robot_range_update(unsigned mm, bool valid, bool forward) {
    uint16_t now = robot_now();
    uint32_t irq = save_and_disable_interrupts();
    if (forward) {
        controller_range(&shared.control, mm, valid, now);
        if (valid)
            controller_clear_cause(&shared.control, F_NO_ECHO);
    }
    restore_interrupts(irq);
}
void robot_imu_ready(bool ready) {
    uint32_t irq = save_and_disable_interrupts();
    shared.imu_ready = ready;
    restore_interrupts(irq);
}
void robot_network_status(bool wifi, bool mqtt) {
    uint32_t irq = save_and_disable_interrupts();
    shared.wifi = wifi;
    shared.mqtt = mqtt;
    if (wifi && mqtt)
        controller_clear_cause(&shared.control, F_WIFI);
    else
        controller_fault(&shared.control, F_WIFI);
    restore_interrupts(irq);
}
void robot_set_peak(unsigned peak) {
    if (peak > 1023) {
        robot_raise_fault(F_IMU);
        return;
    }
    uint32_t irq = save_and_disable_interrupts();
    if (peak > shared.hump_peak)
        shared.hump_peak = peak;
    restore_interrupts(irq);
}
void robot_set_barcode(unsigned code) {
    if (code > 5)
        return;
    uint32_t irq = save_and_disable_interrupts();
    shared.barcode = code;
    restore_interrupts(irq);
}
void robot_scan_request(void) {
    uint32_t irq = save_and_disable_interrupts();
    shared.scan_requested = 1;
    shared.scan_busy = 1;
    shared.scan_side = 0;
    shared.control.range_exempt = 1;
    restore_interrupts(irq);
}
bool robot_scan_take_request(void) {
    uint32_t irq = save_and_disable_interrupts();
    bool value = shared.scan_requested;
    shared.scan_requested = 0;
    restore_interrupts(irq);
    return value;
}
void robot_scan_finish(unsigned side) {
    uint32_t irq = save_and_disable_interrupts();
    shared.scan_side = side & 3u;
    shared.scan_busy = 0;
    shared.control.range_exempt = 0;
    restore_interrupts(irq);
    (void)robot_send_event(EV_SCAN_DONE, side);
}
void robot_set_vehicle(vehicle_state_t state) {
    uint32_t irq = save_and_disable_interrupts();
    if (!shared.control.latched && shared.control.ready)
        shared.control.vehicle = state;
    restore_interrupts(irq);
}
void robot_set_recovery(bool active) {
    uint32_t irq = save_and_disable_interrupts();
    shared.recovery_active = active;
    restore_interrupts(irq);
}
void robot_local_reset(void) {
    uint32_t irq = save_and_disable_interrupts();
    shared.local_reset = 1;
    restore_interrupts(irq);
}
bool robot_take_local_reset(void) {
    uint32_t irq = save_and_disable_interrupts();
    bool value = shared.local_reset;
    shared.local_reset = 0;
    restore_interrupts(irq);
    return value;
}
bool robot_apply_command(command_t cmd) {
    uint16_t now = robot_now();
    uint32_t irq = save_and_disable_interrupts();
    bool fresh = shared.line_valid && shared.motion_valid && !shared.scan_busy &&
                 shared.imu_ready && shared.control.range_valid &&
                 elapsed_ms(now, shared.control.line_at) <= LINE_MAX_AGE_MS &&
                 elapsed_ms(now, shared.control.range_at) <= RANGE_MAX_AGE_MS;
    bool accepted = (cmd != CMD_START || fresh) && controller_command(&shared.control, cmd);
    if (accepted && (cmd == CMD_RESET || cmd == CMD_START)) {
        lifecycle.reset_generation++;
        shared.motion_done = 0;
    }
    restore_interrupts(irq);
    return accepted;
}
bool robot_command(command_t cmd, unsigned sequence) {
    command_msg_t message;
    if (!command_encode(&message, cmd, sequence))
        return false;
    if (cmd == CMD_ESTOP) {
        robot_raise_fault(F_ESTOP);
        return true;
    }
    if (cmd == CMD_STOP) {
        uint32_t irq = save_and_disable_interrupts();
        controller_command(&shared.control, CMD_STOP);
        restore_interrupts(irq);
        return true;
    }
    return tk_snd_mbf(q_cmd_in, &message, sizeof(message), TMO_POL) == E_OK;
}
void robot_safety_step(void) {
    uint16_t now = robot_now();
    uint32_t irq = save_and_disable_interrupts();
    bool healthy = shared.imu_ready && shared.line_ready && shared.control.range_valid;
    if (shared.control.vehicle == VEH_BOOT && healthy && ROBOT_CALIBRATION_CONFIRMED) {
        controller_self_test(&shared.control, true);
    }
    shared.control.ready = healthy && ROBOT_CALIBRATION_CONFIRMED;
    if (controller_may_drive(&shared.control)) {
        for (unsigned i = 0; i < 5; ++i) {
            if (elapsed_ms(now, shared.heartbeat[i]) > 50)
                controller_fault(&shared.control, F_WATCHDOG);
        }
    }
    controller_tick(&shared.control, now);
    restore_interrupts(irq);
}
bool robot_may_drive(void) {
    uint32_t irq = save_and_disable_interrupts();
    bool allowed = controller_may_drive(&shared.control) && !shared.scan_busy;
    restore_interrupts(irq);
    return allowed;
}
bool robot_send_motion(move_t kind, int argument, unsigned speed) {
    motion_msg_t message;
    if (!motion_encode(&message, kind, argument, speed / 2u))
        return false;
    if (tk_snd_mbf(q_motion_cmd, &message, sizeof(message), TMO_POL) == E_OK)
        return true;
    robot_raise_fault(F_OVERFLOW);
    return false;
}
bool robot_send_event(unsigned type, unsigned value) {
    event_msg_t message = {{0}};
    if (!bits_put(message.bytes, 4, 0, 2, type) || !bits_put(message.bytes, 4, 2, 10, value))
        return false;
    bits_put(message.bytes, 4, 12, 16, robot_now());
    if (tk_snd_mbf(q_event, &message, sizeof(message), TMO_POL) == E_OK)
        return true;
    robot_raise_fault(F_OVERFLOW);
    return false;
}
bool robot_decode_event(const event_msg_t *m, unsigned *type, unsigned *value, uint16_t *at) {
    uint32_t t, v, time;
    if ((m->bytes[3] & 0xf0u) != 0)
        return false;
    bits_get(m->bytes, 4, 0, 2, &t);
    bits_get(m->bytes, 4, 2, 10, &v);
    bits_get(m->bytes, 4, 12, 16, &time);
    *type = t;
    *value = v;
    *at = (uint16_t)time;
    return (t == EV_BARCODE && v >= 1 && v <= 5) || (t == EV_HUMP_START && v == 0) ||
           (t == EV_HUMP_END && v <= 1023) || (t == EV_SCAN_DONE && v <= 2);
}
void robot_motion_complete(void) {
    uint32_t irq = save_and_disable_interrupts();
    shared.motion_done = 1;
    restore_interrupts(irq);
}
bool robot_take_motion_complete(void) {
    uint32_t irq = save_and_disable_interrupts();
    bool done = shared.motion_done;
    shared.motion_done = 0;
    restore_interrupts(irq);
    return done;
}
void robot_flush_motion(void) {
    motion_msg_t message;
    while (tk_rcv_mbf(q_motion_cmd, &message, TMO_POL) > 0) {
    }
}
unsigned robot_reset_generation(void) {
    uint32_t irq = save_and_disable_interrupts();
    unsigned value = lifecycle.reset_generation;
    restore_interrupts(irq);
    return value;
}
void robot_reset_subsystems(void) {
    uint32_t irq = save_and_disable_interrupts();
    lifecycle.reset_generation++;
    shared.scan_busy = 0;
    shared.scan_requested = 0;
    shared.control.range_exempt = 0;
    shared.recovery_active = 0;
    shared.motion_done = 0;
    restore_interrupts(irq);
}
