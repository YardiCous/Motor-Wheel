#include "robot.h"
#include <string.h>

static struct {
    struct {
        unsigned mm : 12;
    } coarse[5];
    unsigned phase : 2; /* forward, coarse, fine, return-forward */
    unsigned index : 3;
    unsigned retries : 2;
    unsigned side : 2;
    unsigned waiting : 1;
    unsigned generation : 8;
    uint16_t moved_at, ping_at, started_at;
} scan;
static const unsigned coarse_angles[] = {30, 60, 90, 120, 150};
void obstacle_reset(void) {
    memset(&scan, 0, sizeof(scan));
    scan.generation = robot_reset_generation();
    scan.moved_at = robot_now();
    scan.ping_at = robot_now();
    board_echo_cancel();
    board_servo_angle(90);
}
void obstacle_init(void) {
    obstacle_reset();
}
static void abort_scan(void) {
    robot_raise_fault(F_NAVIGATION);
    board_echo_cancel();
    board_servo_angle(90);
    scan.phase = 3;
    scan.waiting = 0;
    scan.side = 0;
    scan.moved_at = robot_now();
    scan.retries = 0;
}
static void advance(unsigned mm) {
    uint16_t now = robot_now();
    if (scan.phase == 1) {
        scan.coarse[scan.index].mm = mm;
        if (++scan.index < 5)
            board_servo_angle(coarse_angles[scan.index]);
        else {
            unsigned left =
                scan.coarse[3].mm < scan.coarse[4].mm ? scan.coarse[3].mm : scan.coarse[4].mm;
            unsigned right =
                scan.coarse[0].mm < scan.coarse[1].mm ? scan.coarse[0].mm : scan.coarse[1].mm;
            if (left < OBSTACLE_CLEARANCE_MM && right < OBSTACLE_CLEARANCE_MM) {
                abort_scan();
                return;
            }
            scan.side = left >= right ? 1 : 2;
            scan.phase = 2;
            scan.index = 0;
            board_servo_angle(scan.side == 1 ? 105 : 75);
        }
    } else if (scan.phase == 2) {
        if (mm < OBSTACLE_CLEARANCE_MM) {
            abort_scan();
            return;
        }
        if (++scan.index < 2)
            board_servo_angle(scan.side == 1 ? 135 : 45);
        else {
            scan.phase = 3;
            board_servo_angle(90);
        }
    } else if (scan.phase == 3) {
        robot_range_update(mm, true, true);
        robot_scan_finish(scan.side);
        scan.phase = 0;
    }
    scan.moved_at = now;
}
void obstacle_step(void) {
    uint16_t now = robot_now();
    if (scan.generation != robot_reset_generation())
        obstacle_reset();
    if (robot_scan_take_request()) {
        scan.phase = 1;
        scan.index = 0;
        scan.retries = 0;
        scan.side = 0;
        scan.waiting = 0;
        scan.started_at = now;
        scan.moved_at = now;
        board_echo_cancel();
        board_servo_angle(coarse_angles[0]);
    }
    if (scan.phase != 0 && elapsed_ms(now, scan.started_at) > SCAN_TIMEOUT_MS) {
        abort_scan();
        scan.started_at = now;
    }
    if (scan.waiting) {
        unsigned width;
        if (board_echo_take(&width)) {
            scan.waiting = 0;
            unsigned mm = width * 343u / 2000u;
            bool valid = width >= 116u && mm <= 4095u;
            if (!valid) {
                if (scan.phase == 0)
                    robot_range_update(0, false, true);
                else if (scan.retries++ >= SCAN_RETRIES)
                    abort_scan();
                return;
            }
            scan.retries = 0;
            if (scan.phase == 0)
                robot_range_update(mm, true, true);
            else
                advance(mm);
        } else if (elapsed_ms(now, scan.ping_at) >= (ECHO_TIMEOUT_US / 1000u + 1u)) {
            scan.waiting = 0;
            board_echo_cancel();
            if (scan.phase == 0)
                robot_range_update(0, false, true);
            else if (scan.retries++ >= SCAN_RETRIES)
                abort_scan();
        }
        return;
    }
    if (elapsed_ms(now, scan.ping_at) < RANGE_PERIOD_MS ||
        elapsed_ms(now, scan.moved_at) < SERVO_SETTLE_MS)
        return;
    scan.ping_at = now;
    scan.waiting = 1;
    board_range_trigger();
}
