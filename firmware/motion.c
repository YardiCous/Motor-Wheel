#include "robot.h"
#include <stdlib.h>
#include <string.h>

typedef struct {
    signed integral : 19; /* error*ms, bounded +/-250000 */
    signed previous : 10; /* +/-500 mm/s */
    uint16_t last_moved;
} pid_t;
static struct {
    pid_t pid[2];
    uint16_t previous_count[2], previous_at, command_at;
    signed progress_left : 24; /* micrometres, +/-8388607 */
    signed progress_right : 24;
    signed goal_left : 24;
    signed goal_right : 24;
    signed remainder_um : 11;
    unsigned distance_mm : 24; /* signed displacement modulo 2^24 */
    unsigned kind : 3;
    unsigned speed : 7; /* units of 2 mm/s */
    unsigned generation : 8;
} motion;
static int clamp(int value, int lo, int hi) {
    return value < lo ? lo : value > hi ? hi : value;
}
static int count_um(int count) {
    return (int)((int64_t)count * WHEEL_DIAMETER_UM * 3141593 /
                 (ENCODER_COUNTS_PER_REV * 1000000LL));
}
static bool reached(int progress, int goal) {
    return goal >= 0 ? progress >= goal : progress <= goal;
}
void motion_reset(void) {
    motion.kind = MOVE_STOP;
    motion.progress_left = 0;
    motion.progress_right = 0;
    motion.goal_left = 0;
    motion.goal_right = 0;
    memset(motion.pid, 0, sizeof(motion.pid));
    motion.pid[0].last_moved = robot_now();
    motion.pid[1].last_moved = robot_now();
    board_motor_write(0, 0);
}
void motion_init(void) {
    encoder_t l, r;
    board_encoder_snapshot(&l, &r);
    motion.previous_count[0] = l.count;
    motion.previous_count[1] = r.count;
    motion.previous_at = robot_now();
    motion_reset();
}
static int pid_output(unsigned wheel, int target, int measured, unsigned dt, bool moved) {
    pid_t *p = &motion.pid[wheel];
    int error = target - measured;
    int integral = clamp(p->integral + error * (int)dt, -250000, 250000);
    int output = (PID_FEEDFORWARD_Q8 * target + PID_KP_Q8 * error + PID_KI_Q8 * integral +
                  PID_KD_Q8 * (error - p->previous) / (int)dt) /
                 256;
    if (abs(output) < 1000 || (output > 0 && error < 0) || (output < 0 && error > 0))
        p->integral = integral;
    p->previous = error;
    if (moved || abs(target) < 30 || abs(output) < 150)
        p->last_moved = robot_now();
    else if (elapsed_ms(robot_now(), p->last_moved) >= STALL_TIMEOUT_MS)
        robot_raise_fault(F_STALL);
    if (!target) {
        p->integral = 0;
        p->previous = 0;
        return 0;
    }
    return clamp(output, -1000, 1000);
}
static void accept(const motion_msg_t *message) {
    move_t kind;
    int argument;
    unsigned speed;
    if (!motion_decode(message, &kind, &argument, &speed)) {
        robot_raise_fault(F_NAVIGATION);
        return;
    }
    motion_reset();
    motion.kind = kind;
    motion.speed = speed;
    motion.command_at = robot_now();
    if (kind == MOVE_DISTANCE) {
        motion.goal_left = argument * 1000;
        motion.goal_right = argument * 1000;
    } else if (kind == MOVE_TURN) {
        if (argument < -360 || argument > 360) {
            robot_raise_fault(F_NAVIGATION);
            return;
        }
        int path = (int)((int64_t)TRACK_WIDTH_UM * argument * 3141593 / 360000000LL);
        /* Positive angle rotates right: left forward, right backward. */
        motion.goal_left = path;
        motion.goal_right = -path;
    }
}
void motion_step(void) {
    encoder_t l, r;
    board_encoder_snapshot(&l, &r);
    uint16_t now = robot_now();
    unsigned dt = elapsed_ms(now, motion.previous_at);
    if (dt == 0)
        return;
    motion.previous_at = now;
    int dl = encoder_delta(l.count, motion.previous_count[0]) * ENCODER_LEFT_SIGN;
    int dr = encoder_delta(r.count, motion.previous_count[1]) * ENCODER_RIGHT_SIGN;
    motion.previous_count[0] = l.count;
    motion.previous_count[1] = r.count;
    robot_encoder_update(l.count, r.count);
    if (l.invalid || r.invalid || dt > 50 || abs(dl) > 128 || abs(dr) > 128)
        robot_raise_fault(F_ENCODER);
    int ul = count_um(dl), ur = count_um(dr);
    int vl = ul / (int)dt, vr = ur / (int)dt;
    int sum = (ul + ur) / 2 + motion.remainder_um;
    motion.distance_mm = (motion.distance_mm + sum / 1000) & 0xffffffu;
    motion.remainder_um = sum % 1000;
    bool finite = motion.kind == MOVE_DISTANCE || motion.kind == MOVE_TURN;
    int pl = finite ? motion.progress_left + ul : 0;
    int pr = finite ? motion.progress_right + ur : 0;
    if (pl < -8388608 || pl > 8388607 || pr < -8388608 || pr > 8388607)
        robot_raise_fault(F_ENCODER);
    else {
        motion.progress_left = pl;
        motion.progress_right = pr;
    }
    robot_motion_update(vl, vr, motion.distance_mm, (move_t)motion.kind);
    unsigned generation = robot_reset_generation();
    if (generation != motion.generation) {
        motion.generation = generation;
        motion_reset();
    }
    if (!robot_may_drive()) {
        motion_reset();
        robot_flush_motion();
        return;
    }
    motion_msg_t request;
    if (tk_rcv_mbf(q_motion_cmd, &request, TMO_POL) == sizeof(request))
        accept(&request);
    robot_state_t state;
    robot_snapshot(&state);
    if (!state.line_valid || elapsed_ms(now, state.control.line_at) > LINE_MAX_AGE_MS ||
        (!state.control.range_exempt &&
         elapsed_ms(now, state.control.range_at) > RANGE_MAX_AGE_MS)) {
        robot_raise_fault(F_WATCHDOG);
        motion_reset();
        return;
    }
    int tl = 0, tr = 0;
    switch ((move_t)motion.kind) {
    case MOVE_FOLLOW: {
        if (state.control.vehicle == VEH_OBSTACLE) {
            motion_reset();
            return;
        }
        uint32_t raw;
        bits_get(state.line.bytes, 4, 0, 11, &raw);
        int steering = ((int)raw - 1000) * LINE_STEER_GAIN_Q8 / 256;
        /* Barcode/junction patterns carry no useful lateral error. */
        if (state.control.line_mode == LINE_BARCODE_SCAN ||
            state.control.line_mode == LINE_JUNCTION)
            steering = 0;
        tl = clamp((int)(motion.speed * 2u) + steering, 0, MAX_SPEED_MM_S);
        tr = clamp((int)(motion.speed * 2u) - steering, 0, MAX_SPEED_MM_S);
        break;
    }
    case MOVE_SEARCH:
        tl = SEARCH_SPEED_MM_S;
        tr = -(int)SEARCH_SPEED_MM_S;
        break;
    case MOVE_DISTANCE:
    case MOVE_TURN: {
        if (elapsed_ms(now, motion.command_at) > MOTION_TIMEOUT_MS) {
            robot_raise_fault(F_NAVIGATION);
            break;
        }
        bool done_l = reached(motion.progress_left, motion.goal_left);
        bool done_r = reached(motion.progress_right, motion.goal_right);
        if (done_l && done_r) {
            motion_reset();
            robot_motion_complete();
            break;
        }
        tl = done_l ? 0
                    : (motion.goal_left < 0 ? -(int)(motion.speed * 2u) : (int)(motion.speed * 2u));
        tr = done_r
                 ? 0
                 : (motion.goal_right < 0 ? -(int)(motion.speed * 2u) : (int)(motion.speed * 2u));
        if (motion.kind == MOVE_DISTANCE && !done_l && !done_r) {
            int correction = clamp((motion.progress_left - motion.progress_right) / 2000, -25, 25);
            tl -= correction;
            tr += correction;
        }
        break;
    }
    default:
        break;
    }
    /* Forward protection during every moving phase; turning stays interruptible. */
    if (tl > 0 && tr > 0 && state.control.range_mm <= OBSTACLE_STOP_MM) {
        if (state.control.vehicle == VEH_FOLLOW || state.control.vehicle == VEH_HUMP)
            robot_set_vehicle(VEH_OBSTACLE);
        else
            robot_raise_fault(F_NAVIGATION);
        motion_reset();
        return;
    }
    int out_l = pid_output(0, clamp(tl, -250, 250), vl, dt, dl != 0);
    int out_r = pid_output(1, clamp(tr, -250, 250), vr, dt, dr != 0);
    if (!robot_may_drive()) {
        motion_reset();
        return;
    }
    board_motor_write(out_l, out_r);
}
