#include "robot.h"
#include <stdlib.h>
#include <string.h>

static struct {
    unsigned previous : 4;
    unsigned bypass_step : 3;
    unsigned side : 2;
    unsigned retry : 2;
    unsigned waiting : 1;
    unsigned generation : 8;
    uint16_t entered_at;
} navigation;
void navigation_reset(void) {
    memset(&navigation, 0, sizeof(navigation));
    navigation.previous = VEH_BOOT;
    navigation.generation = robot_reset_generation();
    navigation.entered_at = robot_now();
    robot_flush_motion();
    event_msg_t discarded;
    while (tk_rcv_mbf(q_event, &discarded, TMO_POL) > 0) {
    }
    (void)robot_take_motion_complete();
}
static void follow(void) {
    robot_flush_motion();
    robot_set_recovery(false);
    robot_set_vehicle(VEH_FOLLOW);
    (void)robot_send_motion(MOVE_FOLLOW, 0, MAX_SPEED_MM_S);
}
static void bypass_next(void) {
    int direction = navigation.side == 1 ? -1 : 1;
    navigation.waiting = 1;
    /* Offset, pass the obstacle, turn toward the original line, recover it.
     * Every leg is encoder-completed and remains protected by forward range. */
    switch (navigation.bypass_step) {
    case 0:
        (void)robot_send_motion(MOVE_TURN, 90 * direction, TURN_SPEED_MM_S);
        break;
    case 1:
        (void)robot_send_motion(MOVE_DISTANCE, BYPASS_SIDE_MM, BYPASS_SPEED_MM_S);
        break;
    case 2:
        (void)robot_send_motion(MOVE_TURN, -90 * direction, TURN_SPEED_MM_S);
        break;
    case 3:
        (void)robot_send_motion(MOVE_DISTANCE, BYPASS_FORWARD_MM, BYPASS_SPEED_MM_S);
        break;
    case 4:
        (void)robot_send_motion(MOVE_TURN, -90 * direction, TURN_SPEED_MM_S);
        break;
    case 5:
        robot_set_recovery(true);
        navigation.entered_at = robot_now();
        (void)robot_send_motion(MOVE_DISTANCE, BYPASS_SIDE_MM, 120);
        break;
    default:
        navigation.waiting = 0;
        robot_raise_fault(F_NAVIGATION);
        break;
    }
}
static void local_reset(const robot_state_t *state) {
    if (board_estop_active() || controller_may_drive(&state->control))
        return;
    uint32_t left, right;
    bits_get(state->motion.bytes, 8, 0, 9, &left);
    bits_get(state->motion.bytes, 8, 9, 9, &right);
    if (!state->motion_valid || abs((int)left - 250) > 10 || abs((int)right - 250) > 10 ||
        !state->imu_ready || !state->line_ready || !state->control.range_valid ||
        elapsed_ms(robot_now(), state->control.range_at) > RANGE_MAX_AGE_MS)
        return;
    /* Physical RESET acknowledges operator-inspected historical faults while stopped.
     * Continuous sensor causes must have cleared through their owners first. */
    board_clear_encoder_fault();
    robot_clear_cause(F_STALL | F_ENCODER | F_IMPACT | F_NAVIGATION | F_OVERFLOW | F_ESTOP |
                      F_WATCHDOG);
    if (robot_apply_command(CMD_RESET)) {
        robot_reset_subsystems();
        navigation_reset();
    }
}
static void command_step(void) {
    command_msg_t message;
    command_t command;
    uint8_t sequence;
    if (tk_rcv_mbf(q_cmd_in, &message, TMO_POL) != sizeof(message))
        return;
    bool accepted = command_decode(message.bytes, sizeof(message), &command, &sequence) &&
                    robot_apply_command(command);
    if (accepted && command == CMD_RESET) {
        robot_reset_subsystems();
        navigation_reset();
    }
    if (accepted && command == CMD_START) {
        navigation_reset();
        follow();
        navigation.previous = VEH_FOLLOW;
    }
    uint8_t ack = (uint8_t)((sequence & 63u) | (accepted ? 64u : 0u));
    /* ACK overflow is a communications loss, not a motor-control block. */
    (void)tk_snd_mbf(q_ack, &ack, 1, TMO_POL);
}
static void event_step(const robot_state_t *state) {
    event_msg_t message;
    if (tk_rcv_mbf(q_event, &message, TMO_POL) != sizeof(message))
        return;
    unsigned type, value;
    uint16_t at;
    if (!robot_decode_event(&message, &type, &value, &at) || elapsed_ms(robot_now(), at) > 100) {
        if (controller_may_drive(&state->control))
            robot_raise_fault(F_NAVIGATION);
        return;
    }
    if (!controller_may_drive(&state->control))
        return;
    if (type == EV_BARCODE && state->control.vehicle == VEH_FOLLOW) {
        robot_flush_motion();
        robot_set_vehicle(VEH_BARCODE);
        navigation.entered_at = robot_now();
        switch (value) {
        case 1:
            (void)robot_send_motion(MOVE_TURN, -90, TURN_SPEED_MM_S);
            break;
        case 2:
            (void)robot_send_motion(MOVE_TURN, 90, TURN_SPEED_MM_S);
            break;
        case 3:
            (void)robot_send_motion(MOVE_DISTANCE, 150, 120);
            break;
        case 4:
            (void)robot_send_motion(MOVE_TURN, 180, TURN_SPEED_MM_S);
            break;
        case 5:
            (void)robot_command(CMD_STOP, 0);
            break;
        default:
            robot_raise_fault(F_NAVIGATION);
            break;
        }
    } else if (type == EV_HUMP_START && state->control.vehicle == VEH_FOLLOW) {
        robot_set_vehicle(VEH_HUMP);
        (void)robot_send_motion(MOVE_FOLLOW, 0, 120);
    } else if (type == EV_HUMP_END && state->control.vehicle == VEH_HUMP) {
        follow();
    } else if (type == EV_SCAN_DONE && state->control.vehicle == VEH_OBSTACLE) {
        if (!value || value > 2) {
            robot_raise_fault(F_NAVIGATION);
            return;
        }
        navigation.side = value;
        navigation.bypass_step = 0;
        bypass_next();
    }
}
void navigation_step(void) {
    if (navigation.generation != robot_reset_generation())
        navigation_reset();
    robot_state_t state;
    robot_snapshot(&state);
    if (robot_take_local_reset())
        local_reset(&state);
    command_step();
    robot_snapshot(&state);
    event_step(&state);
    robot_snapshot(&state);
    unsigned vehicle = state.control.vehicle;
    if (vehicle != navigation.previous) {
        navigation.entered_at = robot_now();
        if (vehicle == VEH_OBSTACLE) {
            robot_flush_motion();
            navigation.waiting = 0;
            navigation.retry = 0;
            (void)robot_take_motion_complete();
            robot_scan_request();
        } else if (vehicle == VEH_SEARCH) {
            robot_flush_motion();
            (void)robot_send_motion(MOVE_SEARCH, 0, SEARCH_SPEED_MM_S);
        } else if (vehicle == VEH_FOLLOW && navigation.previous == VEH_SEARCH)
            follow();
        navigation.previous = vehicle;
    }
    bool completed = robot_take_motion_complete();
    if (vehicle == VEH_BARCODE && completed)
        follow();
    if (vehicle == VEH_BARCODE &&
        elapsed_ms(robot_now(), navigation.entered_at) > MOTION_TIMEOUT_MS)
        robot_raise_fault(F_NAVIGATION);
    if (vehicle == VEH_OBSTACLE && navigation.waiting) {
        if (navigation.bypass_step == 5 && state.control.line_seen) {
            robot_flush_motion();
            (void)robot_send_motion(MOVE_TURN, navigation.side == 1 ? -90 : 90, TURN_SPEED_MM_S);
            navigation.bypass_step = 6;
        } else if (completed) {
            if (navigation.bypass_step == 6) {
                navigation.waiting = 0;
                follow();
            } else if (navigation.bypass_step == 5) {
                if (navigation.retry < SCAN_RETRIES) {
                    navigation.retry++;
                    (void)robot_send_motion(MOVE_DISTANCE, 50, 120);
                } else
                    robot_raise_fault(F_NAVIGATION);
            } else {
                navigation.bypass_step++;
                bypass_next();
            }
        }
        if (navigation.bypass_step == 5 &&
            elapsed_ms(robot_now(), navigation.entered_at) > SEARCH_TIMEOUT_MS)
            robot_raise_fault(F_NAVIGATION);
    }
}
