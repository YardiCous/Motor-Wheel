#include "controller.h"
#include <string.h>
void controller_init(controller_t *c) {
    memset(c, 0, sizeof(*c));
    c->vehicle = VEH_BOOT;
}
void controller_self_test(controller_t *c, bool passed) {
    c->ready = passed;
    if (passed && !c->latched)
        c->vehicle = VEH_IDLE;
    else if (!passed)
        controller_fault(c, F_WATCHDOG);
}
void controller_fault(controller_t *c, unsigned mask) {
    c->faults |= mask & 1023u;
    if (mask & F_FATAL) {
        c->latched = 1;
        c->vehicle = VEH_FAULT;
    }
}
void controller_clear_cause(controller_t *c, unsigned mask) {
    c->faults &= ~(mask & 1023u);
}
bool controller_command(controller_t *c, command_t cmd) {
    switch (cmd) {
    case CMD_ESTOP:
        controller_fault(c, F_ESTOP);
        return true;
    case CMD_STOP:
        if (!c->latched)
            c->vehicle = VEH_COMPLETE;
        return true;
    case CMD_RESET:
        if (!c->ready || (c->faults & F_FATAL))
            return false;
        if (c->vehicle != VEH_FAULT && c->vehicle != VEH_COMPLETE && c->vehicle != VEH_IDLE)
            return false;
        c->latched = 0;
        c->vehicle = VEH_IDLE;
        return true;
    case CMD_START:
        if (!c->ready || c->latched || c->vehicle != VEH_IDLE)
            return false;
        c->vehicle = VEH_FOLLOW;
        return true;
    default:
        return false;
    }
}
void controller_line(controller_t *c, line_mode_t mode, bool seen, uint16_t now) {
    if ((unsigned)mode > LINE_DECODE) {
        controller_fault(c, F_NAVIGATION);
        return;
    }
    if (c->line_mode != (unsigned)mode)
        c->context_at = now;
    c->line_mode = mode;
    c->line_seen = seen;
    c->line_at = now;
    if (seen)
        c->line_missing = 0;
    else if (!c->line_missing) {
        c->line_missing = 1;
        c->missing_at = now;
    }
}
void controller_range(controller_t *c, unsigned mm, bool valid, uint16_t now) {
    c->range_valid = valid && mm <= 4095;
    c->range_mm = mm <= 4095 ? mm : 4095;
    c->range_at = now;
    if (!c->range_valid)
        controller_fault(c, F_NO_ECHO);
}
bool controller_may_drive(const controller_t *c) {
    return !c->latched && c->ready && c->vehicle >= VEH_FOLLOW && c->vehicle <= VEH_SEARCH;
}
void controller_tick(controller_t *c, uint16_t now) {
    if (!controller_may_drive(c))
        return;
    if (elapsed_ms(now, c->line_at) > LINE_MAX_AGE_MS) {
        controller_fault(c, F_WATCHDOG);
        return;
    }
    if (!c->range_exempt && (!c->range_valid || elapsed_ms(now, c->range_at) > RANGE_MAX_AGE_MS)) {
        controller_fault(c, F_NO_ECHO);
        return;
    }
    if ((c->vehicle == VEH_FOLLOW || c->vehicle == VEH_HUMP) && c->range_mm <= OBSTACLE_STOP_MM) {
        c->vehicle = VEH_OBSTACLE;
        return;
    }
    bool context = c->line_mode == LINE_BARCODE_SCAN || c->line_mode == LINE_JUNCTION ||
                   c->line_mode == LINE_DECODE;
    if (context &&
        elapsed_ms(now, c->context_at) >
            (c->line_mode == LINE_BARCODE_SCAN ? BARCODE_FRAME_TIMEOUT_MS : CONTEXT_TIMEOUT_MS)) {
        controller_fault(c, F_NAVIGATION);
        return;
    }
    if ((c->vehicle == VEH_FOLLOW || c->vehicle == VEH_HUMP) && !context && c->line_missing &&
        elapsed_ms(now, c->missing_at) > LINE_LOST_MS) {
        c->vehicle = VEH_SEARCH;
        c->search_at = now;
    }
    if (c->vehicle == VEH_SEARCH) {
        if (c->line_seen)
            c->vehicle = VEH_FOLLOW;
        else if (elapsed_ms(now, c->search_at) > SEARCH_TIMEOUT_MS)
            controller_fault(c, F_NAVIGATION);
    }
}
