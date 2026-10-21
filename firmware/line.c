#include "hardware/adc.h"
#include "robot.h"
#include <string.h>

/* Streaming forward Code 39: start '*', one command letter, stop '*'.
 * Wide/narrow patterns are the standard Code 39 symbol definitions. */
static struct {
    uint8_t widths[9];
    uint16_t edge_at, frame_at, quiet_at, all_black_at;
    unsigned count : 4;
    unsigned phase : 3; /* 0=track,1=start,2=letter,3=stop,4=quiet */
    unsigned color : 1;
    unsigned gap : 1;
    unsigned command : 3;
    unsigned black_active : 1;
    unsigned generation : 8;
} barcode;
void line_reset(void) {
    memset(&barcode, 0, sizeof(barcode));
    barcode.generation = robot_reset_generation();
}
void line_init(void) {
    line_reset();
}
static int symbol(void) {
    unsigned sorted[9];
    for (unsigned i = 0; i < 9; ++i)
        sorted[i] = barcode.widths[i];
    for (unsigned i = 1; i < 9; ++i) {
        unsigned v = sorted[i], j = i;
        while (j && sorted[j - 1] > v) {
            sorted[j] = sorted[j - 1];
            --j;
        }
        sorted[j] = v;
    }
    if (sorted[0] < BARCODE_MIN_ELEMENT_MS || sorted[8] > BARCODE_MAX_ELEMENT_MS ||
        sorted[6] * 2 < sorted[5] * 3 || sorted[8] > sorted[0] * 4)
        return -1;
    unsigned boundary = (sorted[5] + sorted[6]) / 2;
    unsigned pattern = 0;
    for (unsigned i = 0; i < 9; ++i)
        pattern = (pattern << 1) | (barcode.widths[i] > boundary);
    switch (pattern) {
    case 0x094:
        return 0; /* * */
    case 0x043:
        return 1; /* L */
    case 0x106:
        return 2; /* R */
    case 0x046:
        return 3; /* S */
    case 0x181:
        return 4; /* U */
    case 0x118:
        return 5; /* E */
    default:
        return -1;
    }
}
static void scan(bool black, bool full_width, uint16_t now) {
    if (!barcode.phase) {
        if (full_width) {
            barcode.phase = 1;
            barcode.count = 0;
            barcode.gap = 0;
            barcode.color = 1;
            barcode.edge_at = now;
            barcode.frame_at = now;
        }
        return;
    }
    unsigned age = elapsed_ms(now, barcode.frame_at);
    if (age > BARCODE_FRAME_TIMEOUT_MS || (barcode.phase == 1 && age > 1000)) {
        line_reset();
        return;
    }
    if (barcode.phase == 4) {
        if (black) {
            line_reset();
            return;
        }
        if (elapsed_ms(now, barcode.quiet_at) >= BARCODE_QUIET_MS) {
            unsigned command = barcode.command;
            line_reset();
            robot_set_barcode(command);
            (void)robot_send_event(EV_BARCODE, command);
        }
        return;
    }
    if (black == barcode.color) {
        if (elapsed_ms(now, barcode.edge_at) > BARCODE_MAX_ELEMENT_MS)
            line_reset();
        return;
    }
    unsigned width = elapsed_ms(now, barcode.edge_at);
    barcode.edge_at = now;
    barcode.color = black;
    if (width < BARCODE_MIN_ELEMENT_MS || width > BARCODE_MAX_ELEMENT_MS) {
        line_reset();
        return;
    }
    if (barcode.gap) {
        if (!black) {
            line_reset();
            return;
        }
        barcode.gap = 0;
        barcode.count = 0;
        return;
    }
    barcode.widths[barcode.count++] = (uint8_t)width;
    if (barcode.count == 9) {
        int decoded = symbol();
        if (black || decoded < 0 || (barcode.phase != 2 && decoded != 0) ||
            (barcode.phase == 2 && decoded == 0)) {
            line_reset();
            return;
        }
        if (barcode.phase == 2)
            barcode.command = (unsigned)decoded;
        if (barcode.phase == 3) {
            barcode.phase = 4;
            barcode.quiet_at = now;
        } else {
            barcode.phase++;
            barcode.gap = 1;
        }
    }
}
void line_step(void) {
    if (barcode.generation != robot_reset_generation())
        line_reset();
    unsigned values[3];
    for (unsigned i = 0; i < 3; ++i) {
        adc_select_input(i);
        values[i] = adc_read();
    }
    bool left = IR_BLACK_HIGH ? values[0] >= IR_THRESHOLD_LEFT : values[0] < IR_THRESHOLD_LEFT;
    bool center =
        IR_BLACK_HIGH ? values[1] >= IR_THRESHOLD_CENTER : values[1] < IR_THRESHOLD_CENTER;
    bool right = IR_BLACK_HIGH ? values[2] >= IR_THRESHOLD_RIGHT : values[2] < IR_THRESHOLD_RIGHT;
    uint16_t now = robot_now();
    bool full_width = left && center && right;
    if (full_width && !barcode.black_active) {
        barcode.black_active = 1;
        barcode.all_black_at = now;
    }
    if (!full_width)
        barcode.black_active = 0;
    robot_state_t state;
    robot_snapshot(&state);
    if (state.control.vehicle == VEH_FOLLOW && BARCODE_CODE39)
        scan(center, full_width, now);
    else if (barcode.phase)
        line_reset();
    line_mode_t mode = barcode.phase ? LINE_BARCODE_SCAN : LINE_TRACKING;
    if (full_width && elapsed_ms(now, barcode.all_black_at) > 150) {
        /* A continuous wide region is a junction, never an actionable barcode. */
        barcode.phase = 0;
        mode = LINE_JUNCTION;
    }
    unsigned sum = left + center + right;
    int error = sum ? 1000 * ((int)right - (int)left) / (int)sum : 0;
    line_msg_t message = {{0}};
    bits_put(message.bytes, 4, 0, 11, (unsigned)(error + 1000));
    bits_put(message.bytes, 4, 11, 3, mode);
    bits_put(message.bytes, 4, 14, 1, sum != 0);
    bits_put(message.bytes, 4, 15, 16, now);
    robot_line_update(&message, values[0], values[1], values[2]);
}
