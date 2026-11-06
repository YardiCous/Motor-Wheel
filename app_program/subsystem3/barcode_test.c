/** @file barcode_test.c
 * @brief Motor-off GP28 barcode bench: raw capture, then Code 39 decoding.
 *
 * A scan starts on the first white-to-black edge seen while the wheels are
 * turning (sensor flicker while parked is ignored) and ends after a long
 * white quiet zone. A first "bar" shorter than BARCODE_TEST_MIN_BAR_MS is
 * a blip, not a bar, and is thrown away. Each bar and gap is timed in
 * milliseconds and in encoder edges; edges are used for decoding, so a
 * change of driving speed does not matter.
 */
#include "barcode_test.h"

#include "../board_io.h"
#include "../robot_config.h"

#include <tm/tmonitor.h>

#define BARCODE_TEST_SENSOR 2u
#define BARCODE_TEST_HYSTERESIS 40u
#define BARCODE_TEST_IDLE_REPORT_MS 500u
/* A scan ends after this much white, or 3x the longest element if longer. */
#define BARCODE_TEST_QUIET_MS 300u
#define BARCODE_TEST_QUIET_FACTOR 3u
/* Black this long is the track line or a hand over the sensor, not a bar. */
#define BARCODE_TEST_BLACK_LIMIT_MS 1500u
/* Below this many encoder edges the wheels did not really move. */
#define BARCODE_TEST_MIN_EDGES 100u
#define BARCODE_TEST_WIDTH_LIMIT 65535u
/* The wheels count as moving if they turned within this many ms. */
#define BARCODE_TEST_MOVING_MS 50u
/* A first bar shorter than this is noise: a narrow bar at 70 mm/s is ~60 ms.
 */
#define BARCODE_TEST_MIN_BAR_MS 15u

static bool     sample_black(barcode_test_t * p_test, uint16_t raw);
static void     idle_step(barcode_test_t * p_test, bool b_black, uint16_t raw,
                          uint32_t now);
static void     scan_step(barcode_test_t * p_test, bool b_black, uint32_t now);
static void     start_scan(barcode_test_t * p_test, uint32_t now);
static void     push_element(barcode_test_t * p_test, uint32_t now);
static void     finish_scan(barcode_test_t * p_test, const char * p_reason);
static void     print_elements(const barcode_test_t * p_test);
static void     print_decode(const barcode_test_t * p_test);
static uint16_t moved_since(barcode_test_t * p_test);

/** @brief Motors are held off: nothing else in this build commands them. */
void barcode_test_init(barcode_test_t * p_test, uint8_t phase)
{
    uint32_t left  = 0u;
    uint32_t right = 0u;
    (void)board_io_encoders(&left, &right);
    board_io_motor(0, 0);
    p_test->last_left  = left;
    p_test->last_right = right;
    p_test->report_ms  = board_io_time_ms();
    p_test->moving_ms  = 0u;
    p_test->ignored    = 0u;
    p_test->phase      = phase;
    p_test->b_black    = false;
    p_test->b_scanning = false;
    p_test->scans      = 0u;
    p_test->low        = 4095u;
    p_test->high       = 0u;
    tm_printf(
        (UB *)"\nBARCODE TEST %u: %s\n", (unsigned)phase,
        (UB *)((phase >= 2u) ? "raw data + Code 39 decode" : "raw data only"));
    tm_printf((UB *)"Sensor GP28, threshold %u "
                    "(bigger = black, +/-%u hysteresis)\n",
              (unsigned)ROBOT_IR_THRESHOLD_RIGHT,
              (unsigned)BARCODE_TEST_HYSTERESIS);
    tm_printf((UB *)"Put GP28 just before the barcode and press START "
                    "(GP20): the robot drives %u mm and stops.\n",
              (unsigned)ROBOT_BARCODE_DRIVE_MM);
    tm_printf((UB *)"Recording starts at the first black bar seen while "
                    "the wheels turn.\n");
    tm_printf((UB *)"IDLE lines every 0.5 s show what GP28 reads right "
                    "now.\n\n");
}

/** @brief One 5-ms sample: idle reporting, or timing the current element. */
void barcode_test_step(barcode_test_t * p_test)
{
    uint16_t samples[ROBOT_IR_COUNT] = {0u};
    (void)board_io_line(samples);
    uint16_t raw     = samples[BARCODE_TEST_SENSOR];
    uint32_t now     = board_io_time_ms();
    bool     b_black = sample_black(p_test, raw);
    p_test->low      = (raw < p_test->low) ? raw : p_test->low;
    p_test->high     = (raw > p_test->high) ? raw : p_test->high;

    if (p_test->b_scanning)
    {
        scan_step(p_test, b_black, now);
    }
    else
    {
        idle_step(p_test, b_black, raw, now);
    }

    p_test->b_black = b_black;
}

/** @brief Hysteresis stops a level near the threshold from chattering. */
static bool sample_black(barcode_test_t * p_test, uint16_t raw)
{
    uint32_t threshold = ROBOT_IR_THRESHOLD_RIGHT;
    uint32_t on        = threshold + BARCODE_TEST_HYSTERESIS;
    uint32_t off       = (threshold > BARCODE_TEST_HYSTERESIS)
                             ? (threshold - BARCODE_TEST_HYSTERESIS)
                             : 0u;
    return p_test->b_black ? ((uint32_t)raw >= off) : ((uint32_t)raw >= on);
}

/** @brief Waiting for a bar; report the live level twice a second. */
static void idle_step(barcode_test_t * p_test, bool b_black, uint16_t raw,
                      uint32_t now)
{
    bool b_moving = false;

    if (0u != moved_since(p_test))
    {
        p_test->moving_ms = now;
        b_moving          = true;
    }
    else
    {
        b_moving = (0u != p_test->moving_ms) &&
                   ((now - p_test->moving_ms) < BARCODE_TEST_MOVING_MS);
    }

    if (b_black && (!p_test->b_black) && b_moving)
    {
        start_scan(p_test, now);
    }
    else if ((now - p_test->report_ms) >= BARCODE_TEST_IDLE_REPORT_MS)
    {
        tm_printf((UB *)"IDLE  GP28=%4u %s  (min %4u max %4u, thr %u)%s\n",
                  (unsigned)raw, (UB *)(b_black ? "BLACK" : "white"),
                  (unsigned)p_test->low, (unsigned)p_test->high,
                  (unsigned)ROBOT_IR_THRESHOLD_RIGHT,
                  (UB *)(b_moving ? "" : "  parked: not recording"));
        p_test->report_ms = now;
        p_test->low       = raw;
        p_test->high      = raw;
    }
    else
    {
        /* Nothing to report between idle lines. */
    }
}

/** @brief Each colour change closes one element; long white ends the scan. */
static void scan_step(barcode_test_t * p_test, bool b_black, uint32_t now)
{
    uint32_t elapsed = now - p_test->edge_ms;
    uint32_t quiet = (uint32_t)p_test->longest_ms * BARCODE_TEST_QUIET_FACTOR;
    quiet = (quiet > BARCODE_TEST_QUIET_MS) ? quiet : BARCODE_TEST_QUIET_MS;

    if ((b_black != p_test->b_black) && (0u == p_test->count) &&
        (elapsed < BARCODE_TEST_MIN_BAR_MS))
    {
        /* A blip, not a bar: forget it and wait for a real one. */
        p_test->b_scanning = false;
        ++p_test->ignored;
    }
    else if (b_black != p_test->b_black)
    {
        push_element(p_test, now);

        if (p_test->count >= CODE39_MAX_ELEMENTS)
        {
            p_test->b_overflow = true;
            finish_scan(p_test, "too many bars/gaps");
        }
    }
    else if (b_black && (elapsed >= BARCODE_TEST_BLACK_LIMIT_MS))
    {
        finish_scan(p_test, "black too long (track line? hand?)");
    }
    else if ((!b_black) && (elapsed >= quiet))
    {
        finish_scan(p_test, "white quiet zone");
    }
    else
    {
        /* The current element is still growing. */
    }
}

/** @brief The first bar's leading edge starts the timing. */
static void start_scan(barcode_test_t * p_test, uint32_t now)
{
    p_test->b_scanning  = true;
    p_test->b_overflow  = false;
    p_test->count       = 0u;
    p_test->longest_ms  = 0u;
    p_test->moved_edges = 0u;
    p_test->edge_ms     = now;
    p_test->low         = 4095u;
    p_test->high        = 0u;
    (void)moved_since(p_test);
}

/** @brief Store the element that just ended (its colour is b_black). */
static void push_element(barcode_test_t * p_test, uint32_t now)
{
    uint32_t width_ms = now - p_test->edge_ms;
    width_ms = (width_ms > BARCODE_TEST_WIDTH_LIMIT) ? BARCODE_TEST_WIDTH_LIMIT
                                                     : width_ms;
    uint16_t edges = moved_since(p_test);

    if (p_test->count < CODE39_MAX_ELEMENTS)
    {
        p_test->widths_ms[p_test->count]    = (uint16_t)width_ms;
        p_test->widths_edges[p_test->count] = edges;
        ++p_test->count;
    }

    p_test->moved_edges += edges;
    p_test->longest_ms = ((uint16_t)width_ms > p_test->longest_ms)
                             ? (uint16_t)width_ms
                             : p_test->longest_ms;
    p_test->edge_ms    = now;
}

/** @brief Printing happens only here, after the bars have passed. */
static void finish_scan(barcode_test_t * p_test, const char * p_reason)
{
    ++p_test->scans;
    tm_printf((UB *)"\n=== SCAN %u: %u bars/gaps, ended by %s ===\n",
              (unsigned)p_test->scans, (unsigned)p_test->count,
              (UB *)p_reason);
    tm_printf((UB *)"GP28 during scan: min %u max %u (thr %u). "
                    "Wheels moved %u edges. %u blips ignored.\n",
              (unsigned)p_test->low, (unsigned)p_test->high,
              (unsigned)ROBOT_IR_THRESHOLD_RIGHT,
              (unsigned)p_test->moved_edges, (unsigned)p_test->ignored);
    p_test->ignored = 0u;
    print_elements(p_test);

    if (p_test->phase >= 2u)
    {
        print_decode(p_test);
    }

    tm_printf((UB *)"\n");
    p_test->b_scanning = false;
    p_test->report_ms  = board_io_time_ms();
    p_test->low        = 4095u;
    p_test->high       = 0u;
}

/** @brief Elements alternate BLACK bar, white gap, starting with a bar. */
static void print_elements(const barcode_test_t * p_test)
{
    tm_printf((UB *)"  #  colour   ms  edges\n");

    for (uint8_t index = 0u; index < p_test->count; ++index)
    {
        bool b_bar = (0u == (index & 1u));
        tm_printf((UB *)" %2u  %s %5u %6u\n", (unsigned)(index + 1u),
                  (UB *)(b_bar ? "BLACK" : "white"),
                  (unsigned)p_test->widths_ms[index],
                  (unsigned)p_test->widths_edges[index]);
    }
}

/** @brief Phase 2: turn the widths into characters and one letter. */
static void print_decode(const barcode_test_t * p_test)
{
    bool             b_edges = (p_test->moved_edges >= BARCODE_TEST_MIN_EDGES);
    const uint16_t * p_widths =
        b_edges ? p_test->widths_edges : p_test->widths_ms;
    code39_result_t result = {0};
    code39_decode(p_widths, p_test->count, &result);
    tm_printf((UB *)"DECODE: using %s widths\n",
              (UB *)(b_edges ? "encoder-edge" : "time (ms)"));

    if (CODE39_BAD_COUNT == result.status)
    {
        tm_printf((UB *)"DECODE FAIL: %u bars/gaps; one Code 39 character "
                        "needs 9.\n",
                  (unsigned)p_test->count);
    }
    else if (CODE39_UNCLEAR == result.status)
    {
        tm_printf((UB *)"DECODE FAIL: char %u, wide vs narrow unclear "
                        "(widest narrow %u, narrowest wide %u)\n",
                  (unsigned)(result.bad_char + 1u),
                  (unsigned)result.widest_narrow,
                  (unsigned)result.narrowest_wide);
    }
    else if (CODE39_UNKNOWN == result.status)
    {
        tm_printf((UB *)"DECODE FAIL: char %u pattern 0x%03x is not "
                        "Code 39\n",
                  (unsigned)(result.bad_char + 1u), (unsigned)result.pattern);
    }
    else
    {
        tm_printf((UB *)"DECODE: read \"%s\" %s%s\n", (UB *)result.text,
                  (UB *)(result.b_reversed ? "(backwards)" : "(forwards)"),
                  (UB *)(result.b_partial
                             ? " - partial scan, whole characters only"
                             : ""));
    }

    if (CODE39_OK == result.status)
    {
        tm_printf((UB *)">>> LETTER: %c <<<\n", (int)result.letter);
    }
    else if (CODE39_NOT_LETTER == result.status)
    {
        tm_printf((UB *)"DECODE: not a single capital letter (*X*, *X, X* "
                        "or X)\n");
    }
    else
    {
        /* The failure line above already explains the result. */
    }
}

/** @brief Mean absolute wheel movement since the last call, in edges. */
static uint16_t moved_since(barcode_test_t * p_test)
{
    uint32_t left  = 0u;
    uint32_t right = 0u;
    (void)board_io_encoders(&left, &right);
    int32_t  delta_left  = (int32_t)(left - p_test->last_left);
    int32_t  delta_right = (int32_t)(right - p_test->last_right);
    uint32_t moved_left =
        (delta_left < 0) ? (uint32_t)(-delta_left) : (uint32_t)delta_left;
    uint32_t moved_right =
        (delta_right < 0) ? (uint32_t)(-delta_right) : (uint32_t)delta_right;
    uint32_t moved     = (moved_left + moved_right) / 2u;
    p_test->last_left  = left;
    p_test->last_right = right;
    return (uint16_t)((moved > BARCODE_TEST_WIDTH_LIMIT)
                          ? BARCODE_TEST_WIDTH_LIMIT
                          : moved);
}

/*** End of file. ***/

