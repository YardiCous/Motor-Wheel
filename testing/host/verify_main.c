/** @file verify_main.c
 * @brief Behavioral verification for packed and widened reference builds.
 */
#include "verify_main.h"

#include "../../app_program/data_formats.h"
#include "../../app_program/robot_config.h"
#include "../../app_program/subsystem1/telemetry.h"
#include "../../app_program/subsystem2/motion.h"
#include "../../app_program/subsystem3/line.h"
#include "../../app_program/subsystem4/safety.h"
#include "../../app_program/subsystem4/terrain.h"
#include "../../app_program/subsystem5/navigation.h"
#include "../../app_program/subsystem5/obstacle.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

#define VERIFY_WIDTH_MAX 32u
#define VERIFY_OFFSET_MAX 41u
#define VERIFY_MOTION_STEPS 1000u
#define VERIFY_STATES 8u
#define VERIFY_FAULTS 4096u

#define VERIFY_SIGNED_LIMIT 512
#define VERIFY_HUMP_SAMPLES 100u
#define VERIFY_OVERFLOW_START 104u
#define VERIFY_OVERFLOW_END 230u
#define VERIFY_BARCODE_SYMBOLS 4u
#define VERIFY_LINE_CALIBRATION 32u
#define VERIFY_LOST_SAMPLES 31u
#define VERIFY_SCAN_POINTS 8u
#define VERIFY_BYPASS_STAGES 8u
#define VERIFY_CONFIRMATIONS 3u
#define VERIFY_FIRST_SAMPLE 1u

static void verify_fields(void);
static void verify_vector(void);
static void verify_motion(void);
static void verify_safety(void);
static void verify_terrain(void);
static void verify_line(void);
static void verify_bad_barcode(void);
static void verify_obstacle(void);
static void verify_navigation(void);
static void verify_fragments(void);
static void verify_interlocks(void);
static void line_sample(line_t * p_line, bool b_black, uint16_t distance);
static void motion_ticks(motion_t * p_motion, uint32_t ticks,
                         uint32_t increment, bool b_turn);

/** @brief Assertions report the first reproducible violated contract. */
int main (void)
{
    verify_fields();
    verify_vector();
    verify_motion();
    verify_safety();
    verify_terrain();
    verify_line();
    verify_bad_barcode();
    verify_obstacle();
    verify_navigation();
    verify_fragments();
    verify_interlocks();
    (void)puts("PASS: fields, motion, safety, terrain, line, obstacle, "
               "navigation, MQTT");
    return 0;
}

/** @brief The host decoder independently checks this firmware-format vector.
 */
static void verify_vector (void)
{
    uint8_t  bytes[DATA_FORMATS_TELEMETRY_BYTES] = {0u};
    uint32_t word                                = 0u;
    assert(data_formats_motion_pack(&word, 4095u, 500u, true, 3u, 127u));
    assert(data_formats_write(bytes, 0u, 2u, 1u));
    assert(data_formats_write(bytes, 2u, 3u, 2u));
    assert(data_formats_write(bytes, 5u, 32u, word));
    assert(data_formats_write(bytes, 37u, 12u, 0x123u));
    assert(data_formats_write(bytes, 49u, 12u, 0u));
    assert(data_formats_write(bytes, 61u, 12u, 2048u));
    assert(data_formats_write(bytes, 73u, 12u, 4095u));
    assert(data_formats_write(bytes, 85u, 3u, 4u));
    assert(data_formats_write(bytes, 88u, 7u, 54u));
    assert(data_formats_write(bytes, 95u, 8u, 43u));
    assert(data_formats_write(bytes, 103u, 9u, 400u));
    assert(data_formats_write(bytes, 112u, 6u, 50u));
    assert(data_formats_write(bytes, 118u, 3u, 7u));
    assert(data_formats_write(bytes, 121u, 8u, 0u));
    assert(data_formats_write(bytes, 129u, 3u, 4u));
    assert(data_formats_write(bytes, 132u, 3u, 2u));
    assert(data_formats_write(bytes, 135u, 2u, 3u));
    assert(data_formats_write(bytes, 137u, 1u, 1u));
    assert(data_formats_write(bytes, 138u, 1u, 1u));
    assert(data_formats_write(bytes, 139u, 32u, UINT32_MAX));
    assert(data_formats_write(bytes, 171u, 32u, 2u));
    assert(data_formats_write(bytes, 203u, 10u, 12u));
    assert(data_formats_write(bytes, 213u, 10u, 1012u));
    (void)printf("VECTOR ");

    for (uint8_t index = 0u; index < DATA_FORMATS_TELEMETRY_BYTES; ++index)
    {
        (void)printf("%02x", bytes[index]);
    }

    (void)printf("\n");
}

/** @brief Exercise cross-byte fields and preserve every neighboring bit. */
static void verify_fields (void)
{
    uint8_t bytes[12] = {0u};

    for (uint8_t width = 1u; width <= VERIFY_WIDTH_MAX; ++width)
    {
        for (uint16_t offset = 0u; offset < VERIFY_OFFSET_MAX; ++offset)
        {
            uint32_t maximum =
                (32u == width) ? UINT32_MAX : ((1u << width) - 1u);
            memset(bytes, 0xa5, sizeof(bytes));
            assert(data_formats_write(bytes, offset, width, maximum));
            assert(maximum == data_formats_read(bytes, offset, width));

            for (uint16_t bit = 0u; bit < (sizeof(bytes) * 8u); ++bit)
            {
                if ((bit < offset) || (bit >= (offset + width)))
                {
                    uint32_t expected = (0xa5u >> (bit % 8u)) & 1u;
                    assert(expected == data_formats_read(bytes, bit, 1u));
                }
            }
        }
    }

    uint8_t unchanged[12] = {0u};
    memcpy(unchanged, bytes, sizeof(bytes));
    assert(!data_formats_write(bytes, 0u, 3u, 8u));
    assert(0 == memcmp(bytes, unchanged, sizeof(bytes)));
    assert(!data_formats_write(bytes, 0u, 0u, 0u));

    for (int32_t value = -VERIFY_SIGNED_LIMIT; value < VERIFY_SIGNED_LIMIT;
         ++value)
    {
        assert(value ==
               data_formats_unsign(data_formats_signed(value, 10u), 10u));
    }

    uint32_t word = 0u;
    assert(data_formats_motion_pack(&word, 4095u, 500u, true, 6u, 127u));
    assert(4095u == (word & 4095u));
    assert(500u == ((word >> 12u) & 511u));
    assert(1u == ((word >> 21u) & 1u));
    assert(6u == ((word >> 22u) & 7u));
    assert(127u == (word >> 25u));
    assert(!data_formats_motion_pack(&word, 4096u, 0u, false, 0u, 0u));
    assert(!data_formats_motion_pack(&word, 0u, 512u, false, 0u, 0u));
    assert(!data_formats_motion_pack(&word, 0u, 0u, false, 7u, 0u));
    assert(5 == data_formats_distance_delta(3u, 4094u));
    assert(-5 == data_formats_distance_delta(4094u, 3u));
    assert(data_formats_sequence_new(0u, 15u));
    assert(!data_formats_sequence_new(15u, 0u));
    assert(!data_formats_sequence_new(7u, 7u));
}

/** @brief Distance completion, turning, wrap and stall use real controller
 * code. */
static void verify_motion (void)
{
    motion_t controller = {0};
    motion_init(&controller, 0u, 0u);
    uint32_t command = 0u;
    assert(motion_command(&command, MOTION_MOVE, 1000u, 500u, false));
    assert(motion_accept(&controller, command));
    motion_ticks(&controller, 205u, 23u, false);
    assert(MOTION_IDLE == controller.mode);
    assert(0u != controller.b_done);
    assert(controller.position_um >= 1000000u);
    assert(controller.position_um < 1010000u);
    motion_init(&controller, UINT32_MAX - 2u, UINT32_MAX - 2u);
    assert(motion_command(&command, MOTION_TURN, 110u, 150u, false));
    assert(motion_accept(&controller, command));
    motion_ticks(&controller, 60u, 10u, true);
    assert(MOTION_IDLE == controller.mode);
    assert(controller.position_um < 1000u);
    bool b_valid = false;
    assert(5 == motion_edge_delta(2u, UINT32_MAX - 2u, &b_valid));
    assert(b_valid);
    assert(0 == motion_edge_delta(1000u, 0u, &b_valid));
    assert(!b_valid);
    motion_init(&controller, 0u, 0u);
    assert(motion_command(&command, MOTION_FOLLOW, 0u, 250u, false));
    assert(motion_accept(&controller, command));
    motion_ticks(&controller, 20u, 0u, false);
    assert(0u != controller.b_stall);
    assert(MOTION_STALL == controller.mode);
    int16_t left_pwm  = 10;
    int16_t right_pwm = 10;
    motion_step(&controller, 0u, 0u, 0, true, &left_pwm, &right_pwm);
    assert((0 == left_pwm) && (0 == right_pwm));
    assert(!motion_accept(&controller, command));
    assert(!motion_accept(&controller, UINT32_MAX));
    motion_init(&controller, 0u, 0u);
    assert(motion_accept(&controller, command));
    motion_ticks(&controller, VERIFY_MOTION_STEPS, 12u, false);
    assert(MOTION_FOLLOW == controller.mode);
    assert(controller.position_um > 2000000u);
}

/** @brief Every combination of the twelve faults respects advisory exclusions.
 */
static void verify_safety (void)
{
    for (uint16_t faults = 0u; faults < VERIFY_FAULTS; ++faults)
    {
        safety_t monitor = {0};
        safety_init(&monitor);
        assert(safety_reset(&monitor));
        safety_update(&monitor, faults);
        bool b_fatal = (0u != (faults & DATA_FORMATS_FATAL_MASK));
        assert(b_fatal == (0u != monitor.b_latched));
        assert(!b_fatal == safety_reset(&monitor));
        safety_update(&monitor, 0u);
        assert(b_fatal == (0u != monitor.b_latched));
        assert(safety_reset(&monitor));
    }
}

/** @brief Fine height integration survives 0.435-mm increments and overflow.
 */
static void verify_terrain (void)
{
    terrain_t detector = {0};
    terrain_init(&detector);

    for (uint32_t sample = 0u; sample < ROBOT_IMU_CAL_SAMPLES; ++sample)
    {
        terrain_step(&detector, 0, 0, 1000, 0, 0u, true);
    }

    assert(TERRAIN_LEVEL == detector.state);
    assert(0u != detector.b_valid);
    detector.last_speed = 1012u;

    for (uint16_t sample = VERIFY_FIRST_SAMPLE; sample <= VERIFY_HUMP_SAMPLES;
         ++sample)
    {
        terrain_step(&detector, 87 + 1, 0, 996, 500, sample * 5u, true);
    }

    assert(detector.peak_mm >= 43u);
    assert(detector.peak_mm <= 45u);
    terrain_step(&detector, 0, 0, 1000, 500, 505u, true);
    assert(TERRAIN_CRESTING == detector.state);
    terrain_step(&detector, -100, 0, 995, 500, 510u, true);
    assert(TERRAIN_DESCENDING == detector.state);
    terrain_step(&detector, 0, 0, 1000, 500, 515u, true);
    assert(TERRAIN_LEVEL == detector.state);
    assert(0u != detector.b_finished);
    assert(0u == detector.b_on_hump);

    for (uint16_t sample = VERIFY_OVERFLOW_START; sample < VERIFY_OVERFLOW_END;
         ++sample)
    {
        terrain_step(&detector, 500, 0, 866, 500, sample * 5u, true);
    }

    assert(255u == detector.peak_mm);
    assert(0u != detector.b_overflow);
    assert(0u == detector.b_valid);
    terrain_step(&detector, 0, 0, 3000, 500, 1150u, true);
    assert(0u != detector.b_impact);
    terrain_step(&detector, 0, 0, 1000, 0, 0u, false);
    terrain_step(&detector, 0, 0, 1000, 0, 0u, false);
    terrain_step(&detector, 0, 0, 1000, 0, 0u, false);
    assert(TERRAIN_SENSOR_FAULT == detector.state);
}

/** @brief All four framed barcodes decode once with encoder-measured widths.
 */
static void verify_line (void)
{
    for (uint8_t symbol = 0u; symbol < VERIFY_BARCODE_SYMBOLS; ++symbol)
    {
        line_t detector = {0};
        line_init(&detector);

        for (uint8_t sample = 0u; sample < VERIFY_LINE_CALIBRATION; ++sample)
        {
            line_sample(&detector, false, 0u);
        }

        uint16_t position = 10u;
        line_sample(&detector, true, position);
        position += 30u;
        line_sample(&detector, false, position);
        assert(LINE_SCAN == detector.state);
        position += 6u;
        line_sample(&detector, true, position);
        position += (0u != (symbol & 2u)) ? 18u : 6u;
        line_sample(&detector, false, position);
        position += 6u;
        line_sample(&detector, true, position);
        position += (0u != (symbol & 1u)) ? 18u : 6u;
        line_sample(&detector, false, position);
        position += 6u;
        line_sample(&detector, true, position);
        position += 30u;
        line_sample(&detector, false, position);
        assert(0u != detector.b_barcode_valid);
        assert(symbol == detector.barcode);
        line_sample(&detector, false, position + 1u);
        assert(0u == detector.b_barcode_valid);
    }

    line_t detector = {0};
    line_init(&detector);
    detector.state                 = LINE_TRACKING;
    uint16_t white[ROBOT_IR_COUNT] = {0u};

    for (uint8_t sample = 0u; sample < VERIFY_LOST_SAMPLES; ++sample)
    {
        line_step(&detector, white, 0u, false);
    }

    assert(0u != detector.b_lost);
    assert(LINE_SEARCH == detector.state);
    line_step(&detector, white, 0u, true);
    assert(0u == detector.b_lost);
    detector.state    = LINE_SCAN;
    detector.frame_ms = 995u;
    line_step(&detector, white, 0u, false);
    assert(LINE_SCAN != detector.state);
}

/** @brief Ambiguous bars and reversing cannot generate a command. */
static void verify_bad_barcode (void)
{
    line_t detector = {0};
    line_init(&detector);
    detector.state = LINE_TRACKING;
    line_sample(&detector, true, 10u);
    line_sample(&detector, false, 40u);
    assert(LINE_SCAN == detector.state);
    line_sample(&detector, true, 46u);
    line_sample(&detector, false, 59u);
    assert(LINE_TRACKING == detector.state);
    assert(!detector.b_barcode_valid);
    line_sample(&detector, true, 65u);
    line_sample(&detector, false, 95u);
    assert(LINE_SCAN == detector.state);
    line_sample(&detector, false, 90u);
    assert(LINE_TRACKING == detector.state);
    assert(!detector.b_barcode_valid);
}

/** @brief A scan cannot treat missing echoes as a traversable route. */
static void verify_obstacle (void)
{
    obstacle_t scanner = {0};
    obstacle_init(&scanner);
    assert(obstacle_scan(&scanner));
    assert(!obstacle_scan(&scanner));
    uint16_t samples[8] = {120u, 100u, 30u, 90u, 80u, 110u, 120u, 100u};

    for (uint8_t index = 0u; index < VERIFY_SCAN_POINTS; ++index)
    {
        obstacle_sample(&scanner, samples[index], true);
    }

    assert(OBSTACLE_PLAN == scanner.state);
    assert(0u != scanner.b_left);
    assert(0u != scanner.b_ready);
    assert(100u == obstacle_median(400u, 100u, 2u));
    obstacle_init(&scanner);
    assert(obstacle_scan(&scanner));
    obstacle_sample(&scanner, 0u, false);
    obstacle_sample(&scanner, 0u, false);
    obstacle_sample(&scanner, 0u, false);
    assert(OBSTACLE_ABORT == scanner.state);
    obstacle_init(&scanner);
    assert(obstacle_scan(&scanner));
    obstacle_tick(&scanner, 5000u);
    assert(OBSTACLE_ABORT == scanner.state);
}

/** @brief Barcode, obstacle bypass, recovery and safety arbitration interact.
 */
static void verify_navigation (void)
{
    navigation_t manager = {0};
    navigation_init(&manager);
    assert(!navigation_supervise(&manager, DATA_FORMATS_START, true));
    assert(navigation_supervise(&manager, DATA_FORMATS_START, false));
    navigation_barcode(&manager, DATA_FORMATS_UTURN);
    assert(DATA_FORMATS_BARCODE == manager.state);
    assert(MOTION_TURN == (manager.command & 3u));
    assert(218u == ((manager.command >> 2u) & 4095u));
    navigation_step(&manager, MOTION_IDLE, true, true, false, false, 100u,
                    true, false, false, false, false);
    assert(DATA_FORMATS_FOLLOW == manager.state);
    navigation_step(&manager, MOTION_FOLLOW, false, true, false, false, 35u,
                    true, false, false, false, false);
    assert(DATA_FORMATS_AVOID == manager.state);
    assert(0u != manager.b_stop_requested);
    navigation_step(&manager, MOTION_IDLE, false, false, false, false, 100u,
                    true, true, true, false, false);
    assert(1u == manager.stage);

    for (uint8_t stage = VERIFY_FIRST_SAMPLE; stage < VERIFY_BYPASS_STAGES;
         ++stage)
    {
        navigation_step(&manager, MOTION_IDLE, true, false, false, false, 100u,
                        true, false, true, false, false);
    }

    assert(DATA_FORMATS_SEARCH == manager.state);

    for (uint8_t confirmation = 0u; confirmation < VERIFY_CONFIRMATIONS;
         ++confirmation)
    {
        navigation_step(&manager, MOTION_TURN, false, true, false, false, 100u,
                        true, false, true, false, false);
    }

    assert(DATA_FORMATS_FOLLOW == manager.state);
    navigation_step(&manager, MOTION_FOLLOW, false, true, false, false, 100u,
                    true, false, false, false, true);
    assert(DATA_FORMATS_FAULT == manager.state);
    assert(0u != manager.b_stop_requested);
    assert(navigation_supervise(&manager, DATA_FORMATS_STOP, true));
    assert(0u == manager.b_command_ready);
}

/** @brief A moving action cannot be replaced, and hazard combinations stop. */
static void verify_interlocks (void)
{
    motion_t controller = {0};
    motion_init(&controller, 0u, 0u);
    uint32_t follow = 0u;
    uint32_t move   = 0u;
    assert(motion_command(&follow, MOTION_FOLLOW, 0u, 500u, false));
    assert(motion_command(&move, MOTION_MOVE, 100u, 250u, false));
    uint8_t message[3] = {0u};
    assert(data_formats_write(message, 0u, 24u, move));
    assert(move == data_formats_read(message, 0u, 24u));
    assert(!motion_command(&follow, MOTION_FOLLOW, 0u, 501u, false));
    assert(motion_accept(&controller, follow));
    assert(motion_accept(&controller, move));
    assert(!motion_accept(&controller, follow));
    assert(motion_accept(&controller, 0u));
    assert(MOTION_IDLE == controller.mode);
    navigation_t manager = {0};
    navigation_init(&manager);
    assert(navigation_supervise(&manager, DATA_FORMATS_START, false));
    navigation_step(&manager, MOTION_FOLLOW, false, true, false, false, 100u,
                    true, false, false, true, false);
    assert(250u == ((manager.command >> 14u) & 511u));
    navigation_step(&manager, MOTION_FOLLOW, false, true, false, true, 30u,
                    true, false, false, false, false);
    assert(DATA_FORMATS_FAULT == manager.state);
    assert((manager.b_estop) && (manager.b_stop_requested));
    navigation_init(&manager);
    manager.state      = DATA_FORMATS_SEARCH;
    manager.elapsed_ms = 2990u;
    navigation_step(&manager, MOTION_TURN, false, false, true, false, 100u,
                    true, false, false, false, false);
    assert(manager.b_fault);
}

/** @brief Zero bytes survive the transport and oversized messages are
 * rejected.
 */
static void verify_fragments (void)
{
    telemetry_t receiver = {0};
    uint8_t     payload  = 0u;
    telemetry_begin(&receiver, 1u, true);
    assert(!telemetry_fragment(&receiver, NULL, 0u, false));
    assert(telemetry_fragment(&receiver, &payload, 1u, true));
    assert(0u == receiver.payload);
    assert(!telemetry_fragment(&receiver, &payload, 1u, true));
    telemetry_begin(&receiver, 2u, true);
    assert(!telemetry_fragment(&receiver, &payload, 1u, true));
    telemetry_begin(&receiver, 1u, false);
    assert(!telemetry_fragment(&receiver, &payload, 1u, true));
    telemetry_begin(&receiver, 1u, true);
    payload = 255u;
    assert(!telemetry_fragment(&receiver, &payload, 1u, true));
}

/** @brief Tracking sensors remain on-line while the right lane changes. */
static void line_sample (line_t * p_line, bool b_black, uint16_t distance)
{
    uint16_t samples[ROBOT_IR_COUNT] = {3000u, 1000u, 0u};
    samples[2]                       = b_black ? 3000u : 0u;
    line_step(p_line, samples, distance & 4095u, false);
}

/** @brief Constant synthetic edge rates exercise the physical motion
 * controller. */
static void motion_ticks (motion_t * p_motion, uint32_t ticks,
                         uint32_t increment, bool b_turn)
{
    uint32_t left  = p_motion->last_left;
    uint32_t right = p_motion->last_right;

    for (uint32_t tick = 0u; tick < ticks; ++tick)
    {
        left += increment;
        right             = b_turn ? (right - increment) : (right + increment);
        int16_t left_pwm  = 0;
        int16_t right_pwm = 0;
        motion_step(p_motion, left, right, 0, false, &left_pwm, &right_pwm);
        assert((left_pwm >= -1000) && (left_pwm <= 1000));
        assert((right_pwm >= -1000) && (right_pwm <= 1000));
    }
}

/*** End of file. ***/

