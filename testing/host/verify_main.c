/** @file verify_main.c
 * @brief Behavioral verification for packed and widened reference builds.
 */
#include "verify_main.h"
#include "motor_test.h"
#include "terrain_test.h"

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
#define VERIFY_BARCODE_SYMBOLS 4u
#define VERIFY_LINE_CALIBRATION 32u
#define VERIFY_LOST_SAMPLES 31u
#define VERIFY_SCAN_POINTS OBSTACLE_SCAN_POINTS
#define VERIFY_BYPASS_CYCLES 3000u
#define VERIFY_CONFIRMATIONS 3u
#define VERIFY_FIRST_SAMPLE 1u

static void verify_fields(void);
static void verify_vector(void);
static void verify_motion(void);
static void verify_safety(void);
static void verify_line(void);
static void verify_line_startup(void);
static void verify_bad_barcode(void);
static void verify_obstacle(void);
static void verify_range_restart(void);
static void verify_width_route(void);
static void verify_retry_route(void);
static void verify_body_fallback(void);
static void verify_body_interrupts(void);
static void verify_body_rejections(void);
static void body_sweep(navigation_t * p_manager, const uint16_t * p_ranges);
static void verify_long_side_move(navigation_t * p_manager);
static void retry_tick(navigation_t * p_manager, bool b_done,
                       uint16_t range_cm);
static void verify_scan_edges(void);
static void verify_blocked_width_route(void);
static void verify_settle(navigation_t * p_manager);
static void retry_until_command(navigation_t * p_manager, uint16_t range_cm);
static void verify_navigation(void);
static void verify_bypass_run(navigation_t * p_manager, bool b_clears);
static void verify_return_on_tape(void);
static void verify_side_extra(void);
static void verify_fragments(void);
static void verify_interlocks(void);
static void line_sample(line_t * p_line, bool b_black, uint16_t distance);
static void motion_ticks(motion_t * p_motion, uint32_t ticks,
                         uint32_t increment, bool b_turn);

/** @brief Assertions report the first reproducible violated contract. */
int main(void)
{
#if OBSTACLE_TEST
    verify_body_fallback();
    verify_body_interrupts();
    verify_body_rejections();
    verify_retry_route();
    (void)puts("PASS: 70-mm/s body scan, bounded lateral legs and retries");
    return 0;
#endif /* OBSTACLE_TEST */
    verify_fields();
    verify_vector();
    verify_motion();
    motor_test_run();
    verify_safety();
    terrain_test_run();
    verify_line();
    verify_line_startup();
    verify_bad_barcode();
    verify_obstacle();
    verify_range_restart();
    verify_width_route();
    verify_retry_route();
    verify_body_fallback();
    verify_body_interrupts();
    verify_body_rejections();
    verify_scan_edges();
    verify_blocked_width_route();
    verify_navigation();
    verify_return_on_tape();
    verify_side_extra();
    verify_fragments();
    verify_interlocks();
    (void)puts("PASS: fields, motion, safety, terrain, line, obstacle, "
               "navigation, MQTT");
    return 0;
}

/** @brief The host decoder independently checks this firmware-format vector.
 */
static void verify_vector(void)
{
    uint8_t  bytes[DATA_FORMATS_TELEMETRY_BYTES] = {0u};
    uint32_t word                                = 0u;
    assert(data_formats_motion_pack(&word, 4095u, 500u, true, 3u, 127u));
    assert(data_formats_write(bytes, 0u, 3u, 2u));
    assert(data_formats_write(bytes, 3u, 32u, word));
    assert(data_formats_write(bytes, 35u, 12u, 0x123u));
    assert(data_formats_write(bytes, 47u, 12u, 0u));
    assert(data_formats_write(bytes, 59u, 12u, 2048u));
    assert(data_formats_write(bytes, 71u, 12u, 4095u));
    assert(data_formats_write(bytes, 83u, 3u, 4u));
    assert(data_formats_write(bytes, 86u, 7u, 54u));
    assert(data_formats_write(bytes, 93u, 8u, 43u));
    assert(data_formats_write(bytes, 101u, 9u, 400u));
    assert(data_formats_write(bytes, 110u, 6u, 50u));
    assert(data_formats_write(bytes, 116u, 3u, 7u));
    assert(data_formats_write(bytes, 119u, 8u, 0u));
    assert(data_formats_write(bytes, 127u, 3u, 4u));
    assert(data_formats_write(bytes, 130u, 3u, 2u));
    assert(data_formats_write(bytes, 133u, 2u, 3u));
    assert(data_formats_write(bytes, 135u, 1u, 1u));
    assert(data_formats_write(bytes, 136u, 1u, 1u));
    assert(data_formats_write(bytes, 137u, 32u, UINT32_MAX));
    assert(data_formats_write(bytes, 169u, 32u, 2u));
    assert(data_formats_write(bytes, 201u, 10u, 12u));
    assert(data_formats_write(bytes, 211u, 10u, 1012u));
    (void)printf("VECTOR ");

    for (uint8_t index = 0u; index < DATA_FORMATS_TELEMETRY_BYTES; ++index)
    {
        (void)printf("%02x", bytes[index]);
    }

    (void)printf("\n");
}

/** @brief Exercise cross-byte fields and preserve every neighboring bit. */
static void verify_fields(void)
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
static void verify_motion(void)
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
    controller.integral_left  = 0u;
    controller.integral_right = 0u;
    controller.prior_left     = 0u;
    controller.prior_right    = 0u;
    assert(motion_command(&command, MOTION_MOVE, 100u, 250u, true));
    assert(motion_accept(&controller, command));
    assert(controller.integral_left == 16000u);
    assert(controller.integral_right == 16000u);
    assert(controller.prior_left == 1000u);
    assert(controller.prior_right == 1000u);
    assert(controller.pwm_left == 0);
    assert(controller.pwm_right == 0);
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
static void verify_safety(void)
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

/** @brief All four framed barcodes decode once with encoder-measured widths.
 */
static void verify_line(void)
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

/** @brief Black tape at boot must not skip the 32 calibration samples. */
static void verify_line_startup(void)
{
    line_t detector = {0};
    uint16_t black[ROBOT_IR_COUNT] = {4095u, 4095u, 4095u};
    line_init(&detector);

    for (uint8_t sample = 1u; sample < VERIFY_LINE_CALIBRATION; ++sample)
    {
        line_step(&detector, black, 0u, false);
        assert(LINE_CALIBRATING == detector.state);
        assert(sample == detector.calibration_samples);
        assert(0u == detector.junction_ms);
    }

    line_step(&detector, black, 0u, false);
    assert(LINE_TRACKING == detector.state);
    assert(VERIFY_LINE_CALIBRATION == detector.calibration_samples);
    line_step(&detector, black, 0u, false);
    assert(LINE_JUNCTION == detector.state);
}

/** @brief Ambiguous bars and reversing cannot generate a command. */
static void verify_bad_barcode(void)
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
static void verify_obstacle(void)
{
    obstacle_t scanner = {0};
    obstacle_init(&scanner);
    assert(obstacle_scan(&scanner));
    assert(!obstacle_scan(&scanner));
    for (uint8_t index = 0u; index < VERIFY_SCAN_POINTS; ++index)
    {
        uint16_t range = ((index >= 6u) && (index <= 14u)) ? 26u : 120u;
        range          = (10u == index) ? 25u : range;
        range          = (0u == index) ? 400u : range;
        obstacle_sample(&scanner, range, true);
    }

    assert(OBSTACLE_PLAN == scanner.state);
    assert(0u != scanner.b_left);
    assert(0u != scanner.b_ready);
    assert(134u == scanner.width_mm);
    assert(297u == scanner.side_mm);
    assert(400u == scanner.left_cm);
    assert(120u == scanner.right_cm);
    obstacle_init(&scanner);
    assert(obstacle_scan(&scanner));

    for (uint8_t index = 0u; index < VERIFY_SCAN_POINTS; ++index)
    {
        obstacle_sample(&scanner, (20u == index) ? 27u : 25u, true);
    }

    /* Both edges hidden by the beam (the 6 Nov 22-cm box): no body scan.
     * Go out to the visible face (25 cm x tan 30 = 144 mm) + 230 mm on the
     * more open side; the bypass then steps out until ahead is clear. */
    assert(OBSTACLE_PLAN == scanner.state);
    assert(scanner.b_ready);
    assert(!scanner.b_left);
    assert(0u == scanner.width_mm);
    assert(374u == scanner.side_mm);
    obstacle_init(&scanner);
    assert(obstacle_scan(&scanner));

    for (uint8_t index = 0u; index < VERIFY_SCAN_POINTS; ++index)
    {
        /* Under 10 cm on the far left is the robot itself: ignored. */
        uint16_t range = (index <= 2u) ? 6u : 25u;
        range          = (index >= 18u) ? 28u : range;
        obstacle_sample(&scanner, range, true);
    }

    assert(OBSTACLE_PLAN == scanner.state);
    assert(25u == scanner.left_cm);
    assert(28u == scanner.right_cm);
    assert(!scanner.b_left);
    assert(100u == obstacle_median(400u, 100u, 2u));
    obstacle_init(&scanner);
    assert(obstacle_scan(&scanner));
    obstacle_sample(&scanner, 0u, false);
    obstacle_sample(&scanner, 0u, false);
    obstacle_sample(&scanner, 0u, false);
    assert(OBSTACLE_ABORT == scanner.state);
    obstacle_init(&scanner);
    assert(obstacle_scan(&scanner));
    obstacle_tick(&scanner, ROBOT_SCAN_MS);
    assert(OBSTACLE_ABORT == scanner.state);
}

/** @brief Asymmetric edges and right-hand clearance produce a right plan. */
static void verify_scan_edges(void)
{
    obstacle_t scanner = {0};
    obstacle_init(&scanner);
    assert(obstacle_scan(&scanner));

    for (uint8_t index = 0u; index < VERIFY_SCAN_POINTS; ++index)
    {
        uint16_t range = ((index >= 7u) && (index <= 16u)) ? 26u : 60u;
        range          = (10u == index) ? 25u : range;
        range          = (20u == index) ? 400u : range;
        obstacle_sample(&scanner, range, true);
    }

    assert(OBSTACLE_PLAN == scanner.state);
    /* Left needs less travel (54 mm edge) but 60 cm at 30 degrees leaves
     * only 300 mm of room; the right side has room for its 96 mm edge. */
    assert(150u == scanner.width_mm);
    assert(!scanner.b_left);
    assert(326u == scanner.side_mm);
    assert(60u == scanner.left_cm);
    assert(400u == scanner.right_cm);
    obstacle_init(&scanner);
    assert(obstacle_scan(&scanner));

    for (uint8_t index = 0u; index < VERIFY_SCAN_POINTS; ++index)
    {
        obstacle_sample(&scanner, 400u, true);
    }

    assert(OBSTACLE_ABORT == scanner.state);
    assert(!scanner.b_ready);
}

/** @brief A newly seen side obstruction prevents the lateral move. */
static void verify_blocked_width_route(void)
{
    navigation_t manager = {0};
    navigation_init(&manager);
    assert(navigation_supervise(&manager, DATA_FORMATS_START, false));
    navigation_step(&manager, MOTION_FOLLOW, false, true, false, false, 25u,
                    true, false, false, false, false);
    assert(navigation_obstacle_plan(&manager, 162u, 400u, 120u));
    navigation_step(&manager, MOTION_IDLE, false, false, false, false, 400u,
                    true, true, true, false, false);
    verify_settle(&manager);
    assert(MOTION_TURN == (manager.command & 3u));
    navigation_step(&manager, MOTION_IDLE, true, false, false, false, 20u,
                    true, false, false, false, false);

    for (uint8_t tick = 0u; tick < 40u; ++tick)
    {
        navigation_step(&manager, MOTION_IDLE, false, false, false, false, 20u,
                        true, false, false, false, false);
    }

    assert(manager.b_fault);
    assert(manager.b_stop_requested);
    assert(MOTION_TURN == (manager.command & 3u));
}

/** @brief Out-of-range batches survive scan reset and allow a fresh START. */
static void verify_range_restart(void)
{
    const uint16_t empty[3]  = {0u, 0u, 0u};
    const uint16_t echoes[3] = {90u, 25u, 60u};
    assert(ROBOT_RANGE_MAX_CM == obstacle_range(empty, 0u));
    assert(90u == obstacle_range(echoes, 1u));
    assert(25u == obstacle_range(echoes, 2u));
    assert(60u == obstacle_range(echoes, 3u));
    const uint16_t short_echoes[3] = {1u, 1u, 1u};
    assert(1u == obstacle_range(short_echoes, 3u));

    obstacle_t   scanner = {0};
    navigation_t manager = {0};
    obstacle_init(&scanner);
    navigation_init(&manager);

    for (uint8_t attempt = 0u; attempt < 2u; ++attempt)
    {
        uint16_t range = obstacle_range(empty, 0u);
        obstacle_sample(&scanner, range, true);
        assert(ROBOT_RANGE_MAX_CM == scanner.range_cm);
        assert(navigation_supervise(&manager, DATA_FORMATS_START, false));
        navigation_step(&manager, MOTION_IDLE, false, true, false, false,
                        range, true, false, false, false, false);
        assert(DATA_FORMATS_FOLLOW == manager.state);
        assert(!manager.b_fault);

        /* A real near echo must still stop following and request a scan. */
        range = obstacle_range(echoes, 2u);
        navigation_step(&manager, MOTION_FOLLOW, false, true, false, false,
                        range, true, false, false, false, false);
        assert(DATA_FORMATS_AVOID == manager.state);
        assert(manager.b_stop_requested);
        assert(!manager.b_command_ready);
        assert(manager.b_scan_requested);
        assert(obstacle_scan(&scanner));
        obstacle_init(&scanner);
        assert(30u == scanner.angle);
        assert(OBSTACLE_IDLE == scanner.state);
        assert(navigation_supervise(&manager, DATA_FORMATS_STOP, false));
        assert(navigation_supervise(&manager, DATA_FORMATS_RESET, false));
        navigation_init(&manager);
    }
}

/** @brief Left and right routes use full width then the saved front range. */
static void verify_width_route(void)
{
    for (uint8_t side = 0u; side < 2u; ++side)
    {
        bool         b_left  = (0u == side);
        navigation_t manager = {0};
        navigation_init(&manager);
        assert(navigation_supervise(&manager, DATA_FORMATS_START, false));
        navigation_step(&manager, MOTION_FOLLOW, false, true, false, false,
                        25u, true, false, false, false, false);
        assert(250u == manager.detected_mm);

        for (uint16_t tick = 0u; tick < 600u; ++tick)
        {
            navigation_step(&manager, MOTION_IDLE, false, false, false, false,
                            400u, true, false, false, false, false);
            assert(!manager.b_command_ready);
            assert(!manager.b_fault);
        }

        assert(!navigation_obstacle_plan(&manager, 0u, 400u, 400u));
        assert(!navigation_obstacle_plan(&manager, 5000u, 400u, 400u));
        assert(!navigation_obstacle_plan(&manager, 162u, 20u, 20u));
        assert(navigation_obstacle_plan(&manager, 162u, b_left ? 400u : 120u,
                                        b_left ? 120u : 400u));
        assert(b_left == manager.b_left);
        /* A centred 162-mm box: 81 mm edge + 180 turn radius + 50 margin. */
        assert(311u == manager.side_mm);
        navigation_step(&manager, MOTION_IDLE, false, false, false, false,
                        100u, true, true, b_left, false, false);
        verify_settle(&manager);
        uint32_t turn    = manager.command;
        uint32_t quarter = ROBOT_QUARTER_TURN_MM;
        assert(MOTION_TURN == (turn & 3u));
        assert(quarter == ((turn >> 2u) & 4095u));
        assert(b_left == (0u != (turn >> 23u)));
        navigation_step(&manager, MOTION_IDLE, true, false, false, false, 100u,
                        true, false, false, false, false);
        verify_settle(&manager);
        assert(MOTION_MOVE == (manager.command & 3u));
        assert(311u == ((manager.command >> 2u) & 4095u));
        navigation_step(&manager, MOTION_IDLE, true, false, false, false, 100u,
                        true, false, false, false, false);
        verify_settle(&manager);
        assert(MOTION_TURN == (manager.command & 3u));
        assert(quarter == ((manager.command >> 2u) & 4095u));
        assert(b_left != (0u != (manager.command >> 23u)));
        navigation_step(&manager, MOTION_IDLE, true, false, false, false, 100u,
                        true, false, false, false, false);
        verify_settle(&manager);
        /* The settled check saw clear space ahead: forward stage next. */
        assert(6u == manager.stage);
        navigation_step(&manager, MOTION_IDLE, false, false, false, false,
                        100u, true, false, false, false, false);
        assert(MOTION_MOVE == (manager.command & 3u));
        assert((250u + ROBOT_LENGTH_MM) == ((manager.command >> 2u) & 4095u));
        assert(!manager.b_fault);
    }
}

/** @brief Emulate turn duration, actual heading, and settled body readings. */
static void body_sweep(navigation_t * p_manager, const uint16_t * p_ranges)
{
    int32_t  heading = 0;
    int32_t  turn    = 0;
    uint32_t ticks   = 0u;
    uint32_t turns   = 0u;

    for (uint32_t tick = 0u;
         (tick < 3000u) && (0u == p_manager->stage) && (!p_manager->b_fault);
         ++tick)
    {
        bool b_done = false;
        if (ticks > 0u)
        {
            --ticks;
            if (0u == ticks)
            {
                heading += turn;
                b_done = true;
            }
        }

        uint16_t range = 400u;
        if (2u == p_manager->look)
        {
            int32_t target =
                (int32_t)((p_manager->body_index * ROBOT_QUARTER_TURN_MM) /
                          6u);
            target -= (int32_t)ROBOT_QUARTER_TURN_MM;
            assert(target == heading);
            range = p_ranges[p_manager->body_index];
        }

        p_manager->b_command_ready = false;
        navigation_step(p_manager, MOTION_IDLE, b_done, false, false, false,
                        range, true, false, false, false, false);
        if ((p_manager->b_command_ready) && (0u == p_manager->stage))
        {
            assert(0u == ticks);
            assert(MOTION_TURN == (p_manager->command & 3u));
            uint32_t distance = (p_manager->command >> 2u) & 4095u;
            turn              = (int32_t)distance;
            turn  = (0u != (p_manager->command >> 23u)) ? -turn : turn;
            ticks = ((distance * 100u) + ROBOT_TURN_MM_SECOND - 1u) /
                    ROBOT_TURN_MM_SECOND;
            ++turns;
        }
    }

    assert(0 == heading);
    assert(14u == turns);
}

/** @brief Large estimated widths split into bounded, checked lateral legs. */
static void verify_long_side_move(navigation_t * p_manager)
{
    uint32_t total = p_manager->side_mm;
    uint32_t moved = 0u;
    /* The turn out waits for the robot to settle, then finishes. */
    verify_settle(p_manager);
    assert(MOTION_TURN == (p_manager->command & 3u));
    assert(p_manager->b_left == (0u != (p_manager->command >> 23u)));
    navigation_step(p_manager, MOTION_IDLE, true, false, false, false, 400u,
                    true, false, false, false, false);

    while (2u == p_manager->stage)
    {
        verify_settle(p_manager);
        assert(!p_manager->b_fault);
        assert(MOTION_MOVE == (p_manager->command & 3u));
        uint32_t distance = (p_manager->command >> 2u) & 4095u;
        assert(distance <= ((ROBOT_TURN_MM_SECOND * 8000u) / 1000u));
        assert(distance > 0u);
        moved += distance;
        navigation_step(p_manager, MOTION_IDLE, true, false, false, false,
                        400u, true, false, false, false, false);
    }

    assert(total == moved);
    assert(total == p_manager->side_mm);
    assert(0u == p_manager->side_remaining_mm);
    assert(3u == p_manager->stage);
}

/** @brief Reproduce the flat 30-cm scan, then recover with wider body rays. */
static void verify_body_fallback(void)
{
    const uint16_t wide[13] = {400u, 400u, 50u, 35u, 29u,  26u, 25u,
                               26u,  29u,  35u, 50u, 400u, 350u};
    assert(1868u == obstacle_body_width(wide));
    navigation_t manager = {0};
    navigation_init(&manager);
    assert(navigation_supervise(&manager, DATA_FORMATS_START, false));
    navigation_step(&manager, MOTION_FOLLOW, false, true, false, false, 25u,
                    true, false, false, false, false);
    assert(navigation_body_scan(&manager));
    assert(!navigation_body_scan(&manager));
    body_sweep(&manager, wide);
    assert(!manager.b_fault);
    assert(1u == manager.stage);
    assert(manager.b_left);
    assert(1868u == manager.width_mm);
    /* Centred box: 934 mm edge + 180 turn radius + 50 margin. */
    assert(1164u == manager.side_mm);
    assert(250u == manager.detected_mm);
    verify_long_side_move(&manager);

    uint16_t mirrored[13] = {0u};
    for (uint8_t index = 0u; index < 13u; ++index)
    {
        mirrored[index] = wide[12u - index];
    }

    navigation_init(&manager);
    manager.state       = DATA_FORMATS_AVOID;
    manager.detected_mm = 250u;
    assert(navigation_body_scan(&manager));
    body_sweep(&manager, mirrored);
    assert(!manager.b_fault);
    assert(!manager.b_left);
    verify_long_side_move(&manager);
    mirrored[3] = 0u;
    assert(0u == obstacle_body_width(mirrored));

    uint16_t flat[13] = {0u};
    for (uint8_t index = 0u; index < 13u; ++index)
    {
        flat[index] = 25u;
    }

    assert(0u == obstacle_body_width(flat));
    navigation_init(&manager);
    manager.state = DATA_FORMATS_AVOID;
    assert(navigation_body_scan(&manager));
    body_sweep(&manager, flat);
    assert(manager.b_fault);
    assert(manager.b_stop_requested);
    assert(OBSTACLE_REASON_NO_EDGES == manager.scan_failure);
    assert(0u == manager.side_mm);
}

/** @brief STOP and safety preempt a body sweep without replaying commands. */
static void verify_body_interrupts(void)
{
    navigation_t manager = {0};
    navigation_init(&manager);
    manager.state = DATA_FORMATS_AVOID;
    assert(navigation_body_scan(&manager));
    navigation_step(&manager, MOTION_IDLE, false, false, false, false, 400u,
                    true, false, false, false, false);
    assert(manager.b_command_ready);
    assert(navigation_supervise(&manager, DATA_FORMATS_STOP, false));
    navigation_step(&manager, MOTION_IDLE, true, false, false, false, 400u,
                    true, false, false, false, false);
    assert(DATA_FORMATS_IDLE == manager.state);
    assert(!manager.b_command_ready);
    assert(manager.b_stop_requested);
    manager.state           = DATA_FORMATS_AVOID;
    manager.b_command_ready = true;
    navigation_step(&manager, MOTION_TURN, false, false, false, false, 400u,
                    true, false, false, false, true);
    assert(DATA_FORMATS_FAULT == manager.state);
    assert(!manager.b_command_ready);
    assert(manager.b_stop_requested);
}

/** @brief Body fallback must not invent a route when side space is blocked. */
static void verify_body_rejections(void)
{
    uint16_t     blocked[13] = {100u, 400u, 50u, 35u, 29u,  26u, 25u,
                                26u,  29u,  35u, 50u, 400u, 100u};
    navigation_t manager     = {0};
    navigation_init(&manager);
    manager.state = DATA_FORMATS_AVOID;
    assert(navigation_body_scan(&manager));
    body_sweep(&manager, blocked);
    assert(manager.b_fault);
    assert(OBSTACLE_REASON_BLOCKED == manager.scan_failure);
    assert(0u == manager.side_mm);
    navigation_step(&manager, MOTION_IDLE, false, false, false, false, 400u,
                    true, false, false, false, true);
    assert(navigation_supervise(&manager, DATA_FORMATS_RESET, true));
    assert(manager.b_reset_requested);
    navigation_init(&manager);
    assert(!manager.b_body_scan);
    assert(0u == manager.width_mm);
    blocked[6]    = 400u;
    manager.state = DATA_FORMATS_AVOID;
    assert(navigation_body_scan(&manager));
    body_sweep(&manager, blocked);
    assert(manager.b_fault);
    assert(OBSTACLE_REASON_NO_FRONT == manager.scan_failure);
    assert(0u == manager.side_mm);
}

/** @brief Mimic dispatch consuming each command between navigation ticks. */
static void retry_tick(navigation_t * p_manager, bool b_done,
                       uint16_t range_cm)
{
    p_manager->b_command_ready = false;
    navigation_step(p_manager, MOTION_IDLE, b_done, false, false, false,
                    range_cm, true, false, false, false, false);
}

/** @brief Tick with a fixed range until a command appears (or 60 ticks). */
static void retry_until_command(navigation_t * p_manager, uint16_t range_cm)
{
    for (uint8_t tick = 0u; (tick < 60u) && (!p_manager->b_command_ready);
         ++tick)
    {
        navigation_step(p_manager, MOTION_IDLE, false, false, false, false,
                        range_cm, true, false, false, false, false);
    }
}

/** @brief Both directions repeat: face line, blocked look, face ahead,
 * fixed step; every action waits for the robot to settle first.
 */
static void verify_retry_route(void)
{
    for (uint8_t side = 0u; side < 2u; ++side)
    {
        navigation_t manager = {0};
        navigation_init(&manager);
        manager.state     = DATA_FORMATS_AVOID;
        manager.stage     = 6u;
        manager.side_mm   = 312u;
        manager.b_left    = (0u == side);
        manager.b_waiting = true;

        for (uint8_t retry = 0u; retry < 3u; ++retry)
        {
            /* The forward step finished: no turn until the robot is still. */
            retry_tick(&manager, true, 400u);
            assert(!manager.b_command_ready);
            retry_until_command(&manager, 400u);
            assert(MOTION_TURN == (manager.command & 3u));
            assert(ROBOT_QUARTER_TURN_MM == ((manager.command >> 2u) & 4095u));
            assert(manager.b_left != (0u != (manager.command >> 23u)));
            retry_tick(&manager, true, 15u);
            /* Facing the line, the box is 15 cm away: still beside. */
            retry_until_command(&manager, 15u);
            assert((retry + 1u) == manager.steps);
            assert(!manager.b_margin);
            assert(MOTION_TURN == (manager.command & 3u));
            assert(manager.b_left == (0u != (manager.command >> 23u)));
            retry_tick(&manager, true, 400u);
            retry_until_command(&manager, 400u);
            assert(MOTION_MOVE == (manager.command & 3u));
            assert(((ROBOT_TURN_MM_SECOND * ROBOT_BYPASS_STEP_MS) / 1000u) ==
                   ((manager.command >> 2u) & 4095u));
            assert(!manager.b_fault);
            /* A blocked recheck must cancel a previously requested margin. */
            manager.b_margin = true;
        }
    }
}

/** @brief Supply fresh clear readings while the body settles after a turn. */
static void verify_settle(navigation_t * p_manager)
{
    for (uint8_t tick = 0u; tick < 40u; ++tick)
    {
        navigation_step(p_manager, MOTION_IDLE, false, false, false, false,
                        400u, true, false, false, false, false);
    }
}

/** @brief Barcode, obstacle bypass, recovery and safety arbitration interact.
 */
static void verify_navigation(void)
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
    navigation_step(&manager, MOTION_FOLLOW, false, true, false, false, 25u,
                    true, false, false, false, false);
    assert(DATA_FORMATS_AVOID == manager.state);
    assert(0u != manager.b_stop_requested);
    assert(navigation_obstacle_plan(&manager, 162u, 400u, 120u));
    navigation_step(&manager, MOTION_IDLE, false, false, false, false, 100u,
                    true, true, true, false, false);
    assert(1u == manager.stage);
    verify_bypass_run(&manager, true);
    /* No tape in this run: the whole return leg, then the line sweep. */
    assert(DATA_FORMATS_SEARCH == manager.state);
    assert(!manager.b_fault);
    assert(0u == manager.side_remaining_mm);

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
static void verify_interlocks(void)
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
    navigation_step(&manager, MOTION_FOLLOW, false, true, false, true, 25u,
                    true, false, false, false, false);
    assert(DATA_FORMATS_FAULT == manager.state);
    assert((manager.b_estop) && (manager.b_stop_requested));
    navigation_init(&manager);
    assert(navigation_supervise(&manager, DATA_FORMATS_START, false));
    navigation_step(&manager, MOTION_FOLLOW, false, false, true, false, 100u,
                    true, false, false, false, false);
    assert(DATA_FORMATS_SEARCH == manager.state);
    assert(MOTION_TURN == (manager.command & 3u));
    assert(0u != (manager.command >> 23u));
    uint32_t quarter = (manager.command >> 2u) & 4095u;
    navigation_step(&manager, MOTION_IDLE, true, false, false, false, 100u,
                    true, false, false, false, false);
    assert(1u == manager.stage);
    assert(0u == (manager.command >> 23u));
    assert((quarter * 2u) == ((manager.command >> 2u) & 4095u));
    assert(!manager.b_fault);
    navigation_step(&manager, MOTION_IDLE, true, false, false, false, 100u,
                    true, false, false, false, false);
    assert(manager.b_fault);
    navigation_init(&manager);
    manager.state      = DATA_FORMATS_SEARCH;
    manager.elapsed_ms = 2990u;
    navigation_step(&manager, MOTION_TURN, false, false, true, false, 100u,
                    true, false, false, false, false);
    assert(manager.b_fault);
    navigation_init(&manager);
    assert(navigation_supervise(&manager, DATA_FORMATS_START, false));
    navigation_step(&manager, MOTION_FOLLOW, false, true, false, false, 25u,
                    true, false, false, false, false);
    assert(navigation_obstacle_plan(&manager, 162u, 120u, 400u));
    navigation_step(&manager, MOTION_IDLE, false, false, false, false, 100u,
                    true, true, false, false, false);
    verify_bypass_run(&manager, false);
    assert(manager.b_fault);
}

/** @brief Tape seen on the return leg: stop, roll the axle onto the line,
 * face ahead and follow, with no sweep.
 */
static void verify_return_on_tape(void)
{
    navigation_t manager = {0};
    navigation_init(&manager);
    manager.state             = DATA_FORMATS_AVOID;
    manager.stage             = 10u;
    manager.side_mm           = 300u;
    manager.side_remaining_mm = 450u;
    manager.b_left            = true;
    manager.b_waiting         = true;
    navigation_step(&manager, MOTION_MOVE, false, true, false, false, 400u,
                    true, false, false, false, false);
    assert(11u == manager.stage);
    assert(manager.b_stop_requested);
    assert(manager.b_line_seen);
    manager.b_stop_requested = false;
    verify_settle(&manager);
    assert(MOTION_MOVE == (manager.command & 3u));
    assert(ROBOT_SENSOR_AHEAD_MM == ((manager.command >> 2u) & 4095u));
    navigation_step(&manager, MOTION_IDLE, true, true, false, false, 400u,
                    true, false, false, false, false);
    verify_settle(&manager);
    /* Facing the line with S on the left: a left turn faces ahead. */
    assert(MOTION_TURN == (manager.command & 3u));
    assert(0u != (manager.command >> 23u));
    navigation_step(&manager, MOTION_IDLE, true, true, false, false, 400u,
                    true, false, false, false, false);
    assert(DATA_FORMATS_FOLLOW == manager.state);
    assert(MOTION_FOLLOW == (manager.command & 3u));
    assert(!manager.b_fault);
}

/** @brief Box still ahead after the side move: one extra sideways step. */
static void verify_side_extra(void)
{
    navigation_t manager = {0};
    navigation_init(&manager);
    manager.state       = DATA_FORMATS_AVOID;
    manager.stage       = 4u;
    manager.side_mm     = 217u;
    manager.detected_mm = 250u;
    manager.b_left      = false;
    verify_settle(&manager);
    assert(6u == manager.stage);
    manager.stage      = 4u;
    manager.elapsed_ms = 0u;

    for (uint8_t tick = 0u; tick < 40u; ++tick)
    {
        navigation_step(&manager, MOTION_IDLE, false, false, false, false, 26u,
                        true, false, false, false, false);
    }

    assert(5u == manager.stage);
    assert((217u + ROBOT_BYPASS_SIDE_EXTRA_MM) == manager.side_mm);
    assert(1u == manager.steps);
    navigation_step(&manager, MOTION_IDLE, false, false, false, false, 400u,
                    true, false, false, false, false);
    assert(MOTION_TURN == (manager.command & 3u));
    assert(0u == (manager.command >> 23u));
    navigation_step(&manager, MOTION_IDLE, true, false, false, false, 400u,
                    true, false, false, false, false);
    verify_settle(&manager);
    assert(MOTION_MOVE == (manager.command & 3u));
    assert(ROBOT_BYPASS_SIDE_EXTRA_MM == ((manager.command >> 2u) & 4095u));
}

/** @brief Drive the step-and-check bypass. When b_clears, the obstacle stays
 * ahead for two sideways looks and beside for two forward looks, then clears;
 * otherwise it never clears and the step limit must fault.
 */
static void verify_bypass_run(navigation_t * p_manager, bool b_clears)
{
    for (uint32_t cycle = 0u;
         (cycle < VERIFY_BYPASS_CYCLES) &&
         (DATA_FORMATS_AVOID == p_manager->state) && (!p_manager->b_fault);
         ++cycle)
    {
        bool     b_side  = (p_manager->stage < 6u);
        bool     b_there = (!b_clears) || (p_manager->steps < 2u);
        uint16_t range   = b_side ? 400u : 200u;
        range            = ((8u == p_manager->stage) && b_there) ? 15u : range;
        navigation_step(p_manager, MOTION_IDLE, p_manager->b_waiting, false,
                        false, false, range, true, false, true, false, false);
    }
}

/** @brief Zero bytes survive the transport and oversized messages are
 * rejected.
 */
static void verify_fragments(void)
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
static void line_sample(line_t * p_line, bool b_black, uint16_t distance)
{
    uint16_t samples[ROBOT_IR_COUNT] = {3000u, 1000u, 0u};
    samples[2]                       = b_black ? 3000u : 0u;
    line_step(p_line, samples, distance & 4095u, false);
}

/** @brief Constant synthetic edge rates exercise the physical motion
 * controller. */
static void motion_ticks(motion_t * p_motion, uint32_t ticks,
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

