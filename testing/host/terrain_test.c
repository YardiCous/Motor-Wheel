/** @file terrain_test.c
 * @brief Host tests for terrain.c, driven by a small physics simulator.
 *
 * The simulator produces signals that agree with each other: the forward
 * accelerometer axis reads g*sin(pitch) + the car's own acceleration, and
 * Motion's speed comes from a quantised encoder exactly as motion_cycle()
 * computes it (about 10.6 mm/s per edge in 10 ms). Angles are given as
 * sin x 1000 and cos x 1000, so no maths library is needed.
 */
#include "terrain_test.h"

#include "../../app_program/data_formats.h"
#include "../../app_program/subsystem4/terrain.h"
#include "../../app_program/subsystem4/terrain_config.h"

#include <assert.h>
#include <stdint.h>
#include <stdio.h>

/* Encoder: 102100 um of centre travel per 960 summed edges (motion.c). */
#define SIM_UM_PER_960_EDGES 102100
#define SIM_EDGES 960
#define SIM_SPEED_NUMERATOR 10210
#define SIM_SPEED_LIMIT 511
#define SIM_ODOMETER_MM 4096
#define SIM_UM_PER_MM 1000
#define SIM_STEP_MS 10
/* dv mm/s per 10 ms to milli-g: dv * 100 * 1000 / 9807. */
#define SIM_ACCEL_NUMERATOR 100000
#define SIM_GRAVITY_MM_S2 9807
/* sin and cos x 1000 for the test angles. */
#define SIM_SIN_10 174
#define SIM_COS_10 985
#define SIM_SIN_30 500
#define SIM_COS_30 866
#define SIM_FLAT_COS 1000
#define SIM_LCG_A 1103515245u
#define SIM_LCG_C 12345u
#define SIM_LCG_SHIFT 16u
#define SIM_LCG_RANGE 32768u

typedef struct
{
    terrain_t terrain;
    int32_t   speed_mm_s;  /* true speed */
    int32_t   accel_step;  /* largest speed change per 10 ms */
    int64_t   position_um; /* true odometer */
    int64_t   edges;       /* summed encoder edges */
    int32_t   offset_mg;   /* chip zero-g offset on the forward axis */
    int32_t   noise_mg;    /* +/- uniform noise on every axis */
    uint32_t  seed;
    int32_t   spike_forward_mg; /* added to the next sample only */
    int32_t   turn_mm_s;        /* right wheel minus left wheel / 2 */
    bool      b_stale;
    bool      b_imu_fail;
    uint32_t  starts;
    uint32_t  finishes;
    uint32_t  impacts;
    uint32_t  last_peak_mm;
    bool      b_last_valid;
} sim_t;

static void    sim_init(sim_t * p_sim, int32_t start_mm);
static int32_t sim_noise(sim_t * p_sim);
static void    sim_step(sim_t * p_sim, int32_t target_speed, int32_t sine,
                        int32_t cosine);
static void    sim_drive(sim_t * p_sim, int32_t distance_mm, int32_t speed,
                         int32_t sine, int32_t cosine);
static void    sim_wait(sim_t * p_sim, uint32_t steps, int32_t speed,
                        int32_t sine, int32_t cosine);
static void    sim_calibrate(sim_t * p_sim);
static void    sim_hump(sim_t * p_sim, int32_t speed, int32_t sine,
                        int32_t cosine, int32_t slope_mm);
static void    test_helpers(void);
static void    test_calibration(void);
static void    test_ramp(void);
static void    test_acceleration(void);
static void    test_noise(void);
static void    test_spike_and_timeout(void);
static void    test_motion_gaps(void);
static void    test_reverse(void);
static void    test_saturation(void);
static void    test_sensor_fault(void);
static void    test_impact(void);
static void    test_classification(void);

/** @brief Each group prints one line so the log doubles as evidence. */
void terrain_test_run(void)
{
    test_helpers();
    test_calibration();
    test_ramp();
    test_acceleration();
    test_noise();
    test_spike_and_timeout();
    test_motion_gaps();
    test_reverse();
    test_saturation();
    test_sensor_fault();
    test_impact();
    test_classification();
    (void)puts("PASS: terrain (all groups)");
}

static void sim_init(sim_t * p_sim, int32_t start_mm)
{
    *p_sim             = (sim_t){0};
    p_sim->accel_step  = 10; /* 10 mm/s per 10 ms = 1 m/s^2 */
    p_sim->position_um = (int64_t)start_mm * SIM_UM_PER_MM;
    p_sim->edges = (p_sim->position_um * SIM_EDGES) / SIM_UM_PER_960_EDGES;
    p_sim->seed  = 1u;
    terrain_init(&p_sim->terrain);
}

/** @brief Deterministic uniform noise in -noise_mg..noise_mg. */
static int32_t sim_noise(sim_t * p_sim)
{
    int32_t result = 0;

    if (p_sim->noise_mg > 0)
    {
        p_sim->seed     = (p_sim->seed * SIM_LCG_A) + SIM_LCG_C;
        uint32_t sample = (p_sim->seed >> SIM_LCG_SHIFT) % SIM_LCG_RANGE;
        result          = (((int32_t)sample * ((2 * p_sim->noise_mg) + 1)) /
                  (int32_t)SIM_LCG_RANGE) -
                 p_sim->noise_mg;
    }

    return result;
}

/** @brief Advance the car 10 ms and feed terrain_step() one sample. */
static void sim_step(sim_t * p_sim, int32_t target_speed, int32_t sine,
                     int32_t cosine)
{
    int32_t dv = target_speed - p_sim->speed_mm_s;
    dv         = (dv > p_sim->accel_step) ? p_sim->accel_step : dv;
    dv         = (dv < -p_sim->accel_step) ? -p_sim->accel_step : dv;
    p_sim->speed_mm_s += dv;
    p_sim->position_um += (int64_t)p_sim->speed_mm_s * SIM_STEP_MS;

    /* Encoder and Motion's speed estimate, as in motion_cycle(). */
    int64_t edges = (p_sim->position_um * SIM_EDGES) / SIM_UM_PER_960_EDGES;
    int32_t delta = (int32_t)(edges - p_sim->edges);
    p_sim->edges  = edges;
    int32_t speed = (delta * SIM_SPEED_NUMERATOR) / SIM_EDGES;
    speed         = (speed > SIM_SPEED_LIMIT) ? SIM_SPEED_LIMIT : speed;
    speed         = (speed < -SIM_SPEED_LIMIT) ? -SIM_SPEED_LIMIT : speed;
    int64_t odometer_mm =
        ((edges * SIM_UM_PER_960_EDGES) / SIM_EDGES) / SIM_UM_PER_MM;
    int64_t wrapped =
        ((odometer_mm % SIM_ODOMETER_MM) + SIM_ODOMETER_MM) % SIM_ODOMETER_MM;

    int32_t         accel_mg = (dv * SIM_ACCEL_NUMERATOR) / SIM_GRAVITY_MM_S2;
    terrain_input_t input    = {0};
    input.accel_mg[0]        = (int16_t)(sine + accel_mg + p_sim->offset_mg +
                                  p_sim->spike_forward_mg + sim_noise(p_sim));
    input.accel_mg[1]        = (int16_t)sim_noise(p_sim);
    input.accel_mg[2]        = (int16_t)(cosine + sim_noise(p_sim));
    input.speed_mm_s         = (int16_t)speed;
    input.left_mm_s          = (int16_t)(speed - p_sim->turn_mm_s);
    input.right_mm_s         = (int16_t)(speed + p_sim->turn_mm_s);
    input.distance_mm        = (uint32_t)wrapped;
    input.b_imu_ok           = !p_sim->b_imu_fail;
    input.b_motion_fresh     = !p_sim->b_stale;
    p_sim->spike_forward_mg  = 0;

    terrain_step(&p_sim->terrain, &input);

    if (p_sim->terrain.b_started)
    {
        ++p_sim->starts;
    }

    if (p_sim->terrain.b_finished)
    {
        ++p_sim->finishes;
        p_sim->last_peak_mm = p_sim->terrain.hump_peak_mm;
        p_sim->b_last_valid = p_sim->terrain.b_hump_valid;
    }

    if (p_sim->terrain.b_impact)
    {
        ++p_sim->impacts;
    }
}

/** @brief Drive until the true odometer has moved distance_mm. */
static void sim_drive(sim_t * p_sim, int32_t distance_mm, int32_t speed,
                      int32_t sine, int32_t cosine)
{
    int64_t start = p_sim->position_um;
    int64_t goal  = (int64_t)distance_mm * SIM_UM_PER_MM;
    int64_t moved = 0;

    while (moved < goal)
    {
        sim_step(p_sim, speed, sine, cosine);
        moved = p_sim->position_um - start;
        moved = (moved < 0) ? -moved : moved;
    }
}

static void sim_wait(sim_t * p_sim, uint32_t steps, int32_t speed,
                     int32_t sine, int32_t cosine)
{
    for (uint32_t step = 0u; step < steps; ++step)
    {
        sim_step(p_sim, speed, sine, cosine);
    }
}

/** @brief Stand still on the flat until calibrated, then settle. */
static void sim_calibrate(sim_t * p_sim)
{
    sim_wait(p_sim, 60u, 0, 0, SIM_FLAT_COS);
    assert(p_sim->terrain.b_calibrated);
    assert(TERRAIN_LEVEL == p_sim->terrain.state);
}

/** @brief Flat run-up, slope up, 100-mm flat top, slope down, flat. */
static void sim_hump(sim_t * p_sim, int32_t speed, int32_t sine,
                     int32_t cosine, int32_t slope_mm)
{
    sim_wait(p_sim, 100u, speed, 0, SIM_FLAT_COS);
    sim_drive(p_sim, slope_mm, speed, sine, cosine);
    sim_drive(p_sim, 100, speed, 0, SIM_FLAT_COS);
    sim_drive(p_sim, slope_mm, speed, -sine, cosine);
    sim_drive(p_sim, 200, speed, 0, SIM_FLAT_COS);
}

/** @brief Packing, rounding and the integer maths helpers. */
static void test_helpers(void)
{
    assert(64u == terrain_pitch_field(0));
    assert(64u == terrain_pitch_field(24));
    assert(65u == terrain_pitch_field(25));
    assert(63u == terrain_pitch_field(-25));
    assert(0u == terrain_pitch_field(-3200));
    assert(127u == terrain_pitch_field(3150));
    assert(127u == terrain_pitch_field(9000));
    assert(0u == terrain_pitch_field(-9000));

    for (uint32_t field = 0u; field <= 127u; ++field)
    {
        assert(field == terrain_pitch_field(terrain_pitch_cdeg(field)));
    }

    assert(0u == terrain_peak_mm(-500));
    assert(0u == terrain_peak_mm(49));
    assert(1u == terrain_peak_mm(50));
    assert(52u == terrain_peak_mm(5209));
    assert(255u == terrain_peak_mm(25549));
    assert(255u == terrain_peak_mm(INT16_MAX));

    for (uint32_t peak = 0u; peak <= 255u; ++peak)
    {
        for (uint32_t flags = 0u; flags < 4u; ++flags)
        {
            bool     b_finished = (0u != (flags & 1u));
            bool     b_valid    = (0u != (flags & 2u));
            uint16_t event = terrain_event_pack(b_finished, peak, b_valid);
            /* Same bits robot_os writes with data_formats_write(). */
            uint8_t bytes[2] = {0u, 0u};
            (void)data_formats_write(bytes, 0u, 1u, b_finished ? 1u : 0u);
            (void)data_formats_write(bytes, 1u, 8u, peak);
            (void)data_formats_write(bytes, 9u, 1u, b_valid ? 1u : 0u);
            assert(event == (uint16_t)((uint32_t)bytes[0] |
                                       ((uint32_t)bytes[1] << 8u)));
            bool     b_finished_out = false;
            bool     b_valid_out    = false;
            uint32_t peak_out       = 0u;
            assert(terrain_event_unpack(event, &b_finished_out, &peak_out,
                                        &b_valid_out));
            assert((b_finished == b_finished_out) && (peak == peak_out) &&
                   (b_valid == b_valid_out));
        }
    }

    bool     b_finished = false;
    bool     b_valid    = false;
    uint32_t peak       = 0u;
    assert(!terrain_event_unpack(0x400u, &b_finished, &peak, &b_valid));
    assert(255u ==
           (((uint32_t)terrain_event_pack(true, 999u, true) >> 1u) & 255u));

    assert(0 == terrain_atan2_cdeg(0, 0));
    assert(9000 == terrain_atan2_cdeg(1000, 0));
    assert(-9000 == terrain_atan2_cdeg(-1000, 0));
    assert(18000 == terrain_atan2_cdeg(0, -1000));
    assert(4500 == terrain_atan2_cdeg(1000, 1000));
    int32_t ten   = terrain_atan2_cdeg(SIM_SIN_10, SIM_COS_10);
    int32_t five  = terrain_atan2_cdeg(87, 996);
    int32_t third = terrain_atan2_cdeg(SIM_SIN_30, SIM_COS_30);
    assert((ten > 990) && (ten < 1015));      /* 10.02 deg */
    assert((five > 490) && (five < 510));     /* 4.99 deg */
    assert((third > 2990) && (third < 3010)); /* 30.0 deg */
    assert(0u == terrain_isqrt(0u));
    assert(3u == terrain_isqrt(15u));
    assert(4u == terrain_isqrt(16u));
    assert(999u == terrain_isqrt(999999u));
    assert(1000u == terrain_isqrt(1000000u));
    assert(65535u == terrain_isqrt(UINT32_MAX));
    (void)printf("TERRAIN: helpers ok (atan2 10deg=%d 5deg=%d 30deg=%d "
                 "cdeg)\n",
                 (int)ten, (int)five, (int)third);
}

/** @brief Restart on movement; refuse a wrong mounting; learn offsets. */
static void test_calibration(void)
{
    sim_t sim = {0};
    sim_init(&sim, 0);
    sim_wait(&sim, 20u, 0, 0, SIM_FLAT_COS);
    sim_wait(&sim, 5u, 100, 0, SIM_FLAT_COS); /* pushed: restart */
    sim_wait(&sim, 20u, 0, 0, SIM_FLAT_COS);
    assert(!sim.terrain.b_calibrated);
    sim_wait(&sim, 60u, 0, 0, SIM_FLAT_COS);
    assert(sim.terrain.b_calibrated);

    /* Upside-down or wrong "up" axis: never calibrates. */
    sim_init(&sim, 0);
    sim_wait(&sim, 200u, 0, 0, -SIM_FLAT_COS);
    assert(!sim.terrain.b_calibrated);
    assert(TERRAIN_CALIBRATING == sim.terrain.state);

    /* A +60 mg zero-g offset (datasheet typical) is removed. */
    sim_init(&sim, 0);
    sim.offset_mg = 60;
    sim_calibrate(&sim);
    int32_t baseline = sim.terrain.baseline_x256 / TERRAIN_BASELINE_DIVISOR;
    assert((baseline >= 58) && (baseline <= 62));
    assert((sim.terrain.sin_milli >= -2) && (sim.terrain.sin_milli <= 2));
    (void)printf("TERRAIN: calibration ok (60 mg offset -> baseline %d mg)\n",
                 (int)baseline);
}

/** @brief The brief's reference case: 10 deg over 300 mm = 52.1 mm. */
static void test_ramp(void)
{
    static const int32_t speeds[] = {250, 500};

    for (uint32_t index = 0u; index < 2u; ++index)
    {
        sim_t sim = {0};
        sim_init(&sim, 0);
        sim.offset_mg = 60;
        sim_calibrate(&sim);
        sim_hump(&sim, speeds[index], SIM_SIN_10, SIM_COS_10, 300);
        int32_t peak = sim.terrain.peak_cmm;
        assert(1u == sim.starts);
        assert(1u == sim.finishes);
        assert(sim.b_last_valid);
        /* Brief: 52.1 mm +/- 0.5 mm. (sin 10 deg is fed as 174/1000.) */
        assert((peak >= 5159) && (peak <= 5259));
        assert(52u == sim.last_peak_mm);
        assert((sim.terrain.b_peak_valid) && (52u == sim.terrain.run_peak_mm));
        assert(!sim.terrain.b_on_hump);
        assert(0u == sim.impacts);
        (void)printf("TERRAIN: 10 deg x 300 mm at %d mm/s -> peak %d.%02d mm "
                     "(expected 52.09 +/- 0.5), report %u mm\n",
                     (int)speeds[index], (int)(peak / 100), (int)(peak % 100),
                     (unsigned)sim.last_peak_mm);
    }
}

/** @brief Tribunal 1: speeding up must not look like a climb. */
static void test_acceleration(void)
{
    sim_t sim = {0};
    sim_init(&sim, 0);
    sim_calibrate(&sim);
    sim_wait(&sim, 40u, 500, 0, SIM_FLAT_COS); /* 0 -> 0.4 m/s at 1 m/s^2 */
    int32_t accel = sim.terrain.accel_kin_mg;
    /* 1 m/s^2 = 102 mg. The old code reported about 10. */
    assert((accel >= 90) && (accel <= 115));
    sim_wait(&sim, 200u, 500, 0, SIM_FLAT_COS);
    sim_wait(&sim, 60u, 0, 0, SIM_FLAT_COS); /* brake to a stop */
    sim.accel_step = 20;                     /* 2 m/s^2 */
    sim_wait(&sim, 60u, 500, 0, SIM_FLAT_COS);
    sim_wait(&sim, 60u, 0, 0, SIM_FLAT_COS);
    assert(0u == sim.starts);
    assert(TERRAIN_LEVEL == sim.terrain.state);
    (void)printf("TERRAIN: 1 and 2 m/s^2 on flat -> no hump "
                 "(encoder accel %d mg at 1 m/s^2)\n",
                 (int)accel);
}

/** @brief Sensor noise and motor vibration on the flat. */
static void test_noise(void)
{
    static const int32_t levels[] = {40, 150};

    for (uint32_t index = 0u; index < 2u; ++index)
    {
        sim_t sim = {0};
        sim_init(&sim, 0);
        sim_calibrate(&sim);
        sim.noise_mg = levels[index];
        sim_wait(&sim, 3000u, 500, 0, SIM_FLAT_COS);
        assert(0u == sim.starts);
        assert(0u == sim.impacts);
        (void)printf("TERRAIN: +/-%d mg noise, 30 s at 0.5 m/s -> no hump, "
                     "no impact\n",
                     (int)levels[index]);
    }
}

/** @brief Short spikes never confirm; a 3-s climb is a false trigger. */
static void test_spike_and_timeout(void)
{
    sim_t sim = {0};
    sim_init(&sim, 0);
    sim_calibrate(&sim);
    sim_wait(&sim, 100u, 250, 0, SIM_FLAT_COS);

    for (uint32_t step = 0u; step < 3u; ++step)
    {
        sim.spike_forward_mg = 300;
        sim_step(&sim, 250, 0, SIM_FLAT_COS);
    }

    sim_wait(&sim, 50u, 250, 0, SIM_FLAT_COS);
    assert(0u == sim.starts);

    /* 10 deg for 4 s at 0.1 m/s: never levels off within 3 s. */
    sim_drive(&sim, 400, 100, SIM_SIN_10, SIM_COS_10);
    assert(1u == sim.starts);
    assert(1u == sim.finishes);
    assert(!sim.b_last_valid);
    assert(!sim.terrain.b_on_hump);
    /* Still on the slope: no immediate second hump until it levels. */
    sim_wait(&sim, 50u, 100, SIM_SIN_10, SIM_COS_10);
    assert(1u == sim.starts);
    (void)printf("TERRAIN: 3x300 mg spike ignored; 3-s climb -> invalid, "
                 "no re-trigger\n");
}

/** @brief Stale Motion data is skipped without losing distance; odometer
 * wrap-around changes nothing.
 */
static void test_motion_gaps(void)
{
    sim_t sim = {0};
    sim_init(&sim, 0);
    sim_calibrate(&sim);
    sim_wait(&sim, 100u, 250, 0, SIM_FLAT_COS);
    sim_drive(&sim, 150, 250, SIM_SIN_10, SIM_COS_10);
    sim.b_stale = true;
    sim_wait(&sim, 5u, 250, SIM_SIN_10, SIM_COS_10); /* 12.5 mm unseen */
    sim.b_stale = false;
    sim_drive(&sim, 137, 250, SIM_SIN_10, SIM_COS_10); /* 300 mm in all */
    sim_drive(&sim, 100, 250, 0, SIM_FLAT_COS);
    sim_drive(&sim, 300, 250, -SIM_SIN_10, SIM_COS_10);
    sim_drive(&sim, 200, 250, 0, SIM_FLAT_COS);
    assert((1u == sim.finishes) && (sim.b_last_valid));
    assert(52u == sim.last_peak_mm);
    uint32_t gap_peak = sim.last_peak_mm;

    /* A 150-ms gap on the hump is too long to trust: invalid. */
    sim_init(&sim, 0);
    sim_calibrate(&sim);
    sim_wait(&sim, 100u, 250, 0, SIM_FLAT_COS);
    sim_drive(&sim, 150, 250, SIM_SIN_10, SIM_COS_10);
    sim.b_stale = true;
    sim_wait(&sim, 15u, 250, SIM_SIN_10, SIM_COS_10);
    sim.b_stale = false;
    sim_drive(&sim, 112, 250, SIM_SIN_10, SIM_COS_10);
    sim_drive(&sim, 100, 250, 0, SIM_FLAT_COS);
    sim_drive(&sim, 300, 250, -SIM_SIN_10, SIM_COS_10);
    sim_drive(&sim, 200, 250, 0, SIM_FLAT_COS);
    assert((1u == sim.finishes) && (!sim.b_last_valid));

    /* The ramp crosses the odometer's 4095 -> 0 wrap. */
    sim_init(&sim, 3700);
    sim_calibrate(&sim);
    sim_hump(&sim, 250, SIM_SIN_10, SIM_COS_10, 300);
    assert((1u == sim.finishes) && (sim.b_last_valid));
    assert(52u == sim.last_peak_mm);
    (void)printf("TERRAIN: 50-ms stale gap -> peak %u mm valid; 150-ms gap "
                 "-> invalid; odometer wrap -> peak %u mm\n",
                 (unsigned)gap_peak, (unsigned)sim.last_peak_mm);
}

/** @brief Rolling back down the slope lowers the height. */
static void test_reverse(void)
{
    sim_t sim = {0};
    sim_init(&sim, 0);
    sim_calibrate(&sim);
    sim_wait(&sim, 100u, 250, 0, SIM_FLAT_COS);
    sim_drive(&sim, 200, 250, SIM_SIN_10, SIM_COS_10);
    sim_wait(&sim, 50u, 0, SIM_SIN_10, SIM_COS_10); /* stop */
    int32_t before   = sim.terrain.height_cmm;
    int64_t start_um = sim.position_um;
    sim_drive(&sim, 100, -250, SIM_SIN_10, SIM_COS_10); /* roll back */
    sim_wait(&sim, 50u, 0, SIM_SIN_10, SIM_COS_10);
    int32_t after = sim.terrain.height_cmm;
    /* Expected drop = distance rolled back * sin(10 deg), in 0.01 mm. */
    int64_t back_um  = start_um - sim.position_um;
    int32_t expected = (int32_t)((back_um * SIM_SIN_10) / 10000);
    int32_t drop     = before - after;
    int32_t error    = drop - expected;
    assert(sim.terrain.b_on_hump);
    assert((error > -60) && (error < 60));
    (void)printf("TERRAIN: rolled back %d mm on 10 deg -> height drops "
                 "%d.%02d mm (expected %d.%02d)\n",
                 (int)(back_um / 1000), (int)(drop / 100), (int)(drop % 100),
                 (int)(expected / 100), (int)(expected % 100));
}

/** @brief A hump above 255 mm saturates and is reported invalid. */
static void test_saturation(void)
{
    sim_t sim = {0};
    sim_init(&sim, 0);
    sim_calibrate(&sim);
    sim_hump(&sim, 250, SIM_SIN_30, SIM_COS_30, 600); /* 300 mm high */
    assert(1u == sim.finishes);
    assert(255u == sim.last_peak_mm);
    assert(!sim.b_last_valid);
    assert(!sim.terrain.b_peak_valid);
    (void)printf("TERRAIN: 300-mm hump -> peak saturates at %u, valid=0\n",
                 (unsigned)sim.last_peak_mm);
}

/** @brief One glitch is ridden through; three enter SENSOR_FAULT; the
 * driver is retried at most every 100 ms; recovery keeps calibration.
 */
static void test_sensor_fault(void)
{
    sim_t sim = {0};
    sim_init(&sim, 0);
    assert(sim.terrain.b_request_init);
    sim_calibrate(&sim);
    sim_wait(&sim, 100u, 250, 0, SIM_FLAT_COS);
    sim_drive(&sim, 150, 250, SIM_SIN_10, SIM_COS_10);
    sim.b_imu_fail = true;
    sim_step(&sim, 250, SIM_SIN_10, SIM_COS_10);
    sim.b_imu_fail = false;
    assert(sim.terrain.b_on_hump);
    sim_drive(&sim, 147, 250, SIM_SIN_10, SIM_COS_10); /* 300 mm in all */
    sim_drive(&sim, 100, 250, 0, SIM_FLAT_COS);
    sim_drive(&sim, 300, 250, -SIM_SIN_10, SIM_COS_10);
    sim_drive(&sim, 200, 250, 0, SIM_FLAT_COS);
    assert((1u == sim.finishes) && (sim.b_last_valid));
    assert(52u == sim.last_peak_mm);

    /* Three failures mid-hump: SENSOR_FAULT, hump ended invalid. */
    sim_drive(&sim, 150, 250, SIM_SIN_10, SIM_COS_10);
    assert(sim.terrain.b_on_hump);
    sim.b_imu_fail = true;
    sim_step(&sim, 250, SIM_SIN_10, SIM_COS_10);
    sim_step(&sim, 250, SIM_SIN_10, SIM_COS_10);
    assert(TERRAIN_SENSOR_FAULT != sim.terrain.state);
    sim_step(&sim, 250, SIM_SIN_10, SIM_COS_10);
    assert(TERRAIN_SENSOR_FAULT == sim.terrain.state);
    assert(sim.terrain.b_request_init);
    assert((2u == sim.finishes) && (!sim.b_last_valid));
    assert(!sim.terrain.b_on_hump);
    assert(TERRAIN_MOTION_UNKNOWN == sim.terrain.motion);
    assert(terrain_ready(&sim.terrain));

    uint32_t retries = 0u;

    for (uint32_t step = 0u; step < 100u; ++step) /* 1 s */
    {
        sim_step(&sim, 250, 0, SIM_FLAT_COS);
        retries += sim.terrain.b_request_init ? 1u : 0u;
    }

    assert((retries >= 9u) && (retries <= 10u));
    sim.b_imu_fail = false;
    sim_step(&sim, 250, 0, SIM_FLAT_COS);
    assert(TERRAIN_LEVEL == sim.terrain.state);
    assert(sim.terrain.b_calibrated);
    /* The first, valid hump is still the run's highest peak. */
    assert((sim.terrain.b_peak_valid) && (52u == sim.terrain.run_peak_mm));
    (void)printf("TERRAIN: 1 glitch ridden through (peak 52); 3 -> "
                 "SENSOR_FAULT; %u retries in 1 s; recovered to LEVEL\n",
                 (unsigned)retries);
}

/** @brief Horizontal jolts, the landing gate, and IMPACT hold. */
static void test_impact(void)
{
    sim_t sim = {0};
    sim_init(&sim, 0);
    sim_calibrate(&sim);
    sim_wait(&sim, 100u, 250, 0, SIM_FLAT_COS);
    sim.spike_forward_mg = -2000;
    sim.b_stale          = true; /* must not depend on Motion data */
    sim_step(&sim, 250, 0, SIM_FLAT_COS);
    sim.b_stale = false;
    assert(1u == sim.impacts);
    assert(TERRAIN_IMPACT == sim.terrain.state);
    assert(TERRAIN_MOTION_IMPACT == sim.terrain.motion);
    sim_wait(&sim, 15u, 0, 0, SIM_FLAT_COS);
    assert(TERRAIN_IMPACT == sim.terrain.state);
    sim_wait(&sim, 10u, 0, 0, SIM_FLAT_COS);
    assert(TERRAIN_LEVEL == sim.terrain.state);
    assert(1u == sim.impacts);

    /* A 2 g bump right after the descent is a landing, not a crash... */
    sim_init(&sim, 0);
    sim_calibrate(&sim);
    sim_wait(&sim, 100u, 250, 0, SIM_FLAT_COS);
    sim_drive(&sim, 300, 250, SIM_SIN_10, SIM_COS_10);
    sim_drive(&sim, 100, 250, 0, SIM_FLAT_COS);
    sim_drive(&sim, 300, 250, -SIM_SIN_10, SIM_COS_10);
    sim_wait(&sim, 20u, 250, 0, SIM_FLAT_COS); /* filter settles: finish */
    assert(1u == sim.finishes);
    assert(sim.terrain.gate_ms > 0u);
    sim.spike_forward_mg = 2000;
    sim_step(&sim, 250, 0, SIM_FLAT_COS);
    assert(0u == sim.impacts);
    /* ...but a 3 g hit is still a crash, even inside the gate. */
    sim.spike_forward_mg = 3000;
    sim_step(&sim, 250, 0, SIM_FLAT_COS);
    assert(1u == sim.impacts);
    (void)printf("TERRAIN: 2 g jolt -> IMPACT held 200 ms; 2 g landing "
                 "ignored; 3 g in gate -> IMPACT\n");
}

/** @brief Stationary, accelerating, turning and cruising classes. */
static void test_classification(void)
{
    sim_t sim = {0};
    sim_init(&sim, 0);
    assert(TERRAIN_MOTION_UNKNOWN == sim.terrain.motion);
    sim_calibrate(&sim);
    assert(TERRAIN_MOTION_STATIONARY == sim.terrain.motion);
    sim_wait(&sim, 20u, 500, 0, SIM_FLAT_COS);
    assert(TERRAIN_MOTION_ACCELERATING == sim.terrain.motion);
    sim_wait(&sim, 200u, 500, 0, SIM_FLAT_COS);
    assert(TERRAIN_MOTION_CRUISING == sim.terrain.motion);
    sim.turn_mm_s = 100; /* 200 mm/s difference on a 280-mm track */
    sim_step(&sim, 500, 0, SIM_FLAT_COS);
    assert(TERRAIN_MOTION_TURNING == sim.terrain.motion);
    assert(sim.terrain.b_motion_changed);
    assert((sim.terrain.turn_dps >= 40) && (sim.terrain.turn_dps <= 42));
    sim_step(&sim, 500, 0, SIM_FLAT_COS);
    assert(!sim.terrain.b_motion_changed);
    (void)printf("TERRAIN: classes stationary/accelerating/cruising/turning "
                 "ok (turn %d deg/s)\n",
                 (int)sim.terrain.turn_dps);
}

/*** End of file. ***/

