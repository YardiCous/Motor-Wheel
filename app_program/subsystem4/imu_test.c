/** @file imu_test.c
 * @brief Motor-off IMU bring-up (build with IMU_TEST=1).
 *
 * Lecturer's advice: bring each sensor up on its own before integrating.
 * The output is CSV on the USB console so it can be logged on Windows and
 * parsed. Lines starting with '#' are comments and events.
 */
#include "imu_test.h"

#include "../board_io.h"
#include "../robot_config.h"
#include "terrain_config.h"

#include <tm/tmonitor.h>

#define IMU_TEST_ROW_MS 50u      /* 20 Hz CSV rows */
#define IMU_TEST_RETRY_MS 1000u  /* look for a missing chip once a second */
#define IMU_TEST_TIMING_MS 5000u /* worst-case timing summary period */
/* Boot lines can be printed before the PC opens the COM port, so the scan
 * and chip identity are repeated every 10 s. */
#define IMU_TEST_STATUS_MS 10000u
#define IMU_TEST_FIRST_ADDRESS 0x08u /* 0x00-0x07 and 0x78-0x7F reserved */
#define IMU_TEST_LAST_ADDRESS 0x77u
#define IMU_TEST_PROBE_REG 0x0Fu
#define IMU_TEST_US_LIMIT 65535u
#define IMU_TEST_ODOMETER_MM 4096
#define IMU_TEST_UM_PER_MM 1000
#define IMU_TEST_WHEELS 2

static void scan_bus(void);
static void identify(imu_test_t * p_test);
static void print_identity(const imu_test_t * p_test);
static void sample_encoders(imu_test_t * p_test, terrain_input_t * p_input);
static void report_events(imu_test_t * p_test, uint32_t t_ms);
static void print_row(imu_test_t * p_test, const int16_t * p_raw,
                      const int16_t * p_mg, uint32_t t_ms, bool b_ok);
static uint32_t elapsed_us(uint32_t start_us);

/** @brief Banner, bus scan and chip identity come before any CSV row. */
void imu_test_init(imu_test_t * p_test)
{
    *p_test = (imu_test_t){0};
    board_io_motor(0, 0);
    tm_printf((UB *)"# IMU TEST (Buddy 4): motors disabled in this build\n");
    tm_printf((UB *)"# I2C1: GP2 SDA, GP3 SCL, 400 kHz, 3.3 V\n");
    board_io_i2c_init();
    scan_bus();
    identify(p_test);
    terrain_init(&p_test->terrain);
    (void)board_io_encoders(&p_test->last_left, &p_test->last_right);
    p_test->start_ms       = board_io_time_ms();
    p_test->last_row_ms    = p_test->start_ms;
    p_test->last_retry_ms  = p_test->start_ms;
    p_test->last_timing_ms = p_test->start_ms;
    p_test->last_status_ms = p_test->start_ms;
    tm_printf((UB *)"# Keep the car flat and still for 1 s to calibrate\n");
    tm_printf((UB *)"# pitch/roll columns assume chip X forward, Z up;"
                    " use the raw columns to find the real mounting\n");
    tm_printf((UB *)"# state 0 cal 1 level 2 climb 3 crest 4 descend"
                    " 5 impact 6 fault; motion 0 unknown 1 still"
                    " 2 cruise 3 accel 4 turn 5 climb 6 descend 7 impact\n");
    tm_printf((UB *)"t_ms,ax_raw,ay_raw,az_raw,ax_mg,ay_mg,az_mg,a_mag_mg,"
                    "mx_raw,my_raw,mz_raw,pitch_cdeg,roll_cdeg,dist_mm,"
                    "speed_mm_s,state,motion,sin_milli,height_cmm,"
                    "run_peak_mm,peak_valid,read_us,step_us,ok\n");
}

/** @brief Called every 10 ms by the terrain task slot. */
void imu_test_step(imu_test_t * p_test)
{
    uint32_t        now                  = board_io_time_ms();
    uint32_t        t_ms                 = now - p_test->start_ms;
    terrain_input_t input                = {0};
    int16_t         raw[DRV_LSM303_AXES] = {0};
    bool            b_retry              = (!p_test->b_found) &&
                   ((now - p_test->last_retry_ms) >= IMU_TEST_RETRY_MS);

    if ((p_test->terrain.b_request_init) || (b_retry))
    {
        if (b_retry)
        {
            scan_bus();
        }

        identify(p_test);
        p_test->last_retry_ms = now;
    }

    uint32_t start = board_io_time_us();
    input.b_imu_ok = (p_test->b_found) &&
                     (drv_lsm303_accel(&p_test->imu, raw, input.accel_mg));
    p_test->read_us = elapsed_us(start);
    sample_encoders(p_test, &input);
    start = board_io_time_us();
    terrain_step(&p_test->terrain, &input);
    p_test->step_us     = elapsed_us(start);
    p_test->max_read_us = (p_test->read_us > p_test->max_read_us)
                              ? p_test->read_us
                              : p_test->max_read_us;
    p_test->max_step_us = (p_test->step_us > p_test->max_step_us)
                              ? p_test->step_us
                              : p_test->max_step_us;
    report_events(p_test, t_ms);

    if ((now - p_test->last_row_ms) >= IMU_TEST_ROW_MS)
    {
        p_test->last_row_ms = now;
        print_row(p_test, raw, input.accel_mg, t_ms, input.b_imu_ok);
    }

    if ((now - p_test->last_timing_ms) >= IMU_TEST_TIMING_MS)
    {
        tm_printf((UB *)"# TIMING t=%u max_read_us=%u max_step_us=%u"
                        " (worst case over the last 5 s)\n",
                  (unsigned)t_ms, (unsigned)p_test->max_read_us,
                  (unsigned)p_test->max_step_us);
        p_test->last_timing_ms = now;
        p_test->max_read_us    = 0u;
        p_test->max_step_us    = 0u;
    }

    if ((now - p_test->last_status_ms) >= IMU_TEST_STATUS_MS)
    {
        p_test->last_status_ms = now;
        scan_bus();
        print_identity(p_test);
    }
}

/** @brief Print every 7-bit address that acknowledges a register read. */
static void scan_bus(void)
{
    uint32_t found = 0u;
    tm_printf((UB *)"# I2C1 scan:");

    for (uint8_t address = IMU_TEST_FIRST_ADDRESS;
         address <= IMU_TEST_LAST_ADDRESS; ++address)
    {
        uint8_t value = 0u;

        if (board_io_i2c_read(address, IMU_TEST_PROBE_REG, &value, 1u))
        {
            tm_printf((UB *)" 0x%02x", (unsigned)address);
            ++found;
        }
    }

    tm_printf((UB *)" (%u device(s))\n", (unsigned)found);
}

/** @brief Detect and configure; print only when the result changes. */
static void identify(imu_test_t * p_test)
{
    bool b_found    = drv_lsm303_init(&p_test->imu);
    bool b_changed  = (!p_test->b_reported) || (b_found != p_test->b_found);
    p_test->b_found = b_found;

    if (b_changed)
    {
        print_identity(p_test);
    }

    p_test->b_reported = true;
}

/** @brief Chip name, addresses and the identity bytes that matched. */
static void print_identity(const imu_test_t * p_test)
{
    if (p_test->b_found)
    {
        tm_printf((UB *)"# IMU: %s accel 0x%02x mag 0x%02x"
                        " id %02x %02x %02x, configuration read back ok\n",
                  (UB *)drv_lsm303_name(&p_test->imu),
                  (unsigned)p_test->imu.accel_address,
                  (unsigned)p_test->imu.mag_address,
                  (unsigned)p_test->imu.id[0], (unsigned)p_test->imu.id[1],
                  (unsigned)p_test->imu.id[2]);
    }
    else
    {
        tm_printf((UB *)"# IMU: no LSM303DLHC or LSM303D found."
                        " Check 3.3 V, GND, SDA=GP2, SCL=GP3."
                        " Retrying every 1 s\n");
    }
}

/** @brief Distance and speed from the raw encoders, the same way
 * motion_cycle() computes them, so hand-pushed runs give real distance.
 */
static void sample_encoders(imu_test_t * p_test, terrain_input_t * p_input)
{
    uint32_t left  = 0u;
    uint32_t right = 0u;
    bool     b_ok  = board_io_encoders(&left, &right);
    /* Unsigned subtraction handles the 2^32 wrap; deltas are small. */
    int32_t left_edges  = (int32_t)(left - p_test->last_left);
    int32_t right_edges = (int32_t)(right - p_test->last_right);
    p_test->last_left   = left;
    p_test->last_right  = right;
    p_test->edges += left_edges + right_edges;

    /* um per ms is mm/s: edges * circumference / (edges per turn * 10). */
    int32_t circumference = (int32_t)ROBOT_WHEEL_CIRCUM_UM;
    int32_t per_period =
        (int32_t)ROBOT_ENCODER_EDGES_TURN * (int32_t)ROBOT_IMU_PERIOD_MS;
    p_input->left_mm_s = (int16_t)((left_edges * circumference) / per_period);
    p_input->right_mm_s =
        (int16_t)((right_edges * circumference) / per_period);
    p_input->speed_mm_s =
        (int16_t)(((left_edges + right_edges) * circumference) /
                  (IMU_TEST_WHEELS * per_period));

    int64_t travel_um = (p_test->edges * circumference) /
                        (IMU_TEST_WHEELS * (int32_t)ROBOT_ENCODER_EDGES_TURN);
    int64_t travel_mm = travel_um / IMU_TEST_UM_PER_MM;
    int64_t odometer =
        ((travel_mm % IMU_TEST_ODOMETER_MM) + IMU_TEST_ODOMETER_MM) %
        IMU_TEST_ODOMETER_MM;
    p_input->distance_mm    = (uint32_t)odometer;
    p_input->b_motion_fresh = b_ok;
}

/** @brief Hump, impact, fault and calibration events print immediately. */
static void report_events(imu_test_t * p_test, uint32_t t_ms)
{
    const terrain_t * p_terrain = &p_test->terrain;

    if (p_terrain->b_started)
    {
        tm_printf((UB *)"# EVENT t=%u HUMP_START\n", (unsigned)t_ms);
    }

    if (p_terrain->b_finished)
    {
        tm_printf((UB *)"# EVENT t=%u HUMP_END peak_mm=%u valid=%u"
                        " run_peak_mm=%u\n",
                  (unsigned)t_ms, (unsigned)p_terrain->hump_peak_mm,
                  p_terrain->b_hump_valid ? 1u : 0u,
                  (unsigned)p_terrain->run_peak_mm);
    }

    if (p_terrain->b_impact)
    {
        tm_printf((UB *)"# EVENT t=%u IMPACT\n", (unsigned)t_ms);
    }

    if (p_terrain->state != p_test->last_state)
    {
        if (TERRAIN_SENSOR_FAULT == p_terrain->state)
        {
            tm_printf((UB *)"# EVENT t=%u SENSOR_FAULT\n", (unsigned)t_ms);
        }
        else if (TERRAIN_CALIBRATING == p_test->last_state)
        {
            tm_printf(
                (UB *)"# EVENT t=%u CALIBRATED baseline_mg=%d\n",
                (unsigned)t_ms,
                (int)(p_terrain->baseline_x256 / TERRAIN_BASELINE_DIVISOR));
        }
        else
        {
            /* Other changes are visible in the state column. */
        }

        p_test->last_state = p_terrain->state;
    }
}

/** @brief One CSV row. The magnetometer is read here only (20 Hz). */
static void print_row(imu_test_t * p_test, const int16_t * p_raw,
                      const int16_t * p_mg, uint32_t t_ms, bool b_ok)
{
    int16_t mag[DRV_LSM303_AXES] = {0};
    (void)drv_lsm303_mag(&p_test->imu, mag);
    int32_t  ax         = p_mg[0];
    int32_t  ay         = p_mg[1];
    int32_t  az         = p_mg[2];
    uint32_t horizontal = (uint32_t)((ay * ay) + (az * az));
    uint32_t total      = horizontal + (uint32_t)(ax * ax);
    int32_t pitch = terrain_atan2_cdeg(ax, (int32_t)terrain_isqrt(horizontal));
    int32_t roll  = terrain_atan2_cdeg(ay, az);
    const terrain_t * p_terrain = &p_test->terrain;

    tm_printf((UB *)"%u,%d,%d,%d,%d,%d,%d,%u,%d,%d,%d,%d,%d,%u,%d,%u,%u,"
                    "%d,%d,%u,%u,%u,%u,%u\n",
              (unsigned)t_ms, (int)p_raw[0], (int)p_raw[1], (int)p_raw[2],
              (int)ax, (int)ay, (int)az, (unsigned)terrain_isqrt(total),
              (int)mag[0], (int)mag[1], (int)mag[2], (int)pitch, (int)roll,
              (unsigned)p_terrain->last_distance,
              (int)(p_terrain->speed_x8 / TERRAIN_FAST_DIVISOR),
              (unsigned)p_terrain->state, (unsigned)p_terrain->motion,
              (int)p_terrain->sin_milli, (int)p_terrain->height_cmm,
              (unsigned)p_terrain->run_peak_mm,
              p_terrain->b_peak_valid ? 1u : 0u, (unsigned)p_test->read_us,
              (unsigned)p_test->step_us, b_ok ? 1u : 0u);
}

/** @brief Microseconds since start_us, saturated to the 16-bit field. */
static uint32_t elapsed_us(uint32_t start_us)
{
    uint32_t elapsed = board_io_time_us() - start_us;
    return (elapsed > IMU_TEST_US_LIMIT) ? IMU_TEST_US_LIMIT : elapsed;
}

/*** End of file. ***/

