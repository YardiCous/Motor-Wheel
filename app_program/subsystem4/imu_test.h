/** @file imu_test.h
 * @brief IMU_TEST=1 bring-up: motors off, I2C scan, chip ID, CSV stream.
 *
 * Runs in place of the terrain task (robot_os.c). It also runs the real
 * terrain logic on the live data, so pushing the car over the ramp by hand
 * exercises hump detection and gives on-target timing.
 */
#ifndef IMU_TEST_H
#define IMU_TEST_H

#include "drv_lsm303.h"
#include "terrain.h"

#include <stdbool.h>
#include <stdint.h>

/** @brief Bring-up state; the encoders give distance when pushed by hand. */
typedef struct
{
    drv_lsm303_t imu;
    terrain_t    terrain;
    uint32_t     last_left; /* encoder counts, modulo 2^32 */
    uint32_t     last_right;
    int64_t      edges;    /* left + right edges since start */
    uint32_t     start_ms; /* kernel time at init */
    uint32_t     last_row_ms;
    uint32_t     last_retry_ms;
    uint32_t     last_timing_ms;
    uint32_t     last_status_ms;
    uint32_t     read_us : 16;     /* this cycle's I2C read time */
    uint32_t     step_us : 16;     /* this cycle's terrain_step time */
    uint32_t     max_read_us : 16; /* worst case since the last summary */
    uint32_t     max_step_us : 16;
    uint32_t     last_state : 3; /* terrain state last cycle, for events */
    bool         b_found : 1;    /* a supported chip answered */
    bool         b_reported : 1; /* chip found/lost already printed */
} imu_test_t;

/** @brief Stop the motors, scan I2C1, identify the chip, print the header.
 */
void imu_test_init(imu_test_t * p_test);
/** @brief One 10-ms cycle: sample, run terrain, print at 20 Hz. */
void imu_test_step(imu_test_t * p_test);

#endif /* IMU_TEST_H */

/*** End of file. ***/

