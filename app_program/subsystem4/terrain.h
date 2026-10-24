/** @file terrain.h
 * @brief Gyro-free LSM303DLHC terrain and collision monitoring.
 */
#ifndef TERRAIN_H
#define TERRAIN_H

#include <stdbool.h>
#include <stdint.h>

#define TERRAIN_CALIBRATING 0u
#define TERRAIN_LEVEL 1u
#define TERRAIN_CLIMBING 2u
#define TERRAIN_CRESTING 3u
#define TERRAIN_DESCENDING 4u
#define TERRAIN_IMPACT 5u
#define TERRAIN_SENSOR_FAULT 6u

/** @brief Fine accumulation retains small 100-Hz height increments. */
typedef struct
{
    uint32_t state : 3;
    uint32_t failures : 2;
    uint32_t samples : 6;
    bool     b_valid : 1;
    bool     b_on_hump : 1;
    bool     b_started : 1;
    bool     b_finished : 1;
    bool     b_impact : 1;
    bool     b_overflow : 1;
    uint32_t pitch : 7;
    uint32_t peak_mm : 8;
    uint32_t height_centi_mm : 16;
    uint32_t peak_centi_mm : 16;
    uint32_t baseline_mg : 10;
    uint32_t last_speed : 10;
    uint32_t last_distance : 12;
    uint32_t climb_ms : 12;
    uint32_t hump_ms : 14;
    uint32_t fractional : 5;
    uint32_t calibration_sum : 15;
} terrain_t;

/** @brief Reset calibration and invalidate height until stationary samples. */
void terrain_init(terrain_t * p_terrain);
/** @brief Readings are signed milli-g; motion speed is signed mm/s.
 * @param b_fresh False means I2C or motion data exceeded the 20-ms limit.
 */
void terrain_step(terrain_t * p_terrain, int16_t accel_x, int16_t accel_y,
                  int16_t accel_z, int16_t speed, uint16_t distance,
                  bool b_fresh);

#endif /* TERRAIN_H */

/*** End of file. ***/

