/** @file selftest.c
 * @brief These startup checks never energize hardware or require WiFi.
 */
#include "selftest.h"

#include "data_formats.h"
#include "subsystem4/safety.h"

/** @brief A failed test prevents application tasks from starting. */
bool selftest_run (void)
{
    uint32_t snapshot = 0u;
    bool     b_valid =
        data_formats_motion_pack(&snapshot, 4095u, 500u, true, 6u, 127u);
    b_valid = b_valid && (4095u == (snapshot & 4095u));
    b_valid = b_valid && (127u == (snapshot >> 25u));
    b_valid = b_valid && (5 == data_formats_distance_delta(3u, 4094u));
    uint8_t bytes[2] = {0u};
    b_valid          = b_valid && (data_formats_write(bytes, 3u, 9u, 400u));
    b_valid          = b_valid && (400u == data_formats_read(bytes, 3u, 9u));
    b_valid          = b_valid && (!data_formats_write(bytes, 3u, 9u, 512u));
    safety_t monitor = {0};
    safety_init(&monitor);
    b_valid = b_valid && (safety_reset(&monitor));
    safety_update(&monitor, DATA_FORMATS_FAULT_WIFI);
    b_valid = b_valid && (!monitor.b_latched);
    safety_update(&monitor, DATA_FORMATS_FAULT_IMPACT);
    b_valid = b_valid && (monitor.b_latched) && (!safety_reset(&monitor));
    safety_update(&monitor, 0u);
    b_valid = b_valid && (monitor.b_latched) && (safety_reset(&monitor));
    return b_valid;
}

/*** End of file. ***/

