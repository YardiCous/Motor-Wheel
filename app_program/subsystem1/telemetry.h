/** @file telemetry.h
 * @brief Strict binary MQTT fragment validation, independent of lwIP.
 */
#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>

/** @brief Only a single command byte may be accumulated. */
typedef struct
{
    uint32_t payload : 8;
    uint32_t received : 1;
    bool     b_bad : 1;
    bool     b_complete : 1;
} telemetry_t;

/** @brief Reject unexpected topics and every length other than one byte. */
void telemetry_begin(telemetry_t * p_receiver, uint32_t total,
                     bool b_command_topic);
/** @brief Preserve binary bytes; reject truncation, overrun and extra
 * fragments. */
bool telemetry_fragment(telemetry_t * p_receiver, const uint8_t * p_data,
                        uint16_t length, bool b_last);

#endif /* TELEMETRY_H */

/*** End of file. ***/

