/** @file telemetry.c
 * @brief Binary command reassembly without text conversion or unbounded
 * storage.
 */
#include "telemetry.h"

#include "../data_formats.h"

#include <stddef.h>

/** @brief Reset on each publish header before accepting its payload. */
void telemetry_begin (telemetry_t * p_receiver, uint32_t total,
                     bool b_command_topic)
{
    p_receiver->payload    = 0u;
    p_receiver->received   = 0u;
    p_receiver->b_bad      = (1u != total) || (!b_command_topic);
    p_receiver->b_complete = false;
}

/** @brief A Boolean result tells the transport when a full command is ready.
 */
bool telemetry_fragment (telemetry_t * p_receiver, const uint8_t * p_data,
                        uint16_t length, bool b_last)
{
    bool b_ready = false;
    bool b_fits  = (((uint32_t)length + p_receiver->received) <= 1u) &&
                   ((NULL != p_data) || (0u == length));
    p_receiver->b_bad =
        (p_receiver->b_bad) || ((!b_fits) || (p_receiver->b_complete));

    if (b_fits && (1u == length))
    {
        p_receiver->payload  = p_data[0];
        p_receiver->received = 1u;
    }

    if (b_last)
    {
        p_receiver->b_complete = true;
        uint8_t command        = 0u;
        uint8_t sequence       = 0u;
        /* The stored payload is exactly one eight-bit byte. */
        bool b_valid = data_formats_command_read((uint8_t)p_receiver->payload,
                                                 &command, &sequence);
        b_ready =
            b_valid && (1u == p_receiver->received) && (!p_receiver->b_bad);
    }

    return b_ready;
}

/*** End of file. ***/

