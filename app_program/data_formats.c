/** @file data_formats.c
 * @brief Explicit masks keep compiler bit-field order out of messages.
 */
#include "data_formats.h"

#include <stddef.h>

/** @brief Read each bit using unsigned arithmetic only. */
uint32_t data_formats_read (const volatile uint8_t * p_data, uint16_t offset,
                           uint8_t width)
{
    uint32_t value = 0u;

    for (uint8_t bit = 0u; bit < width; ++bit)
    {
        uint32_t position = offset + bit;
        uint32_t source   = p_data[position / 8u];
        value |= ((source >> (position % 8u)) & 1u) << bit;
    }

    return value;
}

/** @brief Reject overflow before touching any destination byte. */
bool data_formats_write (volatile uint8_t * p_data, uint16_t offset,
                        uint8_t width, uint32_t value)
{
    bool b_valid = (NULL != p_data) && (width > 0u) && (width <= 32u);

    if (b_valid && (width < 32u))
    {
        b_valid = (value < (1u << width));
    }

    if (b_valid)
    {
        for (uint8_t bit = 0u; bit < width; ++bit)
        {
            uint32_t position = offset + bit;
            uint32_t mask     = 1u << (position % 8u);
            uint32_t byte     = p_data[position / 8u];
            byte = (byte & ~mask) | (((value >> bit) & 1u) << (position % 8u));
            /* Clearing and setting one bit preserves an 8-bit result. */
            p_data[position / 8u] = (uint8_t)byte;
        }
    }

    return b_valid;
}

/** @brief The supported widths keep the signed intermediate in int32_t. */
uint32_t data_formats_signed (int32_t value, uint8_t width)
{
    /* Width is 2..16, so the bias fits a positive signed integer. */
    int32_t bias = (int32_t)(1u << (width - 1u));
    /* Callers range-check value; the offset is nonnegative and fits. */
    return (uint32_t)(value + bias);
}

/** @brief Subtraction avoids implementation-defined signed extension. */
int32_t data_formats_unsign (uint32_t value, uint8_t width)
{
    /* Width is 2..16 and both operands fit in int32_t. */
    return (int32_t)value - (int32_t)(1u << (width - 1u));
}

/** @brief A 7-bit 5-ms stamp fixes the tribunal's 33-bit allocation. */
bool data_formats_motion_pack (uint32_t * p_word, uint16_t distance,
                              uint16_t speed, bool b_reverse, uint8_t mode,
                              uint8_t tick)
{
    bool b_valid = (NULL != p_word) && (distance <= 4095u) &&
                   (speed <= 511u) && (mode <= 6u) && (tick <= 127u);

    if (b_valid)
    {
        uint32_t reverse = b_reverse ? 1u : 0u;
        *p_word = distance | ((uint32_t)speed << 12u) | (reverse << 21u) |
                  ((uint32_t)mode << 22u) | ((uint32_t)tick << 25u);
    }

    return b_valid;
}

/** @brief A maximum 20-ms sample age makes the half-range bound generous. */
int16_t data_formats_distance_delta (uint16_t newer, uint16_t older)
{
    int32_t delta = newer;
    delta -= older;

    if (delta > 2047)
    {
        delta -= 4096;
    }
    else if (delta < -2048)
    {
        delta += 4096;
    }
    else
    {
        /* The difference already represents the shortest displacement. */
    }

    /* The modular correction constrains delta to -2048..2047. */
    return (int16_t)delta;
}

/** @brief Payload length is checked by the transport before this call. */
bool data_formats_command_read (uint8_t payload, uint8_t * p_command,
                               uint8_t * p_sequence)
{
    bool b_valid = (0u == (payload & 0xc0u)) && (NULL != p_command) &&
                   (NULL != p_sequence);

    if (b_valid)
    {
        *p_command  = payload & 3u;
        *p_sequence = (payload >> 2u) & 15u;
    }

    return b_valid;
}

/** @brief Inputs are bounded by the four commands and modulo-16 sequence. */
uint8_t data_formats_command_make (uint8_t command, uint8_t sequence)
{
    /* Both masked fields together occupy only six bits. */
    return (uint8_t)((command & 3u) | ((sequence & 15u) << 2u));
}

/** @brief Half-range comparison rejects duplicates and old commands. */
bool data_formats_sequence_new (uint8_t newer, uint8_t older)
{
    uint32_t delta = ((uint32_t)newer + 16u - older) & 15u;
    return (delta > 0u) && (delta < 8u);
}

/*** End of file. ***/

