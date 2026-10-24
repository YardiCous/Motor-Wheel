/** @file safety.c
 * @brief Pure safety policy; robot_os serializes shared access.
 */
#include "safety.h"

#include "../data_formats.h"

/** @brief Startup uses the same explicit reset path as a fault. */
void safety_init (safety_t * p_safety)
{
    p_safety->faults    = 0u;
    p_safety->active    = 0u;
    p_safety->b_latched = true;
}

/** @brief Communications are advisory; physical sensor faults inhibit motion.
 */
void safety_update (safety_t * p_safety, uint16_t active)
{
    uint32_t fatal   = active & DATA_FORMATS_FATAL_MASK;
    p_safety->active = active & 4095u;
    p_safety->faults = (p_safety->faults & DATA_FORMATS_FATAL_MASK) | active;
    p_safety->b_latched = (p_safety->b_latched) || ((0u != fatal));
}

/** @brief No automatic resume occurs when a physical fault disappears. */
bool safety_reset (safety_t * p_safety)
{
    bool b_clear = (0u == (p_safety->active & DATA_FORMATS_FATAL_MASK));

    if (b_clear)
    {
        p_safety->b_latched = false;
        p_safety->faults    = p_safety->active;
    }

    return b_clear;
}

/*** End of file. ***/

