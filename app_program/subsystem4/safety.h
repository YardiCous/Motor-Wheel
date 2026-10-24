/** @file safety.h
 * @brief Latched faults with explicit reset and advisory WiFi/MQTT bits.
 */
#ifndef SAFETY_H
#define SAFETY_H

#include <stdbool.h>
#include <stdint.h>

/** @brief Each of the twelve fault causes occupies one shared bit. */
typedef struct
{
    uint32_t faults : 12;
    uint32_t active : 12;
    bool     b_latched : 1;
} safety_t;

/** @brief Initialize with motors inhibited until startup is complete. */
void safety_init(safety_t * p_safety);
/** @brief Update current conditions and retain historical fatal causes. */
void safety_update(safety_t * p_safety, uint16_t active);
/** @brief RESET succeeds only when every currently active fatal cause clears.
 */
bool safety_reset(safety_t * p_safety);

#endif /* SAFETY_H */

/*** End of file. ***/

