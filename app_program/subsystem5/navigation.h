/** @file navigation.h
 * @brief Mission arbitration with explicit pre-emption and bounded recovery.
 */
#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <stdbool.h>
#include <stdint.h>

#include "obstacle.h"

/** @brief Task-private mission state uses eight values exactly. */
typedef struct
{
    uint32_t state : 3;
    uint32_t stage : 4;
    bool     b_waiting : 1;
    bool     b_scan_requested : 1;
    bool     b_reset_requested : 1;
    bool     b_stop_requested : 1;
    bool     b_command_ready : 1;
    bool     b_fault : 1;
    bool     b_left : 1;
    bool     b_slow : 1;
    uint32_t line_confirm : 2;
    bool     b_estop : 1;
    uint32_t barcode : 2;
    uint32_t elapsed_ms : 14;
    uint32_t look : 3;
    uint32_t left_cm : 9;
    uint32_t right_cm : 9;
    uint32_t side_mm : 12;
    uint32_t steps : 5;
    bool     b_margin : 1;
    uint32_t width_mm : 12;
    uint32_t detected_mm : 12;
    bool b_first_forward : 1;
    bool b_body_scan : 1;
    uint32_t body_index : 4;
    uint32_t side_remaining_mm : 12;
    uint16_t body_ranges[OBSTACLE_BODY_POINTS];
    uint8_t scan_failure;
    uint32_t command;
} navigation_t;

/** @brief Reach IDLE locally without waiting for WiFi. */
void navigation_init(navigation_t * p_navigation);
/** @brief Validate START/STOP/RESET/E-stop against the current state. */
bool navigation_supervise(navigation_t * p_navigation, uint8_t command,
                          bool b_safety_latched);
/** @brief Consume one barcode once, pre-empting line following. */
void navigation_barcode(navigation_t * p_navigation, uint8_t barcode);
/** @brief Accept a bounded width and side clearances for a pending bypass. */
bool navigation_obstacle_plan(navigation_t * p_navigation, uint16_t width_mm,
                               uint16_t left_cm, uint16_t right_cm);
/** @brief Request a centred-sensor body sweep for a pending bypass. */
bool navigation_body_scan(navigation_t * p_navigation);
/** @brief Run one 10-ms arbitration cycle; output is one packed command. */
void navigation_step(navigation_t * p_navigation, uint8_t motion_mode,
                     bool b_done, bool b_line_valid, bool b_line_lost,
                     bool b_on_hump, uint16_t range_cm, bool b_range_valid,
                     bool b_scan_ready, bool b_scan_left, bool b_barcode_zone,
                     bool b_safety_latched);

#endif /* NAVIGATION_H */

/*** End of file. ***/

