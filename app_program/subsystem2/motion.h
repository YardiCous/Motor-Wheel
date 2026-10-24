/** @file motion.h
 * @brief The sole motor controller, with bounded integer PID state.
 */
#ifndef MOTION_H
#define MOTION_H

#include <stdbool.h>
#include <stdint.h>

#define MOTION_IDLE 0u
#define MOTION_FOLLOW 1u
#define MOTION_MOVE 2u
#define MOTION_TURN 3u
#define MOTION_STALL 4u
#define MOTION_ESTOP 5u
#define MOTION_INIT 6u

/** @brief Task-private packed state; never transmitted as a C object. */
typedef struct
{
    uint32_t last_left;
    uint32_t last_right;
    uint32_t position_um : 22;
    uint32_t mode : 3;
    bool     b_reverse : 1;
    bool     b_done : 1;
    bool     b_stall : 1;
    bool     b_encoder_fault : 1;
    bool     b_timeout : 1;
    uint32_t speed : 9;
    uint32_t target_mm : 12;
    uint32_t progress_um : 22;
    uint32_t elapsed_ms : 14;
    uint32_t stall_left_ms : 8;
    uint32_t stall_right_ms : 8;
    uint32_t integral_left : 15;
    uint32_t integral_right : 15;
    uint32_t prior_left : 11;
    uint32_t prior_right : 11;
    uint32_t remainder : 11;
} motion_t;

/** @brief Reset while stopped, anchoring both encoder snapshots. */
void motion_init(motion_t * p_motion, uint32_t left, uint32_t right);
/** @brief Build a 24-bit command with checked physical ranges. */
bool motion_command(uint32_t * p_word, uint8_t mode, uint16_t distance,
                    uint16_t speed, bool b_reverse);
/** @brief STOP overrides all states; manoeuvres preempt FOLLOW only. */
bool motion_accept(motion_t * p_motion, uint32_t command);
/** @brief Run one 10-ms cycle and produce bounded duty values -1000..1000. */
void motion_step(motion_t * p_motion, uint32_t left, uint32_t right,
                 int16_t line_error, bool b_estop, int16_t * p_left_pwm,
                 int16_t * p_right_pwm);
/** @brief Publish one whole 32-bit snapshot, never bit-field writes. */
uint32_t motion_snapshot(const motion_t * p_motion, uint8_t tick,
                         uint16_t speed, bool b_reverse);
/** @brief Return signed modulo-2^32 edge displacement, with a 64-edge guard.
 */
int32_t motion_edge_delta(uint32_t newer, uint32_t older, bool * pb_valid);

#endif /* MOTION_H */

/*** End of file. ***/

