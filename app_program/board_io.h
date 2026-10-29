/** @file board_io.h
 * @brief RP2040 peripherals with one dispatcher for echo and encoder edges.
 */
#ifndef BOARD_IO_H
#define BOARD_IO_H

#include <stdbool.h>
#include <stdint.h>

/** @brief Configure safe outputs and install T-Kernel-wrapped ISRs on core 0.
 */
bool board_io_init(int32_t h_echo_flag);
/** @brief Save and disable core-0 interrupts for a short packed-data access.
 */
uint32_t board_io_lock(void);
/** @brief Restore exactly the previous interrupt mask. */
void board_io_unlock(uint32_t mask);
/** @brief Read the free-running modulo-2^32 microsecond timer. */
uint32_t board_io_time_us(void);
/** @brief Read the kernel's modulo-2^32 millisecond clock. */
uint32_t board_io_time_ms(void);
/** @brief Snapshot both 32-bit modulo encoder counters and edge validity. */
bool board_io_encoders(uint32_t * p_left, uint32_t * p_right);
/** @brief Return latched invalid-transition bits: bit 0 left, bit 1 right. */
uint8_t board_io_encoder_faults(void);
/** @brief Clear an encoder diagnostic only while motor drive is inhibited. */
void board_io_clear_fault(void);
/** @brief Only the Motion task calls this after startup. */
void board_io_motor(int16_t left, int16_t right);
/** @brief Sample all three 12-bit ADC channels with a bounded timeout. */
bool board_io_line(uint16_t * p_samples);
/** @brief Reset I2C1 to 400-kHz master mode; no bus traffic. */
void board_io_i2c_init(void);
/** @brief Read 1..8 consecutive registers from a 7-bit I2C1 target.
 * The caller sets the chip's auto-increment bit in reg when it needs one.
 */
bool board_io_i2c_read(uint8_t address, uint8_t reg, uint8_t * p_bytes,
                       uint8_t length);
/** @brief Write one register on a 7-bit I2C1 target. */
bool board_io_i2c_write(uint8_t address, uint8_t reg, uint8_t value);
/** @brief Set a servo angle 0..60 in three-degree units. */
void board_io_servo(uint8_t angle);
/** @brief Trigger a 10-us pulse using TIMER alarm 3, without busy waiting. */
void board_io_ping(void);
/** @brief Retrieve the ISR-captured echo; an absent echo is invalid. */
bool board_io_range(uint16_t * p_cm);
/** @brief Read active-low local START, STOP, RESET as three individual bits.
 */
uint8_t board_io_buttons(void);
/** @brief Mark each motor-loop start on GP18 for timing measurements. */
void board_io_trace(bool b_high);

#endif /* BOARD_IO_H */

/*** End of file. ***/

