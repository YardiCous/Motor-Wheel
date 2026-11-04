/** @file data_formats.h
 * @brief Portable packed fields; see docs/data_representation.md.
 */
#ifndef DATA_FORMATS_H
#define DATA_FORMATS_H

#include <stdbool.h>
#include <stdint.h>

#define DATA_FORMATS_TELEMETRY_BYTES 28u
#define DATA_FORMATS_COMMAND_BYTES 1u
#define DATA_FORMATS_VERSION 1u
#define DATA_FORMATS_FAULT_STALL 0x001u
#define DATA_FORMATS_FAULT_ENCODER 0x002u
#define DATA_FORMATS_FAULT_IMU 0x004u
#define DATA_FORMATS_FAULT_NO_ECHO 0x008u
#define DATA_FORMATS_FAULT_WIFI 0x010u
#define DATA_FORMATS_FAULT_IMPACT 0x020u
#define DATA_FORMATS_FAULT_ESTOP 0x040u
#define DATA_FORMATS_FAULT_WATCHDOG 0x080u
#define DATA_FORMATS_FAULT_RANGE 0x100u
#define DATA_FORMATS_FAULT_TIMEOUT 0x200u
#define DATA_FORMATS_FAULT_QUEUE 0x400u
#define DATA_FORMATS_FAULT_MQTT 0x800u
/* IMU fault (0x004) is advisory: the run continues with hump height marked
 * invalid (team decision pending, proposed by Buddy 4). IMPACT stays fatal.
 */
#define DATA_FORMATS_FATAL_MASK 0x7ebu

#define DATA_FORMATS_BOOT 0u
#define DATA_FORMATS_IDLE 1u
#define DATA_FORMATS_FOLLOW 2u
#define DATA_FORMATS_BARCODE 3u
#define DATA_FORMATS_HUMP 4u
#define DATA_FORMATS_AVOID 5u
#define DATA_FORMATS_SEARCH 6u
#define DATA_FORMATS_FAULT 7u

#define DATA_FORMATS_STOP 0u
#define DATA_FORMATS_START 1u
#define DATA_FORMATS_RESET 2u
#define DATA_FORMATS_ESTOP 3u
#define DATA_FORMATS_LEFT 0u
#define DATA_FORMATS_RIGHT 1u
#define DATA_FORMATS_STRAIGHT 2u
#define DATA_FORMATS_UTURN 3u

#define DATA_FORMATS_MOTION_BITS (12u + 9u + 1u + 3u + 7u)
#define DATA_FORMATS_PAYLOAD_BITS 223u

#if (DATA_FORMATS_MOTION_BITS != 32u)
#error Motion snapshot must occupy exactly thirty-two bits.
#endif
#if (DATA_FORMATS_PAYLOAD_BITS > (DATA_FORMATS_TELEMETRY_BYTES * 8u))
#error Telemetry buffer is smaller than its fields.
#endif

typedef uint8_t data_formats_word_check_t[(4u == sizeof(uint32_t)) ? 1 : -1];

/** @brief Read an unsigned field from a little-endian bit stream.
 * @pre Width is 1..32 and the caller supplies sufficient buffer space.
 */
uint32_t data_formats_read(const volatile uint8_t * p_data, uint16_t offset,
                           uint8_t width);
/** @brief Validate and write a field without modifying adjacent fields. */
bool data_formats_write(volatile uint8_t * p_data, uint16_t offset,
                        uint8_t width, uint32_t value);
/** @brief Encode a signed value as portable unsigned offset binary. */
uint32_t data_formats_signed(int32_t value, uint8_t width);
/** @brief Decode offset binary without signed bit manipulation. */
int32_t data_formats_unsign(uint32_t value, uint8_t width);
/** @brief Pack distance12, speed9, reverse1, mode3, timestamp7. */
bool data_formats_motion_pack(uint32_t * p_word, uint16_t distance,
                              uint16_t speed, bool b_reverse, uint8_t mode,
                              uint8_t tick);
/** @brief Compute the signed shortest displacement in a modulo-4096 ring.
 * @pre Absolute travel between samples must be less than 2048 mm.
 */
int16_t data_formats_distance_delta(uint16_t newer, uint16_t older);
/** @brief Reject reserved command bits and return command2 and sequence4. */
bool data_formats_command_read(uint8_t payload, uint8_t * p_command,
                               uint8_t * p_sequence);
/** @brief Build command2, sequence4 and two zero reserved bits. */
uint8_t data_formats_command_make(uint8_t command, uint8_t sequence);
/** @brief Accept forward modulo-16 sequence increments of 1..7 only. */
bool data_formats_sequence_new(uint8_t newer, uint8_t older);

#endif /* DATA_FORMATS_H */

/*** End of file. ***/

