#ifndef ROBOT_PACKED_H
#define ROBOT_PACKED_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
/* Explicit little-bit-first encoding, independent of ABI and bit-field order.
 * A byte is the smallest addressable object; unused tail bits must be zero. */
bool bits_put(uint8_t *data, size_t bytes, unsigned offset, unsigned width, uint32_t value);
bool bits_get(const uint8_t *data, size_t bytes, unsigned offset, unsigned width, uint32_t *value);
typedef struct {
    uint8_t bytes[1];
} command_msg_t; /* opcode 2, sequence 6 */
typedef struct {
    uint8_t bytes[3];
} motion_msg_t; /* kind 3, biased argument 14, speed 7 */
typedef struct {
    uint8_t bytes[4];
} line_msg_t; /* error+1000:11, mode:3, seen:1, time16:16 */
typedef struct {
    uint8_t bytes[4];
} event_msg_t; /* type:2, value:10, time16:16 = 28 */
typedef struct {
    uint8_t bytes[8];
} motion_state_msg_t; /* left+250:9,right+250:9,distance modulo:24,mode:3,time16:16 =61 */
typedef struct {
    uint8_t bytes[20];
} telemetry_msg_t; /* 158 bits; see docs */
typedef enum { CMD_START, CMD_STOP, CMD_RESET, CMD_ESTOP } command_t;
typedef enum { MOVE_STOP, MOVE_FOLLOW, MOVE_DISTANCE, MOVE_TURN, MOVE_SEARCH } move_t;
bool command_encode(command_msg_t *out, command_t command, unsigned sequence);
bool command_decode(const uint8_t *in, size_t length, command_t *command, uint8_t *sequence);
bool motion_encode(motion_msg_t *out, move_t kind, int argument, unsigned speed);
bool motion_decode(const motion_msg_t *in, move_t *kind, int *argument, unsigned *speed);
/* All range errors are rejected rather than truncated. */
bool telemetry_set(telemetry_msg_t *out, unsigned field, int32_t value);
bool telemetry_get(const telemetry_msg_t *in, unsigned field, int32_t *value);
enum {
    T_VERSION,
    T_VEHICLE,
    T_FAULTS,
    T_LEFT_SPEED,
    T_RIGHT_SPEED,
    T_LEFT_ENCODER,
    T_RIGHT_ENCODER,
    T_DISTANCE,
    T_IR_LEFT,
    T_IR_CENTER,
    T_IR_RIGHT,
    T_BARCODE,
    T_HUMP,
    T_RANGE,
    T_WIFI,
    T_MQTT,
    T_SEQUENCE,
    T_FIELD_COUNT
};
_Static_assert(sizeof(command_msg_t) == 1, "command wire size");
_Static_assert(sizeof(motion_msg_t) == 3, "motion wire size");
_Static_assert(sizeof(line_msg_t) == 4, "line mailbox size");
_Static_assert(sizeof(motion_state_msg_t) == 8, "stamped motion mailbox size");
_Static_assert(sizeof(event_msg_t) == 4, "event wire size");
_Static_assert(sizeof(telemetry_msg_t) == 20, "telemetry wire size");
#endif
