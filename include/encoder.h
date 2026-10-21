#ifndef ROBOT_ENCODER_H
#define ROBOT_ENCODER_H
#include <stdbool.h>
#include <stdint.h>
typedef struct {
    uint16_t count;
    unsigned previous : 2;
    unsigned invalid : 1;
} encoder_t;
void encoder_init(encoder_t *e, unsigned ab);
/* Call in the one GPIO dispatcher for both A and B edges. Snapshot count and
 * invalid together with interrupts masked; never infer direction from PWM. */
void encoder_edge(encoder_t *e, unsigned ab);
/* Caller guarantees fewer than 32768 counts between snapshots. */
int32_t encoder_delta(uint16_t current, uint16_t previous);
#endif
