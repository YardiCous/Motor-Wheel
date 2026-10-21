#include "encoder.h"
void encoder_init(encoder_t *e, unsigned ab) {
    e->count = 0;
    e->previous = ab & 3u;
    e->invalid = 0;
}
void encoder_edge(encoder_t *e, unsigned ab) {
    static const int8_t step[16] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};
    ab &= 3u;
    if ((e->previous ^ ab) == 3u)
        e->invalid = 1;
    e->count = (uint16_t)(e->count + step[e->previous * 4u + ab]);
    e->previous = ab;
}
int32_t encoder_delta(uint16_t current, uint16_t previous) {
    uint16_t d = (uint16_t)(current - previous);
    return d <= 32767u ? (int32_t)d : (int32_t)d - 65536;
}
