#include "packed.h"
#include <string.h>
bool bits_put(uint8_t *d, size_t n, unsigned o, unsigned w, uint32_t v) {
    if (!d || !w || w > 32 || o > n * 8 || w > n * 8 - o || (w < 32 && v >= (UINT32_C(1) << w)))
        return false;
    for (unsigned i = 0; i < w; ++i) {
        unsigned p = o + i;
        uint8_t mask = (uint8_t)(1u << (p % 8));
        d[p / 8] = (uint8_t)((d[p / 8] & (uint8_t)~mask) | (((v >> i) & 1u) ? mask : 0));
    }
    return true;
}
bool bits_get(const uint8_t *d, size_t n, unsigned o, unsigned w, uint32_t *v) {
    if (!d || !v || !w || w > 32 || o > n * 8 || w > n * 8 - o)
        return false;
    *v = 0;
    for (unsigned i = 0; i < w; ++i)
        *v |= (uint32_t)((d[(o + i) / 8] >> ((o + i) % 8)) & 1u) << i;
    return true;
}
bool command_encode(command_msg_t *out, command_t c, unsigned seq) {
    if (!out || (unsigned)c > CMD_ESTOP || seq > 63)
        return false;
    out->bytes[0] = (uint8_t)((seq << 2) | c);
    return true;
}
bool command_decode(const uint8_t *in, size_t n, command_t *c, uint8_t *seq) {
    if (!in || n != 1 || !c || !seq)
        return false;
    *c = (command_t)(in[0] & 3u);
    *seq = in[0] >> 2;
    return true;
}
bool motion_encode(motion_msg_t *out, move_t k, int a, unsigned s) {
    if (!out || (unsigned)k > MOVE_SEARCH || a < -8192 || a > 8191 || s > 125)
        return false;
    memset(out, 0, sizeof(*out));
    return bits_put(out->bytes, 3, 0, 3, k) &&
           bits_put(out->bytes, 3, 3, 14, (uint32_t)(a + 8192)) &&
           bits_put(out->bytes, 3, 17, 7, s);
}
bool motion_decode(const motion_msg_t *in, move_t *k, int *a, unsigned *s) {
    uint32_t x, y, z;
    if (!in || !k || !a || !s)
        return false;
    bits_get(in->bytes, 3, 0, 3, &x);
    bits_get(in->bytes, 3, 3, 14, &y);
    bits_get(in->bytes, 3, 17, 7, &z);
    if (x > MOVE_SEARCH || z > 125)
        return false;
    *k = (move_t)x;
    *a = (int)y - 8192;
    *s = z;
    return true;
}
/* min/max/width are schema constants, not runtime application storage. */
static const struct {
    int32_t min, max;
    unsigned width;
} schema[T_FIELD_COUNT] = {{0, 1, 1},      {0, 8, 4},      {0, 1023, 10},  {-250, 250, 9},
                           {-250, 250, 9}, {0, 65535, 16}, {0, 65535, 16}, {0, 16777215, 24},
                           {0, 4095, 12},  {0, 4095, 12},  {0, 4095, 12},  {0, 5, 3},
                           {0, 1023, 10},  {0, 4095, 12},  {0, 1, 1},      {0, 1, 1},
                           {0, 63, 6}};
static unsigned offset(unsigned f) {
    unsigned o = 0;
    for (unsigned i = 0; i < f; ++i)
        o += schema[i].width;
    return o;
}
bool telemetry_set(telemetry_msg_t *out, unsigned f, int32_t v) {
    if (!out || f >= T_FIELD_COUNT || v < schema[f].min || v > schema[f].max)
        return false;
    return bits_put(out->bytes, sizeof(out->bytes), offset(f), schema[f].width,
                    (uint32_t)(v - schema[f].min));
}
bool telemetry_get(const telemetry_msg_t *in, unsigned f, int32_t *v) {
    uint32_t raw;
    if (!in || !v || f >= T_FIELD_COUNT)
        return false;
    if (!bits_get(in->bytes, sizeof(in->bytes), offset(f), schema[f].width, &raw))
        return false;
    *v = (int32_t)raw + schema[f].min;
    return *v <= schema[f].max;
}
