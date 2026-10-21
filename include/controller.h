#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H
#include "packed.h"
#include "robot_config.h"
typedef enum {
    VEH_BOOT,
    VEH_IDLE,
    VEH_FOLLOW,
    VEH_BARCODE,
    VEH_HUMP,
    VEH_OBSTACLE,
    VEH_SEARCH,
    VEH_COMPLETE,
    VEH_FAULT
} vehicle_state_t;
typedef enum {
    LINE_CALIBRATING,
    LINE_TRACKING,
    LINE_SEARCH,
    LINE_JUNCTION,
    LINE_BARCODE_SCAN,
    LINE_DECODE
} line_mode_t;
typedef enum {
    OBS_IDLE,
    OBS_COARSE,
    OBS_FINE,
    OBS_PLAN,
    OBS_BYPASS,
    OBS_RECOVER,
    OBS_ABORT
} obstacle_state_t;
typedef enum {
    IMU_INIT,
    IMU_CALIBRATING,
    IMU_LEVEL,
    IMU_CLIMBING,
    IMU_CRESTING,
    IMU_DESCENDING,
    IMU_IMPACT,
    IMU_FAULT
} imu_state_t;
typedef enum { NET_INIT, NET_WIFI, NET_MQTT, NET_ONLINE, NET_RETRY } network_state_t;
enum {
    F_STALL = 1u << 0,
    F_ENCODER = 1u << 1,
    F_IMU = 1u << 2,
    F_NO_ECHO = 1u << 3,
    F_WIFI = 1u << 4,
    F_IMPACT = 1u << 5,
    F_WATCHDOG = 1u << 6,
    F_OVERFLOW = 1u << 7,
    F_NAVIGATION = 1u << 8,
    F_ESTOP = 1u << 9
};
#define F_FATAL (1023u & ~F_WIFI)
/* Private to Navigation; adapter serializes all access. C bit-fields must never
 * be sent as wire data. Padding is ABI dependent; field widths are explicit. */
typedef struct {
    unsigned vehicle : 4;
    unsigned faults : 10;
    unsigned latched : 1;
    unsigned ready : 1;
    unsigned line_mode : 3;
    unsigned line_seen : 1;
    unsigned line_missing : 1;
    unsigned range_valid : 1;
    unsigned range_exempt : 1;
    unsigned range_mm : 12;
    uint16_t line_at, range_at, missing_at, context_at, search_at;
} controller_t;
void controller_init(controller_t *c);
void controller_self_test(controller_t *c, bool passed);
void controller_fault(controller_t *c, unsigned mask);
void controller_clear_cause(controller_t *c, unsigned mask);
bool controller_command(controller_t *c, command_t cmd);
void controller_line(controller_t *c, line_mode_t mode, bool seen, uint16_t now);
void controller_range(controller_t *c, unsigned mm, bool valid, uint16_t now);
void controller_tick(controller_t *c, uint16_t now);
bool controller_may_drive(const controller_t *c);
/* Modulo timestamps: callers must service at <32768 ms intervals. */
static inline uint16_t elapsed_ms(uint16_t now, uint16_t then) {
    return (uint16_t)(now - then);
}
#endif
