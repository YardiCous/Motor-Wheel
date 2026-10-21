#ifndef ROBOT_APP_H
#define ROBOT_APP_H
#include "controller.h"
#include "encoder.h"
#include "kernel_sdk.h"
#include <stdint.h>

enum { EV_BARCODE, EV_HUMP_START, EV_HUMP_END, EV_SCAN_DONE };
enum { FLG_SAFETY = 1u, FLG_ECHO = 2u };
typedef struct {
    controller_t control;
    line_msg_t line;
    motion_state_msg_t motion;
    uint16_t heartbeat[6]; /* motion, line, IMU, navigation, range, safety */
    unsigned adc_left : 12;
    unsigned adc_center : 12;
    unsigned adc_right : 12;
    unsigned encoder_left : 16;
    unsigned encoder_right : 16;
    unsigned hump_peak : 10;
    unsigned barcode : 3;
    unsigned line_valid : 1;
    unsigned motion_valid : 1;
    unsigned imu_ready : 1;
    unsigned line_ready : 1;
    unsigned scan_busy : 1;
    unsigned scan_requested : 1;
    unsigned scan_side : 2; /* 0=none,1=left,2=right */
    unsigned wifi : 1;
    unsigned mqtt : 1;
    unsigned motion_done : 1;
    unsigned local_reset : 1;
    unsigned recovery_active : 1;
} robot_state_t;
extern ID q_motion_cmd, q_event, q_cmd_in, q_ack, evg_robot;
extern ID task_ids[7];
void robot_state_init(void);
void robot_snapshot(robot_state_t *out);
void robot_raise_fault(unsigned mask);
void robot_clear_cause(unsigned mask);
void robot_heartbeat(unsigned task);
void robot_line_update(const line_msg_t *line, unsigned left, unsigned center, unsigned right);
void robot_motion_update(int left, int right, uint32_t distance, move_t mode);
void robot_encoder_update(uint16_t left, uint16_t right);
void robot_range_update(unsigned mm, bool valid, bool forward);
void robot_imu_ready(bool ready);
void robot_network_status(bool wifi, bool mqtt);
void robot_set_peak(unsigned peak);
void robot_set_barcode(unsigned code);
void robot_scan_request(void);
bool robot_scan_take_request(void);
void robot_scan_finish(unsigned side);
void robot_set_vehicle(vehicle_state_t state);
void robot_set_recovery(bool active);
bool robot_take_local_reset(void);
void robot_local_reset(void);
bool robot_command(command_t cmd, unsigned sequence);
bool robot_apply_command(command_t cmd);
void robot_safety_step(void);
bool robot_may_drive(void);
bool robot_send_motion(move_t kind, int argument, unsigned speed_mm_s);
bool robot_send_event(unsigned type, unsigned value);
bool robot_decode_event(const event_msg_t *event, unsigned *type, unsigned *value, uint16_t *at);
void robot_motion_complete(void);
bool robot_take_motion_complete(void);
void robot_flush_motion(void);
void robot_reset_subsystems(void);
void robot_tasks_entry(INT code, void *extra);

void board_init(void);
void board_enable_inputs(void);
void board_buttons_step(void);
bool board_estop_active(void);
void board_encoder_snapshot(encoder_t *left, encoder_t *right);
void board_clear_encoder_fault(void);
void board_motor_write(int left_permille, int right_permille);
void board_servo_angle(unsigned degrees);
void board_range_trigger(void);
bool board_echo_take(unsigned *microseconds);
void board_echo_cancel(void);
uint16_t robot_now(void);

void motion_init(void);
void motion_step(void);
void motion_reset(void);
void line_init(void);
void line_step(void);
void line_reset(void);
void imu_init(void);
void imu_step(void);
void imu_reset(void);
void obstacle_init(void);
void obstacle_step(void);
void obstacle_reset(void);
void navigation_step(void);
void navigation_reset(void);
void network_prepare(void);
void network_init(void);
void network_step(void);
/* Native task-local service states reset via a shared generation counter below. */
unsigned robot_reset_generation(void);
#endif
