#include "hardware/watchdog.h"
#include "pico/stdlib.h"
#include "robot.h"

ID q_motion_cmd, q_event, q_cmd_in, q_ack, evg_robot;
ID task_ids[7];
static void safety_task(INT code, void *extra) {
    (void)code;
    (void)extra;
    watchdog_enable(30, true);
    for (;;) {
        UINT flags;
        tk_wai_flg(evg_robot, FLG_SAFETY, TWF_ORW | TWF_BITCLR, &flags, SAFETY_PERIOD_MS - 1);
        board_buttons_step();
        if (board_estop_active())
            robot_raise_fault(F_ESTOP);
        robot_safety_step();
        robot_heartbeat(5);
        robot_state_t state;
        robot_snapshot(&state);
        bool alive = true;
        for (unsigned i = 0; i < 5; ++i)
            if (elapsed_ms(robot_now(), state.heartbeat[i]) > (i == 0 ? 15u : 50u))
                alive = false;
        /* A dead Motion task cannot write zero; hardware watchdog reboots to low outputs. */
        if (alive)
            watchdog_update();
    }
}
static void motion_task(INT code, void *extra) {
    (void)code;
    (void)extra;
    motion_init();
    uint32_t deadline = kernel_milliseconds();
    for (;;) {
        motion_step();
        robot_heartbeat(0);
        kernel_wait_period(&deadline, MOTION_PERIOD_MS);
    }
}
static void line_task(INT code, void *extra) {
    (void)code;
    (void)extra;
    line_init();
    uint32_t deadline = kernel_milliseconds();
    for (;;) {
        line_step();
        robot_heartbeat(1);
        kernel_wait_period(&deadline, LINE_PERIOD_MS);
    }
}
static void imu_task(INT code, void *extra) {
    (void)code;
    (void)extra;
    imu_init();
    uint32_t deadline = kernel_milliseconds();
    for (;;) {
        imu_step();
        robot_heartbeat(2);
        kernel_wait_period(&deadline, IMU_PERIOD_MS);
    }
}
static void navigation_task(INT code, void *extra) {
    (void)code;
    (void)extra;
    uint32_t deadline = kernel_milliseconds();
    for (;;) {
        navigation_step();
        robot_heartbeat(3);
        kernel_wait_period(&deadline, 10);
    }
}
static void obstacle_task(INT code, void *extra) {
    (void)code;
    (void)extra;
    obstacle_init();
    for (;;) {
        obstacle_step();
        robot_heartbeat(4);
        UINT flags;
        /* ECHO edges wake this task without busy-waiting; timeout advances the FSM. */
        tk_wai_flg(evg_robot, FLG_ECHO, TWF_ORW | TWF_BITCLR, &flags, 4);
    }
}
static void network_task(INT code, void *extra) {
    (void)code;
    (void)extra;
    network_init();
    uint32_t deadline = kernel_milliseconds();
    for (;;) {
        network_step();
        kernel_wait_period(&deadline, 10);
    }
}
static ID message_buffer(unsigned depth, unsigned size) {
    /* micro-T stores a 4-byte length header and rounds each payload to 4 bytes. */
    T_CMBF parameters = {.mbfatr = TA_TFIFO,
                         .bufsz = depth * (sizeof(UW) + ((size + 3u) & ~3u)),
                         .maxmsz = (INT)size};
    return tk_cre_mbf(&parameters);
}
void robot_tasks_entry(INT code, void *extra) {
    (void)code;
    (void)extra;
#ifdef ROBOT_ON_TARGET_TESTS
    extern void robot_kernel_tests(void);
    robot_kernel_tests();
#endif
    robot_state_init();
    q_motion_cmd = message_buffer(MOTION_QUEUE_DEPTH, sizeof(motion_msg_t));
    q_event = message_buffer(EVENT_QUEUE_DEPTH, sizeof(event_msg_t));
    q_cmd_in = message_buffer(COMMAND_QUEUE_DEPTH, sizeof(command_msg_t));
    q_ack = message_buffer(8, 1);
    T_CFLG flags = {.flgatr = TA_WMUL};
    evg_robot = tk_cre_flg(&flags);
    if (q_motion_cmd < 0 || q_event < 0 || q_cmd_in < 0 || q_ack < 0 || evg_robot < 0)
        panic("micro-T object allocation failed");
    static const struct {
        void (*entry)(INT, void *);
        PRI priority;
        SZ stack;
    } definitions[] = {
        {safety_task, PRIORITY_SAFETY, 2048},         {motion_task, PRIORITY_CONTROL, 3072},
        {line_task, PRIORITY_CONTROL, 3072},          {imu_task, PRIORITY_NAVIGATION, 4096},
        {navigation_task, PRIORITY_NAVIGATION, 3072}, {obstacle_task, PRIORITY_OBSTACLE, 3072},
        {network_task, PRIORITY_TELEMETRY, 8192}};
    /* Create all tasks dormant before starting any of them. Init priority is 1. */
    for (unsigned i = 0; i < 7; ++i) {
        T_CTSK task = {.tskatr = TA_HLNG | TA_RNG0,
                       .task = (FP)definitions[i].entry,
                       .itskpri = definitions[i].priority,
                       .stksz = definitions[i].stack};
        task_ids[i] = tk_cre_tsk(&task);
        if (task_ids[i] < E_OK)
            panic("micro-T task allocation failed");
    }
    board_enable_inputs();
    for (unsigned i = 0; i < 6; ++i)
        robot_heartbeat(i);
    for (unsigned i = 0; i < 7; ++i)
        if (tk_sta_tsk(task_ids[i], 0) < E_OK)
            panic("micro-T task start failed");
    tk_exd_tsk();
}
