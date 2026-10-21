/* On-board smoke tests for the actual micro-T scheduler and copied messages. */
#include "robot.h"
#include <assert.h>
#include <stdio.h>

static ID test_buffer;
static ID test_flags;
static void worker(INT code, void *extra) {
    (void)code;
    (void)extra;
    motion_msg_t message;
    assert(motion_encode(&message, MOVE_TURN, -90, 45));
    assert(tk_snd_mbf(test_buffer, &message, sizeof(message), TMO_POL) == E_OK);
    assert(tk_set_flg(test_flags, 3) == E_OK);
    tk_exd_tsk();
}
void robot_kernel_tests(void) {
    T_CMBF buffer = {.bufsz = 8, .maxmsz = sizeof(motion_msg_t)};
    T_CFLG flags = {.flgatr = TA_WMUL};
    test_buffer = tk_cre_mbf(&buffer);
    test_flags = tk_cre_flg(&flags);
    assert(test_buffer > 0 && test_flags > 0);
    T_CTSK task = {.tskatr = TA_HLNG | TA_RNG0, .task = (FP)worker, .itskpri = 2, .stksz = 2048};
    ID worker_id = tk_cre_tsk(&task);
    assert(worker_id > 0 && tk_sta_tsk(worker_id, 0) == E_OK);
    motion_msg_t received;
    assert(tk_rcv_mbf(test_buffer, &received, 20) == sizeof(received));
    move_t kind;
    int argument;
    unsigned speed;
    assert(motion_decode(&received, &kind, &argument, &speed));
    assert(kind == MOVE_TURN && argument == -90 && speed == 45);
    UINT pattern;
    assert(tk_wai_flg(test_flags, 1, TWF_ORW | TWF_BITCLR, &pattern, 20) == E_OK);
    assert(tk_wai_flg(test_flags, 2, TWF_ORW | TWF_BITCLR, &pattern, TMO_POL) == E_OK);
    assert(tk_wai_flg(test_flags, 3, TWF_ORW, &pattern, TMO_POL) == E_TMOUT);
    uint32_t start = kernel_milliseconds();
    assert(tk_dly_tsk(4) == E_OK);
    assert(kernel_milliseconds() - start >= 5 && kernel_milliseconds() - start < 25);
    assert(tk_del_mbf(test_buffer) == E_OK);
    assert(tk_del_flg(test_flags) == E_OK);
    puts("micro-T scheduler, copied message and event tests passed.");
}
