#!/usr/bin/env python3
"""Exercise the real startup adapter with simulated kernel/interrupt boundaries.

This checks startup ordering, not RP2040 interrupt delivery or kernel scheduling.
The production image and on-board tests are still cross-compiled separately.
"""
import pathlib
import subprocess
import tempfile

ROOT = pathlib.Path(__file__).resolve().parents[1]
HEADER = r"""
#ifndef TEST_KERNEL_H
#define TEST_KERNEL_H
#include <stdint.h>
typedef unsigned UINT;
typedef unsigned UW;
typedef int INT;
typedef int ER;
typedef int ID;
typedef unsigned ATR;
typedef void (*FP)();
typedef struct {
    unsigned tskatr;
    FP task;
    int itskpri;
    int stksz;
} T_CTSK;
#define E_OK 0
#define E_PAR (-17)
#define TA_HLNG 1
#define TA_RNG0 0
#define DDS_DISABLE 2
extern unsigned test_primask;
extern void *knl_ctxtsk, *knl_schedtsk;
extern int knl_dispatch_disabled;
#define DISABLE_INTERRUPT ((void)(test_primask = 1))
#define DI(saved) ((saved) = test_primask, test_primask = 1)
#define EI(saved) ((void)(test_primask = (saved)))
#define ENTER_TASK_INDEPENDENT ((void)0)
#define LEAVE_TASK_INDEPENDENT ((void)0)
void knl_dispatch(void);
unsigned knl_get_ipsr(void);
void knl_timer_handler(void);
ER knl_init_Imalloc(void);
ER knl_init_object(void);
ER knl_timer_startup(void);
ID tk_cre_tsk(const T_CTSK *task);
ER tk_sta_tsk(ID id, INT code);
_Noreturn void knl_force_dispatch(void);
#endif
"""
HARNESS = r"""
#include "kernel.h"
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>
static jmp_buf finished;
static unsigned timer_ready, task_ready, sdk_enables_interrupts;
unsigned test_primask;
void *knl_ctxtsk, *knl_schedtsk;
int knl_dispatch_disabled;
void kernel_start(void (*entry)(INT, void *));
void sdk_kernel_install(void (*dispatch)(void), void (*tick)(void)) {
    assert(dispatch && tick);
}
void sdk_kernel_irq(unsigned irq, void (*handler)(void)) {
    (void)irq; (void)handler;
}
_Noreturn void sdk_kernel_panic(const char *message) {
    fprintf(stderr, "unexpected panic: %s\n", message);
    exit(1);
}
void knl_dispatch_entry(void) {}
void knl_tmp_stack_paint(void) {}
void knl_dispatch(void) {}
unsigned knl_get_ipsr(void) { return 16; }
void knl_timer_handler(void) {}
ER knl_init_Imalloc(void) { return E_OK; }
ER knl_init_object(void) {
    knl_dispatch_disabled = 0;
    return E_OK;
}
ER knl_timer_startup(void) {
    assert(!task_ready);
    assert(knl_dispatch_disabled == DDS_DISABLE);
    timer_ready = 1;
    return E_OK;
}
ID tk_cre_tsk(const T_CTSK *task) {
    assert(task->task && timer_ready);
    /* Exercise a possible interrupt-state transition in SDK dependencies. */
    if (sdk_enables_interrupts) test_primask = 0;
    return 1;
}
ER tk_sta_tsk(ID id, INT code) {
    assert(id == 1 && code == 0 && timer_ready);
    assert(knl_dispatch_disabled == DDS_DISABLE);
    task_ready = 1;
    return E_OK;
}
_Noreturn void knl_force_dispatch(void) {
    assert(timer_ready && task_ready);
    longjmp(finished, 1);
}
static void entry(INT code, void *extra) { (void)code; (void)extra; }
int main(void) {
    for (sdk_enables_interrupts = 0; sdk_enables_interrupts < 2; ++sdk_enables_interrupts) {
        timer_ready = task_ready = test_primask = 0;
        if (!setjmp(finished)) kernel_start(entry);
    }
    puts("Kernel startup ordering and dispatch guards passed (2 scenarios).");
}
"""

with tempfile.TemporaryDirectory(prefix="motor-wheel-startup-") as directory:
    work = pathlib.Path(directory)
    (work / "kernel.h").write_text(HEADER)
    (work / "harness.c").write_text(HARNESS)
    subprocess.run([
        "cc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-I", str(work),
        str(ROOT / "firmware/kernel_core.c"), str(work / "harness.c"),
        "-o", str(work / "startup-test"),
    ], check=True)
    subprocess.run([str(work / "startup-test")], check=True)
