/* Kernel-only half of SDK adapter: keep legacy BSP register macros private. */
#include "kernel.h"
#include <stdint.h>
extern void sdk_kernel_install(void (*dispatch)(void), void (*tick)(void));
extern void sdk_kernel_irq(unsigned irq, void (*handler)(void));
extern _Noreturn void sdk_kernel_panic(const char *message);
static uint64_t kernel_arena[48 * 1024 / sizeof(uint64_t)];
void *knl_lowmem_top = kernel_arena;
void *knl_lowmem_limit = kernel_arena + sizeof(kernel_arena) / sizeof(kernel_arena[0]);
static FP handlers[32];
extern void knl_dispatch_entry(void);
extern void knl_tmp_stack_paint(void);

void kernel_irq_enter(void) {
    UINT mask;
    DI(mask);
    ENTER_TASK_INDEPENDENT;
    EI(mask);
}
void kernel_irq_leave(void) {
    UINT mask;
    DI(mask);
    LEAVE_TASK_INDEPENDENT;
    if (knl_ctxtsk != knl_schedtsk && !knl_dispatch_disabled)
        knl_dispatch();
    EI(mask);
}
static void timer_interrupt(void) {
    kernel_irq_enter();
    knl_timer_handler();
    kernel_irq_leave();
}
static void generic_interrupt(void) {
    unsigned irq = knl_get_ipsr() - 16u;
    kernel_irq_enter();
    if (irq < 32 && handlers[irq])
        handlers[irq](irq);
    kernel_irq_leave();
}
ER knl_define_inthdr(INT irq, ATR attributes, FP handler) {
    if (irq < 0 || irq >= 26)
        return E_PAR;
    handlers[irq] = handler;
    sdk_kernel_irq((unsigned)irq,
                   handler ? ((attributes & TA_HLNG) ? generic_interrupt : (void (*)(void))handler)
                           : 0);
    return E_OK;
}
void knl_return_inthdr(void) {}
_Noreturn void kernel_start(void (*entry)(INT, void *)) {
    DISABLE_INTERRUPT;
    knl_tmp_stack_paint();
    sdk_kernel_install(knl_dispatch_entry, timer_interrupt);
    if (knl_init_Imalloc() < E_OK || knl_init_object() < E_OK)
        sdk_kernel_panic("micro-T initialization");
    T_CTSK init = {.tskatr = TA_HLNG | TA_RNG0, .task = (FP)entry, .itskpri = 1, .stksz = 4096};
    ID task = tk_cre_tsk(&init);
    if (task < E_OK || tk_sta_tsk(task, 0) < E_OK)
        sdk_kernel_panic("micro-T initial task");
    if (knl_timer_startup() < E_OK)
        sdk_kernel_panic("micro-T timer");
    knl_force_dispatch();
    for (;;)
        __asm__ volatile("nop");
}
