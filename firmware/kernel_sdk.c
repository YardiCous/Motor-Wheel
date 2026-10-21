/* SDK-only half: the SDK owns startup, clocks and its RAM vector table. */
#include "kernel_sdk.h"
#include "hardware/clocks.h"
#include "hardware/exception.h"
#include "hardware/irq.h"
#include "pico/stdlib.h"

void sdk_kernel_install(void (*dispatch)(void), void (*tick)(void)) {
    if (clock_get_hz(clk_sys) != 125000000u)
        panic("micro-T requires 125 MHz");
    exception_set_exclusive_handler(PENDSV_EXCEPTION, dispatch);
    exception_set_exclusive_handler(SYSTICK_EXCEPTION, tick);
    /* Match the supplied port: SysTick must not interrupt a partial dispatch. */
    exception_set_priority(PENDSV_EXCEPTION, 0xff);
    exception_set_priority(SYSTICK_EXCEPTION, 0xff);
}
void sdk_kernel_irq(unsigned irq, void (*handler)(void)) {
    irq_set_enabled(irq, false);
    irq_handler_t old = irq_get_exclusive_handler(irq);
    if (old)
        irq_remove_handler(irq, old);
    if (handler)
        irq_set_exclusive_handler(irq, handler);
}
_Noreturn void sdk_kernel_panic(const char *message) {
    panic("%s", message);
}
uint32_t kernel_milliseconds(void) {
    SYSTIM time;
    tk_get_otm(&time);
    return time.lo;
}
void kernel_wait_period(uint32_t *deadline, unsigned period_ms) {
    *deadline += period_ms;
    int32_t remaining = (int32_t)(*deadline - kernel_milliseconds());
    if (remaining > 0) {
        /* micro-T adds one tick to delays. Subtract it for absolute cadence. */
        tk_dly_tsk((RELTIM)(remaining - 1));
    } else {
        *deadline = kernel_milliseconds();
        tk_dly_tsk(0); /* yield even after an overrun */
    }
}
