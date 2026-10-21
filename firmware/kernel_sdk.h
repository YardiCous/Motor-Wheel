#ifndef ROBOT_KERNEL_SDK_H
#define ROBOT_KERNEL_SDK_H
#include <stdint.h>
/* newlib ctype.h uses _B; the legacy kernel declares a typedef with that name. */
#pragma push_macro("_B")
#undef _B
#include <tk/errno.h>
#include <tk/syscall.h>
#include <tk/typedef.h>
#pragma pop_macro("_B")
/* Native kernel task signature: INT start_code, void *exinf. */
_Noreturn void kernel_start(void (*entry)(INT, void *));
void kernel_irq_enter(void);
void kernel_irq_leave(void);
uint32_t kernel_milliseconds(void);
void kernel_wait_period(uint32_t *deadline, unsigned period_ms);
#endif
