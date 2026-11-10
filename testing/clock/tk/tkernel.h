#ifndef CLOCK_TEST_TKERNEL_H
#define CLOCK_TEST_TKERNEL_H

#include <stdint.h>
#include <sys/sysdepend/pico_rp2040/sysdef.h>

typedef uint32_t UW;
typedef uint32_t UINT;
typedef uint32_t ATR;
typedef uint64_t UD;
typedef volatile uint32_t _UW;

#define LOCAL static
#define EXPORT
/* Use the real clock implementation and register definitions without ARM
 * context-switch declarations from its local sysdepend.h. */
#define _SYSDEPEND_TARGET_SYSDEPEND_
#define CLKATR_PREI 1u
#define CLKATR_USB 2u
#define CLKATR_ADC 4u
#define CLKATR_RTC 8u

UW in_w(UW address);
void out_w(UW address, UW value);
void set_w(UW address, UW mask);
void clr_w(UW address, UW mask);
void startup_clock(ATR attributes);

#endif
