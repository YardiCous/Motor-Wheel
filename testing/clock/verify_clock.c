/* Exercise real startup_clock() with reset and USB-boot register states. */
#include <tk/tkernel.h>
#include <assert.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#define TICK_ADDRESS 0x4005802cu
#define WATCHDOG_ADDRESS 0x40058000u
#define ONE_US_TICK 0x20cu

typedef struct {
    UW address;
    UW value;
} register_t;

static register_t registers[64];
static size_t register_count;
static unsigned tick_writes;

static UW *register_value(UW address)
{
    for (size_t i = 0; i < register_count; ++i) {
        if (registers[i].address == address) return &registers[i].value;
    }
    assert(register_count < 64);
    registers[register_count].address = address;
    return &registers[register_count++].value;
}

UW in_w(UW address)
{
    if (address == XOSC_STATUS) return XOSC_STATUS_STABLE;
    if (address == PLL_SYS_BASE + PLL_CS ||
        address == PLL_USB_BASE + PLL_CS) {
        return *register_value(address) | PLL_CS_LOCK;
    }
    if (address == CLK_REF + CLK_x_SELECTED) {
        return 1u << (*register_value(CLK_REF) & CLK_REF_CTRL_SRC);
    }
    if (address == CLK_SYS + CLK_x_SELECTED) {
        return 1u << (*register_value(CLK_SYS) & CLK_SYS_CTRL_SRC);
    }
    return *register_value(address);
}

void out_w(UW address, UW value)
{
    if (address == TICK_ADDRESS) {
        /* Tick configuration must follow the switch to the 12 MHz XOSC. */
        assert((*register_value(CLK_REF) & CLK_REF_CTRL_SRC) == 2u);
        assert(*register_value(CLK_REF + CLK_x_DIV) == 0x100u);
        ++tick_writes;
    }
    *register_value(address) = value;
}

void set_w(UW address, UW mask)
{
    out_w(address, in_w(address) | mask);
}

void clr_w(UW address, UW mask)
{
    out_w(address, in_w(address) & ~mask);
}

int main(void)
{
    const UW boot_ticks[] = {0x200u, ONE_US_TICK, 0u, 0x227u};
    for (size_t i = 0; i < sizeof(boot_ticks) / sizeof(boot_ticks[0]); ++i) {
        memset(registers, 0, sizeof(registers));
        register_count = 0;
        tick_writes = 0;
        *register_value(TICK_ADDRESS) = boot_ticks[i];
        *register_value(WATCHDOG_ADDRESS) = 0x07000000u;
        startup_clock(CLKATR_PREI | CLKATR_USB | CLKATR_ADC | CLKATR_RTC);
        assert(tick_writes == 1);
        assert(*register_value(TICK_ADDRESS) == ONE_US_TICK);
        /* Starting the tick must not enable a watchdog reset countdown. */
        assert(*register_value(WATCHDOG_ADDRESS) == 0x07000000u);
    }
    puts("PASS: real clock startup sets 1-us tick after reset and USB boot");
    return 0;
}
