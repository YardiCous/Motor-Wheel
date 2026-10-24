/** @file board_io.c
 * @brief Direct register access is confined to this hardware driver.
 * References: RP2040 datasheet chapters 2.19, 4.3, 4.4, 4.5 and 4.9;
 * ST LSM303DLHC datasheet DM00027543, sections 7 and 8.
 * NOTE: The kernel wraps TA_HLNG handlers with its assembly ISR entry.
 */
#include "board_io.h"

#include "robot_config.h"

#include <tk/tkernel.h>

#if TM_CONSOLE_UART
#error Robot ultrasonic pins require USB console and CRASH_REPORT=0.
#endif /* TM_CONSOLE_UART */

#define BOARD_SIO 0xd0000000u
#define BOARD_IO 0x40014000u
#define BOARD_PADS 0x4001c000u
#define BOARD_PWM 0x40050000u
#define BOARD_ADC 0x4004c000u
#define BOARD_I2C 0x40048000u
#define BOARD_TIMER 0x40054000u
#define BOARD_RESETS 0x4000c000u
#define BOARD_GPIO_IRQ 13u
#define BOARD_TRIGGER_IRQ 3u
#define BOARD_ADC_TIMEOUT_US 100u
#define BOARD_I2C_TIMEOUT_US 1500u
#define BOARD_IMU_BYTES 6u
#define BOARD_GPIO_BANKS 3u
#define BOARD_GPIO_PINS 30u
#define BOARD_MOTOR_TOP 6249u
#define BOARD_IMU_ADDRESS 0x19u

#define BOARD_FIRST_MOTOR_PIN 8u
#define BOARD_LAST_PWM_PIN 12u
#define BOARD_FIRST_ADC_PIN 26u
#define BOARD_LAST_ADC_PIN 28u
#define BOARD_ENCODER_PINS 4u

/** @brief ISR-owned fields are read only while local interrupts are masked. */
typedef struct
{
    uint32_t left;
    uint32_t right;
    uint32_t rise_us;
    uint32_t echo_width : 15;
    uint32_t echo_state : 2;
    uint32_t last_left : 2;
    uint32_t last_right : 2;
    bool     b_encoder_fault : 1;
} capture_t;

static volatile capture_t g_capture        = {0};
static int32_t            gh_echo_flag     = 0;
static const int8_t       g_quadrature[16] = {0, 1, -1, 0,  -1, 0,  0, 1,
                                              1, 0, 0,  -1, 0,  -1, 1, 0};

static uint32_t read_reg(uint32_t address);
static void     write_reg(uint32_t address, uint32_t value);
static void     configure_pin(uint8_t pin, uint8_t function, bool b_output,
                              bool b_pullup);
static bool     i2c_wait(uint32_t offset, uint32_t mask);
static bool     i2c_transfer(uint8_t address, uint8_t value, uint8_t * p_bytes,
                             uint8_t length);
static void     encoder_edge(uint32_t inputs, bool b_left);
static void     board_gpio_isr(uint32_t number);
static void     board_trigger_isr(uint32_t number);

/** @brief All pins are configured before any application task starts. */
bool board_io_init (int32_t h_echo_flag)
{
    gh_echo_flag = h_echo_flag;
    write_reg(BOARD_RESETS + 0x3000u, 0x00004011u);
    uint32_t start = board_io_time_us();

    while (((read_reg(BOARD_RESETS + 8u) & 0x4011u) != 0x4011u) &&
           ((board_io_time_us() - start) < BOARD_I2C_TIMEOUT_US))
    {
        /* Peripheral reset completion is bounded by the hardware timer. */
    }

    for (uint8_t pin = BOARD_FIRST_MOTOR_PIN; pin <= BOARD_LAST_PWM_PIN; ++pin)
    {
        configure_pin(pin, 4u, true, false);
    }

    write_reg(BOARD_PWM + (4u * 20u) + 16u, BOARD_MOTOR_TOP);
    write_reg(BOARD_PWM + (5u * 20u) + 16u, BOARD_MOTOR_TOP);
    write_reg(BOARD_PWM + (4u * 20u) + 4u, 16u);
    write_reg(BOARD_PWM + (5u * 20u) + 4u, 16u);
    board_io_motor(0, 0);
    write_reg(BOARD_PWM + (4u * 20u), 1u);
    write_reg(BOARD_PWM + (5u * 20u), 1u);
    write_reg(BOARD_PWM + (6u * 20u) + 4u, 2000u);
    write_reg(BOARD_PWM + (6u * 20u) + 16u, 19999u);
    board_io_servo(30u);
    write_reg(BOARD_PWM + (6u * 20u), 1u);

    static const uint8_t encoder_pins[4] = {4u, 5u, 16u, 17u};

    for (uint8_t index = 0u; index < BOARD_ENCODER_PINS; ++index)
    {
        configure_pin(encoder_pins[index], 5u, false, true);
    }

    configure_pin(0u, 5u, true, false);
    configure_pin(1u, 5u, false, false);
    configure_pin(2u, 3u, false, true);
    configure_pin(3u, 3u, false, true);
    configure_pin(ROBOT_TRACE_PIN, 5u, true, false);
    configure_pin(ROBOT_START_PIN, 5u, false, true);
    configure_pin(ROBOT_STOP_PIN, 5u, false, true);
    configure_pin(ROBOT_RESET_PIN, 5u, false, true);

    for (uint8_t pin = BOARD_FIRST_ADC_PIN; pin <= BOARD_LAST_ADC_PIN; ++pin)
    {
        configure_pin(pin, 31u, false, false);
        write_reg(BOARD_PADS + 4u + (pin * 4u), 0u);
    }

    write_reg(BOARD_ADC, 1u);
    uint32_t inputs         = read_reg(BOARD_SIO + 4u);
    g_capture.last_left     = (inputs >> 4u) & 3u;
    g_capture.last_right    = (inputs >> 16u) & 3u;
    T_DINT interrupt_config = {0};
    interrupt_config.intatr = TA_HLNG;
    interrupt_config.inthdr = board_gpio_isr;
    bool b_valid = (E_OK == tk_def_int(BOARD_GPIO_IRQ, &interrupt_config));
    interrupt_config.inthdr = board_trigger_isr;
    b_valid =
        b_valid && (E_OK == tk_def_int(BOARD_TRIGGER_IRQ, &interrupt_config));
    write_reg(BOARD_IO + 0xf0u, 0x00cc00c0u);
    write_reg(BOARD_IO + 0xf8u, 0x000000ccu);
    write_reg(BOARD_IO + 0x100u, 0x00cc00c0u);
    write_reg(BOARD_IO + 0x108u, 0x000000ccu);
    EnableInt(BOARD_GPIO_IRQ, 2);
    EnableInt(BOARD_TRIGGER_IRQ, 2);
    return b_valid;
}

/** @brief All callers and writers are pinned to processor 1, including MQTT.
 */
uint32_t board_io_lock (void)
{
    return disint();
}

/** @brief A driver-local compiler extension restores the CPU mask. */
void board_io_unlock (uint32_t mask)
{
    __asm__ volatile("msr primask, %0" : : "r"(mask) : "memory");
}

/** @brief RP2040's raw timer wraps naturally after about 71 minutes. */
uint32_t board_io_time_us (void)
{
    return read_reg(BOARD_TIMER + 0x28u);
}

/** @brief Kernel time avoids deriving milliseconds from a wrapping us timer.
 */
uint32_t board_io_time_ms (void)
{
    SYSTIM time = {0};
    (void)tk_get_tim(&time);
    return time.lo;
}

/** @brief No ISR read-modify-write races with this coherent pair copy. */
bool board_io_encoders (uint32_t * p_left, uint32_t * p_right)
{
    uint32_t mask = board_io_lock();
    *p_left       = g_capture.left;
    *p_right      = g_capture.right;
    bool b_valid  = !g_capture.b_encoder_fault;
    board_io_unlock(mask);
    return b_valid;
}

/** @brief Reset is acknowledged only with the safety latch still asserted. */
void board_io_clear_fault (void)
{
    uint32_t mask             = board_io_lock();
    g_capture.b_encoder_fault = false;
    board_io_unlock(mask);
}

/** @brief Each wheel changes its two bridge channels in one register write. */
void board_io_motor (int16_t left, int16_t right)
{
    int32_t left_drive  = left * ROBOT_MOTOR_LEFT_SIGN;
    int32_t right_drive = right * ROBOT_MOTOR_RIGHT_SIGN;
    left_drive          = (left_drive < -1000) ? -1000 : left_drive;
    left_drive          = (left_drive > 1000) ? 1000 : left_drive;
    right_drive         = (right_drive < -1000) ? -1000 : right_drive;
    right_drive         = (right_drive > 1000) ? 1000 : right_drive;
    /* Saturated magnitudes are nonnegative and at most 1000. */
    uint32_t left_level =
        (uint32_t)((left_drive < 0) ? -left_drive : left_drive);
    uint32_t right_level =
        (uint32_t)((right_drive < 0) ? -right_drive : right_drive);
    left_level  = (left_level * (BOARD_MOTOR_TOP + 1u)) / 1000u;
    right_level = (right_level * (BOARD_MOTOR_TOP + 1u)) / 1000u;
    left_level  = (left_drive < 0) ? (left_level << 16u) : left_level;
    right_level = (right_drive < 0) ? (right_level << 16u) : right_level;
    write_reg(BOARD_PWM + (4u * 20u) + 12u, left_level);
    write_reg(BOARD_PWM + (5u * 20u) + 12u, right_level);
}

/** @brief Each ADC conversion has an independent 100-us deadline. */
bool board_io_line (uint16_t * p_samples)
{
    bool b_valid = true;

    for (uint8_t channel = 0u; channel < ROBOT_IR_COUNT; ++channel)
    {
        write_reg(BOARD_ADC, 1u | ((uint32_t)channel << 12u) | 4u);
        uint32_t start = board_io_time_us();

        while ((0u == (read_reg(BOARD_ADC) & 0x100u)) &&
               ((board_io_time_us() - start) < BOARD_ADC_TIMEOUT_US))
        {
            /* Only the short ADC conversion is polled; echo uses an ISR. */
        }

        b_valid = b_valid && (0u != (read_reg(BOARD_ADC) & 0x100u));
        /* The hardware result register contains a 12-bit ADC value. */
        p_samples[channel] = (uint16_t)(read_reg(BOARD_ADC + 4u) & 4095u);
    }

    return b_valid;
}

/** @brief 400-kHz I2C assumes the repository's 125-MHz peripheral clock. */
bool board_io_imu_init (void)
{
    write_reg(BOARD_I2C + 0x6cu, 0u);
    write_reg(BOARD_I2C, 0x65u);
    write_reg(BOARD_I2C + 0x1cu, 126u);
    write_reg(BOARD_I2C + 0x20u, 187u);
    write_reg(BOARD_I2C + 0xa0u, 10u);
    write_reg(BOARD_I2C + 0x7cu, 38u);
    write_reg(BOARD_I2C + 4u, BOARD_IMU_ADDRESS);
    write_reg(BOARD_I2C + 0x6cu, 1u);
    bool b_valid = i2c_transfer(0x20u, 0x57u, NULL, 0u);
    b_valid      = b_valid && (i2c_transfer(0x23u, 0x98u, NULL, 0u));
    return b_valid;
}

/** @brief BDU and auto-increment provide one coherent high-resolution sample.
 */
bool board_io_imu (int16_t * p_x, int16_t * p_y, int16_t * p_z)
{
    uint8_t bytes[BOARD_IMU_BYTES] = {0u};
    int16_t axes[ROBOT_IR_COUNT]   = {0};
    bool    b_valid = i2c_transfer(0xa8u, 0u, bytes, BOARD_IMU_BYTES);

    if (b_valid)
    {
        for (uint8_t axis = 0u; axis < ROBOT_IR_COUNT; ++axis)
        {
            uint32_t raw =
                bytes[axis * 2u] | ((uint32_t)bytes[(axis * 2u) + 1u] << 8u);
            /* Raw is 0..65535; subtract before scaling to avoid signed shifts.
             */
            int32_t signed_raw =
                (raw >= 32768u) ? ((int32_t)raw - 65536) : (int32_t)raw;
            /* +/-4g HR mode is 2 mg per 12-bit digit, left aligned. */
            axes[axis] = (int16_t)((signed_raw / 16) * 2);
        }

        *p_x = axes[0];
        *p_y = axes[1];
        *p_z = axes[2];
    }

    return b_valid;
}

/** @brief 500..2500-us pulses map the configured 180-degree servo range. */
void board_io_servo (uint8_t angle)
{
    uint32_t bounded = (angle > 60u) ? 60u : angle;
    uint32_t pulse   = 500u + ((bounded * 2000u) / 60u);
    write_reg(BOARD_PWM + (6u * 20u) + 12u, pulse);
}

/** @brief TIMER alarm 3 ends the pulse even if a task is preempted. */
void board_io_ping (void)
{
    uint32_t mask        = board_io_lock();
    g_capture.echo_state = 1u;
    write_reg(BOARD_TIMER + 0x34u, 8u);
    write_reg(BOARD_TIMER + 0x2038u, 8u);
    write_reg(BOARD_SIO + 0x14u, 1u);
    write_reg(BOARD_TIMER + 0x1cu, board_io_time_us() + 10u);
    board_io_unlock(mask);
}

/** @brief The usable 2..400-cm range is a nine-bit value. */
bool board_io_range (uint16_t * p_cm)
{
    uint32_t mask        = board_io_lock();
    uint32_t width       = g_capture.echo_width;
    bool     b_valid     = (3u == g_capture.echo_state);
    g_capture.echo_state = 0u;
    board_io_unlock(mask);
    uint32_t cm = (width + 29u) / 58u;
    b_valid     = b_valid && (cm >= 2u) && (cm <= ROBOT_RANGE_MAX_CM);
    /* Range is no greater than 565 before its validity check. */
    *p_cm = b_valid ? (uint16_t)cm : 0u;
    return b_valid;
}

/** @brief Internal pull-ups let switches close directly to ground. */
uint8_t board_io_buttons (void)
{
    uint32_t input = read_reg(BOARD_SIO + 4u);
    /* Pins 20..22 map to three individual active-low bits. */
    return (uint8_t)((~input >> ROBOT_START_PIN) & 7u);
}

/** @brief The scope captures actual loop starts without console jitter. */
void board_io_trace (bool b_high)
{
    write_reg(BOARD_SIO + (b_high ? 0x14u : 0x18u), 1u << ROBOT_TRACE_PIN);
}

/** @brief The RP2040 physical register addresses fit its 32-bit pointers. */
static uint32_t read_reg (uint32_t address)
{
    /* The address is an aligned peripheral register from the RP2040 map. */
    volatile uint32_t * p_register = (volatile uint32_t *)address;
    return *p_register;
}

/** @brief Only aligned, documented peripheral addresses reach this helper. */
static void write_reg (uint32_t address, uint32_t value)
{
    /* The address is an aligned peripheral register from the RP2040 map. */
    volatile uint32_t * p_register = (volatile uint32_t *)address;
    *p_register                    = value;
}

/** @brief Configure mux, output enable and pad bias before use. */
static void configure_pin (uint8_t pin, uint8_t function, bool b_output,
                          bool b_pullup)
{
    write_reg(BOARD_IO + 4u + (pin * 8u), function);
    write_reg(BOARD_PADS + 4u + (pin * 4u), 0x42u | (b_pullup ? 8u : 0u));
    write_reg(BOARD_SIO + (b_output ? 0x24u : 0x28u), 1u << pin);
    write_reg(BOARD_SIO + 0x18u, 1u << pin);
}

/** @brief Polling I2C is bounded and remains preemptible by control tasks. */
static bool i2c_wait (uint32_t offset, uint32_t mask)
{
    uint32_t start     = board_io_time_us();
    bool     b_aborted = false;

    while ((0u == (read_reg(BOARD_I2C + offset) & mask)) && (!b_aborted) &&
           ((board_io_time_us() - start) < BOARD_I2C_TIMEOUT_US))
    {
        b_aborted = (0u != (read_reg(BOARD_I2C + 0x34u) & 0x40u));
    }

    return (!b_aborted) && (0u != (read_reg(BOARD_I2C + offset) & mask));
}

/** @brief Six receive requests fit entirely in the hardware's 16-entry FIFO.
 */
static bool i2c_transfer (uint8_t address, uint8_t value, uint8_t * p_bytes,
                         uint8_t length)
{
    (void)read_reg(BOARD_I2C + 0x40u);
    write_reg(BOARD_I2C + 0x10u, address);
    bool b_valid = true;

    if (0u == length)
    {
        write_reg(BOARD_I2C + 0x10u, (uint32_t)value | 0x200u);
    }
    else
    {
        for (uint8_t index = 0u; index < length; ++index)
        {
            uint32_t command = 0x100u;
            command |= (0u == index) ? 0x400u : 0u;
            command |= ((index + 1u) == length) ? 0x200u : 0u;
            write_reg(BOARD_I2C + 0x10u, command);
        }

        for (uint8_t index = 0u; index < length; ++index)
        {
            b_valid = b_valid && (i2c_wait(0x70u, 8u));
            /* Reading the data FIFO exposes only the low eight data bits. */
            p_bytes[index] = (uint8_t)(read_reg(BOARD_I2C + 0x10u) & 255u);
        }
    }

    b_valid = b_valid && (i2c_wait(0x34u, 0x200u));
    (void)read_reg(BOARD_I2C + 0x60u);

    if (!b_valid)
    {
        write_reg(BOARD_I2C + 0x6cu, 2u);
        write_reg(BOARD_I2C + 0x6cu, 0u);
        write_reg(BOARD_I2C + 0x6cu, 1u);
    }

    return b_valid;
}

/** @brief A quadrature transition supplies direction during braking too. */
static void encoder_edge (uint32_t inputs, bool b_left)
{
    uint32_t previous = b_left ? g_capture.last_left : g_capture.last_right;
    uint32_t current  = (inputs >> (b_left ? 4u : 16u)) & 3u;
    int32_t  step     = g_quadrature[(previous << 2u) | current];
    step *= b_left ? ROBOT_ENCODER_LEFT_SIGN : ROBOT_ENCODER_RIGHT_SIGN;
    g_capture.b_encoder_fault =
        (g_capture.b_encoder_fault) || ((3u == (previous ^ current)));

    if (b_left)
    {
        g_capture.last_left = current;
        g_capture.left += (step > 0) ? 1u : 0u;
        g_capture.left -= (step < 0) ? 1u : 0u;
    }
    else
    {
        g_capture.last_right = current;
        g_capture.right += (step > 0) ? 1u : 0u;
        g_capture.right -= (step < 0) ? 1u : 0u;
    }
}

/** @brief T-Kernel's TA_HLNG wrapper supplies the actual ISR entry/exit. */
static void board_gpio_isr (uint32_t number)
{
    (void)number;
    uint32_t inputs = read_reg(BOARD_SIO + 4u);
    uint32_t first  = read_reg(BOARD_IO + 0x120u);
    uint32_t third  = read_reg(BOARD_IO + 0x128u);
    write_reg(BOARD_IO + 0xf0u, first);
    write_reg(BOARD_IO + 0xf8u, third);

    if (0u != (first & 0x00cc0000u))
    {
        encoder_edge(inputs, true);
    }

    if (0u != (third & 0xccu))
    {
        encoder_edge(inputs, false);
    }

    if (0u != (first & 0xc0u))
    {
        bool b_high = (0u != (inputs & 2u));

        if (b_high && (1u == g_capture.echo_state))
        {
            g_capture.rise_us    = board_io_time_us();
            g_capture.echo_state = 2u;
        }
        else if ((!b_high) && (2u == g_capture.echo_state))
        {
            uint32_t width       = board_io_time_us() - g_capture.rise_us;
            g_capture.echo_width = (width <= 32767u) ? width : 0u;
            g_capture.echo_state = 3u;
            (void)tk_set_flg(gh_echo_flag, 1u);
        }
        else
        {
            /* Ignore stale edges and unmatched falling edges. */
        }
    }
}

/** @brief The pulse completes without task delay or a busy-wait loop. */
static void board_trigger_isr (uint32_t number)
{
    (void)number;
    write_reg(BOARD_SIO + 0x18u, 1u);
    write_reg(BOARD_TIMER + 0x34u, 8u);
}

/*** End of file. ***/

