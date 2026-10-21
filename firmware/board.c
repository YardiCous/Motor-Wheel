#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"
#include "robot.h"
#include <stdlib.h>

static encoder_t enc_left, enc_right;
static struct {
    uint16_t rise_at;
    unsigned width : 15;
    unsigned armed : 1;
    unsigned rising : 1;
    unsigned ready : 1;
} echo;
static struct {
    uint16_t changed_at;
    unsigned raw : 1;
    unsigned stable : 1;
} buttons[3];
static const unsigned button_pins[] = {PIN_START, PIN_STOP, PIN_RESET};
static unsigned encoder_ab(unsigned a, unsigned b) {
    return (gpio_get(a) << 1u) | gpio_get(b);
}
static void gpio_irq(unsigned gpio, uint32_t events) {
    kernel_irq_enter();
    if (gpio == PIN_ENCODER_L_A || gpio == PIN_ENCODER_L_B) {
        encoder_edge(&enc_left, encoder_ab(PIN_ENCODER_L_A, PIN_ENCODER_L_B));
    } else if (gpio == PIN_ENCODER_R_A || gpio == PIN_ENCODER_R_B) {
        encoder_edge(&enc_right, encoder_ab(PIN_ENCODER_R_A, PIN_ENCODER_R_B));
    } else if (gpio == PIN_US_ECHO && echo.armed) {
        if (events & GPIO_IRQ_EDGE_RISE) {
            echo.rise_at = (uint16_t)time_us_32();
            echo.rising = 1;
        }
        if ((events & GPIO_IRQ_EDGE_FALL) && echo.rising) {
            unsigned width = (uint16_t)((uint16_t)time_us_32() - echo.rise_at);
            echo.width = width <= ECHO_TIMEOUT_US ? width : 0;
            echo.ready = 1;
            echo.armed = 0;
            echo.rising = 0;
            (void)tk_set_flg(evg_robot, FLG_ECHO);
        }
    } else if (gpio == PIN_ESTOP && (events & GPIO_IRQ_EDGE_FALL)) {
        robot_raise_fault(F_ESTOP);
    }
    kernel_irq_leave();
}
void board_init(void) {
    const unsigned motors[] = {PIN_MOTOR_L_A, PIN_MOTOR_L_B, PIN_MOTOR_R_A, PIN_MOTOR_R_B};
    for (unsigned i = 0; i < 4; ++i) {
        gpio_init(motors[i]);
        gpio_put(motors[i], 0);
        gpio_set_dir(motors[i], GPIO_OUT);
    }
    for (unsigned i = 0; i < 4; ++i) {
        gpio_set_function(motors[i], GPIO_FUNC_PWM);
        pwm_config config = pwm_get_default_config();
        pwm_config_set_wrap(&config, PWM_TOP);
        pwm_init(pwm_gpio_to_slice_num(motors[i]), &config, true);
        pwm_set_gpio_level(motors[i], 0);
    }
    gpio_init(PIN_US_TRIG);
    gpio_put(PIN_US_TRIG, 0);
    gpio_set_dir(PIN_US_TRIG, GPIO_OUT);
    gpio_init(PIN_US_ECHO);
    gpio_set_dir(PIN_US_ECHO, GPIO_IN);
    gpio_pull_down(PIN_US_ECHO);
    const unsigned encoders[] = {PIN_ENCODER_L_A, PIN_ENCODER_L_B, PIN_ENCODER_R_A,
                                 PIN_ENCODER_R_B};
    for (unsigned i = 0; i < 4; ++i) {
        gpio_init(encoders[i]);
        gpio_pull_up(encoders[i]);
    }
    encoder_init(&enc_left, encoder_ab(PIN_ENCODER_L_A, PIN_ENCODER_L_B));
    encoder_init(&enc_right, encoder_ab(PIN_ENCODER_R_A, PIN_ENCODER_R_B));
    for (unsigned i = 0; i < 3; ++i) {
        gpio_init(button_pins[i]);
        gpio_pull_up(button_pins[i]);
    }
    gpio_init(PIN_ESTOP);
    gpio_pull_up(PIN_ESTOP);
    adc_init();
    adc_gpio_init(PIN_IR_L);
    adc_gpio_init(PIN_IR_C);
    adc_gpio_init(PIN_IR_R);
    i2c_init(i2c1, 400000);
    gpio_set_function(PIN_IMU_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_IMU_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_IMU_SDA);
    gpio_pull_up(PIN_IMU_SCL);
    gpio_set_function(PIN_SERVO, GPIO_FUNC_PWM);
    pwm_config servo = pwm_get_default_config();
    pwm_config_set_clkdiv(&servo, 125.0f);
    pwm_config_set_wrap(&servo, 19999);
    pwm_init(pwm_gpio_to_slice_num(PIN_SERVO), &servo, true);
    board_servo_angle(90);
}
void board_enable_inputs(void) {
    gpio_set_irq_enabled_with_callback(PIN_US_ECHO, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true,
                                       gpio_irq);
    const unsigned pins[] = {PIN_ENCODER_L_A, PIN_ENCODER_L_B, PIN_ENCODER_R_A, PIN_ENCODER_R_B};
    for (unsigned i = 0; i < 4; ++i)
        gpio_set_irq_enabled(pins[i], GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(PIN_ESTOP, GPIO_IRQ_EDGE_FALL, true);
}
bool board_estop_active(void) {
    return !gpio_get(PIN_ESTOP);
}
void board_buttons_step(void) {
    uint16_t now = robot_now();
    for (unsigned i = 0; i < 3; ++i) {
        bool pressed = !gpio_get(button_pins[i]);
        if (pressed != buttons[i].raw) {
            buttons[i].raw = pressed;
            buttons[i].changed_at = now;
        }
        if (pressed != buttons[i].stable &&
            elapsed_ms(now, buttons[i].changed_at) >= BUTTON_DEBOUNCE_MS) {
            buttons[i].stable = pressed;
            if (pressed) {
                if (i == 0)
                    (void)robot_command(CMD_START, 0);
                if (i == 1)
                    (void)robot_command(CMD_STOP, 0);
                if (i == 2)
                    robot_local_reset();
            }
        }
    }
}
void board_encoder_snapshot(encoder_t *l, encoder_t *r) {
    uint32_t irq = save_and_disable_interrupts();
    *l = enc_left;
    *r = enc_right;
    restore_interrupts(irq);
}
void board_clear_encoder_fault(void) {
    uint32_t irq = save_and_disable_interrupts();
    enc_left.invalid = 0;
    enc_right.invalid = 0;
    restore_interrupts(irq);
}
static void motor(unsigned a, unsigned b, int duty) {
    if (duty > 1000)
        duty = 1000;
    if (duty < -1000)
        duty = -1000;
    pwm_set_gpio_level(a, 0);
    pwm_set_gpio_level(b, 0);
    if (duty)
        pwm_set_gpio_level(duty > 0 ? a : b, (unsigned)abs(duty) * PWM_TOP / 1000u);
}
void board_motor_write(int l, int r) {
    motor(PIN_MOTOR_L_A, PIN_MOTOR_L_B, l * MOTOR_LEFT_SIGN);
    motor(PIN_MOTOR_R_A, PIN_MOTOR_R_B, r * MOTOR_RIGHT_SIGN);
}
void board_servo_angle(unsigned degrees) {
    if (degrees > 180)
        degrees = 180;
    pwm_set_gpio_level(PIN_SERVO, 1000 + degrees * 1000 / 180);
}
void board_range_trigger(void) {
    uint32_t irq = save_and_disable_interrupts();
    echo.armed = 1;
    echo.rising = 0;
    echo.ready = 0;
    restore_interrupts(irq);
    gpio_put(PIN_US_TRIG, 1);
    busy_wait_us_32(10); /* bounded trigger pulse, never busy-wait for echo */
    gpio_put(PIN_US_TRIG, 0);
}
bool board_echo_take(unsigned *width) {
    uint32_t irq = save_and_disable_interrupts();
    bool ready = echo.ready;
    if (ready) {
        *width = echo.width;
        echo.ready = 0;
    }
    restore_interrupts(irq);
    return ready;
}
void board_echo_cancel(void) {
    uint32_t irq = save_and_disable_interrupts();
    echo.armed = 0;
    echo.rising = 0;
    echo.ready = 0;
    restore_interrupts(irq);
}
